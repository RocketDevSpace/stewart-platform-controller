"""
hardware/serial_manager.py

Owns serial connection lifecycle, the background read loop, and a
latest-wins streaming writer. Does NOT format commands — that is
servo_driver.py's responsibility.

Port and baud are injected by the caller (sourced from settings.py there).
No config values are hardcoded here.

Thread model:
- Any thread may call send() / send_latest() (writes are lock-serialized;
  the GUI thread and the vision worker thread both send in production).
- One background reader thread emits complete lines to the receive callback.
- One background writer thread drains the depth-1 "latest" slot, so a
  streaming caller (the vision loop) never blocks on serial I/O and stale
  setpoints are coalesced away instead of queueing.
- Callbacks are invoked on background threads; Qt consumers must bridge via
  signals (the GUI already does).
"""

import logging
import threading
import time
from collections.abc import Callable
from typing import Any

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None  # type: ignore[assignment]

logger = logging.getLogger(__name__)

# Post-open boot wait: opening the port toggles DTR, which resets the
# Arduino; the firmware needs ~2 s to boot before it starts parsing
# commands. The current firmware prints "[READY]\n" when it is up, so
# connect() poll-reads for that banner (up to _READY_WAIT_S) and returns
# as soon as it arrives; if it never does (old firmware may not print
# it), connect() falls back to the plain _ARDUINO_BOOT_DELAY_S wait.
_ARDUINO_BOOT_DELAY_S = 2.0
_READY_BANNER = "[READY"   # matches both "[READY]" (v1) and "[READY v2]"
_READY_WAIT_S = 3.0
# v1 firmware baud; connect() retries here when the configured (v2) rate
# produces no banner.
_LEGACY_BAUD = 115200
_WRITE_TIMEOUT_S = 0.5
_THREAD_JOIN_TIMEOUT_S = 2.0

# RTT telemetry: EMA smoothing weight for new round-trip samples.
_RTT_EMA_ALPHA = 0.2
# Byte prefixes that count as firmware commands for RTT matching (the
# firmware acks each with "[OK] ..." — or "k" for the future terse ack).
_RTT_COMMAND_PREFIXES = (b"S,", b"T,")


class SerialManager:
    """Manages a single serial connection, a reader thread, and a
    latest-wins writer thread."""

    def __init__(self, port: str, baud: int) -> None:
        self._port = port
        self._baud = baud
        self._ser: Any = None
        self._running = False
        self._read_thread: threading.Thread | None = None
        self._write_thread: threading.Thread | None = None
        self._callback: Callable[[str], None] | None = None
        self._disconnect_callback: Callable[[str], None] | None = None
        self._write_lock = threading.Lock()
        # Depth-1 latest-wins slot for streaming senders.
        self._latest_cond = threading.Condition()
        self._latest_data: bytes | None = None
        self._disconnect_reported = False
        # Firmware version parsed from the boot banner: "v2" ([READY v2]),
        # "v1" ([READY]), or "unknown" (no banner seen). Consumers use this
        # to decide protocol capability (v2 = tenth-degree T commands).
        self.firmware_version = "unknown"
        # --- RTT telemetry (write -> firmware ack round trip) ---
        # A sample is only trusted when EXACTLY one command was outstanding
        # at ack time; overlapping commands make the pairing ambiguous and
        # the sample is discarded (unmatched).
        self._rtt_lock = threading.Lock()
        self._rtt_last_write_ts: float | None = None
        self._rtt_outstanding = 0
        self._rtt_ema_ms = 0.0
        self._rtt_last_ms = 0.0
        self._rtt_samples = 0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Open the serial port and start the background threads.

        No-op (returns True) if already connected — reconnecting while a
        reader thread is alive would split the byte stream between two
        threads and corrupt line framing.

        Returns True on success, False on failure (never raises).
        NOTE: blocks ~2 s for the Arduino boot after the DTR reset; call it
        off the GUI thread.
        """
        if serial is None:
            logger.error("pyserial is not installed")
            return False

        if self.is_connected():
            logger.warning("connect() called while already connected — ignored")
            return True

        try:
            self._ser = serial.Serial(
                self._port,
                self._baud,
                timeout=0.1,
                write_timeout=_WRITE_TIMEOUT_S,
            )
            # Arduino resets on the DTR toggle; wait for the firmware's
            # "[READY ...]" banner instead of an open-loop sleep.
            active_baud = self._baud
            banner_seen = self._wait_for_ready_banner()
            if not banner_seen and self._baud != _LEGACY_BAUD:
                # No banner at the configured rate — likely v1 firmware at
                # its legacy baud. Reopen once at 115200 (costs one more
                # boot cycle) before giving up on the banner.
                logger.warning(
                    "no banner at %d baud; retrying at legacy %d",
                    self._baud, _LEGACY_BAUD,
                )
                self._ser.close()
                self._ser = serial.Serial(
                    self._port,
                    _LEGACY_BAUD,
                    timeout=0.1,
                    write_timeout=_WRITE_TIMEOUT_S,
                )
                active_baud = _LEGACY_BAUD
                self._wait_for_ready_banner()
            logger.info(
                "Connected to %s at %d baud (firmware %s)",
                self._port, active_baud, self.firmware_version,
            )
        except Exception as exc:
            logger.error("Could not connect: %s", exc)
            self._ser = None
            return False

        self._running = True
        self._disconnect_reported = False
        self._read_thread = threading.Thread(
            target=self._read_loop, daemon=True, name="serial-read"
        )
        self._read_thread.start()
        self._write_thread = threading.Thread(
            target=self._write_loop, daemon=True, name="serial-write"
        )
        self._write_thread.start()
        return True

    def disconnect(self) -> None:
        """Stop the background threads and close the serial port."""
        self._running = False
        with self._latest_cond:
            self._latest_data = None
            self._latest_cond.notify_all()
        for thread in (self._read_thread, self._write_thread):
            if thread is not None and thread.is_alive():
                thread.join(timeout=_THREAD_JOIN_TIMEOUT_S)
                if thread.is_alive():
                    logger.warning("%s thread did not exit in time", thread.name)
        self._read_thread = None
        self._write_thread = None
        if self._ser is not None:
            try:
                if self._ser.is_open:
                    self._ser.close()
                    logger.info("Disconnected")
            except Exception as exc:
                logger.error("Disconnect failed: %s", exc)
        self._ser = None

    def send(self, data: bytes) -> bool:
        """Write raw bytes to the serial port (synchronous, lock-serialized).

        Returns True on success, False on failure (never raises).
        """
        if not self.is_connected():
            logger.error("Send failed: not connected")
            return False

        try:
            with self._write_lock:
                self._ser.write(data)
            self._note_write(data)
            return True
        except Exception as exc:
            logger.error("Send failed: %s", exc)
            self._handle_link_error(f"write failed: {exc}")
            return False

    def send_latest(self, data: bytes) -> bool:
        """Queue bytes for the streaming writer thread (latest-wins, depth 1).

        If a previous payload is still waiting, it is REPLACED — a control
        loop streaming setpoints must never let stale commands accumulate.
        Returns True if the payload was accepted (link up), False otherwise.
        """
        if not self.is_connected():
            return False
        with self._latest_cond:
            self._latest_data = data
            self._latest_cond.notify()
        return True

    def set_receive_callback(self, callback: Callable[[str], None]) -> None:
        """Register a callback invoked with each complete line received.

        Called on the reader thread. Exceptions raised by the callback are
        logged and do NOT kill the read loop.
        """
        self._callback = callback

    def set_disconnect_callback(self, callback: Callable[[str], None]) -> None:
        """Register a callback invoked once with a reason string when the
        link dies unexpectedly (USB unplug, I/O error). Called on a
        background thread."""
        self._disconnect_callback = callback

    def rtt_stats(self) -> tuple[float, float, int]:
        """Return (ema_ms, last_ms, samples) for the command->ack round trip.

        ema_ms is an exponential moving average (alpha 0.2) of matched
        samples; last_ms is the most recent matched sample; samples is the
        matched-sample count. All zeros until the first matched ack.
        Thread-safe.
        """
        with self._rtt_lock:
            return self._rtt_ema_ms, self._rtt_last_ms, self._rtt_samples

    def is_connected(self) -> bool:
        """Return True if the serial port is currently open."""
        if self._ser is None:
            return False
        try:
            return bool(self._ser.is_open)
        except Exception:
            return False

    # ------------------------------------------------------------------
    # Private
    # ------------------------------------------------------------------

    def _wait_for_ready_banner(self) -> bool:
        """Poll-read for the firmware's "[READY ...]" boot banner.

        Runs during connect(), BEFORE the reader thread starts, so reading
        directly from the port here is safe (single reader). Parses the
        firmware version from the banner ("[READY v2]" -> v2, plain
        "[READY]" -> v1) into self.firmware_version. Returns True when a
        banner arrived, False after _READY_WAIT_S without one (old firmware
        may not print one — by then the plain boot delay has elapsed
        anyway). If polling itself fails, falls back to sleeping out the
        remainder of _ARDUINO_BOOT_DELAY_S.
        """
        start = time.monotonic()
        buffer = ""
        try:
            while (time.monotonic() - start) < _READY_WAIT_S:
                raw = self._ser.read(64)  # bounded by the port timeout
                if raw:
                    buffer += raw.decode("utf-8", errors="ignore")
                    idx = buffer.find(_READY_BANNER)
                    if idx >= 0:
                        # Wait for the banner's closing bracket so a
                        # partially-arrived "[READY v2]" isn't misread
                        # as v1 (the tail may be in flight).
                        tail_deadline = time.monotonic() + 0.2
                        while (
                            "]" not in buffer[idx:]
                            and time.monotonic() < tail_deadline
                        ):
                            more = self._ser.read(64)
                            if more:
                                buffer += more.decode("utf-8", errors="ignore")
                        if "[READY v2]" in buffer:
                            self.firmware_version = "v2"
                        else:
                            self.firmware_version = "v1"
                        logger.info(
                            "Firmware ready banner received (%s)",
                            self.firmware_version,
                        )
                        return True
        except Exception as exc:
            logger.warning(
                "Ready-banner poll failed (%s); falling back to fixed "
                "boot delay", exc,
            )
            remaining = _ARDUINO_BOOT_DELAY_S - (time.monotonic() - start)
            if remaining > 0:
                time.sleep(remaining)
            return False
        logger.warning(
            "No %s banner within %.1f s — assuming old firmware "
            "(boot delay elapsed)", _READY_BANNER, _READY_WAIT_S,
        )
        return False

    def _note_write(self, data: bytes) -> None:
        """Stamp a completed write for RTT pairing (called after every
        successful write, from send() — which is also the _write_loop
        path). Command payloads additionally bump the outstanding count."""
        now = time.perf_counter()
        with self._rtt_lock:
            self._rtt_last_write_ts = now
            if data.startswith(_RTT_COMMAND_PREFIXES):
                self._rtt_outstanding += 1

    def _note_ack(self, line: str) -> None:
        """Fold a firmware ack line into the RTT stats (reader thread).

        Only lines starting with "[OK]" (or exactly "k") are acks. The
        sample is kept only when exactly one command was outstanding —
        otherwise the write->ack pairing is ambiguous and it is discarded.
        The outstanding count resets on every ack either way."""
        if not (line.startswith("[OK]") or line == "k"):
            return
        now = time.perf_counter()
        with self._rtt_lock:
            if self._rtt_outstanding == 1 and self._rtt_last_write_ts is not None:
                sample_ms = (now - self._rtt_last_write_ts) * 1000.0
                self._rtt_last_ms = sample_ms
                if self._rtt_samples == 0:
                    self._rtt_ema_ms = sample_ms
                else:
                    self._rtt_ema_ms = (
                        _RTT_EMA_ALPHA * sample_ms
                        + (1.0 - _RTT_EMA_ALPHA) * self._rtt_ema_ms
                    )
                self._rtt_samples += 1
            self._rtt_outstanding = 0

    def _handle_link_error(self, reason: str) -> None:
        """Mark the link dead and notify the disconnect callback (once)."""
        self._running = False
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        if not self._disconnect_reported:
            self._disconnect_reported = True
            cb = self._disconnect_callback
            if cb is not None:
                try:
                    cb(reason)
                except Exception:
                    logger.exception("disconnect callback raised")

    def _write_loop(self) -> None:
        """Background thread: drain the latest-wins slot."""
        while self._running:
            with self._latest_cond:
                while self._latest_data is None and self._running:
                    self._latest_cond.wait(timeout=0.5)
                data = self._latest_data
                self._latest_data = None
            if data is None or not self._running:
                continue
            self.send(data)

    def _read_loop(self) -> None:
        """Background thread: read bytes, buffer, and emit complete lines."""
        buffer = ""

        while self._running:
            ser = self._ser
            if ser is None:
                break
            try:
                # Low-latency read: block for the FIRST byte (bounded by the
                # port timeout), then drain whatever else is waiting. A plain
                # read(128) would sit out the full timeout waiting for 128
                # bytes, delaying every received line by up to 100 ms.
                raw = ser.read(1)
                if raw:
                    waiting = int(getattr(ser, "in_waiting", 0) or 0)
                    if waiting:
                        raw += ser.read(waiting)
            except Exception as exc:
                logger.error("Read failed: %s", exc)
                self._handle_link_error(f"read failed: {exc}")
                break

            data = raw.decode("utf-8", errors="ignore")
            if not data:
                continue
            buffer += data

            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                self._note_ack(line)
                if self._callback is None:
                    logger.debug("RX %s", line)
                    continue
                try:
                    self._callback(line)
                except Exception:
                    # A misbehaving consumer must not deafen the link.
                    logger.exception("receive callback raised; line dropped")
