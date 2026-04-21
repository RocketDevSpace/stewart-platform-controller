"""
hardware/serial_manager.py

Owns serial connection lifecycle and the background read loop.
Does NOT format commands — that is servo_driver.py's responsibility.

Port and baud are injected by the caller (sourced from settings.py there).
No config values are hardcoded here.
"""

import threading
import time
from collections.abc import Callable
from typing import Any

try:
    import serial  # type: ignore[import]
except ImportError:  # pragma: no cover
    serial = None  # type: ignore[assignment]


class SerialManager:
    """Manages a single serial connection and a background line-reader thread."""

    def __init__(self, port: str, baud: int) -> None:
        self._port = port
        self._baud = baud
        self._ser: Any = None
        self._running = False
        self._read_thread: threading.Thread | None = None
        self._callback: Callable[[str], None] | None = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Open the serial port and start the background read thread.

        Returns True on success, False on failure (never raises).
        """
        if serial is None:
            print("[SERIAL ERROR] pyserial is not installed")
            return False

        try:
            self._ser = serial.Serial(self._port, self._baud, timeout=0.1)
            time.sleep(2)  # allow Arduino reset after DTR toggle
            print(f"[SERIAL] Connected to {self._port} at {self._baud} baud")
        except Exception as exc:
            print(f"[SERIAL ERROR] Could not connect: {exc}")
            self._ser = None
            return False

        self._running = True
        self._read_thread = threading.Thread(
            target=self._read_loop, daemon=True
        )
        self._read_thread.start()
        return True

    def disconnect(self) -> None:
        """Stop the read thread and close the serial port."""
        self._running = False
        if self._ser is not None:
            try:
                if self._ser.is_open:
                    self._ser.close()
                    print("[SERIAL] Disconnected")
            except Exception as exc:
                print(f"[SERIAL ERROR] Disconnect failed: {exc}")
        self._ser = None

    def send(self, data: bytes) -> bool:
        """Write raw bytes to the serial port.

        Returns True on success, False on failure (never raises).
        """
        if not self.is_connected():
            print("[SERIAL ERROR] Not connected")
            return False

        try:
            self._ser.write(data)
            return True
        except Exception as exc:
            print(f"[SERIAL ERROR] Send failed: {exc}")
            return False

    def set_receive_callback(self, callback: Callable[[str], None]) -> None:
        """Register a callback invoked with each complete line received."""
        self._callback = callback

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

    def _read_loop(self) -> None:
        """Background thread: read bytes, buffer, and emit complete lines."""
        buffer = ""

        while self._running and self._ser is not None:
            try:
                raw = self._ser.read(128)
                data = raw.decode("utf-8", errors="ignore")

                if data:
                    buffer += data

                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()

                        if line:
                            if self._callback is not None:
                                self._callback(line)
                            else:
                                print("[SERIAL RX]", line)

            except Exception as exc:
                print(f"[SERIAL ERROR] Read failed: {exc}")
                break
