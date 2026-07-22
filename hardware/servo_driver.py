"""
hardware/servo_driver.py

Owns all servo command formatting and dispatch.
This is the single call site for building "S,..." command strings.

SerialManager is injected — this class does not own the connection.
Safety clipping and the large-move ramp policy are applied before every
send via core/safety.py — including raw commands typed in the GUI.

Firmware protocol (verified against the board; see firmware/README.md):
    "S,a0,a1,a2,a3,a4,a5,speedDelay\\n"
  - angles integer degrees, firmware clamps 0-180
  - speedDelay ms/deg, firmware clamps 0-20; the firmware ramps all servos
    concurrently and acks "[OK] ..." only AFTER the ramp completes.
"""

import logging

from core.safety import NUM_SERVOS, clip_servo_angles, select_speed_delay
from hardware.serial_manager import SerialManager
from settings import (
    DEBUG_PRINTS,
    SERVO_DEDUP_ENABLED,
    SERVO_PROTOCOL,
    SERVO_QUANT_HYST_DEG,
    SERVO_QUANT_HYST_FINE_DEG,
)

logger = logging.getLogger(__name__)

# The firmware writes 90 deg to every servo at boot, so that is the known
# hardware pose before anything has been sent.
_BOOT_ANGLES = [90.0] * NUM_SERVOS


class ServoDriver:
    """Formats servo angle commands and dispatches them over serial."""

    def __init__(self, serial_manager: SerialManager) -> None:
        self._serial = serial_manager
        self._last_sent: list[float] = list(_BOOT_ANGLES)
        # Schmitt-trigger quantizer state: the grid unit each servo is
        # currently committed to. On the legacy whole-degree protocol the
        # grid is 1 deg; on firmware v2's T protocol it is 0.1 deg
        # (committed values are stored in GRID UNITS: degrees or
        # tenth-degrees respectively).
        self._committed: list[int] = [int(a) for a in _BOOT_ANGLES]
        self._committed_grid = 1.0  # deg per grid unit of _committed
        # Last quantized command actually sent (grid units), for dedup.
        # Seeded with the boot pose (firmware boots at 90s on grid 1.0).
        self._last_sent_units: list[int] | None = [int(a) for a in _BOOT_ANGLES]
        self.quant_hyst_deg = float(SERVO_QUANT_HYST_DEG)
        self.quant_hyst_fine_deg = float(SERVO_QUANT_HYST_FINE_DEG)
        self.protocol = str(SERVO_PROTOCOL)
        self.dedup_enabled = bool(SERVO_DEDUP_ENABLED)
        # Telemetry: sends actually suppressed by dedup.
        self.dedup_skips = 0

    def _use_tenth_protocol(self) -> bool:
        return (
            self.protocol == "auto"
            and getattr(self._serial, "firmware_version", "") == "v2"
        )
        # Expected firmware-side ramp duration of the LAST send (ms). The
        # firmware acks only AFTER a ramp and buffers just 64 bytes of
        # commands while ramping — streaming callers (RoutineRunner) must
        # hold off sends for this long after a ramped command or the UART
        # buffer overflows and commands get dropped/corrupted.
        self.last_ramp_ms: float = 0.0

    def rtt_stats(self) -> tuple[float, float, int]:
        """(ema_ms, last_ms, samples) of the serial command->ack round trip.

        Thin delegation to SerialManager.rtt_stats() so consumers (the
        vision worker's timing telemetry) never reach through to the
        serial layer directly."""
        return self._serial.rtt_stats()

    def format_command(self, angles: list[float], speed_delay: int = 0) -> str:
        """Return the Arduino serial command string for the given angles.

        Pure formatting — no side effects, no sending.

        WARNING: does not apply safety clipping. Use send_angles() for
        hardware output.

        Format: "S,<a0>,<a1>,<a2>,<a3>,<a4>,<a5>,<speedDelay>\\n"
        """
        if len(angles) != NUM_SERVOS:
            raise ValueError(
                f"expected {NUM_SERVOS} servo angles, got {len(angles)}"
            )
        ints = [int(round(a)) for a in angles]
        return "S," + ",".join(str(a) for a in ints) + f",{int(speed_delay)}\n"

    def send_angles(self, angles: list[float], streaming: bool = False) -> bool:
        """Clip, ramp-check, format, and send servo angles over serial.

        - Safety clipping from core/safety.py before formatting.
        - Large jumps (vs the last sent pose) are sent with a non-zero
          speedDelay so the firmware ramps them instead of snapping
          (core.safety.select_speed_delay; applies to manual sends too,
          per the 2026-06-11 M9 decision).
        - streaming=True routes through SerialManager.send_latest(): the
          depth-1 latest-wins writer, for control loops. Stale setpoints
          are coalesced away and the caller never blocks on serial I/O.

        Returns True on successful send/enqueue, False on failure
        (never raises on serial problems; raises ValueError on a
        wrong-length or non-finite angle list — caller bugs fail loudly).
        """
        clipped_angles, clips = clip_servo_angles(angles)

        if clips and DEBUG_PRINTS:
            for idx, original, clipped in clips:
                logger.warning(
                    "[SAFETY CLIP] Servo %d: %s -> %s", idx, original, clipped
                )

        # Hybrid dispatch: firmware v2 gets tenth-degree T commands for
        # ordinary (instant) moves; large jumps — and all v1/forced-legacy
        # traffic — go via the legacy S path so the firmware ramp policy
        # still applies.
        ramp_needed = select_speed_delay(self._last_sent, clipped_angles) > 0
        use_tenth = self._use_tenth_protocol() and not ramp_needed

        if use_tenth:
            grid, hyst = 0.1, self.quant_hyst_fine_deg
        else:
            grid, hyst = 1.0, self.quant_hyst_deg
        self._ensure_grid(grid)
        quantized = self._quantize(clipped_angles, grid, hyst)

        if self.dedup_enabled and quantized == self._last_sent_units:
            # The hardware already holds exactly this command — sending it
            # again is pure serial noise. (Safe: no firmware watchdog; the
            # Servo library holds the last pulse width indefinitely.)
            self.dedup_skips += 1
            self.last_ramp_ms = 0.0
            return True

        target = [q * grid for q in quantized]
        if use_tenth:
            cmd = self.format_command_tenth(quantized)
            speed_delay = 0
        else:
            speed_delay = select_speed_delay(self._last_sent, target)
            cmd = self.format_command(target, speed_delay)

        if streaming:
            ok = self._serial.send_latest(cmd.encode())
        else:
            ok = self._serial.send(cmd.encode())
        if ok:
            self.last_ramp_ms = self._ramp_ms(target, speed_delay)
            self._last_sent = target
            self._last_sent_units = list(quantized)
        return ok

    def format_command_tenth(self, tenths: list[int]) -> str:
        """Firmware v2 T command: tenth-degree units, instant write,
        terse "k" ack."""
        if len(tenths) != NUM_SERVOS:
            raise ValueError(
                f"expected {NUM_SERVOS} servo values, got {len(tenths)}"
            )
        return "T," + ",".join(str(int(t)) for t in tenths) + "\n"

    def _ensure_grid(self, grid: float) -> None:
        """Convert the committed Schmitt state when the command grid
        changes (protocol switch between whole-degree and tenth-degree)."""
        if grid == self._committed_grid:
            return
        self._committed = [
            int(round(c * self._committed_grid / grid)) for c in self._committed
        ]
        self._committed_grid = grid
        self._last_sent_units = None  # grids differ; force a real send

    def _quantize(self, angles: list[float], grid: float, hyst_deg: float) -> list[int]:
        """Schmitt-trigger rounding in grid units: each servo keeps its
        committed value until the commanded angle crosses the grid boundary
        by the hysteresis margin. A large jump commits directly to its
        final value in one call — zero added latency for genuine motion;
        boundary noise commits nothing."""
        margin = 0.5 + hyst_deg / grid
        out: list[int] = []
        for i, a in enumerate(angles):
            units = a / grid
            c = self._committed[i]
            if abs(units - c) > margin:
                c = int(round(units))
                self._committed[i] = c
            out.append(c)
        return out

    def _ramp_ms(self, target: list[float], speed_delay: int) -> float:
        if speed_delay <= 0:
            return 0.0
        max_delta = max(
            abs(t - p) for t, p in zip(target, self._last_sent)
        )
        return float(max_delta * speed_delay)

    def send_raw(self, command: str) -> bool:
        """Validate and send a hand-typed protocol command.

        The GUI raw-command box routes here so typed commands get the same
        safety rails as every other send: the command must parse as
        "S,a0..a5[,speedDelay]", angles are safety-clipped, and the
        newline terminator is guaranteed (a bare string without "\\n" would
        sit unexecuted in the Arduino's line buffer).

        Returns True on send, False on parse failure or serial failure.
        """
        text = command.strip()
        if not text.startswith("S,"):
            logger.error("raw command rejected (must start with 'S,'): %r", text)
            return False
        fields = text[2:].split(",")
        if len(fields) not in (NUM_SERVOS, NUM_SERVOS + 1):
            logger.error(
                "raw command rejected (need %d angles [+ speedDelay]): %r",
                NUM_SERVOS, text,
            )
            return False
        try:
            angles = [float(f) for f in fields[:NUM_SERVOS]]
            speed_delay = int(fields[NUM_SERVOS]) if len(fields) > NUM_SERVOS else 0
        except ValueError:
            logger.error("raw command rejected (non-numeric field): %r", text)
            return False

        clipped_angles, clips = clip_servo_angles(angles)
        if clips:
            logger.warning("raw command clipped: %s", clips)
        # Respect an explicit speedDelay but never let a big typed jump snap.
        speed_delay = max(
            speed_delay, select_speed_delay(self._last_sent, clipped_angles)
        )
        cmd = self.format_command(clipped_angles, speed_delay)
        ok = self._serial.send(cmd.encode())
        if ok:
            self.last_ramp_ms = self._ramp_ms(clipped_angles, speed_delay)
            self._last_sent = list(clipped_angles)
            # Raw commands bypass the Schmitt (explicit operator intent)
            # but must keep the quantizer's committed state truthful.
            self._committed_grid = 1.0
            self._committed = [int(round(a)) for a in clipped_angles]
            self._last_sent_units = list(self._committed)
        return ok
