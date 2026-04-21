"""
hardware/servo_driver.py

Owns all servo command formatting and dispatch.
This is the single call site for building "S,..." command strings.

SerialManager is injected — this class does not own the connection.
Safety clipping is applied before every send via core/safety.py.
"""

from core.safety import clip_servo_angles
from hardware.serial_manager import SerialManager
from settings import DEBUG_PRINTS


class ServoDriver:
    """Formats servo angle commands and dispatches them over serial."""

    def __init__(self, serial_manager: SerialManager) -> None:
        self._serial = serial_manager

    def format_command(self, angles: list[float]) -> str:
        """Return the Arduino serial command string for the given angles.

        Pure formatting — no side effects, no sending.
        Angles are rounded to int but NOT clipped here; call send_angles()
        if you want safety clipping applied.

        Format: "S,<a0>,<a1>,<a2>,<a3>,<a4>,<a5>,0\\n"
        """
        ints = [int(round(a)) for a in angles]
        return "S," + ",".join(str(a) for a in ints) + ",0\n"

    def send_angles(self, angles: list[float]) -> bool:
        """Clip, format, and send servo angles over serial.

        Applies safety clipping from core/safety.py before formatting.
        Logs clipped angles if settings.DEBUG_PRINTS is True.

        Returns True on successful send, False on failure (never raises).
        """
        clipped_angles, clips = clip_servo_angles(angles)

        if clips and DEBUG_PRINTS:
            for idx, original, clipped in clips:
                print(
                    f"[SAFETY CLIP] Servo {idx}: {original} → {clipped}"
                )

        cmd = self.format_command(clipped_angles)
        return self._serial.send(cmd.encode())
