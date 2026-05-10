import time

from core.platform_state import BallState
from settings import DEBUG_PRINTS


class BallController:
    """
    PD controller for ball balancing on Stewart platform.

    Inputs:
        Ball position + velocity (mm, mm/s)

    Outputs:
        Desired platform tilt angles (degrees)
            roll_deg  -> rotation about X axis
            pitch_deg -> rotation about Y axis
    """

    def __init__(
        self,
        kp: float = 0.05,
        kd: float = 0.01,
        max_tilt_deg: float = 10.0
    ):
        self.kp = kp
        self.kd = kd
        self.max_tilt_deg = max_tilt_deg

        self.enabled = True

        self.pitch_offset = 0
        self.roll_offset = 0

    # ---------------------------
    # Public Interface
    # ---------------------------

    def set_gains(self, kp: float, kd: float) -> None:
        """Update controller gains live (from GUI sliders)."""
        self.kp = kp
        self.kd = kd

    def set_max_tilt(self, max_tilt_deg: float) -> None:
        """Update safety tilt limit."""
        self.max_tilt_deg = max_tilt_deg

    def enable(self) -> None:
        self.enabled = True

    def disable(self) -> None:
        self.enabled = False

    def compute(self, ball_state: BallState | None) -> tuple[float, float]:
        """
        Compute desired platform tilt.

        Returns:
            (roll_deg, pitch_deg)
        """
        t0 = time.perf_counter()

        if not self.enabled:
            return 0.0, 0.0

        if ball_state is None:
            return 0.0, 0.0

        x = ball_state.x_mm
        y = ball_state.y_mm
        vx = ball_state.vx_mm_s
        vy = ball_state.vy_mm_s

        ex = -x
        ey = -y

        pitch = self.kp * ex + self.kd * (-vx)
        roll = -(self.kp * ey + self.kd * (-vy))

        pitch = pitch + self.pitch_offset
        roll = roll + self.roll_offset

        roll = self._clamp(roll, -self.max_tilt_deg, self.max_tilt_deg)
        pitch = self._clamp(pitch, -self.max_tilt_deg, self.max_tilt_deg)

        t1 = time.perf_counter()

        if DEBUG_PRINTS:
            print(f"PD compute: {(t1-t0)*1000:.3f} ms")

        return roll, pitch

    # ---------------------------
    # Internal Helpers
    # ---------------------------

    @staticmethod
    def _clamp(value: float, min_val: float, max_val: float) -> float:
        return max(min(value, max_val), min_val)
