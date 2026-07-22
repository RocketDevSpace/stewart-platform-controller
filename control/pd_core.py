"""
control/pd_core.py

PDCore — pure PD math for the ball balancer: proportional + derivative
terms, the derivative-term cap (FG-11), the max-tilt clamp, and the
commanded-tilt slew ("tilt rate") limiter (FG-11). No target, trim, or
autotune knowledge; the caller supplies error vectors and offsets.
"""
from collections.abc import Callable
from dataclasses import dataclass


def _clamp(value: float, min_val: float, max_val: float) -> float:
    return max(min(value, max_val), min_val)


@dataclass
class PDResult:
    """Intermediate and final values of one PD step (terms-dict inputs)."""

    p_term: tuple[float, float]
    d_term: tuple[float, float]
    pd_vec: tuple[float, float]
    roll_raw: float
    pitch_raw: float
    roll_clamped: float
    pitch_clamped: float
    roll_cmd: float
    pitch_cmd: float


class PDCore:
    def __init__(
        self,
        kp: float,
        kd: float,
        max_tilt_deg: float,
        max_tilt_rate_deg_s: float,
        d_term_limit_deg: float | None,
        clock: Callable[[], float],
    ) -> None:
        self._clock = clock
        self.kp = float(kp)
        self.kd = float(kd)
        self.max_tilt_deg = float(max_tilt_deg)
        self.max_tilt_rate_deg_s = float(max_tilt_rate_deg_s)
        self.d_term_limit_deg = (
            None if d_term_limit_deg is None else float(d_term_limit_deg)
        )

        # Slew limit state (FG-11)
        self._prev_roll_cmd = 0.0
        self._prev_pitch_cmd = 0.0
        self._prev_cmd_time: float | None = None

    def reset_motion_state(self) -> None:
        self._prev_roll_cmd = 0.0
        self._prev_pitch_cmd = 0.0
        self._prev_cmd_time = None

    def compute(
        self,
        ex: float,
        ey: float,
        vx: float,
        vy: float,
        roll_offset: float,
        pitch_offset: float,
    ) -> PDResult:
        """One PD step from error vector (ex, ey) and ball velocity."""
        p_x = self.kp * ex
        p_y = self.kp * ey
        d_x = self.kd * (-vx)
        d_y = self.kd * (-vy)

        # Derivative term cap (FG-11)
        if self.d_term_limit_deg is not None:
            d_x = _clamp(d_x, -self.d_term_limit_deg, self.d_term_limit_deg)
            d_y = _clamp(d_y, -self.d_term_limit_deg, self.d_term_limit_deg)

        pd_x = p_x + d_x
        pd_y = p_y + d_y

        pitch_raw = pd_x + pitch_offset
        roll_raw = -pd_y + roll_offset
        roll_clamped = _clamp(roll_raw, -self.max_tilt_deg, self.max_tilt_deg)
        pitch_clamped = _clamp(pitch_raw, -self.max_tilt_deg, self.max_tilt_deg)

        # Slew limit (FG-11)
        roll_cmd, pitch_cmd = self._apply_slew_limit(roll_clamped, pitch_clamped)

        return PDResult(
            p_term=(p_x, p_y),
            d_term=(d_x, d_y),
            pd_vec=(pd_x, pd_y),
            roll_raw=roll_raw,
            pitch_raw=pitch_raw,
            roll_clamped=roll_clamped,
            pitch_clamped=pitch_clamped,
            roll_cmd=roll_cmd,
            pitch_cmd=pitch_cmd,
        )

    def _apply_slew_limit(self, roll: float, pitch: float) -> tuple[float, float]:
        now = self._clock()
        if self._prev_cmd_time is None:
            self._prev_cmd_time = now
            self._prev_roll_cmd = roll
            self._prev_pitch_cmd = pitch
            return roll, pitch
        dt = max(1e-4, now - self._prev_cmd_time)
        self._prev_cmd_time = now
        max_step = self.max_tilt_rate_deg_s * dt
        roll_out = self._prev_roll_cmd + _clamp(
            roll - self._prev_roll_cmd, -max_step, max_step
        )
        pitch_out = self._prev_pitch_cmd + _clamp(
            pitch - self._prev_pitch_cmd, -max_step, max_step
        )
        self._prev_roll_cmd = roll_out
        self._prev_pitch_cmd = pitch_out
        return roll_out, pitch_out
