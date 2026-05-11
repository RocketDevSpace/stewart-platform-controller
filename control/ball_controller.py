import math
import time

from core.platform_state import BallState
from settings import (
    AUTO_TRIM_ERROR_LPF_ALPHA,
    AUTO_TRIM_HOME_CAL_MAX_DEG,
    AUTO_TRIM_KI_DEG_PER_MM_S,
    AUTO_TRIM_MAX_DEG,
    AUTO_TRIM_SETTLE_HOLD_S,
    AUTO_TRIM_SETTLE_RADIUS_LPF_ALPHA,
    AUTO_TRIM_SETTLE_RADIUS_MM,
    AUTO_TRIM_SETTLE_SPEED_LPF_ALPHA,
    AUTO_TRIM_SETTLE_SPEED_MM_S,
    AUTO_TRIM_STEP_LIMIT_DEG,
    AUTO_TRIM_TARGET_HOLD_S,
    BALL_TARGET_DEFAULT_X_MM,
    BALL_TARGET_DEFAULT_Y_MM,
    DEBUG_PRINTS,
    MANUAL_PITCH_TRIM_DEG,
    MANUAL_ROLL_TRIM_DEG,
    PD_DEFAULT_KD,
    PD_DEFAULT_KP,
)


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
        kp: float = PD_DEFAULT_KP,
        kd: float = PD_DEFAULT_KD,
        max_tilt_deg: float = 10.0,
        roll_offset: float = 0.0,
        pitch_offset: float = 0.0,
        auto_trim_enabled: bool = False,
    ) -> None:
        self.kp = kp
        self.kd = kd
        self.max_tilt_deg = max_tilt_deg
        self.enabled = True

        # Trim offsets (FG-9)
        self.roll_offset = float(roll_offset)
        self.pitch_offset = float(pitch_offset)
        self.auto_trim_enabled = bool(auto_trim_enabled)

        # Target (FG-9)
        self.target_x_mm = float(BALL_TARGET_DEFAULT_X_MM)
        self.target_y_mm = float(BALL_TARGET_DEFAULT_Y_MM)
        self._manual_target_x_mm = self.target_x_mm
        self._manual_target_y_mm = self.target_y_mm
        self._target_change_time: float | None = None

        # Auto-trim config (FG-9)
        self.auto_trim_ki = float(AUTO_TRIM_KI_DEG_PER_MM_S)
        self.auto_trim_max_deg = float(AUTO_TRIM_MAX_DEG)
        self.auto_trim_home_cal_max_deg = float(AUTO_TRIM_HOME_CAL_MAX_DEG)
        self.auto_trim_settle_speed_mm_s = float(AUTO_TRIM_SETTLE_SPEED_MM_S)
        self.auto_trim_settle_radius_mm = float(AUTO_TRIM_SETTLE_RADIUS_MM)
        self.auto_trim_settle_hold_s = float(AUTO_TRIM_SETTLE_HOLD_S)
        self.auto_trim_step_limit_deg = float(AUTO_TRIM_STEP_LIMIT_DEG)
        self.auto_trim_target_hold_s = float(AUTO_TRIM_TARGET_HOLD_S)
        self.auto_trim_error_lpf_alpha = float(AUTO_TRIM_ERROR_LPF_ALPHA)
        self.auto_trim_settle_speed_lpf_alpha = float(AUTO_TRIM_SETTLE_SPEED_LPF_ALPHA)
        self.auto_trim_settle_radius_lpf_alpha = float(AUTO_TRIM_SETTLE_RADIUS_LPF_ALPHA)

        # Auto-trim runtime state (FG-9)
        self._settled_time_s = 0.0
        self._last_trim_update_time: float | None = None
        self._auto_trim_update_count = 0
        self._trim_err_lp_x = 0.0
        self._trim_err_lp_y = 0.0
        self._settle_speed_lp = 0.0
        self._settle_radius_lp = 0.0
        self._settle_lp_initialized = False
        self._auto_trim_state = "idle"
        self._auto_trim_target_hold_remaining_s = 0.0
        self._auto_trim_last_speed_mm_s = 0.0
        self._auto_trim_last_radius_mm = 0.0
        self._auto_trim_last_roll_step_deg = 0.0
        self._auto_trim_last_pitch_step_deg = 0.0
        self._auto_trim_limit_deg_active = self.auto_trim_max_deg
        self._auto_trim_gate_speed_ok = False
        self._auto_trim_gate_radius_ok = False
        self._auto_trim_gate_radius_bypassed = False
        self._auto_trim_gate_settled_ok = False
        self._auto_trim_gate_reason = "init"
        self.home_calibration_active = False
        self._home_calibration_start_ts = 0.0

    # ---------------------------
    # Public Interface
    # ---------------------------

    def set_gains(self, kp: float, kd: float) -> None:
        self.kp = kp
        self.kd = kd

    def set_max_tilt(self, max_tilt_deg: float) -> None:
        self.max_tilt_deg = max_tilt_deg

    def enable(self) -> None:
        self.enabled = True

    def disable(self) -> None:
        self.enabled = False

    def set_trim(self, roll_offset_deg: float, pitch_offset_deg: float) -> None:
        self.roll_offset = float(roll_offset_deg)
        self.pitch_offset = float(pitch_offset_deg)

    def reset_trim(self) -> None:
        self.roll_offset = float(MANUAL_ROLL_TRIM_DEG)
        self.pitch_offset = float(MANUAL_PITCH_TRIM_DEG)
        self._settled_time_s = 0.0
        self._auto_trim_update_count = 0
        self._trim_err_lp_x = 0.0
        self._trim_err_lp_y = 0.0
        self._settle_speed_lp = 0.0
        self._settle_radius_lp = 0.0
        self._settle_lp_initialized = False

    def set_auto_trim_enabled(self, enabled: bool) -> None:
        self.auto_trim_enabled = bool(enabled)
        if not self.auto_trim_enabled:
            self.home_calibration_active = False
            self._home_calibration_start_ts = 0.0

    def start_home_calibration(self) -> None:
        self.home_calibration_active = True
        self.auto_trim_enabled = True
        self.target_x_mm = float(BALL_TARGET_DEFAULT_X_MM)
        self.target_y_mm = float(BALL_TARGET_DEFAULT_Y_MM)
        now = time.perf_counter()
        self._target_change_time = now
        self._home_calibration_start_ts = now
        self._settled_time_s = 0.0
        self._trim_err_lp_x = 0.0
        self._trim_err_lp_y = 0.0
        self._settle_speed_lp = 0.0
        self._settle_radius_lp = 0.0
        self._settle_lp_initialized = False

    def cancel_home_calibration(self) -> None:
        self.home_calibration_active = False
        self._home_calibration_start_ts = 0.0

    def set_target(self, x_mm: float, y_mm: float) -> None:
        self._manual_target_x_mm = float(x_mm)
        self._manual_target_y_mm = float(y_mm)
        self._settle_lp_initialized = False
        self._target_change_time = time.perf_counter()
        self.target_x_mm = float(x_mm)
        self.target_y_mm = float(y_mm)

    def compute(self, ball_state: BallState | None) -> tuple[float, float]:
        """Compute desired platform tilt. Returns (roll_deg, pitch_deg)."""
        t0 = time.perf_counter()

        if not self.enabled:
            return 0.0, 0.0

        if ball_state is None:
            return 0.0, 0.0

        x = ball_state.x_mm
        y = ball_state.y_mm
        vx = ball_state.vx_mm_s
        vy = ball_state.vy_mm_s

        ex = self.target_x_mm - x
        ey = self.target_y_mm - y

        pitch = self.kp * ex + self.kd * (-vx)
        roll = -(self.kp * ey + self.kd * (-vy))

        # Auto-trim slow integral correction (FG-9)
        self._update_auto_trim(ex, ey, vx, vy)

        pitch = pitch + self.pitch_offset
        roll = roll + self.roll_offset

        roll = self._clamp(roll, -self.max_tilt_deg, self.max_tilt_deg)
        pitch = self._clamp(pitch, -self.max_tilt_deg, self.max_tilt_deg)

        t1 = time.perf_counter()

        if DEBUG_PRINTS:
            print(f"PD compute: {(t1-t0)*1000:.3f} ms")

        return roll, pitch

    # ---------------------------
    # Auto-trim (FG-9)
    # ---------------------------

    def _update_auto_trim(self, ex: float, ey: float, vx: float, vy: float) -> None:
        now = time.perf_counter()
        self._auto_trim_last_roll_step_deg = 0.0
        self._auto_trim_last_pitch_step_deg = 0.0
        self._auto_trim_target_hold_remaining_s = 0.0
        self._auto_trim_last_speed_mm_s = float(math.hypot(vx, vy))
        self._auto_trim_last_radius_mm = float(math.hypot(ex, ey))
        self._auto_trim_limit_deg_active = (
            self.auto_trim_home_cal_max_deg
            if self.home_calibration_active
            else self.auto_trim_max_deg
        )
        self._auto_trim_gate_speed_ok = False
        self._auto_trim_gate_radius_ok = False
        self._auto_trim_gate_radius_bypassed = False
        self._auto_trim_gate_settled_ok = False
        self._auto_trim_gate_reason = "init"

        if self._last_trim_update_time is None:
            self._last_trim_update_time = now
            self._auto_trim_state = "init"
            self._auto_trim_gate_reason = "init_dt"
            return

        dt = max(1e-4, now - self._last_trim_update_time)
        self._last_trim_update_time = now

        if (not self.auto_trim_enabled) or (not self.enabled):
            self._settled_time_s = 0.0
            self._settle_lp_initialized = False
            self._auto_trim_state = "disabled"
            self._auto_trim_gate_reason = "trim_disabled"
            return

        if self._target_change_time is not None:
            hold_remaining = self.auto_trim_target_hold_s - (now - self._target_change_time)
            self._auto_trim_target_hold_remaining_s = max(0.0, hold_remaining)
        if self._auto_trim_target_hold_remaining_s > 0.0:
            self._settled_time_s = 0.0
            self._settle_lp_initialized = False
            self._auto_trim_state = "target_hold"
            self._auto_trim_gate_reason = "target_hold"
            return

        speed = math.hypot(vx, vy)
        radius = math.hypot(ex, ey)
        self._auto_trim_last_speed_mm_s = float(speed)
        self._auto_trim_last_radius_mm = float(radius)

        if not self._settle_lp_initialized:
            self._settle_speed_lp = float(speed)
            self._settle_radius_lp = float(radius)
            self._settle_lp_initialized = True
        else:
            a_speed = self._clamp(self.auto_trim_settle_speed_lpf_alpha, 0.0, 0.999)
            a_radius = self._clamp(self.auto_trim_settle_radius_lpf_alpha, 0.0, 0.999)
            self._settle_speed_lp = (
                a_speed * self._settle_speed_lp + (1.0 - a_speed) * speed
            )
            self._settle_radius_lp = (
                a_radius * self._settle_radius_lp + (1.0 - a_radius) * radius
            )

        speed_ok = self._settle_speed_lp <= self.auto_trim_settle_speed_mm_s
        radius_ok = self._settle_radius_lp <= self.auto_trim_settle_radius_mm
        radius_bypassed = bool(self.home_calibration_active)
        settled = speed_ok and (radius_ok or radius_bypassed)

        self._auto_trim_gate_speed_ok = bool(speed_ok)
        self._auto_trim_gate_radius_ok = bool(radius_ok)
        self._auto_trim_gate_radius_bypassed = bool(radius_bypassed)
        self._auto_trim_gate_settled_ok = bool(settled)

        if not settled:
            self._settled_time_s = 0.0
            self._auto_trim_state = "waiting_settle"
            if not speed_ok:
                self._auto_trim_gate_reason = "wait_speed"
            elif not radius_ok:
                self._auto_trim_gate_reason = "wait_radius"
            else:
                self._auto_trim_gate_reason = "wait_other"
            return

        self._settled_time_s += dt
        if self._settled_time_s < self.auto_trim_settle_hold_s:
            self._auto_trim_state = "holding_settle"
            self._auto_trim_gate_reason = "hold_timer"
            return

        alpha = self._clamp(self.auto_trim_error_lpf_alpha, 0.0, 0.999)
        self._trim_err_lp_x = (
            alpha * self._trim_err_lp_x + (1.0 - alpha) * ex
        )
        self._trim_err_lp_y = (
            alpha * self._trim_err_lp_y + (1.0 - alpha) * ey
        )

        roll_rate = self.auto_trim_ki * (-self._trim_err_lp_y)
        pitch_rate = self.auto_trim_ki * self._trim_err_lp_x
        roll_step = self._clamp(
            roll_rate * dt,
            -self.auto_trim_step_limit_deg,
            self.auto_trim_step_limit_deg,
        )
        pitch_step = self._clamp(
            pitch_rate * dt,
            -self.auto_trim_step_limit_deg,
            self.auto_trim_step_limit_deg,
        )

        trim_limit = max(0.1, float(self._auto_trim_limit_deg_active))
        self.roll_offset = self._clamp(
            self.roll_offset + roll_step, -trim_limit, trim_limit
        )
        self.pitch_offset = self._clamp(
            self.pitch_offset + pitch_step, -trim_limit, trim_limit
        )
        self._auto_trim_last_roll_step_deg = float(roll_step)
        self._auto_trim_last_pitch_step_deg = float(pitch_step)
        self._auto_trim_update_count += 1
        self._auto_trim_state = "updating"
        self._auto_trim_gate_reason = "updating"

    # ---------------------------
    # Internal Helpers
    # ---------------------------

    @staticmethod
    def _clamp(value: float, min_val: float, max_val: float) -> float:
        return max(min(value, max_val), min_val)
