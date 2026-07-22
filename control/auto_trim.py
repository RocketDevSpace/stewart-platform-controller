"""
control/auto_trim.py

AutoTrim — owns the roll/pitch trim offsets and the slow integral trim
correction (FG-9), including the settle gates, low-pass filters, hold
timers, and home-calibration state.

Single-writer rule: the offsets are mutated only here — GUI writes go
through set_offsets(), the integrator steps them in update(), reset()
restores the manual defaults from settings.
"""
import math
from collections.abc import Callable
from typing import Any

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
    MANUAL_PITCH_TRIM_DEG,
    MANUAL_ROLL_TRIM_DEG,
)


def _clamp(value: float, min_val: float, max_val: float) -> float:
    return max(min(value, max_val), min_val)


class AutoTrim:
    def __init__(
        self,
        roll_offset: float,
        pitch_offset: float,
        clock: Callable[[], float],
    ) -> None:
        self._clock = clock

        # Trim offsets (FG-9) — single writer: this class.
        self._roll_offset = float(roll_offset)
        self._pitch_offset = float(pitch_offset)

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

        # Home-calibration state (FG-9)
        self.home_calibration_active = False
        self._home_calibration_start_ts = 0.0

    # ---------------------------
    # Offsets
    # ---------------------------

    @property
    def roll_offset(self) -> float:
        return self._roll_offset

    @property
    def pitch_offset(self) -> float:
        return self._pitch_offset

    def set_offsets(self, roll_offset_deg: float, pitch_offset_deg: float) -> None:
        self._roll_offset = float(roll_offset_deg)
        self._pitch_offset = float(pitch_offset_deg)

    def reset(self) -> None:
        """Restore manual trim defaults and clear the integrator state."""
        self._roll_offset = float(MANUAL_ROLL_TRIM_DEG)
        self._pitch_offset = float(MANUAL_PITCH_TRIM_DEG)
        self._settled_time_s = 0.0
        self._auto_trim_update_count = 0
        self._trim_err_lp_x = 0.0
        self._trim_err_lp_y = 0.0
        self._settle_speed_lp = 0.0
        self._settle_radius_lp = 0.0
        self._settle_lp_initialized = False

    # ---------------------------
    # Gating hooks
    # ---------------------------

    def notify_target_changed(self) -> None:
        """The target moved: restart the settle low-pass filters."""
        self._settle_lp_initialized = False

    def start_home_calibration(self) -> None:
        self.home_calibration_active = True
        self._home_calibration_start_ts = self._clock()
        self._settled_time_s = 0.0
        self._trim_err_lp_x = 0.0
        self._trim_err_lp_y = 0.0
        self._settle_speed_lp = 0.0
        self._settle_radius_lp = 0.0
        self._settle_lp_initialized = False

    def cancel_home_calibration(self) -> None:
        self.home_calibration_active = False
        self._home_calibration_start_ts = 0.0

    # ---------------------------
    # Integrator update (FG-9)
    # ---------------------------

    def update(
        self,
        ex: float,
        ey: float,
        vx: float,
        vy: float,
        enabled: bool,
        target_change_time: float | None,
    ) -> None:
        now = self._clock()
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

        if not enabled:
            self._settled_time_s = 0.0
            self._settle_lp_initialized = False
            self._auto_trim_state = "disabled"
            self._auto_trim_gate_reason = "trim_disabled"
            return

        if target_change_time is not None:
            hold_remaining = self.auto_trim_target_hold_s - (now - target_change_time)
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
            a_speed = _clamp(self.auto_trim_settle_speed_lpf_alpha, 0.0, 0.999)
            a_radius = _clamp(self.auto_trim_settle_radius_lpf_alpha, 0.0, 0.999)
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

        alpha = _clamp(self.auto_trim_error_lpf_alpha, 0.0, 0.999)
        self._trim_err_lp_x = alpha * self._trim_err_lp_x + (1.0 - alpha) * ex
        self._trim_err_lp_y = alpha * self._trim_err_lp_y + (1.0 - alpha) * ey

        roll_rate = self.auto_trim_ki * (-self._trim_err_lp_y)
        pitch_rate = self.auto_trim_ki * self._trim_err_lp_x
        roll_step = _clamp(
            roll_rate * dt,
            -self.auto_trim_step_limit_deg,
            self.auto_trim_step_limit_deg,
        )
        pitch_step = _clamp(
            pitch_rate * dt,
            -self.auto_trim_step_limit_deg,
            self.auto_trim_step_limit_deg,
        )

        trim_limit = max(0.1, float(self._auto_trim_limit_deg_active))
        self._roll_offset = _clamp(
            self._roll_offset + roll_step, -trim_limit, trim_limit
        )
        self._pitch_offset = _clamp(
            self._pitch_offset + pitch_step, -trim_limit, trim_limit
        )
        self._auto_trim_last_roll_step_deg = float(roll_step)
        self._auto_trim_last_pitch_step_deg = float(pitch_step)
        self._auto_trim_update_count += 1
        self._auto_trim_state = "updating"
        self._auto_trim_gate_reason = "updating"

    # ---------------------------
    # Telemetry (terms-dict keys — FROZEN, the GUI reads these)
    # ---------------------------

    def telemetry(self, enabled: bool) -> dict[str, Any]:
        return {
            "trim_settled_s": self._settled_time_s,
            "trim_updates": self._auto_trim_update_count,
            "auto_trim_enabled": enabled,
            "auto_trim_state": self._auto_trim_state,
            "auto_trim_target_hold_remaining_s": self._auto_trim_target_hold_remaining_s,
            "auto_trim_speed_mm_s": self._auto_trim_last_speed_mm_s,
            "auto_trim_radius_mm": self._auto_trim_last_radius_mm,
            "auto_trim_speed_lpf_mm_s": self._settle_speed_lp,
            "auto_trim_radius_lpf_mm": self._settle_radius_lp,
            "auto_trim_speed_thresh_mm_s": self.auto_trim_settle_speed_mm_s,
            "auto_trim_radius_thresh_mm": self.auto_trim_settle_radius_mm,
            "auto_trim_hold_s": self.auto_trim_settle_hold_s,
            "auto_trim_roll_step_deg": self._auto_trim_last_roll_step_deg,
            "auto_trim_pitch_step_deg": self._auto_trim_last_pitch_step_deg,
            "auto_trim_limit_deg_active": self._auto_trim_limit_deg_active,
            "auto_trim_limit_deg_normal": self.auto_trim_max_deg,
            "auto_trim_limit_deg_home_cal": self.auto_trim_home_cal_max_deg,
            "auto_trim_roll_sat": 1.0 if abs(self._roll_offset) >= (
                self._auto_trim_limit_deg_active - 1e-6) else 0.0,
            "auto_trim_pitch_sat": 1.0 if abs(self._pitch_offset) >= (
                self._auto_trim_limit_deg_active - 1e-6) else 0.0,
            "auto_trim_gate_speed_ok": 1.0 if self._auto_trim_gate_speed_ok else 0.0,
            "auto_trim_gate_radius_ok": 1.0 if self._auto_trim_gate_radius_ok else 0.0,
            "auto_trim_gate_radius_bypassed": (
                1.0 if self._auto_trim_gate_radius_bypassed else 0.0
            ),
            "auto_trim_gate_settled_ok": 1.0 if self._auto_trim_gate_settled_ok else 0.0,
            "auto_trim_gate_reason": self._auto_trim_gate_reason,
            "home_calibration_active": self.home_calibration_active,
            "home_calibration_elapsed_s": (
                max(0.0, self._clock() - self._home_calibration_start_ts)
                if self.home_calibration_active and self._home_calibration_start_ts > 0.0
                else 0.0
            ),
        }
