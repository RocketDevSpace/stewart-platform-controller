# -*- coding: utf-8 -*-
"""
Created on Tue Feb 24 16:13:27 2026

@author: hudso
"""

import math
import time
from dataclasses import dataclass
from stewart_control.config import (
    AUTO_TRIM_ENABLED,
    AUTO_TRIM_KI_DEG_PER_MM_S,
    AUTO_TRIM_MAX_DEG,
    AUTO_TRIM_SETTLE_HOLD_S,
    AUTO_TRIM_SETTLE_RADIUS_MM,
    AUTO_TRIM_SETTLE_SPEED_MM_S,
    AUTO_TRIM_STEP_LIMIT_DEG,
    DEBUG_LEVEL,
    LOG_EVERY_N,
    BALL_TARGET_DEFAULT_X_MM,
    BALL_TARGET_DEFAULT_Y_MM,
    MANUAL_PITCH_TRIM_DEG,
    MANUAL_ROLL_TRIM_DEG,
    PD_DEFAULT_KD,
    PD_DEFAULT_KP,
    PD_AUTOTUNE_AUTO_APPLY,
    PD_AUTOTUNE_ENABLED,
    PD_AUTOTUNE_KD_STEP,
    PD_AUTOTUNE_KP_STEP,
    PD_AUTOTUNE_MAX_KD,
    PD_AUTOTUNE_MAX_KP,
    PD_AUTOTUNE_MIN_KD,
    PD_AUTOTUNE_MIN_KP,
    PD_AUTOTUNE_MIN_TRIAL_S,
    PD_AUTOTUNE_SETTLE_HOLD_S,
    PD_AUTOTUNE_SETTLE_RADIUS_MM,
    PD_AUTOTUNE_SETTLE_SPEED_MM_S,
    PD_AUTOTUNE_TARGET_SETTLE_S,
    PD_AUTOTUNE_TEST_DIAGONAL_MM,
    PD_AUTOTUNE_TIMEOUT_S,
)


@dataclass
class BallState:
    """
    Standardized ball state container.
    This mirrors what the CV module should provide.
    """
    x_mm: float
    y_mm: float
    vx_mm_s: float
    vy_mm_s: float


class _PDLegEvaluator:
    def __init__(self):
        self.active = False
        self.start_ts = 0.0
        self.last_ts = 0.0
        self.settle_timer = 0.0
        self.u_x = 0.0
        self.u_y = 0.0
        self.s0 = 0.0
        self.min_s = 0.0
        self.peak_abs_s = 0.0
        self.iae = 0.0
        self.sign_changes = 0
        self.last_s = None

    def start(self, ts: float, x: float, y: float, tx: float, ty: float):
        ex = tx - x
        ey = ty - y
        e0 = math.hypot(ex, ey)
        if e0 <= 1e-6:
            return False
        self.active = True
        self.start_ts = ts
        self.last_ts = ts
        self.settle_timer = 0.0
        self.u_x = ex / e0
        self.u_y = ey / e0
        self.s0 = e0
        self.min_s = e0
        self.peak_abs_s = e0
        self.iae = 0.0
        self.sign_changes = 0
        self.last_s = e0
        return True

    def observe(
        self,
        ts: float,
        x: float,
        y: float,
        vx: float,
        vy: float,
        tx: float,
        ty: float,
        settle_radius_mm: float,
        settle_speed_mm_s: float,
        settle_hold_s: float,
        min_trial_s: float,
        timeout_s: float,
    ):
        if not self.active:
            return None
        ex = tx - x
        ey = ty - y
        err = math.hypot(ex, ey)
        speed = math.hypot(vx, vy)
        dt = max(1e-4, min(0.1, ts - self.last_ts))
        self.last_ts = ts

        s = ex * self.u_x + ey * self.u_y
        self.iae += abs(s) * dt
        self.peak_abs_s = max(self.peak_abs_s, abs(s))
        self.min_s = min(self.min_s, s)
        if self.last_s is not None:
            band = max(2.0, settle_radius_mm)
            if abs(s) > band and abs(self.last_s) > band and (s * self.last_s) < 0.0:
                self.sign_changes += 1
        self.last_s = s

        if err <= settle_radius_mm and speed <= settle_speed_mm_s:
            self.settle_timer += dt
        else:
            self.settle_timer = 0.0

        elapsed = ts - self.start_ts
        settled = self.settle_timer >= settle_hold_s
        timed_out = elapsed >= timeout_s
        if elapsed < min_trial_s:
            return None
        if not settled and not timed_out:
            return None

        overshoot_mm = max(0.0, -self.min_s)
        metrics = {
            "initial_offset_mm": self.s0,
            "settle_time_s": elapsed,
            "overshoot_mm": overshoot_mm,
            "overshoot_ratio": overshoot_mm / max(self.s0, 1e-6),
            "oscillation_crossings": float(self.sign_changes),
            "iae_mm_s": self.iae,
            "peak_abs_proj_mm": self.peak_abs_s,
            "timed_out": 1.0 if timed_out and not settled else 0.0,
            "target_x_mm": float(tx),
            "target_y_mm": float(ty),
        }
        self.active = False
        return metrics


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
        max_tilt_rate_deg_s: float = 300.0,
        d_term_limit_deg: float | None = 2.5,
        debug_level: int = DEBUG_LEVEL,
        log_every_n: int = LOG_EVERY_N,
    ):
        self.kp = kp
        self.kd = kd
        self.max_tilt_deg = max_tilt_deg
        self.max_tilt_rate_deg_s = max_tilt_rate_deg_s
        self.d_term_limit_deg = d_term_limit_deg

        self.enabled = True
        self.debug_level = debug_level
        self.log_every_n = max(1, int(log_every_n))
        self._log_counter = 0
        
        self.pitch_offset = float(MANUAL_PITCH_TRIM_DEG)
        self.roll_offset = float(MANUAL_ROLL_TRIM_DEG)
        self.auto_trim_enabled = bool(AUTO_TRIM_ENABLED)
        self.auto_trim_ki = float(AUTO_TRIM_KI_DEG_PER_MM_S)
        self.auto_trim_max_deg = float(AUTO_TRIM_MAX_DEG)
        self.auto_trim_settle_speed_mm_s = float(AUTO_TRIM_SETTLE_SPEED_MM_S)
        self.auto_trim_settle_radius_mm = float(AUTO_TRIM_SETTLE_RADIUS_MM)
        self.auto_trim_settle_hold_s = float(AUTO_TRIM_SETTLE_HOLD_S)
        self.auto_trim_step_limit_deg = float(AUTO_TRIM_STEP_LIMIT_DEG)
        self._settled_time_s = 0.0
        self._last_trim_update_time = None
        self._auto_trim_update_count = 0
        self._prev_roll_cmd = 0.0
        self._prev_pitch_cmd = 0.0
        self._prev_cmd_time = None
        self.target_x_mm = float(BALL_TARGET_DEFAULT_X_MM)
        self.target_y_mm = float(BALL_TARGET_DEFAULT_Y_MM)
        
        self.pd_autotune_enabled = bool(PD_AUTOTUNE_ENABLED)
        self.pd_autotune_auto_apply = bool(PD_AUTOTUNE_AUTO_APPLY)
        self.pd_autotune_kp_step = float(PD_AUTOTUNE_KP_STEP)
        self.pd_autotune_kd_step = float(PD_AUTOTUNE_KD_STEP)
        self.pd_autotune_min_kp = float(PD_AUTOTUNE_MIN_KP)
        self.pd_autotune_max_kp = float(PD_AUTOTUNE_MAX_KP)
        self.pd_autotune_min_kd = float(PD_AUTOTUNE_MIN_KD)
        self.pd_autotune_max_kd = float(PD_AUTOTUNE_MAX_KD)
        self.pd_autotune_target_settle_s = float(PD_AUTOTUNE_TARGET_SETTLE_S)
        self._pd_autotune_trial_count = 0
        self._pd_autotune_last_message = ""
        self._pd_autotune_has_suggestion = False
        self._pd_autotune_suggested_kp = float(self.kp)
        self._pd_autotune_suggested_kd = float(self.kd)
        self.pd_autotune_test_diag_mm = float(PD_AUTOTUNE_TEST_DIAGONAL_MM)
        self._pd_leg_eval = _PDLegEvaluator()
        self._pd_autotune_leg_index = 0
        self._pd_autotune_leg_initialized = False
        self._manual_target_x_mm = float(self.target_x_mm)
        self._manual_target_y_mm = float(self.target_y_mm)
        self._pd_autotune_center_x_mm = float(self.target_x_mm)
        self._pd_autotune_center_y_mm = float(self.target_y_mm)

    # ---------------------------
    # Public Interface
    # ---------------------------

    def set_gains(self, kp: float, kd: float):
        """Update controller gains live (from GUI sliders)."""
        self.kp = kp
        self.kd = kd
        self._pd_autotune_has_suggestion = False
        self._pd_autotune_suggested_kp = float(kp)
        self._pd_autotune_suggested_kd = float(kd)

    def set_max_tilt(self, max_tilt_deg: float):
        """Update safety tilt limit."""
        self.max_tilt_deg = max_tilt_deg

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def reset_trim(self):
        self.roll_offset = float(MANUAL_ROLL_TRIM_DEG)
        self.pitch_offset = float(MANUAL_PITCH_TRIM_DEG)
        self._settled_time_s = 0.0
        self._auto_trim_update_count = 0

    def set_trim(self, roll_offset_deg: float, pitch_offset_deg: float):
        self.roll_offset = float(roll_offset_deg)
        self.pitch_offset = float(pitch_offset_deg)

    def set_auto_trim_enabled(self, enabled: bool):
        self.auto_trim_enabled = bool(enabled)

    def set_target(self, x_mm: float, y_mm: float):
        self._manual_target_x_mm = float(x_mm)
        self._manual_target_y_mm = float(y_mm)
        if self.pd_autotune_enabled:
            # While autotune is running, interpret slider target as test-center.
            self._pd_autotune_center_x_mm = float(x_mm)
            self._pd_autotune_center_y_mm = float(y_mm)
            self._pd_autotune_leg_initialized = False
            self._pd_leg_eval.active = False
        else:
            self.target_x_mm = float(x_mm)
            self.target_y_mm = float(y_mm)

    def set_pd_autotune(self, enabled: bool, auto_apply: bool | None = None):
        prev_enabled = self.pd_autotune_enabled
        self.pd_autotune_enabled = bool(enabled)
        if auto_apply is not None:
            self.pd_autotune_auto_apply = bool(auto_apply)
        if prev_enabled != self.pd_autotune_enabled:
            self._pd_autotune_leg_initialized = False
            self._pd_leg_eval.active = False
            if self.pd_autotune_enabled:
                self._pd_autotune_center_x_mm = float(self._manual_target_x_mm)
                self._pd_autotune_center_y_mm = float(self._manual_target_y_mm)
            else:
                self.target_x_mm = float(self._manual_target_x_mm)
                self.target_y_mm = float(self._manual_target_y_mm)
        if (not self.pd_autotune_enabled) or self.pd_autotune_auto_apply:
            self._pd_autotune_has_suggestion = False

    def apply_pd_autotune_recommendation(self):
        if not self._pd_autotune_has_suggestion:
            return False, float(self.kp), float(self.kd)
        self.kp = float(self._pd_autotune_suggested_kp)
        self.kd = float(self._pd_autotune_suggested_kd)
        self._pd_autotune_has_suggestion = False
        self._pd_autotune_last_message = f"manual apply kp={self.kp:.4f}, kd={self.kd:.4f}"
        if self.debug_level >= 1:
            print(f"[PD TUNE] {self._pd_autotune_last_message}")
        return True, float(self.kp), float(self.kd)

    def compute(self, ball_state: dict | BallState):
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

        x, y, vx, vy = self._extract_state(ball_state)

        # ---------------------------
        # Control Law
        # ---------------------------

        # Error: want ball at configurable target.
        ex = self.target_x_mm - x
        ey = self.target_y_mm - y

        # PD control
        pitch = self.kp * ex + self.kd * (-vx)
        roll = -(self.kp * ey + self.kd * (-vy))
        
        pitch = pitch + self.pitch_offset
        roll = roll + self.roll_offset

        # ---------------------------
        # Safety Clamping
        # ---------------------------

        roll = self._clamp(roll, -self.max_tilt_deg, self.max_tilt_deg)
        pitch = self._clamp(pitch, -self.max_tilt_deg, self.max_tilt_deg)
        
        t1 = time.perf_counter()
        
        self._log_counter += 1
        if self.debug_level >= 2 and (self._log_counter % self.log_every_n == 0):
            print(f"PD compute: {(t1-t0)*1000:.3f} ms")
        
        # print(f"pitch={pitch:.3f}, roll_y={roll:.3f}")

        return roll, pitch

    def compute_with_terms(self, ball_state: dict | BallState):
        """
        Compute desired platform tilt and expose planar PD terms for diagnostics.
        Returns:
            (roll_deg, pitch_deg, terms_dict)
        """
        if not self.enabled or ball_state is None:
            return 0.0, 0.0, {
                "position_vec_mm": (0.0, 0.0),
                "velocity_vec_mm_s": (0.0, 0.0),
                "pd_vec": (0.0, 0.0),
            }

        x, y, vx, vy = self._extract_state(ball_state)
        tune_info = self._update_pd_autotune(x, y, vx, vy)

        pos_vec_x = self.target_x_mm - x
        pos_vec_y = self.target_y_mm - y
        p_x = self.kp * pos_vec_x
        p_y = self.kp * pos_vec_y
        d_x = self.kd * (-vx)
        d_y = self.kd * (-vy)
        if self.d_term_limit_deg is not None:
            d_x = self._clamp(d_x, -self.d_term_limit_deg, self.d_term_limit_deg)
            d_y = self._clamp(d_y, -self.d_term_limit_deg, self.d_term_limit_deg)

        pd_x = p_x + d_x
        pd_y = p_y + d_y

        self._update_auto_trim(pos_vec_x, pos_vec_y, vx, vy)

        # Map planar control vector to platform axes.
        pitch_raw = pd_x + self.pitch_offset
        roll_raw = -pd_y + self.roll_offset
        roll_clamped = self._clamp(roll_raw, -self.max_tilt_deg, self.max_tilt_deg)
        pitch_clamped = self._clamp(pitch_raw, -self.max_tilt_deg, self.max_tilt_deg)
        roll, pitch = self._apply_slew_limit(roll_clamped, pitch_clamped)
        terms = {
            "position_vec_mm": (pos_vec_x, pos_vec_y),
            "velocity_vec_mm_s": (vx, vy),
            "pd_vec": (pd_x, pd_y),
            "p_term": (p_x, p_y),
            "d_term": (d_x, d_y),
            "roll_raw": roll_raw,
            "pitch_raw": pitch_raw,
            "roll_clamped": roll_clamped,
            "pitch_clamped": pitch_clamped,
            "roll_cmd": roll,
            "pitch_cmd": pitch,
            "roll_offset": self.roll_offset,
            "pitch_offset": self.pitch_offset,
            "trim_settled_s": self._settled_time_s,
            "trim_updates": self._auto_trim_update_count,
            "auto_trim_enabled": self.auto_trim_enabled,
            "kp": self.kp,
            "kd": self.kd,
            "pd_autotune_enabled": self.pd_autotune_enabled,
            "pd_autotune_auto_apply": self.pd_autotune_auto_apply,
            "pd_autotune_trial_count": self._pd_autotune_trial_count,
            "pd_autotune_message": self._pd_autotune_last_message,
            "pd_autotune_has_suggestion": self._pd_autotune_has_suggestion,
            "pd_autotune_suggested_kp": self._pd_autotune_suggested_kp,
            "pd_autotune_suggested_kd": self._pd_autotune_suggested_kd,
            "target_x_mm": self.target_x_mm,
            "target_y_mm": self.target_y_mm,
        }
        if tune_info:
            terms["pd_autotune_event"] = tune_info
        return roll, pitch, terms

    def _update_auto_trim(self, ex, ey, vx, vy):
        now = time.perf_counter()
        if self._last_trim_update_time is None:
            self._last_trim_update_time = now
            return

        dt = max(1e-4, now - self._last_trim_update_time)
        self._last_trim_update_time = now

        if (not self.auto_trim_enabled) or (not self.enabled):
            self._settled_time_s = 0.0
            return

        speed = math.hypot(vx, vy)
        radius = math.hypot(ex, ey)
        settled = (speed <= self.auto_trim_settle_speed_mm_s) and (radius <= self.auto_trim_settle_radius_mm)
        if not settled:
            self._settled_time_s = 0.0
            return

        self._settled_time_s += dt
        if self._settled_time_s < self.auto_trim_settle_hold_s:
            return

        # Very slow integral-style trim to cancel static bias.
        roll_rate = self.auto_trim_ki * (-ey)
        pitch_rate = self.auto_trim_ki * ex
        roll_step = self._clamp(roll_rate * dt, -self.auto_trim_step_limit_deg, self.auto_trim_step_limit_deg)
        pitch_step = self._clamp(pitch_rate * dt, -self.auto_trim_step_limit_deg, self.auto_trim_step_limit_deg)

        self.roll_offset = self._clamp(self.roll_offset + roll_step, -self.auto_trim_max_deg, self.auto_trim_max_deg)
        self.pitch_offset = self._clamp(self.pitch_offset + pitch_step, -self.auto_trim_max_deg, self.auto_trim_max_deg)
        self._auto_trim_update_count += 1

    def _update_pd_autotune(self, x, y, vx, vy):
        if (not self.pd_autotune_enabled) or (not self.enabled):
            return None
        now = time.perf_counter()
        if not self._pd_autotune_leg_initialized:
            self._pd_autotune_leg_index = 0
            tx, ty = self._autotune_leg_target(self._pd_autotune_leg_index)
            self.target_x_mm = tx
            self.target_y_mm = ty
            self._pd_leg_eval.start(now, x, y, tx, ty)
            self._pd_autotune_leg_initialized = True
            msg = f"active test started: target=({tx:.1f},{ty:.1f})"
            self._pd_autotune_last_message = msg
            return {"type": "trial_started", "message": msg}

        metrics = self._pd_leg_eval.observe(
            now,
            x,
            y,
            vx,
            vy,
            self.target_x_mm,
            self.target_y_mm,
            settle_radius_mm=PD_AUTOTUNE_SETTLE_RADIUS_MM,
            settle_speed_mm_s=PD_AUTOTUNE_SETTLE_SPEED_MM_S,
            settle_hold_s=PD_AUTOTUNE_SETTLE_HOLD_S,
            min_trial_s=PD_AUTOTUNE_MIN_TRIAL_S,
            timeout_s=PD_AUTOTUNE_TIMEOUT_S,
        )
        if metrics is None:
            return None

        self._pd_autotune_trial_count += 1
        kp_new, kd_new, rationale = self._recommend_pd_update(metrics)
        changed = (abs(kp_new - self.kp) > 1e-9) or (abs(kd_new - self.kd) > 1e-9)

        if changed and self.pd_autotune_auto_apply:
            self.kp = kp_new
            self.kd = kd_new
            self._pd_autotune_has_suggestion = False
            self._pd_autotune_suggested_kp = float(self.kp)
            self._pd_autotune_suggested_kd = float(self.kd)
            action = f"applied kp={self.kp:.4f}, kd={self.kd:.4f}"
        elif changed:
            self._pd_autotune_has_suggestion = True
            self._pd_autotune_suggested_kp = float(kp_new)
            self._pd_autotune_suggested_kd = float(kd_new)
            action = f"suggest kp={kp_new:.4f}, kd={kd_new:.4f}"
        else:
            self._pd_autotune_has_suggestion = False
            self._pd_autotune_suggested_kp = float(self.kp)
            self._pd_autotune_suggested_kd = float(self.kd)
            action = "hold gains"

        self._pd_autotune_leg_index = 1 - self._pd_autotune_leg_index
        next_tx, next_ty = self._autotune_leg_target(self._pd_autotune_leg_index)
        self.target_x_mm = next_tx
        self.target_y_mm = next_ty
        self._pd_leg_eval.start(now, x, y, next_tx, next_ty)

        msg = (
            f"trial#{self._pd_autotune_trial_count} "
            f"target=({metrics.get('target_x_mm', 0.0):.1f},{metrics.get('target_y_mm', 0.0):.1f}) "
            f"settle={metrics['settle_time_s']:.2f}s "
            f"overshoot={metrics['overshoot_ratio']:.2f} "
            f"cross={metrics['oscillation_crossings']:.0f} "
            f"iae={metrics['iae_mm_s']:.1f} -> {action} ({rationale}); "
            f"next=({next_tx:.1f},{next_ty:.1f})"
        )
        self._pd_autotune_last_message = msg
        if self.debug_level >= 1:
            print(f"[PD TUNE] {msg}")
        return {
            "type": "trial_done",
            "message": msg,
            "metrics": metrics,
            "kp_suggested": kp_new,
            "kd_suggested": kd_new,
            "applied": 1.0 if (changed and self.pd_autotune_auto_apply) else 0.0,
        }

    def _autotune_leg_target(self, leg_index: int):
        sign = 1.0 if (int(leg_index) % 2 == 0) else -1.0
        d = self.pd_autotune_test_diag_mm
        return (self._pd_autotune_center_x_mm + sign * d, self._pd_autotune_center_y_mm + sign * d)

    def _recommend_pd_update(self, metrics):
        kp_new = float(self.kp)
        kd_new = float(self.kd)
        overshoot_ratio = float(metrics.get("overshoot_ratio", 0.0))
        settle_s = float(metrics.get("settle_time_s", 0.0))
        crossings = int(round(metrics.get("oscillation_crossings", 0.0)))
        timed_out = bool(metrics.get("timed_out", 0.0) > 0.5)
        rationale = "balanced"

        if timed_out:
            kp_new = min(self.pd_autotune_max_kp, kp_new + 0.5 * self.pd_autotune_kp_step)
            kd_new = min(self.pd_autotune_max_kd, kd_new + self.pd_autotune_kd_step)
            rationale = "timeout: increase authority and damping"
        elif overshoot_ratio > 0.28 or crossings >= 2:
            kd_new = min(self.pd_autotune_max_kd, kd_new + self.pd_autotune_kd_step)
            kp_new = max(self.pd_autotune_min_kp, kp_new - 0.5 * self.pd_autotune_kp_step)
            rationale = "overshoot/oscillation: add damping"
        elif settle_s > self.pd_autotune_target_settle_s and overshoot_ratio < 0.12:
            kp_new = min(self.pd_autotune_max_kp, kp_new + self.pd_autotune_kp_step)
            rationale = "slow but stable: raise kp"
        elif settle_s < 0.7 * self.pd_autotune_target_settle_s and overshoot_ratio < 0.05 and crossings == 0:
            kd_new = max(self.pd_autotune_min_kd, kd_new - 0.5 * self.pd_autotune_kd_step)
            rationale = "fast and clean: trim kd"

        kp_new = self._clamp(kp_new, self.pd_autotune_min_kp, self.pd_autotune_max_kp)
        kd_new = self._clamp(kd_new, self.pd_autotune_min_kd, self.pd_autotune_max_kd)
        return kp_new, kd_new, rationale

    def _apply_slew_limit(self, roll, pitch):
        now = time.perf_counter()
        if self._prev_cmd_time is None:
            self._prev_cmd_time = now
            self._prev_roll_cmd = roll
            self._prev_pitch_cmd = pitch
            return roll, pitch

        dt = max(1e-4, now - self._prev_cmd_time)
        self._prev_cmd_time = now
        max_step = self.max_tilt_rate_deg_s * dt
        roll_limited = self._prev_roll_cmd + self._clamp(roll - self._prev_roll_cmd, -max_step, max_step)
        pitch_limited = self._prev_pitch_cmd + self._clamp(pitch - self._prev_pitch_cmd, -max_step, max_step)
        self._prev_roll_cmd = roll_limited
        self._prev_pitch_cmd = pitch_limited
        return roll_limited, pitch_limited

    # ---------------------------
    # Internal Helpers
    # ---------------------------

    @staticmethod
    def _extract_state(ball_state: dict | BallState):
        if isinstance(ball_state, dict):
            return (
                float(ball_state["x_mm"]),
                float(ball_state["y_mm"]),
                float(ball_state["vx_mm_s"]),
                float(ball_state["vy_mm_s"]),
            )
        return (
            float(ball_state.x_mm),
            float(ball_state.y_mm),
            float(ball_state.vx_mm_s),
            float(ball_state.vy_mm_s),
        )

    @staticmethod
    def _clamp(value: float, min_val: float, max_val: float):
        return max(min(value, max_val), min_val)
