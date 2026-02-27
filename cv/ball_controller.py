# -*- coding: utf-8 -*-
"""
Created on Tue Feb 24 16:13:27 2026

@author: hudso
"""

import math
import time
from dataclasses import dataclass
from stewart_control.config import DEBUG_LEVEL, LOG_EVERY_N


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
        
        self.pitch_offset = 0
        self.roll_offset = 0
        self._prev_roll_cmd = 0.0
        self._prev_pitch_cmd = 0.0
        self._prev_cmd_time = None

    # ---------------------------
    # Public Interface
    # ---------------------------

    def set_gains(self, kp: float, kd: float):
        """Update controller gains live (from GUI sliders)."""
        self.kp = kp
        self.kd = kd

    def set_max_tilt(self, max_tilt_deg: float):
        """Update safety tilt limit."""
        self.max_tilt_deg = max_tilt_deg

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

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

        # Error: want ball at (0, 0)
        ex = -x
        ey = -y

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

        pos_vec_x = -x
        pos_vec_y = -y
        p_x = self.kp * pos_vec_x
        p_y = self.kp * pos_vec_y
        d_x = self.kd * (-vx)
        d_y = self.kd * (-vy)
        if self.d_term_limit_deg is not None:
            d_x = self._clamp(d_x, -self.d_term_limit_deg, self.d_term_limit_deg)
            d_y = self._clamp(d_y, -self.d_term_limit_deg, self.d_term_limit_deg)

        pd_x = p_x + d_x
        pd_y = p_y + d_y

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
        }
        return roll, pitch, terms

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
