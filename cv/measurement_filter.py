"""
cv/measurement_filter.py

MeasurementFilter — the position/velocity measurement filtering stage
extracted verbatim from BallTracker.process() (perf pass, 2026-07-22):
raw-speed outlier rejection, the adaptive position low-pass (slow/fast
alpha blend bounded by max lag), and the velocity EMA.

Two consumers:
- cv/ball_tracker.py feeds it detected positions per frame (behavior is
  pinned by the synthetic-frame tests in tests/test_ball_tracker.py).
- tools/jitter_bench.py feeds it synthetic position streams to benchmark
  the filter -> controller -> quantizer chain without a camera.

This class is also the seam a future alpha-beta filter drops into.
"""

from __future__ import annotations

import numpy as np

from settings import (
    BALL_VEL_FILTER_ALPHA,
    TRACKER_MAX_SPEED_MM_S,
    TRACKER_POS_FILTER_ALPHA_FAST,
    TRACKER_POS_FILTER_ALPHA_SLOW,
    TRACKER_POS_FILTER_ENABLED,
    TRACKER_POS_FILTER_MAX_LAG_MM,
    TRACKER_POS_FILTER_SPEED_MM_S,
)


class MeasurementFilter:
    """Filters raw (x, y) position measurements into a filtered position
    plus a low-pass filtered velocity estimate."""

    def __init__(self) -> None:
        # --- Parameters (same defaults/clamps the tracker applied) ---
        self.pos_filter_enabled = bool(TRACKER_POS_FILTER_ENABLED)
        self.pos_filter_alpha_slow = float(
            np.clip(TRACKER_POS_FILTER_ALPHA_SLOW, 0.0, 0.98)
        )
        self.pos_filter_alpha_fast = float(
            np.clip(TRACKER_POS_FILTER_ALPHA_FAST, 0.0, 0.98)
        )
        self.pos_filter_speed_mm_s = max(1.0, float(TRACKER_POS_FILTER_SPEED_MM_S))
        self.pos_filter_max_lag_mm = max(0.1, float(TRACKER_POS_FILTER_MAX_LAG_MM))
        self.max_speed_mm_s = float(TRACKER_MAX_SPEED_MM_S)
        self.vel_filter_alpha = float(BALL_VEL_FILTER_ALPHA)
        self._last_pos_filter_alpha = self.pos_filter_alpha_fast

        # --- Per-track state ---
        self.prev_ball_mm: tuple[float, float] | None = None
        self.prev_ball_raw_mm: tuple[float, float] | None = None
        self.prev_time: float | None = None
        self._vx_f: float = 0.0   # low-pass filtered velocity
        self._vy_f: float = 0.0

    def update(
        self, x_mm_raw: float, y_mm_raw: float, t: float
    ) -> tuple[float, float, float, float] | None:
        """Fold one raw measurement in; return (x, y, vx, vy) filtered.

        Returns None when the raw-speed outlier gate rejects the sample
        (state is NOT advanced; the caller must treat it as a detection
        loss and call reset(), matching BallTracker._miss()).
        """
        # ---- Velocity measurement ----
        dt = 0.0
        vx_raw_meas = 0.0
        vy_raw_meas = 0.0
        raw_speed_mm_s = 0.0
        if self.prev_time is not None and self.prev_ball_raw_mm is not None:
            dt = t - self.prev_time
            if dt > 0:
                vx_raw_meas = (x_mm_raw - self.prev_ball_raw_mm[0]) / dt
                vy_raw_meas = (y_mm_raw - self.prev_ball_raw_mm[1]) / dt
                raw_speed_mm_s = float(np.hypot(vx_raw_meas, vy_raw_meas))
                if self.max_speed_mm_s > 0 and raw_speed_mm_s > self.max_speed_mm_s:
                    return None

        # ---- Position filter ----
        if self.pos_filter_enabled and self.prev_ball_mm is not None:
            speed_ratio = min(1.0, raw_speed_mm_s / self.pos_filter_speed_mm_s)
            filter_alpha = (
                self.pos_filter_alpha_slow
                + (self.pos_filter_alpha_fast - self.pos_filter_alpha_slow)
                * speed_ratio
            )
            x_mm = filter_alpha * self.prev_ball_mm[0] + (1.0 - filter_alpha) * x_mm_raw
            y_mm = filter_alpha * self.prev_ball_mm[1] + (1.0 - filter_alpha) * y_mm_raw
            lag = float(np.hypot(x_mm_raw - x_mm, y_mm_raw - y_mm))
            if lag > self.pos_filter_max_lag_mm:
                scale = self.pos_filter_max_lag_mm / max(lag, 1e-6)
                x_mm = x_mm_raw - (x_mm_raw - x_mm) * scale
                y_mm = y_mm_raw - (y_mm_raw - y_mm) * scale
            self._last_pos_filter_alpha = float(filter_alpha)
        else:
            x_mm = x_mm_raw
            y_mm = y_mm_raw

        # ---- Velocity EMA ----
        if self.prev_ball_mm is None or self.prev_time is None or dt <= 0:
            vx = 0.0
            vy = 0.0
            self._vx_f = 0.0
            self._vy_f = 0.0
        else:
            alpha = self.vel_filter_alpha
            self._vx_f = alpha * vx_raw_meas + (1.0 - alpha) * self._vx_f
            self._vy_f = alpha * vy_raw_meas + (1.0 - alpha) * self._vy_f
            vx = self._vx_f
            vy = self._vy_f

        self.prev_ball_raw_mm = (x_mm_raw, y_mm_raw)
        self.prev_ball_mm = (x_mm, y_mm)
        self.prev_time = t

        return x_mm, y_mm, vx, vy

    def reset(self) -> None:
        """Detection-loss reset (matches the old BallTracker._miss()).

        Velocity zeroes and the position filter re-seeds on the next
        update; prev_time/prev_ball_raw_mm deliberately survive so the
        raw-speed gate still spans a short loss (identical to the
        pre-extraction behavior)."""
        self._vx_f = 0.0
        self._vy_f = 0.0
        self.prev_ball_mm = None
