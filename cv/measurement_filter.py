"""
cv/measurement_filter.py

Measurement filtering between raw ball detections and BallState: the
raw-speed outlier gate plus one of three position/velocity filter modes
(TRACKER_FILTER_MODE):

- "alpha_beta" — AlphaBetaFilter2D, an adaptive alpha-beta tracker whose
  gains are scheduled between (ALPHA_MIN, BETA_MIN) at rest and
  (ALPHA_MAX, BETA_MAX) under large innovations or high predicted speed.
  Near-static camera noise barely moves the state; real motion opens the
  gains and tracks within a couple of frames.
- "legacy" — the original adaptive position low-pass (slow/fast alpha
  bounded by max lag) + velocity EMA, extracted verbatim from
  BallTracker.process(). Kept as a regression reference.
- "raw" — passthrough position + raw finite-difference velocity, for
  bench A/B comparison only.

The raw-speed outlier gate runs BEFORE the active filter in every mode,
computed from raw finite differences exactly as before.

Consumers:
- cv/ball_tracker.py feeds it detected positions per frame (pinned by
  tests/test_ball_tracker.py and tests/test_measurement_filter.py).
- tools/jitter_bench.py feeds it synthetic position streams to benchmark
  the filter -> controller -> quantizer chain without a camera
  (--filter-mode overrides the settings default for A/B runs).
"""

from __future__ import annotations

import math

import numpy as np

from settings import (
    BALL_VEL_FILTER_ALPHA,
    TRACKER_AB_ALPHA_MAX,
    TRACKER_AB_ALPHA_MIN,
    TRACKER_AB_BETA_MAX,
    TRACKER_AB_BETA_MIN,
    TRACKER_AB_INNOV_FULL_MM,
    TRACKER_AB_INNOV_OPEN_MM,
    TRACKER_AB_SPEED_FULL_MM_S,
    TRACKER_AB_SPEED_OPEN_MM_S,
    TRACKER_FILTER_MODE,
    TRACKER_MAX_SPEED_MM_S,
    TRACKER_POS_FILTER_ALPHA_FAST,
    TRACKER_POS_FILTER_ALPHA_SLOW,
    TRACKER_POS_FILTER_ENABLED,
    TRACKER_POS_FILTER_MAX_LAG_MM,
    TRACKER_POS_FILTER_SPEED_MM_S,
)

FILTER_MODES = ("alpha_beta", "legacy", "raw")

_DT_MIN_S = 1e-4
_DT_MAX_S = 0.1


def _unit_ramp(value: float, open_at: float, full_at: float) -> float:
    """0 below open_at, 1 above full_at, linear in between."""
    if full_at <= open_at:
        return 1.0 if value >= full_at else 0.0
    return min(1.0, max(0.0, (value - open_at) / (full_at - open_at)))


class AlphaBetaFilter2D:
    """Adaptive 2D alpha-beta position/velocity tracker.

    Per update: predict (constant velocity), measure the innovation, then
    correct with gains scheduled by max(innovation gate, speed gate) —
    small residuals at low speed use the quiet MIN gains, large residuals
    or fast predicted motion open the gains toward MAX. dt is clamped to
    [1e-4, 0.1] s so stalled or duplicate timestamps never blow up the
    velocity correction. The first sample seeds state (x=z, v=0)."""

    def __init__(
        self,
        alpha_min: float = TRACKER_AB_ALPHA_MIN,
        alpha_max: float = TRACKER_AB_ALPHA_MAX,
        beta_min: float = TRACKER_AB_BETA_MIN,
        beta_max: float = TRACKER_AB_BETA_MAX,
        innov_open_mm: float = TRACKER_AB_INNOV_OPEN_MM,
        innov_full_mm: float = TRACKER_AB_INNOV_FULL_MM,
        speed_open_mm_s: float = TRACKER_AB_SPEED_OPEN_MM_S,
        speed_full_mm_s: float = TRACKER_AB_SPEED_FULL_MM_S,
    ) -> None:
        self.alpha_min = float(alpha_min)
        self.alpha_max = float(alpha_max)
        self.beta_min = float(beta_min)
        self.beta_max = float(beta_max)
        self.innov_open_mm = float(innov_open_mm)
        self.innov_full_mm = float(innov_full_mm)
        self.speed_open_mm_s = float(speed_open_mm_s)
        self.speed_full_mm_s = float(speed_full_mm_s)

        self._x: float = 0.0
        self._y: float = 0.0
        self._vx: float = 0.0
        self._vy: float = 0.0
        self._prev_t: float | None = None

    @property
    def vx(self) -> float:
        return self._vx

    @property
    def vy(self) -> float:
        return self._vy

    def update(
        self, z_x: float, z_y: float, t: float
    ) -> tuple[float, float, float, float]:
        """Fold one measurement in; return (x, y, vx, vy)."""
        if self._prev_t is None:
            self._x = float(z_x)
            self._y = float(z_y)
            self._vx = 0.0
            self._vy = 0.0
            self._prev_t = float(t)
            return self._x, self._y, 0.0, 0.0

        dt = min(_DT_MAX_S, max(_DT_MIN_S, float(t) - self._prev_t))

        x_pred = self._x + self._vx * dt
        y_pred = self._y + self._vy * dt
        vx_pred = self._vx
        vy_pred = self._vy

        rx = float(z_x) - x_pred
        ry = float(z_y) - y_pred
        r_mag = math.hypot(rx, ry)

        g_inn = _unit_ramp(r_mag, self.innov_open_mm, self.innov_full_mm)
        g_spd = _unit_ramp(
            math.hypot(vx_pred, vy_pred),
            self.speed_open_mm_s,
            self.speed_full_mm_s,
        )
        g = max(g_inn, g_spd)

        alpha = self.alpha_min + (self.alpha_max - self.alpha_min) * g
        beta = self.beta_min + (self.beta_max - self.beta_min) * g

        self._x = x_pred + alpha * rx
        self._y = y_pred + alpha * ry
        self._vx = vx_pred + (beta / dt) * rx
        self._vy = vy_pred + (beta / dt) * ry
        self._prev_t = float(t)
        return self._x, self._y, self._vx, self._vy

    def reset(self) -> None:
        """Clear state; the next update re-seeds (x=z, v=0)."""
        self._x = 0.0
        self._y = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._prev_t = None


class MeasurementFilter:
    """Filters raw (x, y) position measurements into a filtered position
    plus a filtered velocity estimate, per the active mode."""

    def __init__(self, mode: str = TRACKER_FILTER_MODE) -> None:
        mode = str(mode)
        if mode not in FILTER_MODES:
            raise ValueError(
                f"unknown filter mode {mode!r} (choose from {FILTER_MODES})"
            )
        self.mode = mode

        # --- Legacy-path parameters (same defaults/clamps as before) ---
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

        # --- Alpha-beta path ---
        self._ab = AlphaBetaFilter2D()

        # --- Per-track state ---
        self.prev_ball_mm: tuple[float, float] | None = None
        self.prev_ball_raw_mm: tuple[float, float] | None = None
        self.prev_time: float | None = None
        self._vx_f: float = 0.0   # active filter's velocity (mirrored)
        self._vy_f: float = 0.0

    def update(
        self, x_mm_raw: float, y_mm_raw: float, t: float
    ) -> tuple[float, float, float, float] | None:
        """Fold one raw measurement in; return (x, y, vx, vy) filtered.

        Returns None when the raw-speed outlier gate rejects the sample
        (state is NOT advanced; the caller must treat it as a detection
        loss and call reset(), matching BallTracker._miss()).
        """
        # ---- Raw finite-difference velocity + outlier gate (all modes) ----
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

        if self.mode == "alpha_beta":
            x_mm, y_mm, vx, vy = self._ab.update(x_mm_raw, y_mm_raw, t)
            self._vx_f = vx
            self._vy_f = vy
        elif self.mode == "raw":
            x_mm = x_mm_raw
            y_mm = y_mm_raw
            vx = vx_raw_meas
            vy = vy_raw_meas
            self._vx_f = vx
            self._vy_f = vy
        else:
            x_mm, y_mm, vx, vy = self._update_legacy(
                x_mm_raw, y_mm_raw, dt, vx_raw_meas, vy_raw_meas, raw_speed_mm_s
            )

        self.prev_ball_raw_mm = (x_mm_raw, y_mm_raw)
        self.prev_ball_mm = (x_mm, y_mm)
        self.prev_time = t

        return x_mm, y_mm, vx, vy

    def _update_legacy(
        self,
        x_mm_raw: float,
        y_mm_raw: float,
        dt: float,
        vx_raw_meas: float,
        vy_raw_meas: float,
        raw_speed_mm_s: float,
    ) -> tuple[float, float, float, float]:
        """Original position low-pass + velocity EMA (behavior unchanged)."""
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

        return x_mm, y_mm, vx, vy

    def reset(self) -> None:
        """Detection-loss reset (matches the old BallTracker._miss()).

        Resets whichever filter is active: the alpha-beta state clears and
        re-seeds on the next update; the legacy velocity EMA zeroes and its
        position filter re-seeds. prev_time/prev_ball_raw_mm deliberately
        survive so the raw-speed gate still spans a short loss (identical
        to the pre-extraction behavior)."""
        self._vx_f = 0.0
        self._vy_f = 0.0
        self.prev_ball_mm = None
        self._ab.reset()
