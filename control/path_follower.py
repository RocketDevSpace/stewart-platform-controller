"""
control/path_follower.py

PathFollower — advances a target point along a patterns.Path at a
commanded speed, tapering the advance rate with the ball's tracking
error so the target never outruns the ball:

  factor = clamp((CAPTURE - err) / (CAPTURE - FULL_SPEED), 0, 1)

Full speed while the ball is within PATH_FULL_SPEED_RADIUS_MM of the
target, linearly down to frozen at PATH_CAPTURE_RADIUS_MM. The law is
continuous — "stalled" is telemetry only and auto-resumes as the ball
catches up. The target is seeded ONCE (on the first update() after
start()) at the path point nearest the ball and never re-seeded, so
self-approaching shapes cannot lobe-jump.

States: "idle", "armed", "following", "stalled", "paused", "done".

Pure Python + numpy, injected clock, no Qt, no hardware. One update()
per control cycle.
"""
import math
from collections.abc import Callable
from typing import Any

import numpy as np

from control.patterns import Path
from settings import (
    PATH_CAPTURE_RADIUS_MM,
    PATH_FULL_SPEED_RADIUS_MM,
    PATH_SPEED_MAX_MM_S,
    PATH_SPEED_MIN_MM_S,
    PATH_SPEED_MM_S,
)

STATE_IDLE = "idle"
STATE_ARMED = "armed"
STATE_FOLLOWING = "following"
STATE_STALLED = "stalled"
STATE_PAUSED = "paused"
STATE_DONE = "done"


def _clamp(value: float, min_val: float, max_val: float) -> float:
    return max(min(value, max_val), min_val)


class PathFollower:
    def __init__(
        self,
        clock: Callable[[], float],
        speed_mm_s: float = PATH_SPEED_MM_S,
    ) -> None:
        self._clock = clock
        self._speed_mm_s = _clamp(
            float(speed_mm_s), PATH_SPEED_MIN_MM_S, PATH_SPEED_MAX_MM_S
        )

        self._path: Path | None = None
        # Precomputed geometry (set_path). _cum_s[i] is the arc position of
        # point i; for closed paths _seg_len has the wrap segment appended
        # and _total_len includes it.
        self._points: np.ndarray = np.zeros((0, 2), dtype=np.float64)
        self._cum_s: np.ndarray = np.zeros(1, dtype=np.float64)
        self._seg_len: np.ndarray = np.zeros(0, dtype=np.float64)
        self._total_len = 0.0
        self._closed = False

        self._state = STATE_IDLE
        self._s_target = 0.0
        self._lap = 0
        self._done = False
        self._last_update_t: float | None = None
        self._last_err_mm = 0.0
        self._last_factor = 0.0

    # ---------------------------
    # Read-only state (telemetry)
    # ---------------------------

    @property
    def state(self) -> str:
        return self._state

    @property
    def active(self) -> bool:
        """True in any non-idle state (armed/following/stalled/paused/done)."""
        return self._state != STATE_IDLE

    @property
    def done(self) -> bool:
        return self._done

    @property
    def speed_mm_s(self) -> float:
        return self._speed_mm_s

    def telemetry(self) -> dict[str, Any]:
        """Complete telemetry snapshot — the key set is identical in every state."""
        commanded = (
            self._speed_mm_s * self._last_factor
            if self._state in (STATE_FOLLOWING, STATE_STALLED)
            else 0.0
        )
        progress = (
            self._s_target / self._total_len if self._total_len > 0.0 else 0.0
        )
        return {
            "path_active": self.active,
            "path_state": self._state,
            "path_name": "" if self._path is None else self._path.name,
            "path_progress": float(progress),
            "path_lap": int(self._lap),
            "path_s_mm": float(self._s_target),
            "path_error_mm": float(self._last_err_mm),
            "path_speed_mm_s": float(commanded),
        }

    # ---------------------------
    # Lifecycle
    # ---------------------------

    def set_path(self, path: Path) -> None:
        """Load/replace the path; forces idle and clears progress/lap/done."""
        pts = np.asarray(path.points, dtype=np.float64)
        seg = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
        cum = np.concatenate([[0.0], np.cumsum(seg)])
        if path.closed:
            wrap = float(np.hypot(pts[0, 0] - pts[-1, 0], pts[0, 1] - pts[-1, 1]))
            seg = np.concatenate([seg, [wrap]])
            total = float(cum[-1]) + wrap
        else:
            total = float(cum[-1])

        self._path = path
        self._points = pts
        self._cum_s = cum
        self._seg_len = seg
        self._total_len = total
        self._closed = path.closed

        self._state = STATE_IDLE
        self._s_target = 0.0
        self._lap = 0
        self._done = False
        self._last_update_t = None
        self._last_err_mm = 0.0
        self._last_factor = 0.0

    def start(self) -> bool:
        """Arm the follower; the target seed happens on the FIRST update()."""
        if self._path is None:
            return False
        self._state = STATE_ARMED
        self._lap = 0
        self._done = False
        self._last_update_t = None
        self._last_factor = 0.0
        return True

    def stop(self) -> None:
        self._state = STATE_IDLE
        self._done = False
        self._last_factor = 0.0

    def pause(self) -> None:
        """Freeze advance; only meaningful mid-run (following/stalled)."""
        if self._state in (STATE_FOLLOWING, STATE_STALLED):
            self._state = STATE_PAUSED

    def resume(self) -> None:
        if self._state == STATE_PAUSED:
            self._state = STATE_FOLLOWING

    def set_speed(self, mm_s: float) -> None:
        """Live speed change, clamped to [PATH_SPEED_MIN, PATH_SPEED_MAX]."""
        self._speed_mm_s = _clamp(
            float(mm_s), PATH_SPEED_MIN_MM_S, PATH_SPEED_MAX_MM_S
        )

    # ---------------------------
    # Per-cycle update
    # ---------------------------

    def update(self, ball_x: float, ball_y: float) -> tuple[float, float]:
        """One control-cycle step; returns the target point (x, y) in mm.

        Idle (or no path loaded): a documented no-op — returns the ball's
        own position so a careless caller cannot move the setpoint.
        """
        path = self._path
        if path is None or self._state == STATE_IDLE:
            return float(ball_x), float(ball_y)

        if self._state == STATE_ARMED:
            # Seed ONCE at the path POINT nearest the ball (O(N) scan);
            # never re-seeded afterward — no lobe-jumping on
            # self-approaching shapes.
            d2 = (self._points[:, 0] - ball_x) ** 2 + (
                self._points[:, 1] - ball_y
            ) ** 2
            idx = int(np.argmin(d2))
            self._s_target = float(self._cum_s[idx])
            self._state = STATE_FOLLOWING
            self._last_update_t = self._clock()
            self._last_err_mm = float(math.sqrt(float(d2[idx])))
            self._last_factor = 0.0
            return float(self._points[idx, 0]), float(self._points[idx, 1])

        now = self._clock()
        last = now if self._last_update_t is None else self._last_update_t
        dt = _clamp(now - last, 1e-4, 0.1)
        self._last_update_t = now

        tx, ty = self._point_at(self._s_target)
        self._last_err_mm = math.hypot(ball_x - tx, ball_y - ty)

        if self._state in (STATE_PAUSED, STATE_DONE):
            # Advance frozen; err still tracked above for telemetry.
            self._last_factor = 0.0
            return tx, ty

        denom = max(1e-6, PATH_CAPTURE_RADIUS_MM - PATH_FULL_SPEED_RADIUS_MM)
        factor = _clamp(
            (PATH_CAPTURE_RADIUS_MM - self._last_err_mm) / denom, 0.0, 1.0
        )
        self._last_factor = factor
        # Telemetry only — the advance law is continuous and auto-resumes.
        self._state = STATE_STALLED if factor == 0.0 else STATE_FOLLOWING

        self._s_target += self._speed_mm_s * factor * dt
        if self._closed:
            if self._s_target >= self._total_len > 0.0:
                self._lap += int(self._s_target // self._total_len)
                self._s_target %= self._total_len
        elif self._s_target >= self._total_len:
            # Open path complete: hold the endpoint until stop().
            self._s_target = self._total_len
            self._state = STATE_DONE
            self._done = True
            self._last_factor = 0.0

        return self._point_at(self._s_target)

    # ---------------------------
    # Geometry
    # ---------------------------

    def _point_at(self, s: float) -> tuple[float, float]:
        """Point at arc position s (wraps when closed, clamps when open)."""
        pts = self._points
        n = len(pts)
        if self._closed:
            s = s % self._total_len if self._total_len > 0.0 else 0.0
        else:
            s = _clamp(s, 0.0, self._total_len)
        idx = int(np.searchsorted(self._cum_s, s, side="right")) - 1
        idx = max(0, min(idx, n - 1))
        if self._closed:
            nxt = (idx + 1) % n
            seg = float(self._seg_len[idx])
        else:
            if idx >= n - 1:
                return float(pts[-1, 0]), float(pts[-1, 1])
            nxt = idx + 1
            seg = float(self._cum_s[nxt] - self._cum_s[idx])
        if seg <= 0.0:
            return float(pts[idx, 0]), float(pts[idx, 1])
        frac = (s - float(self._cum_s[idx])) / seg
        px = float(pts[idx, 0] + frac * (pts[nxt, 0] - pts[idx, 0]))
        py = float(pts[idx, 1] + frac * (pts[nxt, 1] - pts[idx, 1]))
        return px, py
