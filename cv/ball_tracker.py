"""
cv/ball_tracker.py

Pure detection pipeline: ArUco corner tracking -> perspective warp -> HSV
masking -> contour analysis -> BallState with filtered velocity.

No camera, no threads, no I/O (M11): frames are handed in via process(),
which makes the whole pipeline testable with synthetic images. Camera
lifecycle lives in cv/camera_source.py; debug views are surfaced through
debug_views() for the GUI's vision monitor (the old cv2.imshow paths are
gone — production always ran with them disabled).

Stale-homography policy (fixes the serve-stale-H-forever defect): the
cached homography from the last successful marker solve is reused between
scheduled redetects (every TRACKER_ARUCO_REDETECT_EVERY_N frames). Once a
redetect FAILS, the tracker switches to attempting detection EVERY frame,
serves the cached H for at most TRACKER_MAX_ARUCO_HOLD_FRAMES of those
attempts, then INVALIDATES the cache and reports loss. Marker loss can
therefore fabricate positions for a bounded ~(redetect_every_n +
hold_frames) frames, never indefinitely.
"""

from __future__ import annotations

import time

import cv2
import numpy as np

from core.platform_state import BallState
from settings import (
    BALL_VEL_FILTER_ALPHA,
    TRACKER_ARUCO_CENTER_FILTER_ALPHA,
    TRACKER_ARUCO_DETECT_SCALE,
    TRACKER_ARUCO_REDETECT_EVERY_N,
    TRACKER_MAX_ARUCO_HOLD_FRAMES,
    TRACKER_MAX_SPEED_MM_S,
    TRACKER_MIN_CIRCULARITY,
    TRACKER_MIN_CONTOUR_AREA,
    TRACKER_MIN_FILL_RATIO,
    TRACKER_MIN_RADIUS_PX,
    TRACKER_POS_FILTER_ALPHA_FAST,
    TRACKER_POS_FILTER_ALPHA_SLOW,
    TRACKER_POS_FILTER_ENABLED,
    TRACKER_POS_FILTER_MAX_LAG_MM,
    TRACKER_POS_FILTER_SPEED_MM_S,
    TRACKER_WARP_GRAY_CACHE_N,
)


class BallTracker:

    def __init__(
        self,
        platform_size_mm: float = 240.0,
        warp_size_px: int = 800,
    ) -> None:
        self.PLATFORM_SIZE_MM = float(platform_size_mm)
        self.WARP_SIZE_PX = int(warp_size_px)

        # --- ArUco setup ---
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.CORNER_IDS = [0, 1, 2, 3]
        self.aruco_world_mm = {
            0: np.array([60.0, 60.0]),
            1: np.array([-60.0, 60.0]),
            2: np.array([-60.0, -60.0]),
            3: np.array([60.0, -60.0]),
        }

        # --- ArUco tracking state ---
        self._last_H: np.ndarray | None = None
        self._aruco_hold_count = 0
        self._aruco_lost = False       # a redetect attempt has failed
        self._marker_centers_lp: dict[int, np.ndarray] = {}
        self._frame_counter = 0
        self.aruco_detect_scale = float(TRACKER_ARUCO_DETECT_SCALE)
        self.aruco_redetect_every_n = max(1, int(TRACKER_ARUCO_REDETECT_EVERY_N))
        self.aruco_center_filter_alpha = float(
            np.clip(TRACKER_ARUCO_CENTER_FILTER_ALPHA, 0.0, 0.98)
        )
        self.max_aruco_hold_frames = int(TRACKER_MAX_ARUCO_HOLD_FRAMES)

        # --- Ball detection params ---
        self.min_radius_px = float(TRACKER_MIN_RADIUS_PX)
        self.min_contour_area = float(TRACKER_MIN_CONTOUR_AREA)
        self.min_circularity = float(TRACKER_MIN_CIRCULARITY)
        self.min_fill_ratio = float(TRACKER_MIN_FILL_RATIO)
        self.max_speed_mm_s = float(TRACKER_MAX_SPEED_MM_S)
        self._morph_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        # --- Position filter state ---
        self.pos_filter_enabled = bool(TRACKER_POS_FILTER_ENABLED)
        self.pos_filter_alpha_slow = float(
            np.clip(TRACKER_POS_FILTER_ALPHA_SLOW, 0.0, 0.98)
        )
        self.pos_filter_alpha_fast = float(
            np.clip(TRACKER_POS_FILTER_ALPHA_FAST, 0.0, 0.98)
        )
        self.pos_filter_speed_mm_s = max(1.0, float(TRACKER_POS_FILTER_SPEED_MM_S))
        self.pos_filter_max_lag_mm = max(0.1, float(TRACKER_POS_FILTER_MAX_LAG_MM))
        self._last_pos_filter_alpha = self.pos_filter_alpha_fast

        # --- Warp brightness cache ---
        self._warp_gray_cache_counter: int = TRACKER_WARP_GRAY_CACHE_N
        self._cached_warp_gray_mean: float = 0.0

        # --- Velocity and position state ---
        self.prev_ball_mm: tuple[float, float] | None = None
        self.prev_ball_raw_mm: tuple[float, float] | None = None
        self.prev_time: float | None = None
        self._vx_f: float = 0.0   # low-pass filtered velocity
        self._vy_f: float = 0.0

        # --- HSV state (defaults from settings; live updates via setter) ---
        from settings import (
            TRACKER_HSV_H_MAX, TRACKER_HSV_H_MIN,
            TRACKER_HSV_S_MAX, TRACKER_HSV_S_MIN,
            TRACKER_HSV_V_MAX, TRACKER_HSV_V_MIN,
        )
        self.hsv_lower = np.array(
            [TRACKER_HSV_H_MIN, TRACKER_HSV_S_MIN, TRACKER_HSV_V_MIN],
            dtype=np.uint8,
        )
        self.hsv_upper = np.array(
            [TRACKER_HSV_H_MAX, TRACKER_HSV_S_MAX, TRACKER_HSV_V_MAX],
            dtype=np.uint8,
        )

        # --- Debug views (read via debug_views()) ---
        self._last_camera_bgr: np.ndarray | None = None
        self._last_warped_bgr: np.ndarray | None = None
        self._last_mask_gray: np.ndarray | None = None
        self._last_processed_ts = 0.0

    # =========================
    # Public API
    # =========================

    def set_hsv_thresholds(
        self, hmin: int, hmax: int, smin: int, smax: int, vmin: int, vmax: int
    ) -> None:
        self.hsv_lower = np.array([int(hmin), int(smin), int(vmin)], dtype=np.uint8)
        self.hsv_upper = np.array([int(hmax), int(smax), int(vmax)], dtype=np.uint8)

    def debug_views(
        self,
    ) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None]:
        """(camera_bgr, warped_bgr, mask_gray) from the last process() call."""
        return self._last_camera_bgr, self._last_warped_bgr, self._last_mask_gray

    def last_processed_ts(self) -> float:
        return self._last_processed_ts

    def process(
        self,
        frame_bgr: np.ndarray,
        frame_ts: float,
        brightness_gain: float = 1.0,
    ) -> BallState | None:
        """Run the full detection pipeline on one frame.

        frame_ts is the capture timestamp (perf_counter domain) used for
        velocity dt. Returns None when no ball (or no marker geometry) is
        found; velocity and position-filter state reset on loss so a
        reacquired ball never blends toward its stale pre-loss position.
        """
        self._last_processed_ts = frame_ts
        frame = cv2.flip(frame_bgr, 1)
        if brightness_gain > 1.02:
            try:
                frame = cv2.convertScaleAbs(frame, alpha=float(brightness_gain), beta=0)
            except Exception:
                pass
        self._last_camera_bgr = frame

        self._frame_counter += 1

        H = self._resolve_homography(frame)
        if H is None:
            return self._miss()

        warped = cv2.warpPerspective(frame, H, (self.WARP_SIZE_PX, self.WARP_SIZE_PX))
        self._last_warped_bgr = warped

        # ---- Adaptive HSV detection ----
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)

        self._warp_gray_cache_counter += 1
        if self._warp_gray_cache_counter >= TRACKER_WARP_GRAY_CACHE_N:
            self._warp_gray_cache_counter = 0
            self._cached_warp_gray_mean = float(
                np.mean(cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY))
            )
        gray_warp_mean = self._cached_warp_gray_mean
        vmin_base = int(self.hsv_lower[2])
        if gray_warp_mean < 45.0:
            relax = min(70, int((45.0 - gray_warp_mean) * 2.0))
            vmin_eff = max(20, vmin_base - relax)
        else:
            vmin_eff = vmin_base

        lower_eff = self.hsv_lower.copy()
        lower_eff[2] = vmin_eff
        mask = cv2.inRange(hsv, lower_eff, self.hsv_upper)
        self._last_mask_gray = mask
        ball = self.find_ball_center(mask)

        # Two-pass retry with further relaxed V-min
        if ball is None and vmin_eff > 20:
            lower_eff2 = self.hsv_lower.copy()
            lower_eff2[2] = max(10, vmin_eff - 20)
            mask = cv2.inRange(hsv, lower_eff2, self.hsv_upper)
            self._last_mask_gray = mask
            ball = self.find_ball_center(mask)

        if ball is None:
            return self._miss()

        (bx, by), _radius = ball

        cx = self.WARP_SIZE_PX // 2
        cy = self.WARP_SIZE_PX // 2
        mm_per_px = self.PLATFORM_SIZE_MM / self.WARP_SIZE_PX
        x_mm_raw = (bx - cx) * mm_per_px
        y_mm_raw = (cy - by) * mm_per_px

        current_time = frame_ts if frame_ts > 0 else time.perf_counter()

        # ---- Velocity measurement ----
        dt = 0.0
        vx_raw_meas = 0.0
        vy_raw_meas = 0.0
        raw_speed_mm_s = 0.0
        if self.prev_time is not None and self.prev_ball_raw_mm is not None:
            dt = current_time - self.prev_time
            if dt > 0:
                vx_raw_meas = (x_mm_raw - self.prev_ball_raw_mm[0]) / dt
                vy_raw_meas = (y_mm_raw - self.prev_ball_raw_mm[1]) / dt
                raw_speed_mm_s = float(np.hypot(vx_raw_meas, vy_raw_meas))
                if self.max_speed_mm_s > 0 and raw_speed_mm_s > self.max_speed_mm_s:
                    return self._miss()

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

        if self.prev_ball_mm is None or self.prev_time is None or dt <= 0:
            vx = 0.0
            vy = 0.0
            self._vx_f = 0.0
            self._vy_f = 0.0
        else:
            alpha = BALL_VEL_FILTER_ALPHA
            self._vx_f = alpha * vx_raw_meas + (1.0 - alpha) * self._vx_f
            self._vy_f = alpha * vy_raw_meas + (1.0 - alpha) * self._vy_f
            vx = self._vx_f
            vy = self._vy_f

        self.prev_ball_raw_mm = (x_mm_raw, y_mm_raw)
        self.prev_ball_mm = (x_mm, y_mm)
        self.prev_time = current_time

        return BallState(x_mm=x_mm, y_mm=y_mm, vx_mm_s=vx, vy_mm_s=vy)

    # =========================
    # Internal: loss handling
    # =========================

    def _miss(self) -> BallState | None:
        """Reset per-track filter state on any detection loss.

        Velocity resets to zero and the position filter re-seeds on the
        next detection — a reacquired ball must never be blended toward
        its stale pre-loss position (that was a live landmine whenever
        TRACKER_POS_FILTER_ENABLED was turned on)."""
        self._vx_f = 0.0
        self._vy_f = 0.0
        self.prev_ball_mm = None
        return None

    # =========================
    # ArUco pipeline
    # =========================

    def _resolve_homography(self, frame: np.ndarray) -> np.ndarray | None:
        """Return the camera->warp homography for this frame, honoring the
        redetect cadence and the bounded stale-hold policy."""
        attempt_detect = (
            self._last_H is None
            or self._aruco_lost  # in loss state: try EVERY frame
            or (self._frame_counter % self.aruco_redetect_every_n == 0)
        )
        if not attempt_detect:
            return self._last_H

        H = self._detect_and_solve(frame)
        if H is not None:
            self._last_H = H
            self._aruco_hold_count = 0
            self._aruco_lost = False
            return H

        # Detection failed: serve the cached H for a bounded number of
        # attempts, then invalidate it — never fabricate positions forever.
        self._aruco_lost = True
        if (
            self._last_H is not None
            and self._aruco_hold_count < self.max_aruco_hold_frames
        ):
            self._aruco_hold_count += 1
            return self._last_H
        self._last_H = None
        return None

    def _detect_and_solve(self, frame: np.ndarray) -> np.ndarray | None:
        """One full marker detection + homography solve; None on failure."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.aruco_detect_scale < 0.99:
            gray_detect = cv2.resize(
                gray, None,
                fx=self.aruco_detect_scale, fy=self.aruco_detect_scale,
                interpolation=cv2.INTER_AREA,
            )
            scale_inv = 1.0 / self.aruco_detect_scale
        else:
            gray_detect = gray
            scale_inv = 1.0

        corners, ids, _ = self.detector.detectMarkers(gray_detect)
        if ids is None:
            gray_eq = cv2.equalizeHist(gray_detect)
            corners, ids, _ = self.detector.detectMarkers(gray_eq)
        if ids is None:
            return None

        ids = ids.flatten()
        marker_centers_px: dict[int, np.ndarray] = {}
        for i, marker_id in enumerate(ids):
            if marker_id in self.CORNER_IDS:
                center_px = self.get_marker_center(corners[i]) * scale_inv
                marker_centers_px[int(marker_id)] = self._filter_marker_center(
                    marker_id, center_px
                )

        if len(marker_centers_px) < 3:
            return None

        if len(marker_centers_px) == 3:
            missing_id = list(
                set(self.CORNER_IDS) - set(marker_centers_px.keys())
            )[0]
            order = self.CORNER_IDS
            idx = order.index(missing_id)
            prev_id = order[(idx - 1) % 4]
            next_id = order[(idx + 1) % 4]
            opposite_id = order[(idx + 2) % 4]
            if not (
                prev_id in marker_centers_px
                and next_id in marker_centers_px
                and opposite_id in marker_centers_px
            ):
                return None
            a = marker_centers_px[prev_id]
            b = marker_centers_px[opposite_id]
            c = marker_centers_px[next_id]
            marker_centers_px[missing_id] = a + c - b

        src_pts = np.array(
            [marker_centers_px[mid] for mid in self.CORNER_IDS],
            dtype=np.float32,
        )
        dst_pts = np.array(
            [self.mm_to_warp_px(*self.aruco_world_mm[mid]) for mid in self.CORNER_IDS],
            dtype=np.float32,
        )
        # Exact 4-point perspective is faster than findHomography
        result: np.ndarray | None = cv2.getPerspectiveTransform(src_pts, dst_pts)
        return result

    def get_marker_center(self, corners: np.ndarray) -> np.ndarray:
        pts = corners[0]
        return np.asarray(np.mean(pts, axis=0))

    def _filter_marker_center(
        self, marker_id: int, center_px: np.ndarray
    ) -> np.ndarray:
        current = np.asarray(center_px, dtype=np.float32)
        prev = self._marker_centers_lp.get(int(marker_id))
        if prev is None:
            filt = current
        else:
            # Reset on sudden jump (camera reopen / platform moved)
            if float(np.hypot(*(current - prev))) > 40.0:
                filt = current
            else:
                alpha = self.aruco_center_filter_alpha
                filt = alpha * prev + (1.0 - alpha) * current
        self._marker_centers_lp[int(marker_id)] = filt
        return filt

    # =========================
    # Ball detection
    # =========================

    def find_ball_center(
        self, mask: np.ndarray
    ) -> tuple[tuple[int, int], int] | None:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._morph_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._morph_kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < self.min_contour_area:
            return None

        perimeter = cv2.arcLength(largest, True)
        if perimeter > 1e-6 and self.min_circularity > 0:
            circularity = float(4.0 * np.pi * area / (perimeter * perimeter))
            if circularity < self.min_circularity:
                return None

        (x, y), radius = cv2.minEnclosingCircle(largest)
        if radius < self.min_radius_px:
            return None

        if self.min_fill_ratio > 0:
            circle_area = float(np.pi * radius * radius)
            if circle_area > 1e-6 and (area / circle_area) < self.min_fill_ratio:
                return None

        return (int(x), int(y)), int(radius)

    # =========================
    # Helpers
    # =========================

    def mm_to_warp_px(self, x_mm: float, y_mm: float) -> list[float]:
        px_per_mm = self.WARP_SIZE_PX / self.PLATFORM_SIZE_MM
        cx = self.WARP_SIZE_PX // 2
        cy = self.WARP_SIZE_PX // 2
        return [cx + x_mm * px_per_mm, cy - y_mm * px_per_mm]
