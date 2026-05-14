import cv2
import numpy as np
import time
import threading

from core.platform_state import BallState
from settings import (
    CAMERA_AUTO_EXPOSURE,
    CAMERA_BUFFER_SIZE,
    CAMERA_EXPOSURE,
    CAMERA_FORCE_BACKEND,
    CAMERA_HEIGHT,
    CAMERA_WIDTH,
    CAMERA_RUNTIME_ADAPTIVE,
    CAMERA_RUNTIME_CHECK_S,
    CAMERA_RUNTIME_MAX_PERIOD_MS,
    CAMERA_RUNTIME_MIN_GRAY,
    CAMERA_RUNTIME_SOFT_GAIN_MAX,
    CAMERA_RUNTIME_TARGET_GRAY,
    CAMERA_RUNTIME_WARMUP_S,
    CAMERA_TARGET_FPS,
    DEBUG_PRINTS,
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

    # =========================
    # INIT
    # =========================

    def __init__(self,
                 camera_index=0,
                 platform_size_mm=240.0,
                 warp_size_px=800,
                 aruco_size_mm=37.5,
                 vel_alpha=0.7,
                 show_debug=True):

        self.PLATFORM_SIZE_MM = platform_size_mm
        self.WARP_SIZE_PX = warp_size_px
        self.ARUCO_SIZE_MM = aruco_size_mm
        self.VEL_ALPHA = vel_alpha
        self.VEL_ARROW_SCALE = 0.15
        self.show_debug = show_debug
        self.camera_index = int(camera_index)

        # --- ArUco setup ---
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.CORNER_IDS = [0, 1, 2, 3]

        self.aruco_world_mm = {
            0: np.array([60.0, 60.0]),
            1: np.array([-60.0, 60.0]),
            2: np.array([-60.0, -60.0]),
            3: np.array([60.0, -60.0])
        }

        # --- ArUco tracking state (FG-6) ---
        self._last_H = None
        self._aruco_hold_count = 0
        self._aruco_fail_streak = 0
        self._marker_centers_lp = {}
        self._frame_counter = 0
        self.aruco_detect_scale = float(TRACKER_ARUCO_DETECT_SCALE)
        self.aruco_redetect_every_n = max(1, int(TRACKER_ARUCO_REDETECT_EVERY_N))
        self.aruco_center_filter_alpha = float(np.clip(TRACKER_ARUCO_CENTER_FILTER_ALPHA, 0.0, 0.98))

        # --- Ball detection params (FG-7) ---
        self.min_radius_px = float(TRACKER_MIN_RADIUS_PX)
        self.min_contour_area = float(TRACKER_MIN_CONTOUR_AREA)
        self.min_circularity = float(TRACKER_MIN_CIRCULARITY)
        self.min_fill_ratio = float(TRACKER_MIN_FILL_RATIO)
        self.max_speed_mm_s = float(TRACKER_MAX_SPEED_MM_S)

        # --- Position filter state (FG-8) ---
        self.pos_filter_enabled = bool(TRACKER_POS_FILTER_ENABLED)
        self.pos_filter_alpha_slow = float(np.clip(TRACKER_POS_FILTER_ALPHA_SLOW, 0.0, 0.98))
        self.pos_filter_alpha_fast = float(np.clip(TRACKER_POS_FILTER_ALPHA_FAST, 0.0, 0.98))
        self.pos_filter_speed_mm_s = max(1.0, float(TRACKER_POS_FILTER_SPEED_MM_S))
        self.pos_filter_max_lag_mm = max(0.1, float(TRACKER_POS_FILTER_MAX_LAG_MM))
        self._last_pos_filter_alpha = self.pos_filter_alpha_fast

        # --- Runtime policy state (FG-4) ---
        self._software_brightness_gain = 1.0
        self._runtime_policy_state = "bootstrap_auto"
        self._runtime_manual_exp = None
        self._runtime_speed_profile_applied = False
        self._runtime_profile_attempts = 0
        self._runtime_recovery_disabled = False
        self._last_runtime_reopen_ts = 0.0
        self._last_runtime_policy_ts = 0.0
        self._last_runtime_mode_change_ts = 0.0
        self._capture_start_ts = time.perf_counter()

        # --- Camera (FG-2, FG-3) ---
        self.cap = self._open_best_camera(camera_index)
        if self.cap is None or not self.cap.isOpened():
            raise RuntimeError("Camera failed to open.")

        # --- Background capture thread (FG-1) ---
        self._capture_running = True
        self._capture_lock = threading.Lock()
        # Double-buffer: capture thread writes into _buf_write_slot; worker reads
        # from the other slot.  No full-frame copy needed on the read path (Opt 3).
        self._frame_bufs = [
            np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8),
            np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8),
        ]
        self._buf_write_slot: int = 0
        self._buf_ready: bool = False
        self._latest_frame_ts = 0.0
        self._last_processed_frame_ts = 0.0
        self._capture_period_ms = 0.0
        self._capture_gray_mean = 0.0
        self._prev_capture_ts = 0.0
        self._slow_capture_streak = 0
        # Latest processed frames — read by worker to populate snapshot
        self._last_camera_bgr = None
        self._last_warped_bgr = None
        self._last_mask_gray = None
        self._capture_fail_streak: int = 0
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

        # --- Warp brightness cache (Opt 4) ---
        self._warp_gray_cache_counter: int = TRACKER_WARP_GRAY_CACHE_N
        self._cached_warp_gray_mean: float = 0.0

        # --- Velocity and position state ---
        self.prev_ball_mm = None
        self.prev_ball_raw_mm = None
        self.prev_time = None
        self.vx_smooth = 0.0
        self.vy_smooth = 0.0

        # --- HSV state ---
        self.hsv_lower = np.array([10, 83, 125], dtype=np.uint8)
        self.hsv_upper = np.array([28, 255, 255], dtype=np.uint8)

        # --- HSV window (optional) ---
        if self.show_debug:
            self._init_hsv_controls()

    # =========================
    # CAMERA HELPERS (FG-2, FG-3)
    # =========================

    @staticmethod
    def _backend_name(backend):
        if backend == cv2.CAP_DSHOW:
            return "DSHOW"
        if backend == cv2.CAP_MSMF:
            return "MSMF"
        if backend == cv2.CAP_ANY:
            return "ANY"
        return str(backend)

    @staticmethod
    def _apply_manual_exposure(cap, exposure_value, settle_reads=0):
        try:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        except Exception:
            pass
        try:
            cap.set(cv2.CAP_PROP_EXPOSURE, float(exposure_value))
            cap.set(cv2.CAP_PROP_EXPOSURE, float(exposure_value))
        except Exception:
            pass
        for _ in range(max(0, int(settle_reads))):
            try:
                cap.read()
            except Exception:
                break

    def _configure_camera(self, cap, force_auto=False):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, CAMERA_BUFFER_SIZE)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FPS, CAMERA_TARGET_FPS)
        use_auto = bool(CAMERA_AUTO_EXPOSURE) or bool(force_auto)
        try:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75 if use_auto else 0.25)
        except Exception:
            pass
        if not use_auto:
            self._apply_manual_exposure(cap, float(CAMERA_EXPOSURE), settle_reads=2)
        try:
            cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        except Exception:
            pass

    def _measure_camera_stats(self, cap, samples=8, warmup=0):
        t_prev = None
        periods = []
        gray_levels = []
        for _ in range(max(0, int(warmup))):
            try:
                cap.read()
            except Exception:
                pass
        for _ in range(samples):
            ret, frame = cap.read()
            if not ret:
                continue
            t_now = time.perf_counter()
            if t_prev is not None:
                periods.append((t_now - t_prev) * 1000.0)
            t_prev = t_now
            try:
                gray_levels.append(float(np.mean(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))))
            except Exception:
                pass
        if not periods:
            return 1e9, 0.0
        gray_mean = float(sum(gray_levels) / len(gray_levels)) if gray_levels else 0.0
        period_ms = float(np.median(np.array(periods, dtype=float)))
        return period_ms, gray_mean

    def _open_best_camera(self, camera_index):
        forced = CAMERA_FORCE_BACKEND.upper()
        if forced == "DSHOW":
            backends = [cv2.CAP_DSHOW]
        elif forced == "MSMF":
            backends = [cv2.CAP_MSMF]
        elif forced == "ANY":
            backends = [cv2.CAP_ANY]
        else:
            backends = [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY]

        best_cap = None
        best_period = 1e9
        best_backend = None

        for backend in backends:
            cap = cv2.VideoCapture(camera_index, backend)
            if not cap.isOpened():
                cap.release()
                continue
            self._configure_camera(cap, force_auto=True)
            period, gray_mean = self._measure_camera_stats(cap, samples=8, warmup=2)
            if DEBUG_PRINTS:
                print(
                    f"[CAMERA] backend={self._backend_name(backend)} "
                    f"period~{period:.1f}ms gray={gray_mean:.1f}"
                )
            if period < best_period:
                if best_cap is not None:
                    best_cap.release()
                best_cap = cap
                best_period = period
                best_backend = backend
            else:
                cap.release()

        if best_cap is not None:
            self.camera_backend = self._backend_name(best_backend)
            self.camera_measured_period_ms = float(best_period)
            self._last_runtime_mode_change_ts = time.perf_counter()
            if DEBUG_PRINTS:
                print(
                    f"[CAMERA] Selected backend={self.camera_backend} "
                    f"period~{best_period:.1f}ms"
                )
        return best_cap

    # =========================
    # RUNTIME ADAPTIVE EXPOSURE (FG-4)
    # =========================

    def _probe_manual_exposure_candidates(self, candidates):
        probe_results = []
        for exp in candidates:
            self._apply_manual_exposure(self.cap, float(exp), settle_reads=2)
            p_ms, g_mean = self._measure_camera_stats(self.cap, samples=4, warmup=1)
            probe_results.append((float(p_ms), float(g_mean), float(exp)))

        if not probe_results:
            return None

        max_period = float(CAMERA_RUNTIME_MAX_PERIOD_MS)
        min_gray = float(CAMERA_RUNTIME_MIN_GRAY)
        acceptable = [r for r in probe_results if r[0] <= (max_period + 2.0) and r[1] >= min_gray]
        if acceptable:
            fastest = min(r[0] for r in acceptable)
            near_fast = [r for r in acceptable if r[0] <= (fastest + 6.0)]
            best = max(near_fast, key=lambda r: r[1])
        else:
            bright_enough = [r for r in probe_results if r[1] >= max(15.0, min_gray - 10.0)]
            if bright_enough:
                best = min(bright_enough, key=lambda r: r[0])
            else:
                best = min(probe_results, key=lambda r: r[0])

        self._apply_manual_exposure(self.cap, best[2], settle_reads=2)
        self._runtime_manual_exp = best[2]
        self._runtime_policy_state = "manual"
        self._last_runtime_mode_change_ts = time.perf_counter()
        return best

    def _evaluate_runtime_camera_policy(self, now):
        if not CAMERA_RUNTIME_ADAPTIVE or self._runtime_recovery_disabled:
            return
        if (now - self._capture_start_ts) < float(CAMERA_RUNTIME_WARMUP_S):
            return
        if (now - self._last_runtime_policy_ts) < float(CAMERA_RUNTIME_CHECK_S):
            return
        self._last_runtime_policy_ts = now
        self._runtime_profile_attempts += 1

        period_ms = float(self._capture_period_ms) if self._capture_period_ms > 0 else float(getattr(self, "camera_measured_period_ms", 1e9))
        gray_mean = float(self._capture_gray_mean) if self._capture_gray_mean > 0 else 0.0
        max_period = float(CAMERA_RUNTIME_MAX_PERIOD_MS)
        min_gray = float(CAMERA_RUNTIME_MIN_GRAY)

        if period_ms > max_period + 2.0 and (now - self._last_runtime_mode_change_ts) > 2.5:
            candidates = list({float(CAMERA_EXPOSURE), -4.0, -5.0, -6.0})
            if self._runtime_manual_exp is not None:
                candidates = [self._runtime_manual_exp] + [c for c in candidates if c != self._runtime_manual_exp]
            best = self._probe_manual_exposure_candidates(candidates)
            if best is not None:
                self._runtime_speed_profile_applied = True
                if DEBUG_PRINTS:
                    print(f"[CAMERA] runtime: manual exp={best[2]:.2f} period~{best[0]:.1f}ms")
        elif gray_mean < min_gray and (now - self._last_runtime_mode_change_ts) > 2.5 and self._runtime_policy_state != "manual":
            candidates = list({float(CAMERA_EXPOSURE), -4.0, -3.0, -5.0})
            best = self._probe_manual_exposure_candidates(candidates)
            if best is not None:
                self._runtime_speed_profile_applied = True
                if DEBUG_PRINTS:
                    print(f"[CAMERA] runtime brightened: exp={best[2]:.2f} gray={best[1]:.1f}")

        self._update_software_gain(gray_mean)

    # =========================
    # SOFTWARE GAIN (FG-5)
    # =========================

    def _update_software_gain(self, gray_level):
        target_gray = float(CAMERA_RUNTIME_TARGET_GRAY)
        max_gain = float(CAMERA_RUNTIME_SOFT_GAIN_MAX)
        if gray_level <= 0.5:
            desired_gain = max_gain
        elif gray_level < target_gray:
            desired_gain = min(max_gain, max(1.0, target_gray / gray_level))
        else:
            desired_gain = 1.0
        self._software_brightness_gain = (
            0.85 * self._software_brightness_gain + 0.15 * desired_gain
        )

    def _apply_software_brightness(self, frame):
        gain = float(self._software_brightness_gain)
        if gain <= 1.02:
            return frame
        try:
            return cv2.convertScaleAbs(frame, alpha=gain, beta=0)
        except Exception:
            return frame

    # =========================
    # BACKGROUND CAPTURE (FG-1)
    # =========================

    def _capture_loop(self):
        while self._capture_running:
            ret, frame = self.cap.read()
            if not ret:
                self._capture_fail_streak += 1
                time.sleep(0.001)
                continue
            self._capture_fail_streak = 0
            now = time.perf_counter()
            gray_sample = float(np.mean(frame[:, :, 1]))
            # Re-allocate buffers if camera returns a different resolution
            if frame.shape != self._frame_bufs[0].shape:
                self._frame_bufs = [np.zeros_like(frame), np.zeros_like(frame)]
            # Copy into write slot outside the lock (the expensive pixel copy)
            np.copyto(self._frame_bufs[self._buf_write_slot], frame)
            with self._capture_lock:
                if self._prev_capture_ts > 0:
                    dt_ms = (now - self._prev_capture_ts) * 1000.0
                    if self._capture_period_ms > 0:
                        self._capture_period_ms = 0.9 * self._capture_period_ms + 0.1 * dt_ms
                    else:
                        self._capture_period_ms = dt_ms
                    slow_now = dt_ms > (float(CAMERA_RUNTIME_MAX_PERIOD_MS) * 2.0)
                    if slow_now:
                        self._slow_capture_streak += 1
                    else:
                        self._slow_capture_streak = max(0, self._slow_capture_streak - 1)
                self._capture_gray_mean = (
                    0.92 * self._capture_gray_mean + 0.08 * gray_sample
                    if self._capture_gray_mean > 0
                    else gray_sample
                )
                self._prev_capture_ts = now
                self._buf_write_slot ^= 1
                self._latest_frame_ts = now
                self._buf_ready = True
            self._evaluate_runtime_camera_policy(now)

    # =========================
    # HSV TRACKBARS
    # =========================

    def _init_hsv_controls(self):
        def nothing(x): pass

        cv2.namedWindow("HSV Controls", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("HSV Controls", 500, 300)

        cv2.createTrackbar("H Min", "HSV Controls", int(self.hsv_lower[0]), 179, nothing)
        cv2.createTrackbar("H Max", "HSV Controls", int(self.hsv_upper[0]), 179, nothing)
        cv2.createTrackbar("S Min", "HSV Controls", int(self.hsv_lower[1]), 255, nothing)
        cv2.createTrackbar("S Max", "HSV Controls", int(self.hsv_upper[1]), 255, nothing)
        cv2.createTrackbar("V Min", "HSV Controls", int(self.hsv_lower[2]), 255, nothing)
        cv2.createTrackbar("V Max", "HSV Controls", int(self.hsv_upper[2]), 255, nothing)

    def set_hsv_thresholds(self, hmin, hmax, smin, smax, vmin, vmax):
        self.hsv_lower = np.array([int(hmin), int(smin), int(vmin)], dtype=np.uint8)
        self.hsv_upper = np.array([int(hmax), int(smax), int(vmax)], dtype=np.uint8)

    # =========================
    # ARUCO HELPERS (FG-6)
    # =========================

    def get_marker_center(self, corners):
        pts = corners[0]
        return np.mean(pts, axis=0)

    def _filter_marker_center(self, marker_id, center_px):
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
    # BALL DETECTION (FG-7)
    # =========================

    def find_ball_center(self, mask):
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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
    # HELPERS
    # =========================

    def mm_to_warp_px(self, x_mm, y_mm):
        px_per_mm = self.WARP_SIZE_PX / self.PLATFORM_SIZE_MM
        cx = self.WARP_SIZE_PX // 2
        cy = self.WARP_SIZE_PX // 2

        px = cx + x_mm * px_per_mm
        py = cy - y_mm * px_per_mm
        return [px, py]

    # =========================
    # MAIN UPDATE (NON-BLOCKING)
    # =========================

    def update(self):
        t0 = time.perf_counter()

        # Read from double-buffer (FG-1 + Opt 3): grab slot index under lock,
        # then access pixel data without holding the lock.
        with self._capture_lock:
            if not self._buf_ready:
                return None
            frame_ts = self._latest_frame_ts
            read_slot = 1 - self._buf_write_slot

        # Skip stale frames
        if (
            frame_ts > 0
            and self._last_processed_frame_ts > 0
            and frame_ts <= self._last_processed_frame_ts
        ):
            return None

        frame = self._frame_bufs[read_slot].copy()
        t1 = time.perf_counter()

        frame = cv2.flip(frame, 1)
        frame = self._apply_software_brightness(frame)  # FG-5
        self._last_camera_bgr = frame

        self._frame_counter += 1

        # ---- ArUco detection (FG-6) ----
        H = None
        warped = None
        use_cached_h = (
            self._last_H is not None
            and (self._frame_counter % self.aruco_redetect_every_n != 0)
        )

        if use_cached_h:
            H = self._last_H
            t2 = t1
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Downscale for ArUco detection speed
            if self.aruco_detect_scale < 0.99:
                gray_detect = cv2.resize(
                    gray,
                    None,
                    fx=self.aruco_detect_scale,
                    fy=self.aruco_detect_scale,
                    interpolation=cv2.INTER_AREA,
                )
                scale_inv = 1.0 / self.aruco_detect_scale
            else:
                gray_detect = gray
                scale_inv = 1.0

            corners, ids, _ = self.detector.detectMarkers(gray_detect)

            # Retry with histogram equalization on failure (FG-6)
            if ids is None:
                gray_eq = cv2.equalizeHist(gray_detect)
                corners, ids, _ = self.detector.detectMarkers(gray_eq)

            if ids is None:
                self._aruco_fail_streak += 1
                # Use cached homography for up to TRACKER_MAX_ARUCO_HOLD_FRAMES frames
                if self._last_H is not None and self._aruco_hold_count < TRACKER_MAX_ARUCO_HOLD_FRAMES:
                    H = self._last_H
                    self._aruco_hold_count += 1
                    t2 = time.perf_counter()
                else:
                    self._debug_show(frame, None, None)
                    return None
            else:
                self._aruco_fail_streak = 0
                ids = ids.flatten()

                marker_centers_px = {}
                for i, marker_id in enumerate(ids):
                    if marker_id in self.CORNER_IDS:
                        center_px = self.get_marker_center(corners[i]) * scale_inv
                        # Low-pass filter each marker center (FG-6)
                        marker_centers_px[marker_id] = self._filter_marker_center(marker_id, center_px)

                t2 = time.perf_counter()

                if len(marker_centers_px) < 3:
                    if self._last_H is not None and self._aruco_hold_count < TRACKER_MAX_ARUCO_HOLD_FRAMES:
                        H = self._last_H
                        self._aruco_hold_count += 1
                    else:
                        self._debug_show(frame, None, None)
                        return None
                else:
                    if len(marker_centers_px) == 3:
                        missing_id = list(set(self.CORNER_IDS) - set(marker_centers_px.keys()))[0]
                        order = self.CORNER_IDS
                        idx = order.index(missing_id)
                        prev_id = order[(idx - 1) % 4]
                        next_id = order[(idx + 1) % 4]
                        opposite_id = order[(idx + 2) % 4]
                        if (
                            prev_id in marker_centers_px
                            and next_id in marker_centers_px
                            and opposite_id in marker_centers_px
                        ):
                            A = marker_centers_px[prev_id]
                            B = marker_centers_px[opposite_id]
                            C = marker_centers_px[next_id]
                            marker_centers_px[missing_id] = A + C - B
                        else:
                            if self._last_H is not None and self._aruco_hold_count < TRACKER_MAX_ARUCO_HOLD_FRAMES:
                                H = self._last_H
                                self._aruco_hold_count += 1
                            else:
                                self._debug_show(frame, None, None)
                                return None

                    if H is None:
                        src_pts = np.array(
                            [marker_centers_px[mid] for mid in self.CORNER_IDS],
                            dtype=np.float32,
                        )
                        dst_pts = np.array(
                            [self.mm_to_warp_px(*self.aruco_world_mm[mid]) for mid in self.CORNER_IDS],
                            dtype=np.float32,
                        )
                        # Exact 4-point perspective is faster than findHomography
                        if len(src_pts) == 4:
                            H = cv2.getPerspectiveTransform(src_pts, dst_pts)
                        else:
                            H, _ = cv2.findHomography(src_pts, dst_pts)
                        if H is None:
                            self._debug_show(frame, None, None)
                            return None
                        self._last_H = H
                        self._aruco_hold_count = 0

        warped = cv2.warpPerspective(frame, H, (self.WARP_SIZE_PX, self.WARP_SIZE_PX))
        self._last_warped_bgr = warped

        # ---- Adaptive HSV detection (FG-7) ----
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)

        if self.show_debug:
            hmin = cv2.getTrackbarPos("H Min", "HSV Controls")
            hmax = cv2.getTrackbarPos("H Max", "HSV Controls")
            smin = cv2.getTrackbarPos("S Min", "HSV Controls")
            smax = cv2.getTrackbarPos("S Max", "HSV Controls")
            vmin = cv2.getTrackbarPos("V Min", "HSV Controls")
            vmax = cv2.getTrackbarPos("V Max", "HSV Controls")
            self.hsv_lower = np.array([hmin, smin, vmin], dtype=np.uint8)
            self.hsv_upper = np.array([hmax, smax, vmax], dtype=np.uint8)

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

        # Two-pass retry with further relaxed V-min (FG-7)
        if ball is None and vmin_eff > 20:
            lower_eff2 = self.hsv_lower.copy()
            lower_eff2[2] = max(10, vmin_eff - 20)
            mask = cv2.inRange(hsv, lower_eff2, self.hsv_upper)
            ball = self.find_ball_center(mask)

        if ball is None:
            self._debug_show(frame, warped, mask)
            return None

        (bx, by), radius = ball

        cx = self.WARP_SIZE_PX // 2
        cy = self.WARP_SIZE_PX // 2

        mm_per_px = self.PLATFORM_SIZE_MM / self.WARP_SIZE_PX
        x_mm_raw = (bx - cx) * mm_per_px
        y_mm_raw = (cy - by) * mm_per_px

        current_time = frame_ts if frame_ts > 0 else time.perf_counter()
        t3 = time.perf_counter()

        # ---- Velocity and position filter (FG-8) ----
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

                # Speed outlier rejection (FG-7)
                if self.max_speed_mm_s > 0 and raw_speed_mm_s > self.max_speed_mm_s:
                    self._debug_show(frame, warped, mask)
                    return None

        # Position filter (FG-8)
        if self.pos_filter_enabled and self.prev_ball_mm is not None:
            speed_ratio = min(1.0, raw_speed_mm_s / self.pos_filter_speed_mm_s)
            filter_alpha = (
                self.pos_filter_alpha_slow
                + (self.pos_filter_alpha_fast - self.pos_filter_alpha_slow) * speed_ratio
            )
            x_mm = filter_alpha * self.prev_ball_mm[0] + (1.0 - filter_alpha) * x_mm_raw
            y_mm = filter_alpha * self.prev_ball_mm[1] + (1.0 - filter_alpha) * y_mm_raw
            # Bound phase lag
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
        else:
            self.vx_smooth = self.VEL_ALPHA * self.vx_smooth + (1 - self.VEL_ALPHA) * vx_raw_meas
            self.vy_smooth = self.VEL_ALPHA * self.vy_smooth + (1 - self.VEL_ALPHA) * vy_raw_meas
            vx = self.vx_smooth
            vy = self.vy_smooth

        self.prev_ball_raw_mm = (x_mm_raw, y_mm_raw)
        self.prev_ball_mm = (x_mm, y_mm)
        self.prev_time = current_time
        self._last_processed_frame_ts = frame_ts

        self._debug_show(frame, warped, mask, bx, by, vx, vy)

        if DEBUG_PRINTS:
            print(f"Capture: {(t1-t0)*1000:.1f} ms")
            print(f"Aruco: {(t2-t1)*1000:.1f} ms")
            print(f"Ball: {(t3-t2)*1000:.1f} ms")

        return BallState(x_mm=x_mm, y_mm=y_mm, vx_mm_s=vx, vy_mm_s=vy)

    # =========================
    # DEBUG DISPLAY
    # =========================

    def _debug_show(self, frame, warped, mask, bx=None, by=None, vx=0, vy=0):
        if not self.show_debug:
            return

        cv2.imshow("Camera View", frame)

        if warped is not None:
            if bx is not None:
                cv2.circle(warped, (bx, by), 5, (0, 0, 255), -1)

                arrow_px_x = int(bx + vx * self.VEL_ARROW_SCALE)
                arrow_px_y = int(by - vy * self.VEL_ARROW_SCALE)

                cv2.arrowedLine(warped,
                                (bx, by),
                                (arrow_px_x, arrow_px_y),
                                (255, 0, 0),
                                2)

            cv2.imshow("Warped Platform View", warped)

        if mask is not None:
            cv2.imshow("Ball Mask", mask)

        cv2.waitKey(1)

    # =========================
    # CLEANUP
    # =========================

    def release(self):
        self._capture_running = False
        self._capture_thread.join(timeout=2.0)
        if self.cap:
            self.cap.release()
        if self.show_debug:
            cv2.destroyAllWindows()
