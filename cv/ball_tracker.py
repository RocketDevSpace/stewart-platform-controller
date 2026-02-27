import cv2
import numpy as np
import time
import threading

from stewart_control.config import (
    CAMERA_ALLOW_FALLBACK_RESOLUTION,
    CAMERA_AUTO_EXPOSURE,
    CAMERA_EXPOSURE,
    CAMERA_FALLBACK_HEIGHT,
    CAMERA_FALLBACK_WIDTH,
    CAMERA_FORCE_BACKEND,
    CAMERA_BUFFER_SIZE,
    CAMERA_HEIGHT,
    CAMERA_TARGET_FPS,
    CAMERA_WIDTH,
    DEBUG_LEVEL,
    LOG_EVERY_N,
    TRACKER_ARUCO_DETECT_SCALE,
    TRACKER_ARUCO_REDETECT_EVERY_N,
    TRACKER_MAX_SPEED_MM_S,
    TRACKER_MIN_CIRCULARITY,
    TRACKER_MIN_CONTOUR_AREA,
    TRACKER_MIN_FILL_RATIO,
    TRACKER_MIN_RADIUS_PX,
    TRACKER_MAX_ARUCO_HOLD_FRAMES,
    TRACKER_WARP_SIZE_PX,
)


class BallTracker:
    def __init__(
        self,
        camera_index=0,
        platform_size_mm=240.0,
        warp_size_px=TRACKER_WARP_SIZE_PX,
        vel_alpha=0.7,
        pd_kp=0.005,
        pd_kd=0.010,
        aruco_detect_scale=TRACKER_ARUCO_DETECT_SCALE,
        aruco_redetect_every_n=TRACKER_ARUCO_REDETECT_EVERY_N,
        min_radius_px=TRACKER_MIN_RADIUS_PX,
        min_contour_area=TRACKER_MIN_CONTOUR_AREA,
        max_speed_mm_s=TRACKER_MAX_SPEED_MM_S,
        min_circularity=TRACKER_MIN_CIRCULARITY,
        min_fill_ratio=TRACKER_MIN_FILL_RATIO,
        debug_level=DEBUG_LEVEL,
        log_every_n=LOG_EVERY_N,
    ):
        self.PLATFORM_SIZE_MM = platform_size_mm
        self.WARP_SIZE_PX = warp_size_px
        self.VEL_ALPHA = vel_alpha

        self.VEL_ARROW_SCALE = 0.15
        self.POS_ARROW_SCALE = 0.20
        self.PD_ARROW_SCALE = 40.0

        self.pd_kp = float(pd_kp)
        self.pd_kd = float(pd_kd)
        self.min_radius_px = float(min_radius_px)
        self.min_contour_area = float(min_contour_area)
        self.max_speed_mm_s = float(max_speed_mm_s)
        self.min_circularity = float(min_circularity)
        self.min_fill_ratio = float(min_fill_ratio)
        self.aruco_detect_scale = float(aruco_detect_scale)
        self.aruco_redetect_every_n = max(1, int(aruco_redetect_every_n))
        self.debug_level = debug_level
        self.log_every_n = max(1, int(log_every_n))
        self._log_counter = 0
        self._frame_counter = 0
        self._last_H = None
        self._aruco_hold_count = 0
        self._aruco_fail_streak = 0

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        # Keep detector close to OpenCV defaults for robustness across lighting/camera modes.
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.CORNER_IDS = [0, 1, 2, 3]
        self.aruco_world_mm = {
            0: np.array([60.0, 60.0]),
            1: np.array([-60.0, 60.0]),
            2: np.array([-60.0, -60.0]),
            3: np.array([60.0, -60.0]),
        }

        self.cap = self._open_best_camera(camera_index)
        if self.cap is None or not self.cap.isOpened():
            raise RuntimeError("Camera failed to open.")
        self.camera_backend = getattr(self, "camera_backend", "unknown")
        self.camera_measured_period_ms = getattr(self, "camera_measured_period_ms", 0.0)
        self.camera_mode = getattr(self, "camera_mode", f"{CAMERA_WIDTH}x{CAMERA_HEIGHT}")

        self._capture_running = True
        self._capture_lock = threading.Lock()
        self._latest_frame = None
        self._latest_frame_ts = 0.0
        self._capture_period_ms = 0.0
        self._prev_capture_ts = 0.0
        self._slow_capture_streak = 0
        self._runtime_speed_profile_applied = False
        self._last_runtime_profile_apply_ts = 0.0
        self._runtime_profile_attempts = 0
        self.camera_index = int(camera_index)
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()
        self._last_processed_frame_ts = 0.0

        self.prev_ball_mm = None
        self.prev_time = None
        self.vx_smooth = 0.0
        self.vy_smooth = 0.0
        self.hsv_lower = np.array([10, 83, 125], dtype=np.uint8)
        self.hsv_upper = np.array([28, 255, 255], dtype=np.uint8)
        self.last_status_reason = "init"
        self.last_profile_ms = {
            "frame_age": 0.0,
            "capture_period": 0.0,
        }

    @staticmethod
    def _backend_name(backend):
        if backend == cv2.CAP_DSHOW:
            return "DSHOW"
        if backend == cv2.CAP_MSMF:
            return "MSMF"
        if backend == cv2.CAP_ANY:
            return "ANY"
        return str(backend)

    def _configure_camera(self, cap):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, CAMERA_BUFFER_SIZE)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FPS, CAMERA_TARGET_FPS)
        try:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75 if CAMERA_AUTO_EXPOSURE else 0.25)
        except Exception:
            pass
        if not CAMERA_AUTO_EXPOSURE:
            try:
                cap.set(cv2.CAP_PROP_EXPOSURE, CAMERA_EXPOSURE)
            except Exception:
                pass
        try:
            cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        except Exception:
            pass

    def _configure_camera_fallback(self, cap):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_FALLBACK_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_FALLBACK_HEIGHT)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, CAMERA_BUFFER_SIZE)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FPS, CAMERA_TARGET_FPS)

    def _measure_camera_stats(self, cap, samples=18):
        t_prev = None
        periods = []
        gray_levels = []
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
        return float(sum(periods) / len(periods)), gray_mean

    def _open_best_camera(self, camera_index):
        if CAMERA_FORCE_BACKEND.upper() == "DSHOW":
            backends = [cv2.CAP_DSHOW]
        elif CAMERA_FORCE_BACKEND.upper() == "MSMF":
            backends = [cv2.CAP_MSMF]
        elif CAMERA_FORCE_BACKEND.upper() == "ANY":
            backends = [cv2.CAP_ANY]
        else:
            backends = [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY]
        best_cap = None
        best_period = 1e9
        best_backend = None
        best_mode = f"{CAMERA_WIDTH}x{CAMERA_HEIGHT}"
        best_gray = 0.0

        for backend in backends:
            cap = cv2.VideoCapture(camera_index, backend)
            if not cap.isOpened():
                cap.release()
                continue
            self._configure_camera(cap)
            period, gray_mean = self._measure_camera_stats(cap)
            mode = f"{CAMERA_WIDTH}x{CAMERA_HEIGHT}"

            # If period is too slow, try a small manual-exposure sweep and keep
            # the fastest setting that is still bright enough for detection.
            # This recovery runs even when auto-exposure is configured, because
            # some webcams drop to ~10 FPS under auto exposure in dim scenes.
            if period > 45.0:
                tuned = []
                for exp in (-4.0, -5.0, -6.0):
                    try:
                        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
                        cap.set(cv2.CAP_PROP_EXPOSURE, exp)
                    except Exception:
                        pass
                    p_try, g_try = self._measure_camera_stats(cap, samples=10)
                    tuned.append((p_try, g_try, exp))
                if tuned:
                    bright_candidates = [t for t in tuned if t[1] >= 25.0]
                    if bright_candidates:
                        p_best, g_best, exp_best = min(bright_candidates, key=lambda t: t[0])
                    else:
                        p_best, g_best, exp_best = min(tuned, key=lambda t: t[0])
                    period, gray_mean = p_best, g_best
                    try:
                        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
                        cap.set(cv2.CAP_PROP_EXPOSURE, exp_best)
                    except Exception:
                        pass

            # Keep image-quality-first defaults; only downshift resolution if explicitly enabled.
            if CAMERA_ALLOW_FALLBACK_RESOLUTION and period > 45.0:
                self._configure_camera_fallback(cap)
                period_fallback, gray_fallback = self._measure_camera_stats(cap)
                if period_fallback < period:
                    period = period_fallback
                    gray_mean = gray_fallback
                    mode = f"{CAMERA_FALLBACK_WIDTH}x{CAMERA_FALLBACK_HEIGHT}"
            if period < best_period:
                if best_cap is not None:
                    best_cap.release()
                best_cap = cap
                best_period = period
                best_backend = backend
                best_mode = mode
                best_gray = gray_mean
            else:
                cap.release()

        if best_cap is not None:
            self.camera_backend = self._backend_name(best_backend)
            self.camera_measured_period_ms = float(best_period)
            self.camera_mode = best_mode
            self.camera_gray_mean = float(best_gray)
        if best_cap is not None and self.debug_level >= 1:
            print(
                f"[CAMERA] Selected backend={self.camera_backend} "
                f"mode={self.camera_mode} period~{best_period:.1f}ms"
            )
            try:
                print(
                    "[CAMERA] props "
                    f"fps={best_cap.get(cv2.CAP_PROP_FPS):.1f} "
                    f"w={best_cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f} "
                    f"h={best_cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f} "
                    f"exp={best_cap.get(cv2.CAP_PROP_EXPOSURE):.3f} "
                    f"gray={getattr(self, 'camera_gray_mean', 0.0):.1f}"
                )
            except Exception:
                pass
        return best_cap

    def _capture_loop(self):
        while self._capture_running:
            ret, frame = self.cap.read()
            if not ret:
                self._slow_capture_streak += 1
                if self._slow_capture_streak >= 12:
                    self._apply_runtime_speed_profile()
                    self._slow_capture_streak = 0
                time.sleep(0.001)
                continue
            now = time.perf_counter()
            if self._prev_capture_ts > 0:
                dt_ms = (now - self._prev_capture_ts) * 1000.0
                slow_now = (dt_ms > 45.0) or (self._capture_period_ms > 45.0 and self._capture_period_ms > 0.0)
                if slow_now:
                    self._slow_capture_streak += 1
                else:
                    self._slow_capture_streak = max(0, self._slow_capture_streak - 1)
                if self._slow_capture_streak >= 10:
                    self._apply_runtime_speed_profile()
                    self._slow_capture_streak = 0
            with self._capture_lock:
                self._latest_frame = frame
                if self._prev_capture_ts > 0:
                    self._capture_period_ms = 0.9 * self._capture_period_ms + 0.1 * dt_ms if self._capture_period_ms > 0 else dt_ms
                self._prev_capture_ts = now
                self._latest_frame_ts = now

    def _apply_runtime_speed_profile(self):
        # Force a stable low-latency mode if runtime capture drifts.
        now = time.perf_counter()
        if (now - self._last_runtime_profile_apply_ts) < 1.5:
            return
        self._last_runtime_profile_apply_ts = now
        self._runtime_profile_attempts += 1
        try:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            self.cap.set(cv2.CAP_PROP_FPS, max(30, CAMERA_TARGET_FPS))
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, CAMERA_BUFFER_SIZE)
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            self.cap.set(cv2.CAP_PROP_EXPOSURE, max(float(CAMERA_EXPOSURE), -4.0))
            # Let camera settle before measuring.
            for _ in range(5):
                self.cap.read()
            p_ms, g_mean = self._measure_camera_stats(self.cap, samples=10)
            self._runtime_speed_profile_applied = True
            if self.debug_level >= 1:
                print(
                    "[CAMERA] Runtime speed profile enabled "
                    f"(fps_target={max(30, CAMERA_TARGET_FPS)}, exp={max(float(CAMERA_EXPOSURE), -4.0):.1f}, "
                    f"period~{p_ms:.1f}ms, gray={g_mean:.1f}, tries={self._runtime_profile_attempts})"
                )
            if p_ms > 50.0:
                self._reopen_camera_runtime_profile()
        except Exception:
            pass

    def _reopen_camera_runtime_profile(self):
        backend_order = []
        b = str(getattr(self, "camera_backend", "")).upper()
        if b == "DSHOW":
            backend_order = [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY]
        elif b == "MSMF":
            backend_order = [cv2.CAP_MSMF, cv2.CAP_DSHOW, cv2.CAP_ANY]
        else:
            backend_order = [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY]

        best_cap = None
        best_period = 1e9
        best_backend = None
        best_gray = 0.0
        for backend in backend_order:
            cap = cv2.VideoCapture(self.camera_index, backend)
            if not cap.isOpened():
                cap.release()
                continue
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, CAMERA_BUFFER_SIZE)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FPS, max(30, CAMERA_TARGET_FPS))
            try:
                cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
                cap.set(cv2.CAP_PROP_EXPOSURE, max(float(CAMERA_EXPOSURE), -4.0))
            except Exception:
                pass
            p_ms, g_mean = self._measure_camera_stats(cap, samples=10)
            if p_ms < best_period:
                if best_cap is not None:
                    best_cap.release()
                best_cap = cap
                best_period = p_ms
                best_backend = backend
                best_gray = g_mean
            else:
                cap.release()

        if best_cap is not None:
            old = self.cap
            self.cap = best_cap
            self.camera_backend = self._backend_name(best_backend)
            self.camera_measured_period_ms = float(best_period)
            self.camera_gray_mean = float(best_gray)
            self._capture_period_ms = best_period
            self._prev_capture_ts = 0.0
            try:
                old.release()
            except Exception:
                pass
            if self.debug_level >= 1:
                print(
                    "[CAMERA] Runtime reopen selected "
                    f"backend={self.camera_backend} period~{best_period:.1f}ms gray={best_gray:.1f}"
                )

    def set_pd_gains(self, kp, kd):
        self.pd_kp = float(kp)
        self.pd_kd = float(kd)

    def set_hsv_thresholds(self, hmin, hmax, smin, smax, vmin, vmax):
        self.hsv_lower = np.array([int(hmin), int(smin), int(vmin)], dtype=np.uint8)
        self.hsv_upper = np.array([int(hmax), int(smax), int(vmax)], dtype=np.uint8)

    def mm_to_warp_px(self, x_mm, y_mm):
        px_per_mm = self.WARP_SIZE_PX / self.PLATFORM_SIZE_MM
        cx = self.WARP_SIZE_PX // 2
        cy = self.WARP_SIZE_PX // 2
        px = cx + x_mm * px_per_mm
        py = cy - y_mm * px_per_mm
        return [px, py]

    @staticmethod
    def get_marker_center(corners):
        pts = corners[0]
        return np.mean(pts, axis=0)

    def find_ball_center(self, mask):
        kernel = np.ones((3, 3), np.uint8)
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
        if perimeter <= 1e-6:
            return None
        circularity = float(4.0 * np.pi * area / (perimeter * perimeter))
        if self.min_circularity > 0 and circularity < self.min_circularity:
            return None
        (x, y), radius = cv2.minEnclosingCircle(largest)
        if radius < self.min_radius_px:
            return None
        circle_area = float(np.pi * radius * radius)
        fill_ratio = float(area / circle_area) if circle_area > 1e-6 else 0.0
        if self.min_fill_ratio > 0 and fill_ratio < self.min_fill_ratio:
            return None
        return (int(x), int(y)), int(radius), float(area), circularity, fill_ratio, mask

    def reset_motion_state(self):
        self.prev_ball_mm = None
        self.prev_time = None
        self.vx_smooth = 0.0
        self.vy_smooth = 0.0

    def _draw_vectors(self, warped, bx, by, vx, vy, x_mm, y_mm):
        center_px = (self.WARP_SIZE_PX // 2, self.WARP_SIZE_PX // 2)
        cv2.drawMarker(
            warped, center_px, (0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=16, thickness=2
        )
        cv2.circle(warped, (bx, by), 5, (0, 0, 255), -1)

        vel_end = (int(bx + vx * self.VEL_ARROW_SCALE), int(by - vy * self.VEL_ARROW_SCALE))
        cv2.arrowedLine(warped, (bx, by), vel_end, (255, 0, 0), 2)

        pos_vec_x = -x_mm
        pos_vec_y = -y_mm
        pos_end = (int(bx + pos_vec_x * self.POS_ARROW_SCALE), int(by - pos_vec_y * self.POS_ARROW_SCALE))
        cv2.arrowedLine(warped, (bx, by), pos_end, (0, 255, 255), 2)

        pd_x = self.pd_kp * pos_vec_x + self.pd_kd * (-vx)
        pd_y = self.pd_kp * pos_vec_y + self.pd_kd * (-vy)
        pd_end = (int(bx + pd_x * self.PD_ARROW_SCALE), int(by - pd_y * self.PD_ARROW_SCALE))
        cv2.arrowedLine(warped, (bx, by), pd_end, (0, 255, 0), 2)

        cv2.putText(warped, "Center", (center_px[0] + 8, center_px[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(warped, "Vel", (vel_end[0] + 4, vel_end[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(warped, "Pos->Center", (pos_end[0] + 4, pos_end[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(warped, "PD", (pd_end[0] + 4, pd_end[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)

    def update(self, return_debug_frames=False):
        t0 = time.perf_counter()
        with self._capture_lock:
            frame_ts = self._latest_frame_ts
            latest_frame = self._latest_frame
        if latest_frame is None:
            self.last_status_reason = "no_camera_frame"
            self.last_profile_ms = {
                "frame_age": 0.0,
                "capture_period": self._capture_period_ms,
                "slow_streak": float(self._slow_capture_streak),
                "runtime_profile_applied": 1.0 if self._runtime_speed_profile_applied else 0.0,
                "runtime_profile_tries": float(self._runtime_profile_attempts),
            }
            return None
        if frame_ts > 0 and self._last_processed_frame_ts > 0 and frame_ts <= self._last_processed_frame_ts:
            self.last_status_reason = "stale_frame_repeat"
            self.last_profile_ms = {
                "frame_age": 0.0,
                "capture_period": self._capture_period_ms,
                "slow_streak": float(self._slow_capture_streak),
                "runtime_profile_applied": 1.0 if self._runtime_speed_profile_applied else 0.0,
                "runtime_profile_tries": float(self._runtime_profile_attempts),
            }
            return None
        frame = latest_frame.copy()
        t1 = time.perf_counter()
        frame_age_ms = max(0.0, (t1 - frame_ts) * 1000.0) if frame_ts > 0 else 0.0
        self.last_profile_ms = {
            "frame_age": frame_age_ms,
            "capture_period": self._capture_period_ms,
            "slow_streak": float(self._slow_capture_streak),
            "runtime_profile_applied": 1.0 if self._runtime_speed_profile_applied else 0.0,
            "runtime_profile_tries": float(self._runtime_profile_attempts),
        }

        self._frame_counter += 1
        frame = cv2.flip(frame, 1)
        H = None
        warped = None
        mask = None

        t_aruco0 = time.perf_counter()
        homography_ms = 0.0
        aruco_retry_ms = 0.0
        mean_gray = 0.0
        aruco_id_count = 0
        use_cached_h = (self._last_H is not None) and (self._frame_counter % self.aruco_redetect_every_n != 0)
        if use_cached_h:
            H = self._last_H
            t2 = t1
            self._aruco_hold_count = min(self._aruco_hold_count + 1, TRACKER_MAX_ARUCO_HOLD_FRAMES)
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            mean_gray = float(np.mean(gray))
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
            if ids is None:
                # Recovery pass for low-contrast frames while keeping the fast path cheap.
                t_retry0 = time.perf_counter()
                gray_eq = cv2.equalizeHist(gray_detect)
                corners, ids, _ = self.detector.detectMarkers(gray_eq)
                aruco_retry_ms = (time.perf_counter() - t_retry0) * 1000.0
            if ids is None:
                self._aruco_fail_streak += 1
                if self._last_H is not None and self._aruco_hold_count < TRACKER_MAX_ARUCO_HOLD_FRAMES:
                    H = self._last_H
                    t2 = time.perf_counter()
                    self._aruco_hold_count += 1
                else:
                    self.last_status_reason = f"no_aruco_ids(gray={mean_gray:.1f})"
                    return None
            if ids is not None:
                ids = ids.flatten()
                aruco_id_count = int(ids.size)
                self._aruco_fail_streak = 0
            else:
                ids = np.array([], dtype=np.int32)

            marker_centers_px = {}
            for i, marker_id in enumerate(ids):
                if marker_id in self.CORNER_IDS:
                    marker_centers_px[marker_id] = self.get_marker_center(corners[i]) * scale_inv
            t2 = time.perf_counter()

            if ids.size > 0:
                if len(marker_centers_px) < 3:
                    if self._last_H is not None and self._aruco_hold_count < TRACKER_MAX_ARUCO_HOLD_FRAMES:
                        H = self._last_H
                        self._aruco_hold_count += 1
                    else:
                        self.last_status_reason = "insufficient_aruco_points"
                        return None
                if len(marker_centers_px) == 3:
                    missing_id = list(set(self.CORNER_IDS) - set(marker_centers_px.keys()))[0]
                    order = self.CORNER_IDS
                    idx = order.index(missing_id)
                    prev_id = order[(idx - 1) % 4]
                    next_id = order[(idx + 1) % 4]
                    opposite_id = order[(idx + 2) % 4]
                    if prev_id not in marker_centers_px or next_id not in marker_centers_px or opposite_id not in marker_centers_px:
                        if self._last_H is not None and self._aruco_hold_count < TRACKER_MAX_ARUCO_HOLD_FRAMES:
                            H = self._last_H
                            self._aruco_hold_count += 1
                        else:
                            self.last_status_reason = "aruco_reconstruct_failed"
                            return None
                    A = marker_centers_px[prev_id]
                    B = marker_centers_px[opposite_id]
                    C = marker_centers_px[next_id]
                    marker_centers_px[missing_id] = A + C - B

            if H is None:
                src_pts = []
                dst_pts = []
                for mid in self.CORNER_IDS:
                    src_pts.append(marker_centers_px[mid])
                    x_mm, y_mm = self.aruco_world_mm[mid]
                    dst_pts.append(self.mm_to_warp_px(x_mm, y_mm))
                src_pts = np.array(src_pts, dtype=np.float32)
                dst_pts = np.array(dst_pts, dtype=np.float32)

                t_h0 = time.perf_counter()
                if len(src_pts) == 4:
                    H = cv2.getPerspectiveTransform(src_pts, dst_pts)
                else:
                    H, _ = cv2.findHomography(src_pts, dst_pts)
                t_h1 = time.perf_counter()
                homography_ms = (t_h1 - t_h0) * 1000.0
                if H is None:
                    self.last_status_reason = "homography_failed"
                    return None
                self._last_H = H
                self._aruco_hold_count = 0
        t_aruco1 = time.perf_counter()

        t_w0 = time.perf_counter()
        warped = cv2.warpPerspective(frame, H, (self.WARP_SIZE_PX, self.WARP_SIZE_PX), flags=cv2.INTER_LINEAR)
        t_w1 = time.perf_counter()
        t_m0 = time.perf_counter()
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        t_m1 = time.perf_counter()
        t_c0 = time.perf_counter()
        ball = self.find_ball_center(mask)
        if ball is None:
            self.last_status_reason = "ball_contour_invalid"
            return None
        (bx, by), radius_px, contour_area, circularity, fill_ratio, mask = ball
        t_c1 = time.perf_counter()

        t_k0 = time.perf_counter()
        cx = self.WARP_SIZE_PX // 2
        cy = self.WARP_SIZE_PX // 2
        mm_per_px = self.PLATFORM_SIZE_MM / self.WARP_SIZE_PX
        x_mm = (bx - cx) * mm_per_px
        y_mm = (cy - by) * mm_per_px
        current_time = frame_ts if frame_ts > 0 else time.perf_counter()
        t3 = time.perf_counter()

        if self.prev_ball_mm is None or self.prev_time is None:
            vx = 0.0
            vy = 0.0
            dt = 0.0
        else:
            dt = current_time - self.prev_time
            if dt <= 0:
                vx = 0.0
                vy = 0.0
            else:
                vx_raw = (x_mm - self.prev_ball_mm[0]) / dt
                vy_raw = (y_mm - self.prev_ball_mm[1]) / dt
                speed_raw = np.hypot(vx_raw, vy_raw)
                if self.max_speed_mm_s > 0 and speed_raw > self.max_speed_mm_s:
                    self.last_status_reason = f"velocity_outlier_{speed_raw:.1f}"
                    return None
                self.vx_smooth = self.VEL_ALPHA * self.vx_smooth + (1 - self.VEL_ALPHA) * vx_raw
                self.vy_smooth = self.VEL_ALPHA * self.vy_smooth + (1 - self.VEL_ALPHA) * vy_raw
                vx = self.vx_smooth
                vy = self.vy_smooth

        self.prev_ball_mm = (x_mm, y_mm)
        self.prev_time = current_time
        t_k1 = time.perf_counter()
        t_o0 = time.perf_counter()
        if return_debug_frames:
            self._draw_vectors(warped, bx, by, vx, vy, x_mm, y_mm)
        t_o1 = time.perf_counter()

        self._log_counter += 1
        if self.debug_level >= 2 and (self._log_counter % self.log_every_n == 0):
            print(f"Capture: {(t1 - t0) * 1000:.1f} ms")
            print(f"Aruco: {(t2 - t1) * 1000:.1f} ms")
            print(f"Ball: {(t3 - t2) * 1000:.1f} ms")

        result = {
            "x_mm": x_mm,
            "y_mm": y_mm,
            "vx_mm_s": vx,
            "vy_mm_s": vy,
            "frame_ts": float(frame_ts),
            "profile_ms": {
                "capture_fetch": (t1 - t0) * 1000.0,
                "frame_age": frame_age_ms,
                "capture_period": self._capture_period_ms,
                "slow_streak": float(self._slow_capture_streak),
                "runtime_profile_applied": 1.0 if self._runtime_speed_profile_applied else 0.0,
                "runtime_profile_tries": float(self._runtime_profile_attempts),
                "aruco_detect": (t_aruco1 - t_aruco0) * 1000.0,
                "aruco_retry": aruco_retry_ms,
                "homography": homography_ms,
                "warp": (t_w1 - t_w0) * 1000.0,
                "hsv_mask": (t_m1 - t_m0) * 1000.0,
                "contour": (t_c1 - t_c0) * 1000.0,
                "kinematics": (t_k1 - t_k0) * 1000.0,
                "overlay": (t_o1 - t_o0) * 1000.0,
                "total": (time.perf_counter() - t0) * 1000.0,
                "used_cached_h": 1.0 if use_cached_h else 0.0,
            },
            "quality": {
                "radius_px": float(radius_px),
                "contour_area": contour_area,
                "circularity": circularity,
                "fill_ratio": fill_ratio,
                "dt_s": float(dt),
                "gray_mean": mean_gray,
                "aruco_ids": float(aruco_id_count),
            },
        }
        if frame_ts > 0:
            self._last_processed_frame_ts = frame_ts
        self.last_status_reason = "ok"
        if return_debug_frames:
            result["camera_bgr"] = frame
            result["warped_bgr"] = warped
            result["mask_gray"] = mask
        return result

    def release(self):
        self._capture_running = False
        if self._capture_thread and self._capture_thread.is_alive():
            self._capture_thread.join(timeout=0.5)
        if self.cap:
            self.cap.release()
