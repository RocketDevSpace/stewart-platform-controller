import cv2
import numpy as np
import time
import threading

from stewart_control.config import (
    BALL_TARGET_DEFAULT_X_MM,
    BALL_TARGET_DEFAULT_Y_MM,
    CAMERA_ALLOW_FALLBACK_RESOLUTION,
    CAMERA_AUTO_EXPOSURE,
    CAMERA_EXPOSURE,
    CAMERA_FALLBACK_HEIGHT,
    CAMERA_FALLBACK_WIDTH,
    CAMERA_FORCE_BACKEND,
    CAMERA_GAIN,
    CAMERA_BUFFER_SIZE,
    CAMERA_HEIGHT,
    CAMERA_RUNTIME_ADAPTIVE,
    CAMERA_RUNTIME_CHECK_S,
    CAMERA_RUNTIME_MAX_PERIOD_MS,
    CAMERA_RUNTIME_MIN_GRAY,
    CAMERA_RUNTIME_SOFT_GAIN_MAX,
    CAMERA_RUNTIME_TARGET_GRAY,
    CAMERA_RUNTIME_WARMUP_S,
    CAMERA_TARGET_FPS,
    CAMERA_WIDTH,
    DEBUG_LEVEL,
    LOG_EVERY_N,
    TRACKER_ARUCO_DETECT_SCALE,
    TRACKER_ARUCO_REDETECT_EVERY_N,
    TRACKER_MAX_SPEED_MM_S,
    TRACKER_POS_FILTER_ALPHA_FAST,
    TRACKER_POS_FILTER_ALPHA_SLOW,
    TRACKER_POS_FILTER_ENABLED,
    TRACKER_POS_FILTER_SPEED_MM_S,
    TRACKER_MIN_CIRCULARITY,
    TRACKER_MIN_CONTOUR_AREA,
    TRACKER_MIN_FILL_RATIO,
    TRACKER_MIN_RADIUS_PX,
    TRACKER_ARUCO_CENTER_FILTER_ALPHA,
    TRACKER_MAX_ARUCO_HOLD_FRAMES,
    TRACKER_WARP_SIZE_PX,
    TRACKER_HSV_H_MAX,
    TRACKER_HSV_H_MIN,
    TRACKER_HSV_S_MAX,
    TRACKER_HSV_S_MIN,
    TRACKER_HSV_V_MAX,
    TRACKER_HSV_V_MIN,
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
        aruco_center_filter_alpha=TRACKER_ARUCO_CENTER_FILTER_ALPHA,
        pos_filter_enabled=TRACKER_POS_FILTER_ENABLED,
        pos_filter_alpha_slow=TRACKER_POS_FILTER_ALPHA_SLOW,
        pos_filter_alpha_fast=TRACKER_POS_FILTER_ALPHA_FAST,
        pos_filter_speed_mm_s=TRACKER_POS_FILTER_SPEED_MM_S,
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
        self.aruco_center_filter_alpha = float(np.clip(aruco_center_filter_alpha, 0.0, 0.98))
        self.pos_filter_enabled = bool(pos_filter_enabled)
        self.pos_filter_alpha_slow = float(np.clip(pos_filter_alpha_slow, 0.0, 0.98))
        self.pos_filter_alpha_fast = float(np.clip(pos_filter_alpha_fast, 0.0, 0.98))
        self.pos_filter_speed_mm_s = max(1.0, float(pos_filter_speed_mm_s))
        self.debug_level = debug_level
        self.log_every_n = max(1, int(log_every_n))
        self._log_counter = 0
        self._frame_counter = 0
        self._last_H = None
        self._aruco_hold_count = 0
        self._aruco_fail_streak = 0
        self._marker_centers_lp = {}

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
        self._capture_gray_mean = 0.0
        self._prev_capture_ts = 0.0
        self._slow_capture_streak = 0
        self._capture_fail_streak = 0
        self._runtime_speed_profile_applied = False
        self._last_runtime_profile_apply_ts = 0.0
        self._runtime_profile_attempts = 0
        self._runtime_recovery_disabled = False
        self._last_runtime_reopen_ts = 0.0
        self._capture_start_ts = time.perf_counter()
        self._last_runtime_policy_ts = 0.0
        self._last_runtime_mode_change_ts = 0.0
        self._runtime_policy_state = "bootstrap_auto"
        self._runtime_manual_exp = None
        self._software_brightness_gain = 1.0
        self.camera_index = int(camera_index)
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()
        self._last_processed_frame_ts = 0.0

        self.prev_ball_mm = None
        self.prev_ball_raw_mm = None
        self.prev_time = None
        self.vx_smooth = 0.0
        self.vy_smooth = 0.0
        self._last_pos_filter_alpha = self.pos_filter_alpha_fast
        self.hsv_lower = np.array([TRACKER_HSV_H_MIN, TRACKER_HSV_S_MIN, TRACKER_HSV_V_MIN], dtype=np.uint8)
        self.hsv_upper = np.array([TRACKER_HSV_H_MAX, TRACKER_HSV_S_MAX, TRACKER_HSV_V_MAX], dtype=np.uint8)
        self.target_x_mm = float(BALL_TARGET_DEFAULT_X_MM)
        self.target_y_mm = float(BALL_TARGET_DEFAULT_Y_MM)
        self.last_status_reason = "init"
        self.last_profile_ms = {
            "frame_age": 0.0,
            "capture_period": 0.0,
            "capture_gray": 0.0,
            "sw_gain": 1.0,
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

    @staticmethod
    def _safe_cam_get(cap, prop, default=float("nan")):
        try:
            return float(cap.get(prop))
        except Exception:
            return float(default)

    @staticmethod
    def _fourcc_to_str(value):
        try:
            iv = int(value)
            return "".join([chr((iv >> (8 * i)) & 0xFF) for i in range(4)])
        except Exception:
            return "????"

    def _camera_props_snapshot(self, cap):
        fps = self._safe_cam_get(cap, cv2.CAP_PROP_FPS)
        w = self._safe_cam_get(cap, cv2.CAP_PROP_FRAME_WIDTH)
        h = self._safe_cam_get(cap, cv2.CAP_PROP_FRAME_HEIGHT)
        auto_exp = self._safe_cam_get(cap, cv2.CAP_PROP_AUTO_EXPOSURE)
        exp = self._safe_cam_get(cap, cv2.CAP_PROP_EXPOSURE)
        gain = self._safe_cam_get(cap, cv2.CAP_PROP_GAIN)
        fourcc = self._safe_cam_get(cap, cv2.CAP_PROP_FOURCC)
        return {
            "fps": fps,
            "w": w,
            "h": h,
            "auto_exp": auto_exp,
            "exp": exp,
            "gain": gain,
            "fourcc": fourcc,
        }

    def _log_camera_probe(self, label, backend_name, props, period_ms=None, gray_mean=None, details=None, extra=""):
        msg = (
            f"[CAMERA][{label}] backend={backend_name} "
            f"fps={props['fps']:.1f} w={props['w']:.0f} h={props['h']:.0f} "
            f"auto_exp={props['auto_exp']:.2f} exp={props['exp']:.3f} "
            f"gain={props['gain']:.2f} fourcc={self._fourcc_to_str(props['fourcc'])}"
        )
        if period_ms is not None:
            msg += f" period={float(period_ms):.1f}ms"
        if gray_mean is not None:
            msg += f" gray={float(gray_mean):.1f}"
        if details:
            msg += (
                f" n={int(details.get('n', 0))}"
                f" p10={float(details.get('p10', 0.0)):.1f}ms"
                f" p90={float(details.get('p90', 0.0)):.1f}ms"
                f" min={float(details.get('min', 0.0)):.1f}ms"
                f" max={float(details.get('max', 0.0)):.1f}ms"
            )
        if extra:
            msg += f" {extra}"
        print(msg)

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
            cap.set(cv2.CAP_PROP_GAIN, float(CAMERA_GAIN))
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

    @staticmethod
    def _apply_manual_exposure(cap, exposure_value: float, settle_reads: int = 0):
        auto_set_ok = False
        exp_set_ok_1 = False
        exp_set_ok_2 = False
        try:
            auto_set_ok = bool(cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25))
        except Exception:
            pass
        try:
            exp_set_ok_1 = bool(cap.set(cv2.CAP_PROP_EXPOSURE, float(exposure_value)))
            exp_set_ok_2 = bool(cap.set(cv2.CAP_PROP_EXPOSURE, float(exposure_value)))
        except Exception:
            pass
        for _ in range(max(0, int(settle_reads))):
            try:
                cap.read()
            except Exception:
                break
        return {
            "auto_set_ok": auto_set_ok,
            "exp_set_ok_1": exp_set_ok_1,
            "exp_set_ok_2": exp_set_ok_2,
        }

    def _measure_camera_stats(self, cap, samples=8, warmup=0, with_details=False):
        t_prev = None
        periods = []
        gray_levels = []
        ok_frames = 0
        for _ in range(max(0, int(warmup))):
            try:
                cap.read()
            except Exception:
                pass
        for _ in range(samples):
            ret, frame = cap.read()
            if not ret:
                continue
            ok_frames += 1
            t_now = time.perf_counter()
            if t_prev is not None:
                periods.append((t_now - t_prev) * 1000.0)
            t_prev = t_now
            try:
                gray_levels.append(float(np.mean(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))))
            except Exception:
                pass
        if not periods:
            if with_details:
                return 1e9, 0.0, {"n": 0, "p10": 0.0, "p90": 0.0, "min": 0.0, "max": 0.0, "ok_frames": ok_frames}
            return 1e9, 0.0
        gray_mean = float(sum(gray_levels) / len(gray_levels)) if gray_levels else 0.0
        # Median is more robust to a dropped frame right after a property change.
        period_arr = np.array(periods, dtype=float)
        period_ms = float(np.median(period_arr))
        if with_details:
            details = {
                "n": len(periods),
                "p10": float(np.percentile(period_arr, 10)),
                "p90": float(np.percentile(period_arr, 90)),
                "min": float(np.min(period_arr)),
                "max": float(np.max(period_arr)),
                "ok_frames": ok_frames,
            }
            return period_ms, gray_mean, details
        return period_ms, gray_mean

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

        if self.debug_level >= 1:
            print(
                "[CAMERA] startup config "
                f"target={CAMERA_WIDTH}x{CAMERA_HEIGHT}@{CAMERA_TARGET_FPS} "
                f"forced_backend={CAMERA_FORCE_BACKEND} "
                f"bootstrap_auto=yes steady_pref_auto={'yes' if CAMERA_AUTO_EXPOSURE else 'no'} "
                f"manual_exp={float(CAMERA_EXPOSURE):.2f} gain={float(CAMERA_GAIN):.2f}"
            )
            print(
                "[CAMERA] runtime adapt "
                f"enabled={'yes' if CAMERA_RUNTIME_ADAPTIVE else 'no'} "
                f"warmup={float(CAMERA_RUNTIME_WARMUP_S):.1f}s "
                f"check={float(CAMERA_RUNTIME_CHECK_S):.1f}s "
                f"max_period={float(CAMERA_RUNTIME_MAX_PERIOD_MS):.1f}ms "
                f"min_gray={float(CAMERA_RUNTIME_MIN_GRAY):.1f}"
            )

        for backend in backends:
            cap = cv2.VideoCapture(camera_index, backend)
            if not cap.isOpened():
                cap.release()
                continue
            backend_name = self._backend_name(backend)
            if self.debug_level >= 1:
                self._log_camera_probe(
                    "open",
                    backend_name,
                    self._camera_props_snapshot(cap),
                )
            # Always bootstrap with auto-exposure first for robust startup.
            self._configure_camera(cap, force_auto=True)
            if self.debug_level >= 1:
                self._log_camera_probe(
                    "configured",
                    backend_name,
                    self._camera_props_snapshot(cap),
                )
            period, gray_mean, details = self._measure_camera_stats(
                cap, samples=8, warmup=2, with_details=True
            )
            mode = f"{CAMERA_WIDTH}x{CAMERA_HEIGHT}"
            if self.debug_level >= 1:
                self._log_camera_probe(
                    "baseline",
                    backend_name,
                    self._camera_props_snapshot(cap),
                    period_ms=period,
                    gray_mean=gray_mean,
                    details=details,
                )

            # Keep image-quality-first defaults; only downshift resolution if explicitly enabled.
            if CAMERA_ALLOW_FALLBACK_RESOLUTION and period > 45.0:
                self._configure_camera_fallback(cap)
                period_fallback, gray_fallback = self._measure_camera_stats(cap, samples=4, warmup=1)
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
            self._runtime_policy_state = "bootstrap_auto"
            self._runtime_manual_exp = None
            self._software_brightness_gain = 1.0
            self._last_runtime_mode_change_ts = time.perf_counter()
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
                    f"auto_exp={best_cap.get(cv2.CAP_PROP_AUTO_EXPOSURE):.2f} "
                    f"exp={best_cap.get(cv2.CAP_PROP_EXPOSURE):.3f} "
                    f"gray={getattr(self, 'camera_gray_mean', 0.0):.1f}"
                )
                if getattr(self, "camera_gray_mean", 0.0) < 18.0:
                    print("[CAMERA] warning: startup image is very dark; HSV detection may fail.")
            except Exception:
                pass
        return best_cap

    def _capture_loop(self):
        while self._capture_running:
            ret, frame = self.cap.read()
            if not ret:
                self._capture_fail_streak += 1
                self._slow_capture_streak = min(self._slow_capture_streak + 1, 1000)
                if (
                    (not self._runtime_recovery_disabled)
                    and self._capture_fail_streak >= 120
                    and (time.perf_counter() - self._last_runtime_reopen_ts) > 8.0
                ):
                    self._last_runtime_reopen_ts = time.perf_counter()
                    self._reopen_camera_runtime_profile()
                    self._capture_fail_streak = 0
                time.sleep(0.001)
                continue
            self._capture_fail_streak = 0
            now = time.perf_counter()
            dt_ms = 0.0
            if self._prev_capture_ts > 0:
                dt_ms = (now - self._prev_capture_ts) * 1000.0
                slow_now = dt_ms > (CAMERA_RUNTIME_MAX_PERIOD_MS * 2.0)
                if slow_now:
                    self._slow_capture_streak += 1
                else:
                    self._slow_capture_streak = max(0, self._slow_capture_streak - 1)
            gray_sample = float(np.mean(frame[:, :, 1]))
            with self._capture_lock:
                self._latest_frame = frame
                if self._prev_capture_ts > 0:
                    self._capture_period_ms = 0.9 * self._capture_period_ms + 0.1 * dt_ms if self._capture_period_ms > 0 else dt_ms
                self._capture_gray_mean = (
                    0.92 * self._capture_gray_mean + 0.08 * gray_sample
                    if self._capture_gray_mean > 0
                    else gray_sample
                )
                self._prev_capture_ts = now
                self._latest_frame_ts = now
            self._evaluate_runtime_camera_policy(now)

    def _set_auto_exposure_mode(self):
        try:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
        except Exception:
            pass
        try:
            self.cap.set(cv2.CAP_PROP_FPS, CAMERA_TARGET_FPS)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, CAMERA_BUFFER_SIZE)
        except Exception:
            pass
        p_ms, g_mean = self._measure_camera_stats(self.cap, samples=4, warmup=1)
        self._runtime_policy_state = "auto"
        self._runtime_manual_exp = None
        self._last_runtime_mode_change_ts = time.perf_counter()
        return p_ms, g_mean

    def _set_manual_exposure_mode(self, exposure_value):
        set_result = self._apply_manual_exposure(self.cap, float(exposure_value), settle_reads=2)
        p_ms, g_mean = self._measure_camera_stats(self.cap, samples=4, warmup=1)
        self._runtime_policy_state = "manual"
        self._runtime_manual_exp = float(exposure_value)
        self._last_runtime_mode_change_ts = time.perf_counter()
        return p_ms, g_mean, set_result

    def _probe_manual_exposure_candidates(self, candidates):
        probe_results = []
        for exp in candidates:
            p_ms, g_mean, set_result = self._set_manual_exposure_mode(exp)
            probe_results.append((float(p_ms), float(g_mean), float(exp), set_result))
            if self.debug_level >= 1:
                self._log_camera_probe(
                    "runtime_probe",
                    getattr(self, "camera_backend", "unknown"),
                    self._camera_props_snapshot(self.cap),
                    period_ms=p_ms,
                    gray_mean=g_mean,
                    extra=(
                        f"requested_exp={float(exp):.2f} "
                        f"set_ok(auto={int(set_result.get('auto_set_ok', False))},"
                        f"exp1={int(set_result.get('exp_set_ok_1', False))},"
                        f"exp2={int(set_result.get('exp_set_ok_2', False))})"
                    ),
                )
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

        self._set_manual_exposure_mode(best[2])
        return best

    def _update_software_gain(self, gray_level):
        target_gray = float(CAMERA_RUNTIME_TARGET_GRAY)
        max_gain = float(CAMERA_RUNTIME_SOFT_GAIN_MAX)
        if gray_level <= 0.5:
            desired_gain = max_gain
        elif gray_level < target_gray:
            desired_gain = min(max_gain, max(1.0, target_gray / gray_level))
        else:
            desired_gain = 1.0
        self._software_brightness_gain = 0.85 * self._software_brightness_gain + 0.15 * desired_gain

    def _evaluate_runtime_camera_policy(self, now):
        if not CAMERA_RUNTIME_ADAPTIVE or self._runtime_recovery_disabled:
            return
        if (now - self._capture_start_ts) < float(CAMERA_RUNTIME_WARMUP_S):
            return
        if (now - self._last_runtime_policy_ts) < float(CAMERA_RUNTIME_CHECK_S):
            return
        self._last_runtime_policy_ts = now
        self._runtime_profile_attempts += 1

        period_ms = float(self._capture_period_ms) if self._capture_period_ms > 0 else float(self.camera_measured_period_ms)
        gray_mean = float(self._capture_gray_mean) if self._capture_gray_mean > 0 else float(getattr(self, "camera_gray_mean", 0.0))
        max_period = float(CAMERA_RUNTIME_MAX_PERIOD_MS)
        min_gray = float(CAMERA_RUNTIME_MIN_GRAY)

        if self.debug_level >= 1:
            print(
                "[CAMERA] runtime policy "
                f"state={self._runtime_policy_state} period={period_ms:.1f}ms gray={gray_mean:.1f} "
                f"sw_gain={self._software_brightness_gain:.2f} attempts={self._runtime_profile_attempts}"
            )

        changed = False
        if period_ms > max_period + 2.0 and (now - self._last_runtime_mode_change_ts) > 2.5:
            candidates = [float(CAMERA_EXPOSURE), -4.0, -5.0, -6.0]
            if self._runtime_manual_exp is not None:
                candidates = [self._runtime_manual_exp] + candidates
            seen = set()
            ordered = []
            for c in candidates:
                if c in seen:
                    continue
                seen.add(c)
                ordered.append(c)
            best = self._probe_manual_exposure_candidates(ordered)
            if best is not None and self.debug_level >= 1:
                print(
                    "[CAMERA] runtime policy selected manual "
                    f"exp={best[2]:.2f} period~{best[0]:.1f}ms gray={best[1]:.1f}"
                )
                changed = True
        elif gray_mean < min_gray and (now - self._last_runtime_mode_change_ts) > 2.5:
            # If cadence is acceptable but image is dark, prefer brighter manual exposure first.
            if self._runtime_policy_state != "manual":
                base = float(CAMERA_EXPOSURE)
                bright_candidates = [base, -4.0, -3.0, -5.0]
                seen = set()
                ordered = []
                for c in bright_candidates:
                    if c in seen:
                        continue
                    seen.add(c)
                    ordered.append(c)
                best = self._probe_manual_exposure_candidates(ordered)
                if best is not None and self.debug_level >= 1:
                    print(
                        "[CAMERA] runtime policy brightened manual "
                        f"exp={best[2]:.2f} period~{best[0]:.1f}ms gray={best[1]:.1f}"
                    )
                    changed = True

        self._update_software_gain(gray_mean)
        if changed:
            self._runtime_speed_profile_applied = True
            self.camera_gray_mean = gray_mean

    def _reopen_camera_runtime_profile(self):
        if self._runtime_recovery_disabled:
            return
        forced = str(CAMERA_FORCE_BACKEND).upper()
        if forced == "DSHOW":
            backend = cv2.CAP_DSHOW
        elif forced == "MSMF":
            backend = cv2.CAP_MSMF
        elif forced == "ANY":
            backend = cv2.CAP_ANY
        else:
            b = str(getattr(self, "camera_backend", "")).upper()
            backend = cv2.CAP_DSHOW if b == "DSHOW" else cv2.CAP_ANY

        cap = cv2.VideoCapture(self.camera_index, backend)
        if not cap.isOpened():
            cap.release()
            return
        self._configure_camera(cap, force_auto=True)
        p_ms, g_mean = self._measure_camera_stats(cap, samples=6, warmup=1)
        old = self.cap
        self.cap = cap
        self.camera_backend = self._backend_name(backend)
        self.camera_measured_period_ms = float(p_ms)
        self.camera_gray_mean = float(g_mean)
        self._capture_period_ms = p_ms
        self._capture_gray_mean = g_mean
        self._prev_capture_ts = 0.0
        self._runtime_policy_state = "bootstrap_auto"
        self._runtime_manual_exp = None
        self._last_runtime_mode_change_ts = time.perf_counter()
        try:
            old.release()
        except Exception:
            pass
        if self.debug_level >= 1:
            print(
                "[CAMERA] Runtime reopen selected "
                f"backend={self.camera_backend} period~{p_ms:.1f}ms gray={g_mean:.1f}"
            )

    def set_pd_gains(self, kp, kd):
        self.pd_kp = float(kp)
        self.pd_kd = float(kd)

    def set_hsv_thresholds(self, hmin, hmax, smin, smax, vmin, vmax):
        self.hsv_lower = np.array([int(hmin), int(smin), int(vmin)], dtype=np.uint8)
        self.hsv_upper = np.array([int(hmax), int(smax), int(vmax)], dtype=np.uint8)

    def set_target_mm(self, x_mm, y_mm):
        self.target_x_mm = float(x_mm)
        self.target_y_mm = float(y_mm)

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
        self.prev_ball_raw_mm = None
        self.prev_time = None
        self.vx_smooth = 0.0
        self.vy_smooth = 0.0
        self._last_pos_filter_alpha = self.pos_filter_alpha_fast

    def _filter_marker_center(self, marker_id, center_px):
        current = np.asarray(center_px, dtype=np.float32)
        prev = self._marker_centers_lp.get(int(marker_id))
        if prev is None:
            filt = current
        else:
            if float(np.hypot(*(current - prev))) > 40.0:
                # Sudden jump (camera reopen/reposition): avoid dragging stale history.
                filt = current
            else:
                alpha = self.aruco_center_filter_alpha
                filt = alpha * prev + (1.0 - alpha) * current
        self._marker_centers_lp[int(marker_id)] = filt
        return filt

    def _apply_software_brightness(self, frame):
        gain = float(self._software_brightness_gain)
        if gain <= 1.02:
            return frame
        try:
            return cv2.convertScaleAbs(frame, alpha=gain, beta=0)
        except Exception:
            return frame

    def _draw_vectors(self, warped, bx, by, vx, vy, x_mm, y_mm):
        center_px = (self.WARP_SIZE_PX // 2, self.WARP_SIZE_PX // 2)
        target_px_f = self.mm_to_warp_px(self.target_x_mm, self.target_y_mm)
        target_px = (int(target_px_f[0]), int(target_px_f[1]))
        cv2.drawMarker(warped, center_px, (110, 110, 110), markerType=cv2.MARKER_CROSS, markerSize=12, thickness=1)
        cv2.drawMarker(warped, target_px, (0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=16, thickness=2)
        cv2.circle(warped, (bx, by), 5, (0, 0, 255), -1)

        vel_end = (int(bx + vx * self.VEL_ARROW_SCALE), int(by - vy * self.VEL_ARROW_SCALE))
        cv2.arrowedLine(warped, (bx, by), vel_end, (255, 0, 0), 2)

        pos_vec_x = self.target_x_mm - x_mm
        pos_vec_y = self.target_y_mm - y_mm
        pos_end = (int(bx + pos_vec_x * self.POS_ARROW_SCALE), int(by - pos_vec_y * self.POS_ARROW_SCALE))
        cv2.arrowedLine(warped, (bx, by), pos_end, (0, 255, 255), 2)

        pd_x = self.pd_kp * pos_vec_x + self.pd_kd * (-vx)
        pd_y = self.pd_kp * pos_vec_y + self.pd_kd * (-vy)
        pd_end = (int(bx + pd_x * self.PD_ARROW_SCALE), int(by - pd_y * self.PD_ARROW_SCALE))
        cv2.arrowedLine(warped, (bx, by), pd_end, (0, 255, 0), 2)

        cv2.putText(warped, "Center", (center_px[0] + 8, center_px[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (140, 140, 140), 1, cv2.LINE_AA)
        cv2.putText(warped, "Target", (target_px[0] + 8, target_px[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(warped, "Vel", (vel_end[0] + 4, vel_end[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(warped, "Pos->Target", (pos_end[0] + 4, pos_end[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
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
                "capture_gray": self._capture_gray_mean,
                "sw_gain": float(self._software_brightness_gain),
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
                "capture_gray": self._capture_gray_mean,
                "sw_gain": float(self._software_brightness_gain),
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
            "capture_gray": self._capture_gray_mean,
            "sw_gain": float(self._software_brightness_gain),
            "slow_streak": float(self._slow_capture_streak),
            "runtime_profile_applied": 1.0 if self._runtime_speed_profile_applied else 0.0,
            "runtime_profile_tries": float(self._runtime_profile_attempts),
        }

        self._frame_counter += 1
        frame = cv2.flip(frame, 1)
        frame = self._apply_software_brightness(frame)
        H = None
        warped = None
        mask = None

        def _failure_result(
            reason: str,
            *,
            warped_img=None,
            mask_img=None,
            mean_gray_val=0.0,
            aruco_ids_val=0,
            homography_ms_val=0.0,
            aruco_retry_ms_val=0.0,
            used_cached_h_val=0.0,
            warp_ms_val=0.0,
            hsv_ms_val=0.0,
            contour_ms_val=0.0,
        ):
            self.last_status_reason = reason
            fail_profile = {
                "capture_fetch": (t1 - t0) * 1000.0,
                "frame_age": frame_age_ms,
                "capture_period": self._capture_period_ms,
                "capture_gray": self._capture_gray_mean,
                "sw_gain": float(self._software_brightness_gain),
                "slow_streak": float(self._slow_capture_streak),
                "runtime_profile_applied": 1.0 if self._runtime_speed_profile_applied else 0.0,
                "runtime_profile_tries": float(self._runtime_profile_attempts),
                "aruco_detect": 0.0,
                "aruco_retry": float(aruco_retry_ms_val),
                "homography": float(homography_ms_val),
                "warp": float(warp_ms_val),
                "hsv_mask": float(hsv_ms_val),
                "contour": float(contour_ms_val),
                "kinematics": 0.0,
                "overlay": 0.0,
                "total": (time.perf_counter() - t0) * 1000.0,
                "used_cached_h": float(used_cached_h_val),
            }
            self.last_profile_ms = {
                "frame_age": frame_age_ms,
                "capture_period": self._capture_period_ms,
                "capture_gray": self._capture_gray_mean,
                "sw_gain": float(self._software_brightness_gain),
                "slow_streak": float(self._slow_capture_streak),
                "runtime_profile_applied": 1.0 if self._runtime_speed_profile_applied else 0.0,
                "runtime_profile_tries": float(self._runtime_profile_attempts),
            }
            if not return_debug_frames:
                return None
            local_warped = warped_img
            local_mask = mask_img
            if local_mask is None:
                src = local_warped if local_warped is not None else frame
                hsv_dbg = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
                local_mask = cv2.inRange(hsv_dbg, self.hsv_lower, self.hsv_upper)
            return {
                "tracking_valid": False,
                "reason": reason,
                "frame_ts": float(frame_ts),
                "profile_ms": fail_profile,
                "quality": {
                    "radius_px": 0.0,
                    "contour_area": 0.0,
                    "circularity": 0.0,
                    "fill_ratio": 0.0,
                    "dt_s": 0.0,
                    "gray_mean": float(mean_gray_val),
                    "warp_gray_mean": 0.0,
                    "vmin_eff": float(self.hsv_lower[2]),
                    "aruco_ids": float(aruco_ids_val),
                },
                "camera_bgr": frame,
                "warped_bgr": local_warped,
                "mask_gray": local_mask,
            }

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
                    return _failure_result(f"no_aruco_ids(gray={mean_gray:.1f})", mean_gray_val=mean_gray)
            if ids is not None:
                ids = ids.flatten()
                aruco_id_count = int(ids.size)
                self._aruco_fail_streak = 0
            else:
                ids = np.array([], dtype=np.int32)

            marker_centers_px = {}
            for i, marker_id in enumerate(ids):
                if marker_id in self.CORNER_IDS:
                    center_px = self.get_marker_center(corners[i]) * scale_inv
                    marker_centers_px[marker_id] = self._filter_marker_center(marker_id, center_px)
            t2 = time.perf_counter()

            if ids.size > 0:
                if len(marker_centers_px) < 3:
                    if self._last_H is not None and self._aruco_hold_count < TRACKER_MAX_ARUCO_HOLD_FRAMES:
                        H = self._last_H
                        self._aruco_hold_count += 1
                    else:
                        return _failure_result(
                            "insufficient_aruco_points",
                            mean_gray_val=mean_gray,
                            aruco_ids_val=aruco_id_count,
                        )
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
                            return _failure_result(
                                "aruco_reconstruct_failed",
                                mean_gray_val=mean_gray,
                                aruco_ids_val=aruco_id_count,
                            )
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
                    return _failure_result(
                        "homography_failed",
                        mean_gray_val=mean_gray,
                        aruco_ids_val=aruco_id_count,
                        homography_ms_val=homography_ms,
                    )
                self._last_H = H
                self._aruco_hold_count = 0
        t_aruco1 = time.perf_counter()

        t_w0 = time.perf_counter()
        warped = cv2.warpPerspective(frame, H, (self.WARP_SIZE_PX, self.WARP_SIZE_PX), flags=cv2.INTER_LINEAR)
        t_w1 = time.perf_counter()
        gray_warp_mean = float(np.mean(cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)))
        t_m0 = time.perf_counter()
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        vmin_base = int(self.hsv_lower[2])
        if gray_warp_mean < 45.0:
            relax = min(70, int((45.0 - gray_warp_mean) * 2.0))
            vmin_eff = max(20, vmin_base - relax)
        else:
            vmin_eff = vmin_base
        lower_eff = self.hsv_lower.copy()
        lower_eff[2] = vmin_eff
        mask = cv2.inRange(hsv, lower_eff, self.hsv_upper)
        t_m1 = time.perf_counter()
        t_c0 = time.perf_counter()
        ball = self.find_ball_center(mask)
        if ball is None and vmin_eff > 20:
            # One extra relaxed pass helps when camera exposure momentarily dips.
            lower_eff2 = self.hsv_lower.copy()
            lower_eff2[2] = max(10, vmin_eff - 20)
            mask = cv2.inRange(hsv, lower_eff2, self.hsv_upper)
            ball = self.find_ball_center(mask)
            vmin_eff = int(lower_eff2[2])
        if ball is None:
            return _failure_result(
                "ball_contour_invalid",
                warped_img=warped,
                mask_img=mask,
                mean_gray_val=mean_gray,
                aruco_ids_val=aruco_id_count,
                aruco_retry_ms_val=aruco_retry_ms,
                homography_ms_val=homography_ms,
                used_cached_h_val=1.0 if use_cached_h else 0.0,
                warp_ms_val=(t_w1 - t_w0) * 1000.0,
                hsv_ms_val=(t_m1 - t_m0) * 1000.0,
                contour_ms_val=(time.perf_counter() - t_c0) * 1000.0,
            )
        (bx, by), radius_px, contour_area, circularity, fill_ratio, mask = ball
        t_c1 = time.perf_counter()

        t_k0 = time.perf_counter()
        cx = self.WARP_SIZE_PX // 2
        cy = self.WARP_SIZE_PX // 2
        mm_per_px = self.PLATFORM_SIZE_MM / self.WARP_SIZE_PX
        x_mm_raw = (bx - cx) * mm_per_px
        y_mm_raw = (cy - by) * mm_per_px
        current_time = frame_ts if frame_ts > 0 else time.perf_counter()
        t3 = time.perf_counter()

        dt = 0.0
        raw_speed_mm_s = 0.0
        filter_alpha = self.pos_filter_alpha_fast
        if self.prev_time is not None:
            dt = current_time - self.prev_time
        if self.prev_ball_raw_mm is not None and dt > 0:
            raw_dx = x_mm_raw - self.prev_ball_raw_mm[0]
            raw_dy = y_mm_raw - self.prev_ball_raw_mm[1]
            raw_speed_mm_s = float(np.hypot(raw_dx / dt, raw_dy / dt))
        if self.pos_filter_enabled and self.prev_ball_mm is not None:
            speed_ratio = min(1.0, raw_speed_mm_s / self.pos_filter_speed_mm_s)
            filter_alpha = self.pos_filter_alpha_slow + (
                self.pos_filter_alpha_fast - self.pos_filter_alpha_slow
            ) * speed_ratio
            x_mm = filter_alpha * self.prev_ball_mm[0] + (1.0 - filter_alpha) * x_mm_raw
            y_mm = filter_alpha * self.prev_ball_mm[1] + (1.0 - filter_alpha) * y_mm_raw
        else:
            x_mm = x_mm_raw
            y_mm = y_mm_raw
        self._last_pos_filter_alpha = float(filter_alpha)

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
                    return _failure_result(
                        f"velocity_outlier_{speed_raw:.1f}",
                        warped_img=warped,
                        mask_img=mask,
                        mean_gray_val=mean_gray,
                        aruco_ids_val=aruco_id_count,
                        aruco_retry_ms_val=aruco_retry_ms,
                        homography_ms_val=homography_ms,
                        used_cached_h_val=1.0 if use_cached_h else 0.0,
                        warp_ms_val=(t_w1 - t_w0) * 1000.0,
                        hsv_ms_val=(t_m1 - t_m0) * 1000.0,
                        contour_ms_val=(t_c1 - t_c0) * 1000.0,
                    )
                self.vx_smooth = self.VEL_ALPHA * self.vx_smooth + (1 - self.VEL_ALPHA) * vx_raw
                self.vy_smooth = self.VEL_ALPHA * self.vy_smooth + (1 - self.VEL_ALPHA) * vy_raw
                vx = self.vx_smooth
                vy = self.vy_smooth

        self.prev_ball_raw_mm = (x_mm_raw, y_mm_raw)
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
                "capture_gray": self._capture_gray_mean,
                "sw_gain": float(self._software_brightness_gain),
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
                "x_raw_mm": float(x_mm_raw),
                "y_raw_mm": float(y_mm_raw),
                "raw_speed_mm_s": float(raw_speed_mm_s),
                "pos_filter_alpha": float(self._last_pos_filter_alpha),
                "gray_mean": mean_gray,
                "warp_gray_mean": gray_warp_mean,
                "vmin_eff": float(vmin_eff),
                "aruco_ids": float(aruco_id_count),
                "sw_gain": float(self._software_brightness_gain),
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
