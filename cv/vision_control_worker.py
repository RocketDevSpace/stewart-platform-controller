"""
cv/vision_control_worker.py

VisionControlWorker: QObject that owns BallTracker + BallController and runs
the full vision/PD/IK loop in a dedicated QThread. Communicates with the GUI
exclusively via Qt signals (snapshot_ready, camera_ready, error, stopped).

ControlSnapshot is a dataclass carrying the full per-frame telemetry package.
The GUI calls mark_snapshot_consumed() after processing each snapshot; the
worker will not emit the next snapshot until that call arrives (backpressure).
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from PyQt5 import QtCore

from control.ball_controller import BallController
from core.ik_engine import IKEngine
from core.platform_state import BallState, Pose
from cv.ball_tracker import BallTracker
from settings import (
    AUTO_TRIM_ENABLED,
    BALL_TARGET_DEFAULT_X_MM,
    BALL_TARGET_DEFAULT_Y_MM,
    DEBUG_PRINTS,
    LOG_EVERY_N,
    MANUAL_PITCH_TRIM_DEG,
    MANUAL_ROLL_TRIM_DEG,
    PD_DEFAULT_KD,
    PD_DEFAULT_KP,
    TRACKER_HSV_H_MAX,
    TRACKER_HSV_H_MIN,
    TRACKER_HSV_S_MAX,
    TRACKER_HSV_S_MIN,
    TRACKER_HSV_V_MAX,
    TRACKER_HSV_V_MIN,
)

# tracker-internal keys we zero-fill (our BallTracker.update() returns BallState,
# not a dict with profile_ms, so these sub-step timings are unavailable)
_TRK_ZERO_KEYS: tuple[str, ...] = (
    "trk_capture",
    "trk_frame_age",
    "trk_slow_streak",
    "trk_runtime_applied",
    "trk_runtime_tries",
    "trk_aruco",
    "trk_aruco_retry",
    "trk_h",
    "trk_warp",
    "trk_hsv",
    "trk_contour",
    "trk_kin",
    "trk_overlay",
    "trk_cached_h",
    "trk_radius_px",
    "trk_area",
    "trk_circularity",
    "trk_fill",
    "trk_dt_s",
    "trk_warp_gray",
    "trk_vmin_eff",
    "trk_aruco_ids",
    "trk_raw_speed_mm_s",
    "trk_raw_vx_mm_s",
    "trk_raw_vy_mm_s",
    "trk_pos_filter_alpha",
    "trk_pos_lag_mm",
    "trk_pos_filter_enabled",
)

_EMPTY_CONTROL_TERMS: dict = {
    "position_vec_mm": (0.0, 0.0),
    "velocity_vec_mm_s": (0.0, 0.0),
    "pd_vec": (0.0, 0.0),
}


@dataclass
class ControlSnapshot:
    """Full per-frame telemetry package passed from worker to GUI."""
    timestamp: float
    ball_state: BallState | None
    pose: dict
    servo_angles: list
    ik_success: bool
    timings_ms: dict
    ik_result: dict
    control_terms: dict
    tracking_valid: bool
    reason: str = ""
    miss_count: int = 0
    camera_bgr: object | None = None
    warped_bgr: object | None = None
    mask_gray: object | None = None
    worker_emit_perf_ts: float = field(default=0.0)


class VisionControlWorker(QtCore.QObject):
    """
    Owns BallTracker + BallController. Runs the vision/PD/IK loop in a
    QThread at VISION_LOOP_HZ via an internal QTimer.

    Signal contract:
      snapshot_ready(object)  — ControlSnapshot; GUI calls mark_snapshot_consumed()
      camera_ready(dict)      — fired once after tracker opens; contains HSV + cam info
      error(str)              — non-fatal diagnostic string
      stopped()               — emitted when the worker has fully shut down
    """

    snapshot_ready = QtCore.pyqtSignal(object)
    camera_ready = QtCore.pyqtSignal(object)
    error = QtCore.pyqtSignal(str)
    stopped = QtCore.pyqtSignal()

    def __init__(
        self,
        ik_solver: IKEngine,
        z_provider,
        kp: float = PD_DEFAULT_KP,
        kd: float = PD_DEFAULT_KD,
        max_tilt_deg: float = 8.0,
        camera_index: int = 0,
        loop_hz: int = 50,
        snapshot_hz: int = 20,
        command_sender=None,
        roll_offset: float = MANUAL_ROLL_TRIM_DEG,
        pitch_offset: float = MANUAL_PITCH_TRIM_DEG,
        auto_trim_enabled: bool = AUTO_TRIM_ENABLED,
    ) -> None:
        super().__init__()

        self._ik = ik_solver
        self._z_provider = z_provider
        self._loop_hz = max(1, int(loop_hz))
        self._snapshot_hz = max(1, int(snapshot_hz))
        self._command_sender = command_sender
        self._camera_index = int(camera_index)

        # Cached init params — applied to controller on start()
        self._kp_init = float(kp)
        self._kd_init = float(kd)
        self._max_tilt_deg = float(max_tilt_deg)
        self._roll_offset_init = float(roll_offset)
        self._pitch_offset_init = float(pitch_offset)
        self._auto_trim_enabled_init = bool(auto_trim_enabled)

        self._timer: QtCore.QTimer | None = None
        self._running = False
        self._counter = 0

        self._miss_count = 0
        self._was_missing = False
        self._snapshot_inflight = False
        self._last_snapshot_emit_perf = 0.0

        self._prev_arm_points = None

        self.ball_tracker: BallTracker | None = None
        self.ball_controller: BallController | None = None

        self._pending_hsv: tuple[int, int, int, int, int, int] = (
            int(TRACKER_HSV_H_MIN), int(TRACKER_HSV_H_MAX),
            int(TRACKER_HSV_S_MIN), int(TRACKER_HSV_S_MAX),
            int(TRACKER_HSV_V_MIN), int(TRACKER_HSV_V_MAX),
        )

    # ------------------------------------------------------------------
    # Lifecycle slots
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot()
    def start(self) -> None:
        if self._running:
            return

        if self.ball_tracker is None:
            try:
                self.ball_tracker = BallTracker(
                    camera_index=self._camera_index,
                    show_debug=False,
                )
                self.ball_controller = BallController(
                    kp=self._kp_init,
                    kd=self._kd_init,
                    max_tilt_deg=self._max_tilt_deg,
                    roll_offset=self._roll_offset_init,
                    pitch_offset=self._pitch_offset_init,
                    auto_trim_enabled=self._auto_trim_enabled_init,
                )
                self.ball_controller.set_target(
                    float(BALL_TARGET_DEFAULT_X_MM),
                    float(BALL_TARGET_DEFAULT_Y_MM),
                )
                self.ball_tracker.set_hsv_thresholds(*self._pending_hsv)
            except Exception as exc:
                self.error.emit(f"camera init failed: {exc}")
                self.stopped.emit()
                return

            self.camera_ready.emit({
                "hsv_lower": self.ball_tracker.hsv_lower.tolist(),
                "hsv_upper": self.ball_tracker.hsv_upper.tolist(),
                "backend": getattr(self.ball_tracker, "camera_backend", "unknown"),
                "mode": getattr(self.ball_tracker, "_runtime_policy_state", ""),
                "period_ms": float(getattr(self.ball_tracker, "camera_measured_period_ms", 0.0)),
                "gray": float(getattr(self.ball_tracker, "_capture_gray_mean", 0.0)),
            })

        self._running = True
        self._counter = 0
        self._miss_count = 0
        self._was_missing = False
        self._snapshot_inflight = False
        self._last_snapshot_emit_perf = 0.0
        self._prev_arm_points = None

        self._timer = QtCore.QTimer(self)
        self._timer.setTimerType(QtCore.Qt.PreciseTimer)
        self._timer.timeout.connect(self._tick)
        self._timer.start(int(1000 / self._loop_hz))

    @QtCore.pyqtSlot()
    def stop(self) -> None:
        self._running = False
        if self._timer is not None:
            self._timer.stop()
            self._timer.deleteLater()
            self._timer = None
        try:
            if self.ball_tracker is not None:
                self.ball_tracker.release()
        except Exception:
            pass
        self.ball_tracker = None
        self.ball_controller = None
        self.stopped.emit()

    @QtCore.pyqtSlot()
    def mark_snapshot_consumed(self) -> None:
        self._snapshot_inflight = False

    # ------------------------------------------------------------------
    # Configuration slots (can be called while running)
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot(float, float)
    def set_gains(self, kp: float, kd: float) -> None:
        self._kp_init = float(kp)
        self._kd_init = float(kd)
        if self.ball_controller is not None:
            self.ball_controller.set_gains(float(kp), float(kd))

    @QtCore.pyqtSlot(int, int, int, int, int, int)
    def set_hsv(
        self,
        hmin: int,
        hmax: int,
        smin: int,
        smax: int,
        vmin: int,
        vmax: int,
    ) -> None:
        self._pending_hsv = (
            int(hmin), int(hmax), int(smin), int(smax), int(vmin), int(vmax)
        )
        if self.ball_tracker is not None:
            self.ball_tracker.set_hsv_thresholds(*self._pending_hsv)

    @QtCore.pyqtSlot(float, float)
    def set_target(self, x_mm: float, y_mm: float) -> None:
        if self.ball_controller is not None:
            self.ball_controller.set_target(float(x_mm), float(y_mm))

    @QtCore.pyqtSlot(float, float)
    def set_trim(self, roll_offset_deg: float, pitch_offset_deg: float) -> None:
        self._roll_offset_init = float(roll_offset_deg)
        self._pitch_offset_init = float(pitch_offset_deg)
        if self.ball_controller is not None:
            self.ball_controller.set_trim(float(roll_offset_deg), float(pitch_offset_deg))

    @QtCore.pyqtSlot(bool)
    def set_auto_trim_enabled(self, enabled: bool) -> None:
        self._auto_trim_enabled_init = bool(enabled)
        if self.ball_controller is not None:
            self.ball_controller.set_auto_trim_enabled(bool(enabled))

    @QtCore.pyqtSlot()
    def reset_trim(self) -> None:
        if self.ball_controller is not None:
            self.ball_controller.reset_trim()

    @QtCore.pyqtSlot(bool)
    def set_home_calibration(self, enabled: bool) -> None:
        if self.ball_controller is None:
            return
        if bool(enabled):
            self.ball_controller.start_home_calibration()
        else:
            self.ball_controller.cancel_home_calibration()

    @QtCore.pyqtSlot(bool)
    def set_pd_autotune_enabled(self, enabled: bool) -> None:
        if self.ball_controller is not None:
            self.ball_controller.set_pd_autotune(bool(enabled), None)

    @QtCore.pyqtSlot(bool)
    def set_pd_autotune_auto_apply(self, enabled: bool) -> None:
        if self.ball_controller is not None:
            self.ball_controller.set_pd_autotune(
                self.ball_controller.pd_autotune_enabled, bool(enabled)
            )

    @QtCore.pyqtSlot()
    def apply_pd_autotune_recommendation(self) -> None:
        if self.ball_controller is not None:
            self.ball_controller.apply_pd_autotune_recommendation()

    # ------------------------------------------------------------------
    # Inner loop
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot()
    def _tick(self) -> None:
        if not self._running or self.ball_tracker is None or self.ball_controller is None:
            return

        loop_start = time.perf_counter()
        self._counter += 1

        try:
            # Read latest_ts under lock for stale-check and latency timing.
            with self.ball_tracker._capture_lock:
                latest_ts = self.ball_tracker._latest_frame_ts

            # Stale frame: no new frame since last processed — skip silently.
            # Do NOT reset controller state or advance miss count.
            if (
                latest_ts > 0
                and self.ball_tracker._last_processed_frame_ts > 0
                and latest_ts <= self.ball_tracker._last_processed_frame_ts
            ):
                return

            t0 = time.perf_counter()
            ball_state = self.ball_tracker.update()
            t1 = time.perf_counter()

            now_perf = time.perf_counter()
            should_emit = (
                self._last_snapshot_emit_perf <= 0.0
                or (now_perf - self._last_snapshot_emit_perf)
                >= (1.0 / float(self._snapshot_hz))
            )

            if ball_state is None:
                self._miss_count += 1
                self._was_missing = True

                timings_ms = self._build_miss_timings(t0, t1, loop_start, latest_ts)
                snapshot = ControlSnapshot(
                    timestamp=time.time(),
                    ball_state=None,
                    pose={
                        "x": 0.0, "y": 0.0,
                        "z": float(self._z_provider()),
                        "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                    },
                    servo_angles=[],
                    ik_success=False,
                    timings_ms=timings_ms,
                    ik_result={},
                    control_terms=_EMPTY_CONTROL_TERMS,
                    tracking_valid=False,
                    reason="no_ball_detected",
                    miss_count=self._miss_count,
                    worker_emit_perf_ts=time.perf_counter(),
                )
                if should_emit and not self._snapshot_inflight:
                    self._snapshot_inflight = True
                    self._last_snapshot_emit_perf = now_perf
                    self.snapshot_ready.emit(snapshot)
                return

            # Valid detection.
            if self._was_missing:
                self.ball_controller.reset_motion_state()
                self._was_missing = False
            self._miss_count = 0

            t2 = time.perf_counter()
            roll_deg, pitch_deg, control_terms = (
                self.ball_controller.compute_with_terms(ball_state)
            )
            t3 = time.perf_counter()

            z = float(self._z_provider())
            pose = {
                "x": 0.0, "y": 0.0, "z": z,
                "roll": roll_deg, "pitch": pitch_deg, "yaw": 0.0,
            }
            ik_result = self._ik.solve(
                Pose(x=0.0, y=0.0, z=z, roll=roll_deg, pitch=pitch_deg, yaw=0.0),
                self._prev_arm_points,
            )
            t4 = time.perf_counter()

            servo_angles: list[int] = []
            if ik_result.get("success", False):
                self._prev_arm_points = ik_result.get("arm_points")
                servo_angles = [
                    max(0, min(180, int(round(a))))
                    for a in ik_result["servo_angles_deg"]
                ]

            t_cmd0 = time.perf_counter()
            if servo_angles and self._command_sender is not None:
                try:
                    self._command_sender(servo_angles)
                except Exception as exc:
                    self.error.emit(f"command send failed: {exc}")
            t_cmd1 = time.perf_counter()

            timings_ms = self._build_valid_timings(
                t0, t1, t2, t3, t4, t_cmd0, t_cmd1, loop_start, latest_ts
            )

            if DEBUG_PRINTS and (self._counter % LOG_EVERY_N == 0):
                print(
                    "[VISION]",
                    f"ball={timings_ms['ball_update']:.2f}ms",
                    f"pd={timings_ms['pd_compute']:.2f}ms",
                    f"ik={timings_ms['ik_solve']:.2f}ms",
                    f"total={timings_ms['total']:.2f}ms",
                )

            snapshot = ControlSnapshot(
                timestamp=time.time(),
                ball_state=ball_state,
                pose=pose,
                servo_angles=servo_angles,
                ik_success=ik_result.get("success", False),
                timings_ms=timings_ms,
                ik_result=ik_result,
                control_terms=control_terms,
                tracking_valid=True,
                reason="ok",
                miss_count=0,
                worker_emit_perf_ts=time.perf_counter(),
            )
            if should_emit and not self._snapshot_inflight:
                self._snapshot_inflight = True
                self._last_snapshot_emit_perf = now_perf
                self.snapshot_ready.emit(snapshot)

        except Exception as exc:
            self.error.emit(str(exc))

    # ------------------------------------------------------------------
    # Timing helpers
    # ------------------------------------------------------------------

    def _build_miss_timings(
        self,
        t0: float,
        t1: float,
        loop_start: float,
        latest_ts: float,
    ) -> dict:
        tracker = self.ball_tracker
        timings_ms: dict = {
            "ball_update": (t1 - t0) * 1000.0,
            "pd_compute": 0.0,
            "ik_solve": 0.0,
            "serial_enqueue": 0.0,
            "total": (time.perf_counter() - loop_start) * 1000.0,
            "frame_to_worker_ms": 0.0,
            "worker_to_gui_ms": 0.0,
            "frame_to_cmd": 0.0,
            "trk_cap_period": float(getattr(tracker, "_capture_period_ms", 0.0)),
            "trk_gray_mean": float(getattr(tracker, "_capture_gray_mean", 0.0)),
        }
        for key in _TRK_ZERO_KEYS:
            timings_ms.setdefault(key, 0.0)
        return timings_ms

    def _build_valid_timings(
        self,
        t0: float,
        t1: float,
        t2: float,
        t3: float,
        t4: float,
        t_cmd0: float,
        t_cmd1: float,
        loop_start: float,
        latest_ts: float,
    ) -> dict:
        tracker = self.ball_tracker
        frame_to_worker = (
            max(0.0, (t1 - latest_ts) * 1000.0) if latest_ts > 0 else 0.0
        )
        frame_to_cmd = (
            max(0.0, (t_cmd1 - latest_ts) * 1000.0) if latest_ts > 0 else 0.0
        )
        timings_ms: dict = {
            "ball_update": (t1 - t0) * 1000.0,
            "pd_compute": (t3 - t2) * 1000.0,
            "ik_solve": (t4 - t3) * 1000.0,
            "serial_enqueue": (t_cmd1 - t_cmd0) * 1000.0,
            "total": (time.perf_counter() - loop_start) * 1000.0,
            "frame_to_worker_ms": frame_to_worker,
            "worker_to_gui_ms": 0.0,
            "frame_to_cmd": frame_to_cmd,
            "trk_cap_period": float(getattr(tracker, "_capture_period_ms", 0.0)),
            "trk_gray_mean": float(getattr(tracker, "_capture_gray_mean", 0.0)),
        }
        for key in _TRK_ZERO_KEYS:
            timings_ms.setdefault(key, 0.0)
        return timings_ms
