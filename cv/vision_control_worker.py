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

from typing import Callable, TextIO

import time
from dataclasses import dataclass, field

import numpy as np
from PyQt5 import QtCore

from control.ball_controller import BallController
from core.ik_engine import IKEngine
from core.platform_state import BallState, IKResult, Pose
from cv.ball_tracker import BallTracker
from cv.camera_source import CameraSource
from settings import (
    AUTO_TRIM_ENABLED,
    BALL_TARGET_DEFAULT_X_MM,
    BALL_TARGET_DEFAULT_Y_MM,
    DEBUG_PRINTS,
    LOG_EVERY_N,
    MANUAL_PITCH_TRIM_DEG,
    MANUAL_ROLL_TRIM_DEG,
    MAX_TILT_DEG,
    PD_DEFAULT_KD,
    PD_DEFAULT_KP,
    TRACKER_HSV_H_MAX,
    TRACKER_HSV_H_MIN,
    TRACKER_HSV_S_MAX,
    TRACKER_HSV_S_MIN,
    TRACKER_HSV_V_MAX,
    TRACKER_HSV_V_MIN,
    TRACKER_REACQUIRE_VALID_FRAMES,
    TRACKER_WARP_SIZE_PX,
    VISION_MISS_NEUTRAL_AFTER_FRAMES,
    VISION_NEUTRAL_RESEND_S,
)

# Timing dicts contain ONLY genuinely measured values — the 28 zero-filled
# "trk_*" placeholder keys are gone (the GUI plot was rendering fabricated
# zeros as data). Semantics of the frame_to_* keys:
#   frame_to_worker_ms: capture timestamp -> after tracker.process() returned
#   frame_to_cmd:       capture timestamp -> after the command send call
# Both measured against the timestamp of the frame actually processed.
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
    ik_result: IKResult | None
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
        kp: float = PD_DEFAULT_KP,
        kd: float = PD_DEFAULT_KD,
        max_tilt_deg: float = MAX_TILT_DEG,
        camera_index: int = 0,
        loop_hz: int = 50,
        snapshot_hz: int = 20,
        command_sender: Callable[..., object] | None = None,
        roll_offset: float = MANUAL_ROLL_TRIM_DEG,
        pitch_offset: float = MANUAL_PITCH_TRIM_DEG,
        auto_trim_enabled: bool = AUTO_TRIM_ENABLED,
        rtt_provider: Callable[[], tuple[float, float, int]] | None = None,
        position_log_path: str | None = None,
    ) -> None:
        super().__init__()

        self._ik = ik_solver
        # z setpoint pushed from the GUI via the set_z slot (queued signal
        # delivery replaces the old cross-thread z_provider lambda).
        self._z_setpoint = 0.0
        self._loop_hz = max(1, int(loop_hz))
        self._snapshot_hz = max(1, int(snapshot_hz))
        self._command_sender = command_sender
        self._camera_index = int(camera_index)
        # Serial RTT telemetry source (a plain callable, like
        # command_sender — the worker never touches SerialManager).
        self._rtt_provider = rtt_provider
        # Optional per-frame position log ("t,x,y" lines) for
        # tools/jitter_bench.py --csv replay.
        self._position_log_path = position_log_path or None
        self._position_log_file: TextIO | None = None
        self._position_log_lines = 0

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
        self._valid_streak = 0
        self._snapshot_inflight = False
        self._last_snapshot_emit_perf = 0.0

        self._prev_arm_points: np.ndarray | None = None
        self._last_frame_ts = 0.0
        self._last_error_emit = 0.0
        self._last_neutral_send = 0.0

        self.camera: CameraSource | None = None
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

        if self.ball_tracker is None or self.camera is None:
            try:
                # Camera first (the blocking probe runs on this QThread, off
                # the GUI); tracker/controller failures must release it —
                # a half-initialized worker used to leak the open camera
                # and then no-op forever on restart.
                if self.camera is None:
                    self.camera = CameraSource(camera_index=self._camera_index)
                    self.camera.open()
                self.ball_tracker = BallTracker(
                    platform_size_mm=240.0,
                    warp_size_px=int(TRACKER_WARP_SIZE_PX),
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
                if self.camera is not None:
                    try:
                        self.camera.close()
                    except Exception:
                        pass
                    self.camera = None
                self.ball_tracker = None
                self.ball_controller = None
                self.error.emit(f"camera init failed: {exc}")
                self.stopped.emit()
                return

            cam_stats = self.camera.stats()
            self.camera_ready.emit({
                "hsv_lower": self.ball_tracker.hsv_lower.tolist(),
                "hsv_upper": self.ball_tracker.hsv_upper.tolist(),
                "backend": cam_stats.backend,
                "mode": cam_stats.policy_mode,
                "period_ms": cam_stats.period_ms,
                "gray": cam_stats.gray_mean,
            })

        self._running = True
        self._counter = 0
        self._miss_count = 0
        self._was_missing = False
        self._valid_streak = 0
        self._snapshot_inflight = False
        self._last_snapshot_emit_perf = 0.0
        self._prev_arm_points = None
        self._last_frame_ts = 0.0
        self._last_neutral_send = 0.0

        if self._position_log_path and self._position_log_file is None:
            try:
                self._position_log_file = open(
                    self._position_log_path, "a", encoding="utf-8"
                )
                self._position_log_lines = 0
            except OSError as exc:
                self._position_log_file = None
                self.error.emit(f"position log open failed: {exc}")

        self._timer = QtCore.QTimer(self)
        self._timer.setTimerType(QtCore.Qt.TimerType.PreciseTimer)
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
            if self.camera is not None:
                self.camera.close()
        except Exception:
            pass
        self.camera = None
        self.ball_tracker = None
        self.ball_controller = None
        if self._position_log_file is not None:
            try:
                self._position_log_file.close()
            except OSError:
                pass
            self._position_log_file = None
        self.stopped.emit()

    @QtCore.pyqtSlot()
    def mark_snapshot_consumed(self) -> None:
        self._snapshot_inflight = False

    # ------------------------------------------------------------------
    # Configuration slots (can be called while running)
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot(float)
    def set_z(self, z_mm: float) -> None:
        """Store the platform z setpoint used for every commanded pose."""
        self._z_setpoint = float(z_mm)

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
        if (
            not self._running
            or self.camera is None
            or self.ball_tracker is None
            or self.ball_controller is None
        ):
            return

        loop_start = time.perf_counter()
        self._counter += 1

        try:
            # Stale frame: no new frame since last processed — skip silently.
            # Do NOT reset controller state or advance miss count.
            if not self.camera.has_new_frame(self._last_frame_ts):
                return

            grabbed = self.camera.read_latest()
            if grabbed is None:
                return
            frame, latest_ts = grabbed
            self._last_frame_ts = latest_ts

            t0 = time.perf_counter()
            ball_state = self.ball_tracker.process(
                frame, latest_ts,
                brightness_gain=self.camera.stats().software_gain,
            )
            t1 = time.perf_counter()

            now_perf = time.perf_counter()
            should_emit = (
                self._last_snapshot_emit_perf <= 0.0
                or (now_perf - self._last_snapshot_emit_perf)
                >= (1.0 / float(self._snapshot_hz))
            )

            cam_view, warped_view, mask_view = self.ball_tracker.debug_views()

            if ball_state is None:
                self._miss_count += 1
                self._was_missing = True
                self._valid_streak = 0

                # Neutral-pose fallback: on sustained ball loss, level the
                # platform (throttled). This is the SAFETY action — it runs
                # outside the reacquire gate and needs no tracking_valid.
                self._maybe_send_neutral()

                timings_ms = self._build_miss_timings(t0, t1, loop_start, latest_ts)
                snapshot = ControlSnapshot(
                    timestamp=time.time(),
                    ball_state=None,
                    pose={
                        "x": 0.0, "y": 0.0,
                        "z": self._z_setpoint,
                        "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                    },
                    servo_angles=[],
                    ik_success=False,
                    timings_ms=timings_ms,
                    ik_result=None,
                    control_terms=_EMPTY_CONTROL_TERMS,
                    tracking_valid=False,
                    reason="no_ball_detected",
                    miss_count=self._miss_count,
                    camera_bgr=cam_view,
                    warped_bgr=warped_view,
                    mask_gray=mask_view,
                    worker_emit_perf_ts=time.perf_counter(),
                )
                if should_emit and not self._snapshot_inflight:
                    self._snapshot_inflight = True
                    self._last_snapshot_emit_perf = now_perf
                    self.snapshot_ready.emit(snapshot)
                return

            # Valid detection.
            self._log_position(ball_state)
            if self._was_missing:
                self.ball_controller.reset_motion_state()
                self._was_missing = False
                self._valid_streak = 0
            self._miss_count = 0
            self._valid_streak += 1

            t2 = time.perf_counter()
            roll_deg, pitch_deg, control_terms = (
                self.ball_controller.compute_with_terms(ball_state)
            )
            t3 = time.perf_counter()

            z = self._z_setpoint
            pose = {
                "x": 0.0, "y": 0.0, "z": z,
                "roll": roll_deg, "pitch": pitch_deg, "yaw": 0.0,
            }
            ik_result = self._ik.solve(
                Pose(x=0.0, y=0.0, z=z, roll=roll_deg, pitch=pitch_deg, yaw=0.0),
                self._prev_arm_points,
            )
            t4 = time.perf_counter()

            servo_angles: list[float] = []
            if ik_result.success:
                self._prev_arm_points = ik_result.arm_points
                # No inline clamping: the ServoDriver send path applies the
                # single safety clip in core/safety.py.
                servo_angles = [float(a) for a in ik_result.servo_angles_deg]

            # Reacquire gating lives HERE, on the command path, counting
            # real processed frames — a single spurious detection after
            # ball loss must not drive the servos. (It used to live in the
            # GUI snapshot handler, which runs AFTER the send and sees only
            # subsampled snapshots: it gated nothing.)
            send_gated = self._valid_streak < TRACKER_REACQUIRE_VALID_FRAMES

            t_cmd0 = time.perf_counter()
            if servo_angles and not send_gated and self._command_sender is not None:
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
                ik_success=ik_result.success,
                timings_ms=timings_ms,
                ik_result=ik_result,
                control_terms=control_terms,
                tracking_valid=True,
                reason="reacquire_gating" if send_gated else "ok",
                miss_count=0,
                camera_bgr=cam_view,
                warped_bgr=warped_view,
                mask_gray=mask_view,
                worker_emit_perf_ts=time.perf_counter(),
            )
            if should_emit and not self._snapshot_inflight:
                self._snapshot_inflight = True
                self._last_snapshot_emit_perf = now_perf
                self.snapshot_ready.emit(snapshot)

        except Exception as exc:
            # Rate-limit: a persistent per-tick failure at loop rate would
            # flood the GUI log with up to VISION_LOOP_HZ messages/second.
            now = time.perf_counter()
            if (now - self._last_error_emit) >= 1.0:
                self._last_error_emit = now
                self.error.emit(str(exc))

    # ------------------------------------------------------------------
    # Neutral-pose fallback
    # ------------------------------------------------------------------

    def _maybe_send_neutral(self) -> None:
        """Send a level pose at the current z after sustained misses.

        Fires once the miss streak reaches VISION_MISS_NEUTRAL_AFTER_FRAMES
        and then at most every VISION_NEUTRAL_RESEND_S. The throttle stamp
        advances on every attempt (matching the previous GUI-side policy)
        so an unsolvable pose cannot spam IK at loop rate.
        """
        if self._miss_count < VISION_MISS_NEUTRAL_AFTER_FRAMES:
            return
        now = time.perf_counter()
        if (now - self._last_neutral_send) <= VISION_NEUTRAL_RESEND_S:
            return
        self._last_neutral_send = now
        ik = self._ik.solve(
            Pose(x=0.0, y=0.0, z=self._z_setpoint,
                 roll=0.0, pitch=0.0, yaw=0.0),
            None,
        )
        if ik.success and self._command_sender is not None:
            try:
                # Safety clamping happens inside the send path (core/safety
                # via ServoDriver).
                self._command_sender([float(a) for a in ik.servo_angles_deg])
            except Exception as exc:
                self.error.emit(f"neutral send failed: {exc}")

    # ------------------------------------------------------------------
    # Position log (jitter_bench --csv replay input)
    # ------------------------------------------------------------------

    def _log_position(self, ball_state: BallState) -> None:
        """Append one "t,x,y" line for a valid frame (no-op when disabled)."""
        f = self._position_log_file
        if f is None:
            return
        try:
            f.write(
                f"{time.perf_counter():.6f},"
                f"{ball_state.x_mm:.3f},{ball_state.y_mm:.3f}\n"
            )
            self._position_log_lines += 1
            if self._position_log_lines % 30 == 0:
                f.flush()
        except OSError as exc:
            self._position_log_file = None
            self.error.emit(f"position log write failed: {exc}")

    # ------------------------------------------------------------------
    # Timing helpers
    # ------------------------------------------------------------------

    def _serial_rtt_ms(self) -> float:
        """Current serial RTT EMA in ms (0.0 when no provider/samples)."""
        if self._rtt_provider is None:
            return 0.0
        try:
            ema_ms, _last_ms, samples = self._rtt_provider()
        except Exception:
            return 0.0
        return float(ema_ms) if samples > 0 else 0.0

    def _capture_stats_pair(self) -> tuple[float, float]:
        if self.camera is None:
            return 0.0, 0.0
        st = self.camera.stats()
        return float(st.period_ms), float(st.gray_mean)

    def _build_miss_timings(
        self,
        t0: float,
        t1: float,
        loop_start: float,
        latest_ts: float,
    ) -> dict:
        period_ms, gray_mean = self._capture_stats_pair()
        return {
            "ball_update": (t1 - t0) * 1000.0,
            "pd_compute": 0.0,
            "ik_solve": 0.0,
            "serial_enqueue": 0.0,
            "total": (time.perf_counter() - loop_start) * 1000.0,
            "frame_to_worker_ms": (
                max(0.0, (t1 - latest_ts) * 1000.0) if latest_ts > 0 else 0.0
            ),
            "worker_to_gui_ms": 0.0,
            "frame_to_cmd": 0.0,
            "serial_rtt_ms": self._serial_rtt_ms(),
            "trk_cap_period": period_ms,
            "trk_gray_mean": gray_mean,
        }

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
        period_ms, gray_mean = self._capture_stats_pair()
        frame_to_worker = (
            max(0.0, (t1 - latest_ts) * 1000.0) if latest_ts > 0 else 0.0
        )
        frame_to_cmd = (
            max(0.0, (t_cmd1 - latest_ts) * 1000.0) if latest_ts > 0 else 0.0
        )
        return {
            "ball_update": (t1 - t0) * 1000.0,
            "pd_compute": (t3 - t2) * 1000.0,
            "ik_solve": (t4 - t3) * 1000.0,
            "serial_enqueue": (t_cmd1 - t_cmd0) * 1000.0,
            "total": (time.perf_counter() - loop_start) * 1000.0,
            "frame_to_worker_ms": frame_to_worker,
            "worker_to_gui_ms": 0.0,
            "frame_to_cmd": frame_to_cmd,
            "serial_rtt_ms": self._serial_rtt_ms(),
            "trk_cap_period": period_ms,
            "trk_gray_mean": gray_mean,
        }
