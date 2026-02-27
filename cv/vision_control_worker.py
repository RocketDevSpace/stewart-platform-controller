from dataclasses import dataclass
import time

from PyQt5 import QtCore

from stewart_control.config import DEBUG_LEVEL, LOG_EVERY_N
from stewart_control.cv.ball_controller import BallController, BallState
from stewart_control.cv.ball_tracker import BallTracker


@dataclass
class ControlSnapshot:
    timestamp: float
    frame_ts: float
    worker_emit_perf_ts: float
    ball_state: BallState
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


class VisionControlWorker(QtCore.QObject):
    snapshot_ready = QtCore.pyqtSignal(object)
    error = QtCore.pyqtSignal(str)
    stopped = QtCore.pyqtSignal()

    def __init__(
        self,
        ik_solver,
        z_provider,
        kp=0.005,
        kd=0.010,
        max_tilt_deg=8.0,
        camera_index=0,
        loop_hz=50,
        snapshot_hz=20,
        tracker_debug=False,
        emit_camera_every_n=1,
        command_sender=None,
    ):
        super().__init__()
        self.ik_solver = ik_solver
        self.z_provider = z_provider
        self.loop_hz = max(1, int(loop_hz))
        self.snapshot_hz = max(1, int(snapshot_hz))
        self.command_sender = command_sender

        self._timer = None
        self._running = False
        self._counter = 0
        self._emit_camera_every_n = max(1, int(emit_camera_every_n))
        self._miss_count = 0
        self._was_missing = False
        self._stale_count = 0
        self._last_snapshot_emit_perf = 0.0
        self._snapshot_inflight = False

        self.ball_controller = BallController(kp=kp, kd=kd, max_tilt_deg=max_tilt_deg)
        self.ball_tracker = BallTracker(
            camera_index=camera_index,
            pd_kp=kp,
            pd_kd=kd,
        )

    @QtCore.pyqtSlot()
    def start(self):
        if self._running:
            return
        self._running = True
        self._timer = QtCore.QTimer(self)
        self._timer.setTimerType(QtCore.Qt.PreciseTimer)
        self._timer.timeout.connect(self._tick)
        self._timer.start(int(1000 / self.loop_hz))

    @QtCore.pyqtSlot()
    def stop(self):
        self._running = False
        if self._timer is not None:
            self._timer.stop()
            self._timer.deleteLater()
            self._timer = None
        try:
            self.ball_tracker.release()
        except Exception:
            pass
        self.stopped.emit()

    @QtCore.pyqtSlot()
    def mark_snapshot_consumed(self):
        self._snapshot_inflight = False

    @QtCore.pyqtSlot(float, float)
    def set_gains(self, kp, kd):
        self.ball_controller.set_gains(kp, kd)
        self.ball_tracker.set_pd_gains(kp, kd)

    @QtCore.pyqtSlot(int, int, int, int, int, int)
    def set_hsv(self, hmin, hmax, smin, smax, vmin, vmax):
        self.ball_tracker.set_hsv_thresholds(hmin, hmax, smin, smax, vmin, vmax)

    @QtCore.pyqtSlot()
    def _tick(self):
        if not self._running:
            return

        loop_start = time.perf_counter()
        try:
            t0 = time.perf_counter()
            emit_frames = (self._counter % self._emit_camera_every_n == 0)
            ball_state_dict = self.ball_tracker.update(return_debug_frames=emit_frames)
            t1 = time.perf_counter()
            now_perf = time.perf_counter()
            should_emit_snapshot = (
                self._last_snapshot_emit_perf <= 0.0
                or (now_perf - self._last_snapshot_emit_perf) >= (1.0 / float(self.snapshot_hz))
            )
            if ball_state_dict is None:
                reason = self.ball_tracker.last_status_reason or "no_ball_detected"
                is_stale = reason == "stale_frame_repeat"
                if is_stale:
                    self._stale_count += 1
                    # Do not emit stale-frame snapshots; they can flood the queued
                    # cross-thread signal path and delay fresh control updates.
                    return
                else:
                    self._miss_count += 1
                    self._was_missing = True
                    self.ball_tracker.reset_motion_state()
                timings_ms = {
                    "ball_update": (t1 - t0) * 1000.0,
                    "pd_compute": 0.0,
                    "ik_solve": 0.0,
                    "total": (time.perf_counter() - loop_start) * 1000.0,
                }
                tracker_profile = getattr(self.ball_tracker, "last_profile_ms", {}) or {}
                timings_ms["trk_frame_age"] = float(tracker_profile.get("frame_age", 0.0))
                timings_ms["trk_cap_period"] = float(tracker_profile.get("capture_period", 0.0))
                snapshot = ControlSnapshot(
                    timestamp=time.time(),
                    frame_ts=0.0,
                    worker_emit_perf_ts=time.perf_counter(),
                    ball_state=BallState(0.0, 0.0, 0.0, 0.0),
                    pose={"x": 0.0, "y": 0.0, "z": float(self.z_provider()), "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
                    servo_angles=[],
                    ik_success=False,
                    timings_ms=timings_ms,
                    ik_result={},
                    control_terms={"position_vec_mm": (0.0, 0.0), "velocity_vec_mm_s": (0.0, 0.0), "pd_vec": (0.0, 0.0)},
                    tracking_valid=False,
                    reason=reason,
                    miss_count=self._miss_count,
                    camera_bgr=None,
                    warped_bgr=None,
                    mask_gray=None,
                )
                if should_emit_snapshot and (not self._snapshot_inflight):
                    self._last_snapshot_emit_perf = now_perf
                    self._snapshot_inflight = True
                    self.snapshot_ready.emit(snapshot)
                return
            self._stale_count = 0
            self._miss_count = 0
            if self._was_missing:
                # Avoid derivative spikes immediately after reacquisition.
                self.ball_tracker.reset_motion_state()
                self._was_missing = False

            ball_state = BallState(
                x_mm=ball_state_dict["x_mm"],
                y_mm=ball_state_dict["y_mm"],
                vx_mm_s=ball_state_dict["vx_mm_s"],
                vy_mm_s=ball_state_dict["vy_mm_s"],
            )

            roll_deg, pitch_deg, control_terms = self.ball_controller.compute_with_terms(ball_state)
            t2 = time.perf_counter()

            pose = {
                "x": 0,
                "y": 0,
                "z": float(self.z_provider()),
                "roll": roll_deg,
                "pitch": pitch_deg,
                "yaw": 0,
            }

            ik_result = self.ik_solver.solve_pose(
                pose["x"],
                pose["y"],
                pose["z"],
                pose["roll"],
                pose["pitch"],
                pose["yaw"],
                prev_arm_points=None,
            )
            t3 = time.perf_counter()

            servo_angles = []
            if ik_result.get("success", False):
                servo_angles = [int(round(a)) for a in ik_result["servo_angles_deg"]]

            t_cmd0 = time.perf_counter()
            if servo_angles and self.command_sender is not None:
                safe_angles = [max(0, min(180, int(a))) for a in servo_angles]
                cmd = "S," + ",".join(str(a) for a in safe_angles) + ",0\n"
                try:
                    self.command_sender(cmd, "latest")
                except Exception as exc:
                    self.error.emit(f"worker command send failed: {exc}")
            t_cmd1 = time.perf_counter()

            timings_ms = {
                "ball_update": ball_state_dict.get("profile_ms", {}).get("total", (t1 - t0) * 1000.0),
                "pd_compute": (t2 - t1) * 1000.0,
                "ik_solve": (t3 - t2) * 1000.0,
                "total": (time.perf_counter() - loop_start) * 1000.0,
            }
            timings_ms["cmd_enqueue_worker"] = (t_cmd1 - t_cmd0) * 1000.0
            tracker_profile = ball_state_dict.get("profile_ms", {})
            timings_ms["trk_capture"] = tracker_profile.get("capture_fetch", 0.0)
            timings_ms["trk_frame_age"] = tracker_profile.get("frame_age", 0.0)
            timings_ms["trk_cap_period"] = tracker_profile.get("capture_period", 0.0)
            timings_ms["trk_aruco"] = tracker_profile.get("aruco_detect", 0.0)
            timings_ms["trk_aruco_retry"] = tracker_profile.get("aruco_retry", 0.0)
            timings_ms["trk_h"] = tracker_profile.get("homography", 0.0)
            timings_ms["trk_warp"] = tracker_profile.get("warp", 0.0)
            timings_ms["trk_hsv"] = tracker_profile.get("hsv_mask", 0.0)
            timings_ms["trk_contour"] = tracker_profile.get("contour", 0.0)
            timings_ms["trk_kin"] = tracker_profile.get("kinematics", 0.0)
            timings_ms["trk_overlay"] = tracker_profile.get("overlay", 0.0)
            timings_ms["trk_cached_h"] = tracker_profile.get("used_cached_h", 0.0)
            quality = ball_state_dict.get("quality", {})
            timings_ms["trk_radius_px"] = quality.get("radius_px", 0.0)
            timings_ms["trk_area"] = quality.get("contour_area", 0.0)
            timings_ms["trk_circularity"] = quality.get("circularity", 0.0)
            timings_ms["trk_fill"] = quality.get("fill_ratio", 0.0)
            timings_ms["trk_dt_s"] = quality.get("dt_s", 0.0)
            timings_ms["trk_gray_mean"] = quality.get("gray_mean", 0.0)
            timings_ms["trk_aruco_ids"] = quality.get("aruco_ids", 0.0)
            frame_ts = float(ball_state_dict.get("frame_ts", 0.0))
            if frame_ts > 0:
                timings_ms["frame_to_worker_ms"] = max(0.0, (time.perf_counter() - frame_ts) * 1000.0)
                timings_ms["frame_to_cmd_worker"] = max(0.0, (t_cmd1 - frame_ts) * 1000.0)
            else:
                timings_ms["frame_to_worker_ms"] = 0.0
                timings_ms["frame_to_cmd_worker"] = 0.0

            self._counter += 1
            if DEBUG_LEVEL >= 2 and (self._counter % LOG_EVERY_N == 0):
                print(
                    "[VISION]",
                    f"ball={timings_ms['ball_update']:.2f}ms",
                    f"pd={timings_ms['pd_compute']:.2f}ms",
                    f"ik={timings_ms['ik_solve']:.2f}ms",
                    f"total={timings_ms['total']:.2f}ms",
                )

            snapshot = ControlSnapshot(
                timestamp=time.time(),
                frame_ts=frame_ts,
                worker_emit_perf_ts=time.perf_counter(),
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
                camera_bgr=ball_state_dict.get("camera_bgr"),
                warped_bgr=ball_state_dict.get("warped_bgr"),
                mask_gray=ball_state_dict.get("mask_gray"),
            )
            if should_emit_snapshot and (not self._snapshot_inflight):
                self._last_snapshot_emit_perf = now_perf
                self._snapshot_inflight = True
                self.snapshot_ready.emit(snapshot)
        except Exception as exc:
            self.error.emit(str(exc))
