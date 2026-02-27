from dataclasses import dataclass
import time

from PyQt5 import QtCore

from stewart_control.config import DEBUG_LEVEL, LOG_EVERY_N
from stewart_control.cv.ball_controller import BallController, BallState
from stewart_control.cv.ball_tracker import BallTracker


@dataclass
class ControlSnapshot:
    timestamp: float
    ball_state: BallState
    pose: dict
    servo_angles: list
    ik_success: bool
    timings_ms: dict
    ik_result: dict
    control_terms: dict
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
        tracker_debug=False,
        emit_camera_every_n=2,
    ):
        super().__init__()
        self.ik_solver = ik_solver
        self.z_provider = z_provider
        self.loop_hz = max(1, int(loop_hz))

        self._timer = None
        self._running = False
        self._counter = 0
        self._emit_camera_every_n = max(1, int(emit_camera_every_n))

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
            if ball_state_dict is None:
                return

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

            timings_ms = {
                "ball_update": (t1 - t0) * 1000.0,
                "pd_compute": (t2 - t1) * 1000.0,
                "ik_solve": (t3 - t2) * 1000.0,
                "total": (time.perf_counter() - loop_start) * 1000.0,
            }

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
                ball_state=ball_state,
                pose=pose,
                servo_angles=servo_angles,
                ik_success=ik_result.get("success", False),
                timings_ms=timings_ms,
                ik_result=ik_result,
                control_terms=control_terms,
                camera_bgr=ball_state_dict.get("camera_bgr"),
                warped_bgr=ball_state_dict.get("warped_bgr"),
                mask_gray=ball_state_dict.get("mask_gray"),
            )
            self.snapshot_ready.emit(snapshot)
        except Exception as exc:
            self.error.emit(str(exc))
