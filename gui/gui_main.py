import sys
import time
from collections import deque

import cv2
from PyQt5 import QtCore, QtGui
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QApplication

from stewart_control.comms.serial_sender import SerialSender
from stewart_control.config import (
    CAMERA_INDEX,
    GUI_LOG_MAX_LINES,
    LOG_EVERY_N,
    SERIAL_PORT,
    TIMING_PLOT_POINTS,
    VISUALIZER_HZ,
    VISION_LOOP_HZ,
)
from stewart_control.cv.vision_control_worker import VisionControlWorker
from stewart_control.gui.gui_layout import StewartGUIView
from stewart_control.routines.routines import ROUTINES
from stewart_control.visualization.visualizer3d import StewartVisualizer


class _DefaultIKSolver:
    @staticmethod
    def solve_pose(x, y, z, roll, pitch, yaw, prev_arm_points=None):
        from stewart_control.kinematics import ik_solver

        return ik_solver.solve_pose(x, y, z, roll, pitch, yaw, prev_arm_points)


class StewartGUIController(StewartGUIView):
    serial_line_received = pyqtSignal(str)
    vision_gains_updated = pyqtSignal(float, float)
    vision_hsv_updated = pyqtSignal(int, int, int, int, int, int)

    def __init__(self, ik_solver=None):
        self.ik_solver = ik_solver or _DefaultIKSolver()
        super().__init__()

        self.preview_mode = False
        self.current_routine_steps = []
        self.current_routine_index = 0
        self.vision_enabled = False
        self._last_visualizer_update = 0.0
        self._vision_z_setpoint = 0.0
        self._timing_plot_update_every = 5
        self._vision_counter = 0
        self._y_zoom = 1.0
        self._last_neutral_send = 0.0

        self._vision_thread = None
        self._vision_worker = None

        self._timing_keys = ["ball_update", "pd_compute", "ik_solve", "serial_enqueue", "visualizer_gui", "total"]
        self._tracker_keys = ["trk_capture", "trk_aruco", "trk_h", "trk_warp", "trk_hsv", "trk_contour", "trk_kin", "trk_overlay"]
        self._timing_colors = {
            "ball_update": "#38bdf8",
            "pd_compute": "#34d399",
            "ik_solve": "#f43f5e",
            "serial_enqueue": "#a78bfa",
            "visualizer_gui": "#f59e0b",
            "total": "#e2e8f0",
        }
        timing_capacity = max(TIMING_PLOT_POINTS, int(VISION_LOOP_HZ * 35))
        self._timing_history = {k: deque(maxlen=timing_capacity) for k in self._timing_keys}
        self._timing_timestamps = deque(maxlen=timing_capacity)
        self._timing_window_s = 30.0

        self.serial = SerialSender(SERIAL_PORT)
        self.serial_line_received.connect(self.append_serial_line)
        self.serial.connect()
        self.serial.set_receive_callback(lambda line: self.serial_line_received.emit(line))

        self.visualizer = StewartVisualizer(self.visualizer_canvas)
        self._init_timing_plot_style()
        self._init_signals()
        self._init_routines()
        self._set_camera_enabled(False)
        self.monitor_window.show()

    def _init_signals(self):
        for axis, (lbl, sld) in self.sliders.items():
            sld.valueChanged.connect(lambda val, a=axis, l=lbl: self._on_pose_slider(a, val, l))
        for name, (lbl, sld) in self.hsv_controls.items():
            sld.valueChanged.connect(lambda val, n=name, l=lbl: self._on_hsv_slider(n, val, l))

        self.kp_slider.valueChanged.connect(self.update_pd_gains)
        self.kd_slider.valueChanged.connect(self.update_pd_gains)
        self.send_button.clicked.connect(self.send_to_arduino)
        self.raw_serial_send_btn.clicked.connect(self.send_raw_serial_command)
        self.demo_list.currentIndexChanged.connect(self.on_routine_changed)
        self.vision_button.clicked.connect(self.toggle_vision_mode)
        self.cancel_vision_btn.clicked.connect(self.disable_vision_mode)
        self.cancel_routine_btn.clicked.connect(self.cancel_routine_preview)
        self.open_monitor_button.clicked.connect(self.monitor_window.showNormal)

        self.timing_canvas.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.timing_canvas.grabGesture(QtCore.Qt.PinchGesture)
        self.timing_canvas.installEventFilter(self)
        self.timing_canvas.mpl_connect("scroll_event", self._on_timing_scroll)

    def eventFilter(self, obj, event):
        if obj is self.timing_canvas and event.type() == QtCore.QEvent.Gesture:
            pinch = event.gesture(QtCore.Qt.PinchGesture)
            if pinch is not None:
                self._apply_y_zoom(1.0 / float(pinch.scaleFactor()))
                return True
        return super().eventFilter(obj, event)

    def _on_timing_scroll(self, event):
        if event.button == "up":
            self._apply_y_zoom(0.9)
        elif event.button == "down":
            self._apply_y_zoom(1.1)

    def _apply_y_zoom(self, scale):
        self._y_zoom *= float(scale)
        self._y_zoom = max(0.2, min(10.0, self._y_zoom))
        self._update_timing_diagnostics(force_redraw=True)

    def _init_routines(self):
        for name in ROUTINES.keys():
            self.demo_list.addItem(name)

    def _init_timing_plot_style(self):
        self.timing_figure.patch.set_facecolor("#0f1726")
        self.timing_ax.set_facecolor("#0f1726")
        self.timing_ax.set_title("Vision Loop Step Timings (Last 30s)", color="#d6e2ff")
        self.timing_ax.tick_params(colors="#9fb4d9")
        self.timing_ax.grid(True, alpha=0.2, color="#2a3b59")

    def _log_preview(self, msg):
        self.preview_output.append(msg)
        self._trim_text_widget(self.preview_output)

    def _log_serial(self, msg):
        self.serial_monitor.append(msg)
        self._trim_text_widget(self.serial_monitor)

    @staticmethod
    def _trim_text_widget(widget):
        doc = widget.document()
        while doc.blockCount() > GUI_LOG_MAX_LINES:
            cursor = QtGui.QTextCursor(doc)
            cursor.movePosition(QtGui.QTextCursor.Start)
            cursor.select(QtGui.QTextCursor.BlockUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()

    def append_serial_line(self, line):
        self._log_serial(line)

    def _on_pose_slider(self, axis, value, label):
        label.setText(f"{axis}: {value}")
        self._vision_z_setpoint = float(self.sliders["Z"][1].value())
        self.visualizer.update_platform(self._current_pose_from_sliders())

    def _on_hsv_slider(self, name, value, label):
        label.setText(f"{name}: {value}")
        self._emit_hsv_values()

    def _emit_hsv_values(self):
        values = [self.hsv_controls[k][1].value() for k in ["H Min", "H Max", "S Min", "S Max", "V Min", "V Max"]]
        self.vision_hsv_updated.emit(*values)

    def _current_pose_from_sliders(self):
        return {
            "x": self.sliders["X"][1].value(),
            "y": self.sliders["Y"][1].value(),
            "z": self.sliders["Z"][1].value(),
            "roll": self.sliders["Roll"][1].value(),
            "pitch": self.sliders["Pitch"][1].value(),
            "yaw": self.sliders["Yaw"][1].value(),
        }

    def _set_pose_sliders_enabled(self, enabled):
        for _, sld in self.sliders.values():
            sld.setEnabled(enabled)

    def _set_camera_enabled(self, enabled):
        labels = [
            self.monitor_window.camera_view_label,
            self.monitor_window.warped_view_label,
            self.monitor_window.hsv_view_label,
        ]
        titles = ["Camera View", "Warped Vector View", "HSV/Mask View"]
        for lbl, title in zip(labels, titles):
            if enabled:
                lbl.setText(f"{title} (Waiting...)")
                lbl.setStyleSheet("border:1px solid #2a3b59; background:#0f1726;")
            else:
                lbl.clear()
                lbl.setText(f"{title} (Disabled)")
                lbl.setStyleSheet("border:1px solid #2a3b59; background:#060b14; color:#5f738f;")

    @staticmethod
    def _cv_to_pixmap(img, target_w, target_h):
        if img is None:
            return None
        if len(img.shape) == 2:
            rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        else:
            rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qimg = QtGui.QImage(rgb.data, w, h, ch * w, QtGui.QImage.Format_RGB888)
        return QtGui.QPixmap.fromImage(qimg).scaled(
            target_w, target_h, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation
        )

    def _update_camera_views(self, snapshot):
        for widget, frame in [
            (self.monitor_window.camera_view_label, snapshot.camera_bgr),
            (self.monitor_window.warped_view_label, snapshot.warped_bgr),
            (self.monitor_window.hsv_view_label, snapshot.mask_gray),
        ]:
            pix = self._cv_to_pixmap(frame, widget.width(), widget.height())
            if pix is not None:
                widget.setPixmap(pix)

    def update_pd_gains(self):
        kp = self.kp_slider.value() / 1000.0
        kd = self.kd_slider.value() / 1000.0
        self.kp_label.setText(f"Kp: {kp:.3f}")
        self.kd_label.setText(f"Kd: {kd:.3f}")
        self.vision_gains_updated.emit(kp, kd)

    def send_raw_serial_command(self):
        cmd = self.raw_serial_input.text().strip()
        if not cmd:
            return
        self.serial.send_command(cmd.encode())
        self._log_serial(f">>> {cmd}")

    def on_routine_changed(self):
        routine_name = self.demo_list.currentText()
        if routine_name == "(Choose a routine...)":
            self.cancel_routine_preview()
            return
        if routine_name not in ROUTINES:
            return
        self.start_routine_preview(routine_name)

    def start_routine_preview(self, routine_name):
        poses = ROUTINES[routine_name]()
        if not poses:
            return
        self.preview_mode = True
        self.current_routine_steps = poses
        self.current_routine_index = 0
        self.cancel_routine_btn.setEnabled(True)
        self._set_pose_sliders_enabled(False)
        self._log_preview(f"[INFO] Previewing routine: {routine_name}")
        self._routine_timer_start()

    def _routine_timer_start(self):
        if not hasattr(self, "_routine_timer"):
            self._routine_timer = QtCore.QTimer(self)
            self._routine_timer.timeout.connect(self._routine_step)
        self._routine_timer.start(20)

    def _routine_timer_stop(self):
        if hasattr(self, "_routine_timer"):
            self._routine_timer.stop()

    def _routine_step(self):
        if not self.preview_mode or not self.current_routine_steps:
            return
        pose = self.current_routine_steps[self.current_routine_index]
        self.current_routine_index = (self.current_routine_index + 1) % len(self.current_routine_steps)
        for key, (lbl, sld) in self.sliders.items():
            pose_key = key.lower()
            sld.blockSignals(True)
            sld.setValue(int(pose[pose_key]))
            lbl.setText(f"{key}: {int(pose[pose_key])}")
            sld.blockSignals(False)
        self.visualizer.update_platform(pose)

    def cancel_routine_preview(self):
        self.preview_mode = False
        self.current_routine_steps = []
        self.current_routine_index = 0
        self.cancel_routine_btn.setEnabled(False)
        self._routine_timer_stop()
        if not self.vision_enabled:
            self._set_pose_sliders_enabled(True)

    def send_to_arduino(self):
        pose = self._current_pose_from_sliders()
        ik_result = self.ik_solver.solve_pose(
            pose["x"], pose["y"], pose["z"], pose["roll"], pose["pitch"], pose["yaw"], prev_arm_points=None
        )
        if not ik_result.get("success", False):
            self._log_preview("[WARN] IK failed.")
            return
        angles = [int(round(a)) for a in ik_result["servo_angles_deg"]]
        cmd = "S," + ",".join(str(a) for a in angles) + ",0\n"
        self.serial.send_command(cmd.encode())
        self._log_preview(f"[SEND] {cmd.strip()}")

    def toggle_vision_mode(self):
        if self.vision_enabled:
            self.disable_vision_mode()
        else:
            self.enable_vision_mode()

    def enable_vision_mode(self):
        if self.vision_enabled:
            return
        self.vision_enabled = True
        self._vision_counter = 0
        self._last_visualizer_update = 0.0
        self._set_pose_sliders_enabled(False)
        self.cancel_vision_btn.setEnabled(True)
        self.vision_button.setText("Vision ACTIVE")
        self._set_camera_enabled(True)
        self.monitor_window.showNormal()
        self.monitor_window.raise_()
        self.monitor_window.activateWindow()
        for key in self._timing_keys:
            self._timing_history[key].clear()
        self._timing_timestamps.clear()
        self._start_vision_worker()
        self._log_preview("[INFO] Vision mode enabled.")

    def disable_vision_mode(self):
        if not self.vision_enabled and self._vision_worker is None:
            return
        self.vision_enabled = False
        self.cancel_vision_btn.setEnabled(False)
        self.vision_button.setText("Stopping Vision...")
        self._set_camera_enabled(False)
        self._stop_vision_worker_async()
        self.timing_summary_label.setText("Vision Timing Avg (ms): waiting for data...")

    def _start_vision_worker(self):
        if self._vision_thread is not None:
            return
        self._vision_thread = QtCore.QThread(self)
        self._vision_worker = VisionControlWorker(
            ik_solver=self.ik_solver,
            z_provider=lambda: self._vision_z_setpoint,
            kp=self.kp_slider.value() / 1000.0,
            kd=self.kd_slider.value() / 1000.0,
            max_tilt_deg=8.0,
            camera_index=CAMERA_INDEX,
            loop_hz=VISION_LOOP_HZ,
            emit_camera_every_n=1,
        )
        self._vision_worker.moveToThread(self._vision_thread)
        self._vision_thread.finished.connect(self._on_vision_thread_finished)
        self._vision_thread.started.connect(self._vision_worker.start)
        self._vision_worker.snapshot_ready.connect(self._on_control_snapshot)
        self._vision_worker.error.connect(self._on_vision_error)
        self._vision_worker.stopped.connect(self._on_vision_worker_stopped)
        self.vision_gains_updated.connect(self._vision_worker.set_gains)
        self.vision_hsv_updated.connect(self._vision_worker.set_hsv)
        self._vision_thread.start()
        self.update_pd_gains()
        self._emit_hsv_values()

    def _stop_vision_worker_async(self):
        if self._vision_worker is None:
            self._on_vision_worker_stopped()
            return
        try:
            self.vision_gains_updated.disconnect(self._vision_worker.set_gains)
        except TypeError:
            pass
        try:
            self.vision_hsv_updated.disconnect(self._vision_worker.set_hsv)
        except TypeError:
            pass
        QtCore.QMetaObject.invokeMethod(self._vision_worker, "stop", QtCore.Qt.QueuedConnection)

    def _on_vision_worker_stopped(self):
        if self._vision_thread is not None:
            self._vision_thread.quit()
        else:
            self._on_vision_thread_finished()

    def _on_vision_thread_finished(self):
        if self._vision_thread is not None:
            self._vision_thread.deleteLater()
        self._vision_thread = None
        self._vision_worker = None
        self.vision_button.setText("Enable Vision Mode")
        if not self.preview_mode:
            self._set_pose_sliders_enabled(True)
        self._log_preview("[INFO] Vision mode disabled.")

    def _on_vision_error(self, msg):
        self._log_preview(f"[VISION ERROR] {msg}")

    def _on_control_snapshot(self, snapshot):
        if not self.vision_enabled:
            return

        if not snapshot.tracking_valid:
            now = time.perf_counter()
            if now - self._last_neutral_send > 0.2:
                neutral_pose = {
                    "x": 0.0,
                    "y": 0.0,
                    "z": float(self._vision_z_setpoint),
                    "roll": 0.0,
                    "pitch": 0.0,
                    "yaw": 0.0,
                }
                ik = self.ik_solver.solve_pose(
                    neutral_pose["x"],
                    neutral_pose["y"],
                    neutral_pose["z"],
                    neutral_pose["roll"],
                    neutral_pose["pitch"],
                    neutral_pose["yaw"],
                    prev_arm_points=None,
                )
                if ik.get("success", False):
                    safe_angles = [max(0, min(180, int(round(a)))) for a in ik["servo_angles_deg"]]
                    cmd = "S," + ",".join(str(a) for a in safe_angles) + ",0\n"
                    self.serial.enqueue_command(cmd, policy="latest")
                    self._last_neutral_send = now
            self._vision_counter += 1
            if self._vision_counter % LOG_EVERY_N == 0:
                self._log_preview("[TRACK] no ball detected -> neutral hold")
            return

        if not snapshot.ik_success:
            self._vision_counter += 1
            if self._vision_counter % LOG_EVERY_N == 0:
                self._log_preview("[WARN] IK failed in vision loop.")
            return

        safe_angles = [max(0, min(180, int(a))) for a in snapshot.servo_angles]
        cmd = "S," + ",".join(str(a) for a in safe_angles) + ",0\n"
        t_serial0 = time.perf_counter()
        self.serial.enqueue_command(cmd, policy="latest")
        t_serial1 = time.perf_counter()

        vis_ms = 0.0
        now_perf = time.perf_counter()
        if now_perf - self._last_visualizer_update >= (1.0 / max(1, VISUALIZER_HZ)):
            self._last_visualizer_update = now_perf
            t_vis0 = time.perf_counter()
            self.visualizer.update_platform(snapshot.pose, ik_result=snapshot.ik_result)
            t_vis1 = time.perf_counter()
            vis_ms = (t_vis1 - t_vis0) * 1000.0

        timings = dict(snapshot.timings_ms)
        timings["serial_enqueue"] = (t_serial1 - t_serial0) * 1000.0
        timings["visualizer_gui"] = vis_ms

        now = time.time()
        self._timing_timestamps.append(now)
        for key in self._timing_keys:
            self._timing_history[key].append(float(timings.get(key, 0.0)))
        for key in self._tracker_keys:
            self._timing_history.setdefault(key, deque(maxlen=self._timing_timestamps.maxlen))
            self._timing_history[key].append(float(timings.get(key, 0.0)))
        self._trim_timing_history(now)

        self._update_camera_views(snapshot)
        self._update_timing_diagnostics()
        self._vision_counter += 1

        if self._vision_counter % LOG_EVERY_N == 0:
            bs = snapshot.ball_state
            terms = snapshot.control_terms
            self._log_preview(
                "[CONTROL TRACE] "
                f"ball=({bs.x_mm:.2f},{bs.y_mm:.2f})mm "
                f"vel=({bs.vx_mm_s:.2f},{bs.vy_mm_s:.2f})mm/s "
                f"pos_err=({terms['position_vec_mm'][0]:.2f},{terms['position_vec_mm'][1]:.2f}) "
                f"pd_vec=({terms['pd_vec'][0]:.4f},{terms['pd_vec'][1]:.4f}) "
                f"pose=(r{snapshot.pose['roll']:.3f},p{snapshot.pose['pitch']:.3f},z{snapshot.pose['z']:.2f}) "
                f"cmd={safe_angles}"
            )

    def _trim_timing_history(self, now):
        while self._timing_timestamps and (now - self._timing_timestamps[0]) > self._timing_window_s:
            self._timing_timestamps.popleft()
            for key in self._timing_keys:
                if self._timing_history[key]:
                    self._timing_history[key].popleft()
            for key in self._tracker_keys:
                if key in self._timing_history and self._timing_history[key]:
                    self._timing_history[key].popleft()

    def _update_timing_diagnostics(self, force_redraw=False):
        if not self._timing_history["total"]:
            return
        avgs = {}
        for key in self._timing_keys:
            values = self._timing_history[key]
            avgs[key] = (sum(values) / len(values)) if values else 0.0
        self.timing_summary_label.setText(
            "Vision Timing Avg (ms): "
            f"ball={avgs['ball_update']:.2f}, pd={avgs['pd_compute']:.2f}, "
            f"ik={avgs['ik_solve']:.2f}, serQ={avgs['serial_enqueue']:.2f}, "
            f"vis={avgs['visualizer_gui']:.2f}, total={avgs['total']:.2f}, zoom={self._y_zoom:.2f}x"
        )

        if self._vision_counter % LOG_EVERY_N == 0:
            trk = {}
            for k in self._tracker_keys:
                values = self._timing_history.get(k, [])
                trk[k] = (sum(values) / len(values)) if values else 0.0
            self._log_preview(
                "[TRACKER AVG] "
                f"fetch={trk['trk_capture']:.2f}ms "
                f"aruco={trk['trk_aruco']:.2f}ms "
                f"H={trk['trk_h']:.2f}ms "
                f"warp={trk['trk_warp']:.2f}ms "
                f"hsv={trk['trk_hsv']:.2f}ms "
                f"contour={trk['trk_contour']:.2f}ms "
                f"kin={trk['trk_kin']:.2f}ms "
                f"overlay={trk['trk_overlay']:.2f}ms"
            )

        if not force_redraw and (self._vision_counter % self._timing_plot_update_every != 0):
            return
        if not self._timing_timestamps:
            return

        now = self._timing_timestamps[-1]
        x = [t - now for t in self._timing_timestamps]
        self.timing_ax.cla()
        self.timing_ax.set_facecolor("#0f1726")
        self.timing_ax.set_title("Vision Loop Step Timings (Last 30s)", color="#d6e2ff")
        self.timing_ax.tick_params(colors="#9fb4d9")
        self.timing_ax.grid(True, alpha=0.2, color="#2a3b59")
        self.timing_ax.set_xlim(-self._timing_window_s, 0.0)
        self.timing_ax.set_xlabel("Time (s, rolling)", color="#9fb4d9")
        self.timing_ax.set_ylabel("ms", color="#9fb4d9")

        y_max = 1.0
        for key in self._timing_keys:
            y = list(self._timing_history[key])
            if y:
                self.timing_ax.plot(x, y, label=key, linewidth=1.4, color=self._timing_colors[key])
                y_max = max(y_max, max(y))
        self.timing_ax.set_ylim(0.0, y_max * self._y_zoom)
        self.timing_ax.legend(loc="upper right", fontsize=7)
        self.timing_canvas.draw_idle()

    def closeEvent(self, event):
        self.cancel_routine_preview()
        self.disable_vision_mode()
        self.serial.disconnect()
        self.monitor_window.close()
        super().closeEvent(event)


StewartGUILayout = StewartGUIController


def run_gui():
    app = QApplication(sys.argv)
    window = StewartGUIController()
    window.show()
    sys.exit(app.exec_())
