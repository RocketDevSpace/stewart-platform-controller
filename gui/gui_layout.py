from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from stewart_control.comms.serial_sender import SerialSender
from stewart_control.config import (
    CAMERA_DEBUG_WINDOWS,
    CAMERA_INDEX,
    DEBUG_LEVEL,
    GUI_LOG_MAX_LINES,
    LOG_EVERY_N,
    SERIAL_PORT,
    TIMING_PLOT_POINTS,
    VISUALIZER_HZ,
    VISION_LOOP_HZ,
)
from stewart_control.cv.vision_control_worker import VisionControlWorker
from stewart_control.routines.routines import ROUTINES
from stewart_control.visualization.visualizer3d import StewartVisualizer

import time
from collections import deque


class _DefaultIKSolver:
    @staticmethod
    def solve_pose(x, y, z, roll, pitch, yaw, prev_arm_points=None):
        from stewart_control.kinematics import ik_solver

        return ik_solver.solve_pose(x, y, z, roll, pitch, yaw, prev_arm_points)


class StewartGUILayout(QWidget):
    serial_line_received = pyqtSignal(str)
    vision_gains_updated = pyqtSignal(float, float)

    def __init__(self, ik_solver=None):
        super().__init__()
        self.ik_solver = ik_solver or _DefaultIKSolver()

        self.preview_mode = False
        self.current_routine_steps = []
        self.current_routine_index = 0
        self.visualizer_update_skip = 5
        self.vision_enabled = False
        self._vision_counter = 0
        self._last_visualizer_update = 0.0
        self._timing_plot_update_every = 5

        self._routine_timer_mode = None
        self._routine_send_handler = None
        self._routine_send_index = 0

        self._vision_thread = None
        self._vision_worker = None
        self._timing_keys = [
            "ball_update",
            "pd_compute",
            "ik_solve",
            "serial_enqueue",
            "visualizer_gui",
            "total",
        ]
        self._timing_colors = {
            "ball_update": "#1f77b4",
            "pd_compute": "#2ca02c",
            "ik_solve": "#d62728",
            "serial_enqueue": "#9467bd",
            "visualizer_gui": "#ff7f0e",
            "total": "#111111",
        }
        self._timing_history = {k: deque(maxlen=TIMING_PLOT_POINTS) for k in self._timing_keys}

        self.routine_timer = QTimer()
        self.routine_timer.setTimerType(QtCore.Qt.PreciseTimer)
        self.routine_timer.timeout.connect(self._on_routine_timer_tick)

        self.init_ui()
        self._vision_z_setpoint = float(self.sliders["Z"].value())
        self.send_button.clicked.connect(self.send_to_arduino)

        self.visualizer = StewartVisualizer(self.canvas)

        self.serial_line_received.connect(self.append_serial_line)
        self.serial = SerialSender(SERIAL_PORT)
        self.serial.connect()
        self.serial.set_receive_callback(lambda line: self.serial_line_received.emit(line))

    def _log_preview(self, msg):
        self.preview_output.append(msg)
        self._trim_text_widget(self.preview_output)

    def _log_serial(self, msg):
        self.serial_monitor.append(msg)
        self._trim_text_widget(self.serial_monitor)

    def _trim_text_widget(self, widget):
        doc = widget.document()
        overflow = doc.blockCount() - GUI_LOG_MAX_LINES
        if overflow <= 0:
            return
        cursor = QtGui.QTextCursor(doc)
        cursor.movePosition(QtGui.QTextCursor.Start)
        for _ in range(overflow):
            cursor.select(QtGui.QTextCursor.BlockUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()

    def append_serial_line(self, line):
        self._log_serial(line)

    def init_ui(self):
        main_layout = QHBoxLayout()

        slider_layout = QVBoxLayout()
        self.sliders = {}
        axes = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
        for ax in axes:
            lbl = QLabel(f"{ax}: 0")
            sld = QSlider(QtCore.Qt.Horizontal)
            sld.setMinimum(-100)
            sld.setMaximum(100)
            sld.setValue(0)
            sld.valueChanged.connect(lambda val, a=ax, l=lbl: self.update_slider(a, val, l))
            slider_layout.addWidget(lbl)
            slider_layout.addWidget(sld)
            self.sliders[ax] = sld

        control_layout = QVBoxLayout()
        self.demo_list = QtWidgets.QComboBox()
        self.demo_list.addItem("(Choose a routine...)")
        for name in ROUTINES.keys():
            self.demo_list.addItem(name)
        control_layout.addWidget(QLabel("Demo Routines:"))
        control_layout.addWidget(self.demo_list)

        self.cancel_routine_btn = QPushButton("Cancel Routine")
        self.cancel_routine_btn.setStyleSheet("background-color: darkred; color: white; font-weight: bold;")
        self.cancel_routine_btn.clicked.connect(self.cancel_routine_preview)
        self.cancel_routine_btn.setEnabled(False)
        control_layout.addWidget(self.cancel_routine_btn)

        control_layout.addWidget(QLabel("Ball Balancing Control"))
        self.vision_button = QPushButton("Enable Vision Mode")
        self.vision_button.setStyleSheet("background-color: darkgreen; color: white;")
        self.vision_button.clicked.connect(self.toggle_vision_mode)
        control_layout.addWidget(self.vision_button)

        self.cancel_vision_btn = QPushButton("Cancel Vision Mode")
        self.cancel_vision_btn.setStyleSheet("background-color: darkred; color: white;")
        self.cancel_vision_btn.clicked.connect(self.disable_vision_mode)
        self.cancel_vision_btn.setEnabled(False)
        control_layout.addWidget(self.cancel_vision_btn)

        self.kp_label = QLabel("Kp: 0.05")
        self.kp_slider = QSlider(QtCore.Qt.Horizontal)
        self.kp_slider.setMinimum(0)
        self.kp_slider.setMaximum(100)
        self.kp_slider.setValue(5)
        self.kp_slider.valueChanged.connect(self.update_pd_gains)
        control_layout.addWidget(self.kp_label)
        control_layout.addWidget(self.kp_slider)

        self.kd_label = QLabel("Kd: 0.010")
        self.kd_slider = QSlider(QtCore.Qt.Horizontal)
        self.kd_slider.setMinimum(0)
        self.kd_slider.setMaximum(100)
        self.kd_slider.setValue(10)
        self.kd_slider.valueChanged.connect(self.update_pd_gains)
        control_layout.addWidget(self.kd_label)
        control_layout.addWidget(self.kd_slider)

        self.send_button = QPushButton("SEND TO ARDUINO")
        self.send_button.setStyleSheet("background-color: red; color: white; font-size: 16pt;")
        control_layout.addWidget(self.send_button)

        self.demo_list.currentIndexChanged.connect(self.on_routine_changed)

        self.preview_output = QTextEdit()
        self.preview_output.setReadOnly(True)
        self.preview_output.setPlaceholderText("Command preview will appear here...")
        self.preview_output.textChanged.connect(lambda: self._trim_text_widget(self.preview_output))
        control_layout.addWidget(QLabel("Command Preview:"))
        control_layout.addWidget(self.preview_output)

        self.raw_serial_input = QtWidgets.QLineEdit()
        self.raw_serial_input.setPlaceholderText("Type raw serial command here, e.g. S,90,90,90,90,90,90")
        control_layout.addWidget(QLabel("Direct Serial Command:"))
        control_layout.addWidget(self.raw_serial_input)

        self.raw_serial_send_btn = QPushButton("Send Command")
        self.raw_serial_send_btn.clicked.connect(self.send_raw_serial_command)
        control_layout.addWidget(self.raw_serial_send_btn)

        self.serial_monitor = QTextEdit()
        self.serial_monitor.setReadOnly(True)
        self.serial_monitor.setPlaceholderText("Arduino Serial Monitor output will appear here...")
        self.serial_monitor.textChanged.connect(lambda: self._trim_text_widget(self.serial_monitor))
        control_layout.addWidget(QLabel("Serial Monitor:"))
        control_layout.addWidget(self.serial_monitor)

        self.timing_summary_label = QLabel("Vision Timing Avg (ms): waiting for data...")
        self.timing_summary_label.setWordWrap(True)
        control_layout.addWidget(self.timing_summary_label)

        self.timing_figure = Figure(figsize=(5, 2.2))
        self.timing_canvas = FigureCanvas(self.timing_figure)
        self.timing_canvas.setFixedHeight(220)
        self.timing_ax = self.timing_figure.add_subplot(111)
        self.timing_ax.set_title("Vision Loop Step Timings (ms)")
        self.timing_ax.set_xlabel("Samples")
        self.timing_ax.set_ylabel("ms")
        control_layout.addWidget(self.timing_canvas)

        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setFixedSize(500, 500)

        main_layout.addLayout(slider_layout)
        main_layout.addLayout(control_layout)
        main_layout.addWidget(self.canvas)
        self.setLayout(main_layout)

    def _on_routine_timer_tick(self):
        if self._routine_timer_mode == "preview":
            self._routine_step()
        elif self._routine_timer_mode == "send" and self._routine_send_handler is not None:
            self._routine_send_handler()

    def _set_routine_timer_mode(self, mode):
        self._routine_timer_mode = mode
        if mode != "send":
            self._routine_send_handler = None

    def send_raw_serial_command(self):
        cmd = self.raw_serial_input.text().strip()
        if not cmd:
            self._log_preview("[WARN] No command entered.")
            return

        confirm = QtWidgets.QMessageBox.question(
            self,
            "Confirm Send",
            f"Send the following command to Arduino?\n{cmd}",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
        )
        if confirm != QtWidgets.QMessageBox.Yes:
            self._log_preview("[INFO] Send canceled by user.")
            return

        self.serial.send_command(cmd.encode())
        self._log_serial(f">>> {cmd}")
        self._log_preview(f"[INFO] Command sent: {cmd}")

    def run_selected_routine(self):
        routine_name = self.demo_list.currentText()
        if routine_name not in ROUTINES:
            self._log_preview("[ERROR] Routine not found.")
            return

        poses = ROUTINES[routine_name]()
        self._log_preview(f"[INFO] Running routine: {routine_name}")
        self._log_preview(f"[INFO] Steps: {len(poses)}")
        self.current_routine_steps = poses
        self.current_routine_index = 0
        self._set_routine_timer_mode("preview")
        self.routine_timer.start(20)

    def on_routine_changed(self):
        routine_name = self.demo_list.currentText()
        if routine_name == "(Choose a routine...)":
            self.cancel_routine_preview()
            return
        if routine_name not in ROUTINES:
            self._log_preview("[ERROR] Routine not found.")
            return
        self.start_routine_preview(routine_name)

    def start_routine_preview(self, routine_name):
        self._log_preview(f"[INFO] Previewing routine: {routine_name}")

        current_pose = {
            "x": self.sliders["X"].value(),
            "y": self.sliders["Y"].value(),
            "z": self.sliders["Z"].value(),
            "roll": self.sliders["Roll"].value(),
            "pitch": self.sliders["Pitch"].value(),
            "yaw": self.sliders["Yaw"].value(),
        }
        ik_seed = self.ik_solver.solve_pose(
            current_pose["x"],
            current_pose["y"],
            current_pose["z"],
            current_pose["roll"],
            current_pose["pitch"],
            current_pose["yaw"],
            prev_arm_points=self.visualizer.prev_arm_points,
        )
        self.visualizer.prev_arm_points = ik_seed.get("arm_points", None)

        poses = ROUTINES[routine_name]()
        if not poses or len(poses) < 2:
            self._log_preview("[ERROR] Routine has no poses.")
            return

        self.preview_mode = True
        self.current_routine_steps = poses
        self.current_routine_index = 0
        for slider in self.sliders.values():
            slider.setEnabled(False)

        self.cancel_routine_btn.setEnabled(True)
        self._set_routine_timer_mode("preview")
        self.routine_timer.start(20)

    def cancel_routine_preview(self):
        if self.preview_mode:
            self._log_preview("[INFO] Routine preview cancelled.")

        self.preview_mode = False
        self.routine_timer.stop()
        self._set_routine_timer_mode(None)

        self.current_routine_steps = []
        self.current_routine_index = 0

        if not self.vision_enabled:
            for slider in self.sliders.values():
                slider.setEnabled(True)

        self.send_button.setEnabled(True)
        self.cancel_routine_btn.setEnabled(False)

        if self.demo_list.currentText() != "(Choose a routine...)":
            self.demo_list.blockSignals(True)
            self.demo_list.setCurrentIndex(0)
            self.demo_list.blockSignals(False)

        pose = self._current_pose_from_sliders()
        self.visualizer.update_platform(pose)

    def _routine_step(self):
        if not self.preview_mode or len(self.current_routine_steps) == 0:
            return

        pose = self.current_routine_steps[self.current_routine_index]
        self.current_routine_index += 1
        if self.current_routine_index >= len(self.current_routine_steps):
            self.current_routine_index = 0

        self._set_slider_pose(pose)
        self.visualizer.update_platform(pose)

    def _set_slider_pose(self, pose):
        keys = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
        pose_keys = ["x", "y", "z", "roll", "pitch", "yaw"]
        for ax in keys:
            self.sliders[ax].blockSignals(True)
        for ax, key in zip(keys, pose_keys):
            self.sliders[ax].setValue(int(pose[key]))
        self._vision_z_setpoint = float(pose["z"])
        for ax in keys:
            self.sliders[ax].blockSignals(False)

    def _current_pose_from_sliders(self):
        return {
            "x": self.sliders["X"].value(),
            "y": self.sliders["Y"].value(),
            "z": self.sliders["Z"].value(),
            "roll": self.sliders["Roll"].value(),
            "pitch": self.sliders["Pitch"].value(),
            "yaw": self.sliders["Yaw"].value(),
        }

    def update_slider(self, axis, value, label):
        label.setText(f"{axis}: {value}")
        self._vision_z_setpoint = float(self.sliders["Z"].value())
        self.visualizer.update_platform(self._current_pose_from_sliders())

    def send_pose_direct(self, pose):
        try:
            ik_result = self.ik_solver.solve_pose(
                pose["x"],
                pose["y"],
                pose["z"],
                pose["roll"],
                pose["pitch"],
                pose["yaw"],
                prev_arm_points=self.visualizer.prev_arm_points,
            )
            if not ik_result["success"]:
                self._log_preview("[WARN] IK failed at this step.")
                return

            angles = [int(round(a)) for a in ik_result["servo_angles_deg"]]
            cmd = "S," + ",".join(str(a) for a in angles) + ",0\n"
            self._log_preview(f"[SEND] {cmd.strip()}")
            self.serial.send_command(cmd.encode())
        except Exception as e:
            self._log_preview(f"[ERROR] Routine send failed: {e}")

    def send_to_arduino(self):
        if self.preview_mode:
            self.routine_timer.stop()
            self._set_routine_timer_mode(None)
            self._log_preview("[PROFILE] Building routine command list...")
            t_build_start = time.perf_counter()
            all_commands = []
            ik_fail_count = 0

            for pose in self.current_routine_steps:
                try:
                    ik_result = self.ik_solver.solve_pose(
                        pose["x"],
                        pose["y"],
                        pose["z"],
                        pose["roll"],
                        pose["pitch"],
                        pose["yaw"],
                        prev_arm_points=self.visualizer.prev_arm_points,
                    )
                    if not ik_result["success"]:
                        ik_fail_count += 1
                        continue

                    angles = [int(round(a)) for a in ik_result["servo_angles_deg"]]
                    safe_angles, clipped = self.safety_clip_servos(angles)
                    cmd = "S," + ",".join(str(a) for a in safe_angles) + ",0\n"
                    if clipped:
                        for idx, original, new in clipped:
                            self._log_preview(
                                f'<span style="color:red;">[SAFETY CLIP] Servo {idx}: {original} -> {new}</span>'
                            )
                    all_commands.append((cmd, pose))
                except Exception as e:
                    ik_fail_count += 1
                    self._log_preview(f"[ERROR] Routine IK calculation failed: {e}")

            build_ms = (time.perf_counter() - t_build_start) * 1000
            self._log_preview(
                f"[PROFILE] Routine command generation complete: {len(all_commands)} valid steps, "
                f"{ik_fail_count} failed. Build time = {build_ms:.2f} ms"
            )
            if not all_commands:
                self._log_preview("[ERROR] No valid servo commands generated for routine.")
                return

            cmds_text = "\n".join(c.strip() for c, _ in all_commands)
            confirmed = self.confirm_large_text(
                "Confirm Send Routine",
                f"Send the following {len(all_commands)} servo commands to Arduino?",
                cmds_text,
            )
            if not confirmed:
                self._log_preview("[INFO] Routine send canceled by user.")
                return

            for slider in self.sliders.values():
                slider.setEnabled(False)
            self.send_button.setEnabled(False)
            self.cancel_routine_btn.setEnabled(True)

            self._routine_send_index = 0
            self._profile_last_tick = None
            self._profile_total_time = 0.0
            self._profile_step_count = 0

            def send_next_step():
                step_total_start = time.perf_counter()
                tick_now = time.perf_counter()
                if self._profile_last_tick is not None:
                    dt_ms = (tick_now - self._profile_last_tick) * 1000
                else:
                    dt_ms = None
                self._profile_last_tick = tick_now

                if self._routine_send_index >= len(all_commands):
                    self._log_preview("[INFO] Routine sent successfully.")
                    if self._profile_step_count > 0:
                        avg_step_ms = (self._profile_total_time / self._profile_step_count) * 1000
                        hz = 1000.0 / avg_step_ms if avg_step_ms > 0 else 0
                        self._log_preview(
                            f"[PROFILE] Routine complete. Avg step time = {avg_step_ms:.2f} ms (~{hz:.1f} Hz)"
                        )

                    self.send_button.setEnabled(True)
                    self.cancel_routine_btn.setEnabled(self.preview_mode)
                    if not self.vision_enabled:
                        for slider in self.sliders.values():
                            slider.setEnabled(True)

                    if self.preview_mode:
                        self._set_routine_timer_mode("preview")
                        self.routine_timer.start(20)
                    else:
                        self.routine_timer.stop()
                        self._set_routine_timer_mode(None)
                    return

                cmd, pose = all_commands[self._routine_send_index]
                t_serial_start = time.perf_counter()
                try:
                    self.serial.send_command(cmd.encode())
                except Exception as e:
                    self._log_preview(f"[SERIAL ERROR] Send failed: {e}")
                t_serial_end = time.perf_counter()

                t_slider_start = time.perf_counter()
                self._set_slider_pose(pose)
                t_slider_end = time.perf_counter()

                t_vis_start = time.perf_counter()
                did_vis_update = False
                if self._routine_send_index % self.visualizer_update_skip == 0:
                    did_vis_update = True
                    self.visualizer.update_platform(pose)
                t_vis_end = time.perf_counter()

                self._routine_send_index += 1
                step_total_end = time.perf_counter()

                serial_ms = (t_serial_end - t_serial_start) * 1000
                slider_ms = (t_slider_end - t_slider_start) * 1000
                vis_ms = (t_vis_end - t_vis_start) * 1000
                step_total_ms = (step_total_end - step_total_start) * 1000

                self._profile_total_time += (step_total_end - step_total_start)
                self._profile_step_count += 1
                if self._routine_send_index % 10 == 0:
                    hz = 1000.0 / step_total_ms if step_total_ms > 0 else 0
                    dt_str = "N/A" if dt_ms is None else f"{dt_ms:.2f}ms"
                    self._log_preview(
                        f"[PROFILE] step {self._routine_send_index}/{len(all_commands)} | "
                        f"tick_dt={dt_str} | total={step_total_ms:.2f}ms (~{hz:.1f}Hz) | "
                        f"serial={serial_ms:.2f}ms | sliders={slider_ms:.2f}ms | "
                        f"vis={vis_ms:.2f}ms (updated={did_vis_update})"
                    )

            self._set_routine_timer_mode("send")
            self._routine_send_handler = send_next_step
            self.routine_timer.start(20)
            return

        pose = self._current_pose_from_sliders()
        try:
            t_ik_start = time.perf_counter()
            ik_result = self.ik_solver.solve_pose(
                pose["x"],
                pose["y"],
                pose["z"],
                pose["roll"],
                pose["pitch"],
                pose["yaw"],
                prev_arm_points=self.visualizer.prev_arm_points,
            )
            t_ik_end = time.perf_counter()
            if not ik_result["success"]:
                self._log_preview("[WARN] IK solver failed.")
                return

            angles = [int(round(a)) for a in ik_result["servo_angles_deg"]]
            safe_angles, clipped = self.safety_clip_servos(angles)
            cmd = "S," + ",".join(str(a) for a in safe_angles) + ",0\n"
            if clipped:
                for idx, original, new in clipped:
                    self._log_preview(f'<span style="color:red;">[SAFETY CLIP] Servo {idx}: {original} -> {new}</span>')

            ik_ms = (t_ik_end - t_ik_start) * 1000
            self._log_preview(f"[PROFILE] IK solve time = {ik_ms:.2f} ms")
        except Exception as e:
            self._log_preview(f"[ERROR] IK solver failed: {e}")
            return

        confirm = QtWidgets.QMessageBox.question(
            self,
            "Confirm Send",
            f"Send the following command to Arduino?\n{cmd.strip()}",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
        )
        if confirm != QtWidgets.QMessageBox.Yes:
            self._log_preview("[INFO] Send canceled by user.")
            return

        self._log_preview(f"[SEND] {cmd.strip()}")
        try:
            t_serial_start = time.perf_counter()
            self.serial.send_command(cmd.encode())
            t_serial_end = time.perf_counter()
            serial_ms = (t_serial_end - t_serial_start) * 1000
            self._log_preview(f"[PROFILE] Serial send time = {serial_ms:.2f} ms")
        except Exception as e:
            self._log_preview(f"[SERIAL ERROR] Send failed: {e}")
            return

        t_vis_start = time.perf_counter()
        self.visualizer.update_platform(pose, ik_result=ik_result)
        t_vis_end = time.perf_counter()
        self._log_preview(f"[PROFILE] Visualizer update time = {(t_vis_end - t_vis_start) * 1000:.2f} ms")

    def confirm_large_text(self, title, message, text):
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle(title)
        dialog.resize(700, 500)

        layout = QtWidgets.QVBoxLayout(dialog)
        label = QtWidgets.QLabel(message)
        layout.addWidget(label)

        text_box = QtWidgets.QTextEdit()
        text_box.setPlainText(text)
        text_box.setReadOnly(True)
        layout.addWidget(text_box)

        button_layout = QtWidgets.QHBoxLayout()
        yes_btn = QtWidgets.QPushButton("Yes")
        no_btn = QtWidgets.QPushButton("No")
        button_layout.addStretch()
        button_layout.addWidget(yes_btn)
        button_layout.addWidget(no_btn)
        layout.addLayout(button_layout)

        result = {"confirmed": False}
        yes_btn.clicked.connect(lambda: (result.update({"confirmed": True}), dialog.accept()))
        no_btn.clicked.connect(lambda: (result.update({"confirmed": False}), dialog.reject()))
        dialog.exec_()
        return result["confirmed"]

    def closeEvent(self, event):
        if self.routine_timer.isActive():
            self.routine_timer.stop()
        self._set_routine_timer_mode(None)
        self.disable_vision_mode()
        if hasattr(self, "serial"):
            self.serial.disconnect()
        event.accept()

    def safety_clip_servos(self, angles):
        clipped = []
        safe_angles = angles.copy()
        for i, angle in enumerate(angles):
            original = angle
            if i in [0, 2, 4] and angle > 170:
                safe_angles[i] = 170
                clipped.append((i, original, 170))
            if i in [1, 3, 5] and angle < 10:
                safe_angles[i] = 10
                clipped.append((i, original, 10))
        return safe_angles, clipped

    def update_pd_gains(self):
        kp = self.kp_slider.value() / 1000.0
        kd = self.kd_slider.value() / 1000.0
        self.kp_label.setText(f"Kp: {kp:.3f}")
        self.kd_label.setText(f"Kd: {kd:.3f}")
        self.vision_gains_updated.emit(kp, kd)

    def toggle_vision_mode(self):
        if not self.vision_enabled:
            self.enable_vision_mode()
        else:
            self.disable_vision_mode()

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
            tracker_debug=CAMERA_DEBUG_WINDOWS,
        )
        self._vision_worker.moveToThread(self._vision_thread)
        self._vision_thread.started.connect(self._vision_worker.start)
        self._vision_worker.snapshot_ready.connect(self._on_control_snapshot)
        self._vision_worker.error.connect(self._on_vision_error)
        self._vision_worker.stopped.connect(self._vision_thread.quit)
        self.vision_gains_updated.connect(self._vision_worker.set_gains)
        self._vision_thread.start()

    def _stop_vision_worker(self):
        if self._vision_worker is None:
            return
        try:
            self.vision_gains_updated.disconnect(self._vision_worker.set_gains)
        except TypeError:
            pass
        QtCore.QMetaObject.invokeMethod(self._vision_worker, "stop", QtCore.Qt.QueuedConnection)
        if self._vision_thread is not None:
            self._vision_thread.wait(1500)
            if self._vision_thread.isRunning():
                self._vision_thread.quit()
                self._vision_thread.wait(500)
            self._vision_thread.deleteLater()
        self._vision_worker = None
        self._vision_thread = None

    def enable_vision_mode(self):
        if self.vision_enabled:
            return
        self._log_preview("[INFO] Vision mode enabled.")
        self.vision_enabled = True
        self._vision_counter = 0
        self._last_visualizer_update = 0.0
        for key in self._timing_keys:
            self._timing_history[key].clear()
        self.vision_button.setText("Vision Mode ACTIVE")
        self.vision_button.setStyleSheet("background-color: orange; color: black;")
        self.cancel_vision_btn.setEnabled(True)
        for slider in self.sliders.values():
            slider.setEnabled(False)
        self._start_vision_worker()

    def disable_vision_mode(self):
        if not self.vision_enabled and self._vision_worker is None:
            return
        if self.vision_enabled:
            self._log_preview("[INFO] Vision mode disabled.")
        self.vision_enabled = False
        self.vision_button.setText("Enable Vision Mode")
        self.vision_button.setStyleSheet("background-color: darkgreen; color: white;")
        self.cancel_vision_btn.setEnabled(False)
        self._stop_vision_worker()
        if not self.preview_mode:
            for slider in self.sliders.values():
                slider.setEnabled(True)
        self.timing_summary_label.setText("Vision Timing Avg (ms): waiting for data...")

    def _on_vision_error(self, msg):
        self._log_preview(f"[VISION ERROR] {msg}")

    def _on_control_snapshot(self, snapshot):
        if not self.vision_enabled:
            return
        if not snapshot.ik_success:
            self._vision_counter += 1
            if self._vision_counter % LOG_EVERY_N == 0:
                self._log_preview("[WARN] IK failed in vision loop.")
            return

        safe_angles, clipped = self.safety_clip_servos(snapshot.servo_angles)
        cmd = "S," + ",".join(str(a) for a in safe_angles) + ",0\n"
        t_serial0 = time.perf_counter()
        self.serial.enqueue_command(cmd, policy="latest")
        t_serial1 = time.perf_counter()

        now = time.perf_counter()
        vis_ms = 0.0
        if now - self._last_visualizer_update >= (1.0 / max(1, VISUALIZER_HZ)):
            self._last_visualizer_update = now
            t_vis0 = time.perf_counter()
            self.visualizer.update_platform(snapshot.pose, ik_result=snapshot.ik_result)
            t_vis1 = time.perf_counter()
            vis_ms = (t_vis1 - t_vis0) * 1000.0

        timings = dict(snapshot.timings_ms)
        timings["serial_enqueue"] = (t_serial1 - t_serial0) * 1000.0
        timings["visualizer_gui"] = vis_ms
        for key in self._timing_keys:
            self._timing_history[key].append(float(timings.get(key, 0.0)))

        self._update_timing_diagnostics()

        self._vision_counter += 1
        if DEBUG_LEVEL >= 2 and (self._vision_counter % LOG_EVERY_N == 0):
            t = timings
            self._log_preview(
                "[VISION] "
                f"ball={t['ball_update']:.2f}ms "
                f"pd={t['pd_compute']:.2f}ms "
                f"ik={t['ik_solve']:.2f}ms "
                f"serQ={t['serial_enqueue']:.2f}ms "
                f"vis={t['visualizer_gui']:.2f}ms "
                f"total={t['total']:.2f}ms "
                f"clips={len(clipped)}"
            )

    def _update_timing_diagnostics(self):
        if len(self._timing_history["total"]) == 0:
            return

        averages = {}
        for key in self._timing_keys:
            data = self._timing_history[key]
            if data:
                averages[key] = sum(data) / len(data)
            else:
                averages[key] = 0.0

        self.timing_summary_label.setText(
            "Vision Timing Avg (ms): "
            f"ball={averages['ball_update']:.2f}, "
            f"pd={averages['pd_compute']:.2f}, "
            f"ik={averages['ik_solve']:.2f}, "
            f"serQ={averages['serial_enqueue']:.2f}, "
            f"vis={averages['visualizer_gui']:.2f}, "
            f"total={averages['total']:.2f}"
        )

        if self._vision_counter % LOG_EVERY_N == 0:
            self._log_preview(
                "[VISION AVG] "
                f"ball={averages['ball_update']:.2f}ms, "
                f"pd={averages['pd_compute']:.2f}ms, "
                f"ik={averages['ik_solve']:.2f}ms, "
                f"serQ={averages['serial_enqueue']:.2f}ms, "
                f"vis={averages['visualizer_gui']:.2f}ms, "
                f"total={averages['total']:.2f}ms"
            )

        if self._vision_counter % self._timing_plot_update_every != 0:
            return

        self.timing_ax.cla()
        self.timing_ax.set_title("Vision Loop Step Timings (ms)")
        self.timing_ax.set_xlabel("Samples")
        self.timing_ax.set_ylabel("ms")

        for key in self._timing_keys:
            y = list(self._timing_history[key])
            if not y:
                continue
            x = list(range(len(y)))
            self.timing_ax.plot(x, y, label=key, linewidth=1.5, color=self._timing_colors[key])

        self.timing_ax.grid(True, alpha=0.3)
        self.timing_ax.legend(loc="upper right", fontsize=7)
        self.timing_canvas.draw_idle()
