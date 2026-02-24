from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QLabel, QSlider, QPushButton, QVBoxLayout, QHBoxLayout, QTextEdit, QWidget, QMessageBox
from PyQt5.QtCore import pyqtSignal, QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from stewart_control.visualization.visualizer3d import StewartVisualizer
from stewart_control.comms.serial_sender import SerialSender
from stewart_control.comms.arduino_protocol import format_command
from stewart_control.routines.routines import ROUTINES
from stewart_control.cv.ball_tracker import BallTracker
from stewart_control.cv.ball_controller import BallController
import time



class StewartGUILayout(QWidget):
    serial_line_received = pyqtSignal(str)

    def __init__(self, ik_solver=None):
        super().__init__()
    
        # --- Internal flags and data structures ---
        self.preview_mode = False
        self.current_routine_steps = []
        self.current_routine_index = 0
        self.ik_solver = ik_solver  # inject IK solver for real servo angles
        
        self.visualizer_update_skip = 5
    
        # --- Create the timer BEFORE it’s connected ---
        self.routine_timer = QTimer()  # just define it, don’t connect yet
        self.routine_timer.setTimerType(QtCore.Qt.PreciseTimer)

    
        # --- Build UI ---
        self.init_ui()  # THIS must happen before you access any buttons or sliders!
    
        # --- Now widgets exist, safe to connect ---
        self.routine_timer.timeout.connect(self._routine_step)
        self.send_button.clicked.connect(self.send_to_arduino)
    
        # --- Visualizer ---
        self.visualizer = StewartVisualizer(self.canvas)
    
        # --- Serial ---
        self.serial_line_received.connect(self.append_serial_line)
        self.serial = SerialSender('COM4')
        self.serial.connect()
        self.serial.set_receive_callback(lambda line: self.serial_line_received.emit(line))
        
        self.profile_enabled = True
        self.profile_every_n = 10  # print every 10 steps
        self._profile_step_counter = 0
        
        # --- Ball balancing system ---
        self.ball_tracker = BallTracker()
        self.ball_controller = BallController(
            kp=0.014,
            kd=0.024,
            max_tilt_deg=8.0
        )
        
        self.vision_enabled = False
        
        # Timer for control loop (separate from routine timer)
        self.vision_timer = QTimer()
        self.vision_timer.setTimerType(QtCore.Qt.PreciseTimer)
        self.vision_timer.timeout.connect(self._vision_control_step)

        
    
        
    def append_serial_line(self, line):
        self.serial_monitor.append(line)

    def init_ui(self):
        main_layout = QHBoxLayout()
        
        # --- Left panel: sliders ---
        slider_layout = QVBoxLayout()
        self.sliders = {}
        axes = ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']
        for ax in axes:
            lbl = QLabel(f'{ax}: 0')
            sld = QSlider(QtCore.Qt.Horizontal)
            sld.setMinimum(-100)
            sld.setMaximum(100)
            sld.setValue(0)
            sld.valueChanged.connect(lambda val, a=ax, l=lbl: self.update_slider(a, val, l))
            slider_layout.addWidget(lbl)
            slider_layout.addWidget(sld)
            self.sliders[ax] = sld

        # --- Middle panel: controls ---
        control_layout = QVBoxLayout()
        self.demo_list = QtWidgets.QComboBox()
        self.demo_list.addItem("(Choose a routine...)")
        for name in ROUTINES.keys():
            self.demo_list.addItem(name)
        control_layout.addWidget(QLabel('Demo Routines:'))
        control_layout.addWidget(self.demo_list)
        
        self.cancel_routine_btn = QPushButton("❌ Cancel Routine")
        self.cancel_routine_btn.setStyleSheet("background-color: darkred; color: white; font-weight: bold;")
        self.cancel_routine_btn.clicked.connect(self.cancel_routine_preview)
        self.cancel_routine_btn.setEnabled(False)
        control_layout.addWidget(self.cancel_routine_btn)


        # --- Vision Controls ---
        control_layout.addWidget(QLabel("Ball Balancing Control"))
        
        self.vision_button = QPushButton('Enable Vision Mode')
        self.vision_button.setStyleSheet("background-color: darkgreen; color: white;")
        self.vision_button.clicked.connect(self.toggle_vision_mode)
        control_layout.addWidget(self.vision_button)
        
        self.cancel_vision_btn = QPushButton("🛑 Cancel Vision Mode")
        self.cancel_vision_btn.setStyleSheet("background-color: darkred; color: white;")
        self.cancel_vision_btn.clicked.connect(self.disable_vision_mode)
        self.cancel_vision_btn.setEnabled(False)
        control_layout.addWidget(self.cancel_vision_btn)
        
        # --- Kp Slider ---
        self.kp_label = QLabel("Kp: 0.03")
        self.kp_slider = QSlider(QtCore.Qt.Horizontal)
        self.kp_slider.setMinimum(0)
        self.kp_slider.setMaximum(100)
        self.kp_slider.setValue(30)  # 0.03 scaled by 1000
        self.kp_slider.valueChanged.connect(self.update_pd_gains)
        
        control_layout.addWidget(self.kp_label)
        control_layout.addWidget(self.kp_slider)
        
        # --- Kd Slider ---
        self.kd_label = QLabel("Kd: 0.008")
        self.kd_slider = QSlider(QtCore.Qt.Horizontal)
        self.kd_slider.setMinimum(0)
        self.kd_slider.setMaximum(100)
        self.kd_slider.setValue(8)  # 0.008 scaled by 1000
        self.kd_slider.valueChanged.connect(self.update_pd_gains)
        
        control_layout.addWidget(self.kd_label)
        control_layout.addWidget(self.kd_slider)

        self.send_button = QPushButton('SEND TO ARDUINO')
        self.send_button.setStyleSheet("background-color: red; color: white; font-size: 16pt;")
        control_layout.addWidget(self.send_button)

        self.demo_list.currentIndexChanged.connect(self.on_routine_changed)


        self.preview_output = QTextEdit()
        self.preview_output.setReadOnly(True)
        self.preview_output.setPlaceholderText('Command preview will appear here...')
        control_layout.addWidget(QLabel('Command Preview:'))
        control_layout.addWidget(self.preview_output)

        # In init_ui(), under control_layout
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
        control_layout.addWidget(QLabel("Serial Monitor:"))
        control_layout.addWidget(self.serial_monitor)


        # --- Right panel: Matplotlib canvas ---
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setFixedSize(500, 500)

        # Combine layouts
        main_layout.addLayout(slider_layout)
        main_layout.addLayout(control_layout)
        main_layout.addWidget(self.canvas)
        self.setLayout(main_layout)
        
    def send_raw_serial_command(self):
        cmd = self.raw_serial_input.text().strip()
        if not cmd:
            self.preview_output.append("[WARN] No command entered.")
            return
    
        confirm = QtWidgets.QMessageBox.question(
            self,
            "Confirm Send",
            f"Send the following command to Arduino?\n{cmd}",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No
        )
        if confirm != QtWidgets.QMessageBox.Yes:
            self.preview_output.append("[INFO] Send canceled by user.")
            return
    
        self.serial.send_command(cmd.encode())  # pass as string
        self.serial_monitor.append(f">>> {cmd}")
        self.preview_output.append(f"[INFO] Command sent: {cmd}")

        
    def run_selected_routine(self):
        routine_name = self.demo_list.currentText()
    
        if routine_name not in ROUTINES:
            self.preview_output.append("[ERROR] Routine not found.")
            return
    
        routine_func = ROUTINES[routine_name]
    
        # generate poses
        poses = routine_func()
    
        self.preview_output.append(f"[INFO] Running routine: {routine_name}")
        self.preview_output.append(f"[INFO] Steps: {len(poses)}")
    
        # store routine state for animation
        self.current_routine_steps = poses
        self.current_routine_index = 0
    
        # start timer-based playback
        self.routine_timer.start(20)  # 500ms per step
        
    def on_routine_changed(self):
        routine_name = self.demo_list.currentText()
    
        # If default option selected -> exit preview mode
        if routine_name == "(Choose a routine...)":
            self.cancel_routine_preview()
            return
    
        # Otherwise start preview mode
        if routine_name not in ROUTINES:
            self.preview_output.append("[ERROR] Routine not found.")
            return
    
        self.start_routine_preview(routine_name)
        
        
    def start_routine_preview(self, routine_name):
        self.preview_output.append(f"[INFO] Previewing routine: {routine_name}")
        
        # Get current pose from sliders to seed prev_arm_points
        current_pose = {
            "x": self.sliders["X"].value(),
            "y": self.sliders["Y"].value(),
            "z": self.sliders["Z"].value(),
            "roll": self.sliders["Roll"].value(),
            "pitch": self.sliders["Pitch"].value(),
            "yaw": self.sliders["Yaw"].value()
        }
        ik_seed = self.ik_solver.solve_pose(
            current_pose['x'], current_pose['y'], current_pose['z'],
            current_pose['roll'], current_pose['pitch'], current_pose['yaw'],
            prev_arm_points=self.visualizer.prev_arm_points
        )
        
        # seed prev_arm_points
        self.visualizer.prev_arm_points = ik_seed.get("arm_points", None)
    
        routine_func = ROUTINES[routine_name]
        poses = routine_func()
    
        if not poses or len(poses) < 2:
            self.preview_output.append("[ERROR] Routine has no poses.")
            return
    
        self.preview_mode = True
    
        # Store routine state
        self.current_routine_steps = poses
        self.current_routine_index = 0
    
        # Disable manual controls
        for slider in self.sliders.values():
            slider.setEnabled(False)
    
        self.cancel_routine_btn.setEnabled(True)
    
        # Start timer loop animation
        self.routine_timer.start(20)  # speed of preview (ms)
        
    def cancel_routine_preview(self):
        if self.preview_mode:
            self.preview_output.append("[INFO] Routine preview cancelled.")
    
        self.preview_mode = False
    
        # stop timer
        self.routine_timer.stop()
    
        # RESET timer connections to default preview step
        try:
            self.routine_timer.timeout.disconnect()
        except TypeError:
            pass
        self.routine_timer.timeout.connect(self._routine_step)
    
        self.current_routine_steps = []
        self.current_routine_index = 0
    
        for slider in self.sliders.values():
            slider.setEnabled(True)
    
        self.send_button.setEnabled(True)
        self.cancel_routine_btn.setEnabled(False)
    
        if self.demo_list.currentText() != "(Choose a routine...)":
            self.demo_list.blockSignals(True)
            self.demo_list.setCurrentIndex(0)
            self.demo_list.blockSignals(False)
    
        pose = {
            "x": self.sliders["X"].value(),
            "y": self.sliders["Y"].value(),
            "z": self.sliders["Z"].value(),
            "roll": self.sliders["Roll"].value(),
            "pitch": self.sliders["Pitch"].value(),
            "yaw": self.sliders["Yaw"].value()
        }
    
        self.visualizer.update_platform(pose)


    def _routine_step(self):
        if not self.preview_mode:
            return
    
        if len(self.current_routine_steps) == 0:
            return
    
        pose = self.current_routine_steps[self.current_routine_index]
    
        # increment and wrap (looping)
        self.current_routine_index += 1
        if self.current_routine_index >= len(self.current_routine_steps):
            self.current_routine_index = 0
    
        # Update sliders (even though disabled)
        for ax in ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]:
            self.sliders[ax].blockSignals(True)
        
        self.sliders["X"].setValue(int(pose["x"]))
        self.sliders["Y"].setValue(int(pose["y"]))
        self.sliders["Z"].setValue(int(pose["z"]))
        self.sliders["Roll"].setValue(int(pose["roll"]))
        self.sliders["Pitch"].setValue(int(pose["pitch"]))
        self.sliders["Yaw"].setValue(int(pose["yaw"]))
        
        for ax in ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]:
            self.sliders[ax].blockSignals(False)

    
        # Update visualizer preview
        self.visualizer.update_platform({
            "x": pose["x"],
            "y": pose["y"],
            "z": pose["z"],
            "roll": pose["roll"],
            "pitch": pose["pitch"],
            "yaw": pose["yaw"]
        })
    
    
    def update_slider(self, axis, value, label):
        label.setText(f'{axis}: {value}')
        pos = {
            'x': self.sliders['X'].value(),
            'y': self.sliders['Y'].value(),
            'z': self.sliders['Z'].value(),
            'roll': self.sliders['Roll'].value(),
            'pitch': self.sliders['Pitch'].value(),
            'yaw': self.sliders['Yaw'].value()
        }
        self.visualizer.update_platform(pos)
    
        
    def send_pose_direct(self, pose):
        try:
            ik_result = self.ik_solver.solve_pose(
                pose['x'], pose['y'], pose['z'],
                pose['roll'], pose['pitch'], pose['yaw'],
                prev_arm_points=self.visualizer.prev_arm_points
            )
    
            if not ik_result["success"]:
                self.preview_output.append("[WARN] IK failed at this step.")
                return
    
            angles = ik_result["servo_angles_deg"]
            angles = [int(round(a)) for a in angles]
    
            cmd = "S," + ",".join(str(a) for a in angles) + ",0\n"

    
            self.preview_output.append(f"[SEND] {cmd.strip()}")
    
            # send serial
            self.serial.send_command(cmd.encode())
    
        except Exception as e:
            self.preview_output.append(f"[ERROR] Routine send failed: {e}")



    def send_to_arduino(self):
        """Send either current slider pose or full routine to Arduino with confirmation + profiling."""
    
        if self.preview_mode:
            self.preview_output.append("[PROFILE] Building routine command list...")
    
            t_build_start = time.perf_counter()
    
            # --- Step 1: Generate all routine commands ---
            all_commands = []
            ik_fail_count = 0
    
            for pose in self.current_routine_steps:
                try:
                    t_ik_start = time.perf_counter()
    
                    ik_result = self.ik_solver.solve_pose(
                        pose['x'], pose['y'], pose['z'],
                        pose['roll'], pose['pitch'], pose['yaw'],
                        prev_arm_points=self.visualizer.prev_arm_points
                    )
    
                    t_ik_end = time.perf_counter()
    
                    if not ik_result["success"]:
                        ik_fail_count += 1
                        continue
    
                    angles = [int(round(a)) for a in ik_result["servo_angles_deg"]]

                    safe_angles, clipped = self.safety_clip_servos(angles)
                    
                    cmd = "S," + ",".join(str(a) for a in safe_angles) + ",0\n"
                    
                    if clipped:
                        for idx, original, new in clipped:
                            self.preview_output.append(
                                f'<span style="color:red;">[SAFETY CLIP] Servo {idx}: {original} → {new}</span>'
                            )

    
                    all_commands.append((cmd, pose))
    
                except Exception as e:
                    ik_fail_count += 1
                    self.preview_output.append(f"[ERROR] Routine IK calculation failed: {e}")
                    continue
    
            t_build_end = time.perf_counter()
    
            build_ms = (t_build_end - t_build_start) * 1000
            self.preview_output.append(
                f"[PROFILE] Routine command generation complete: "
                f"{len(all_commands)} valid steps, {ik_fail_count} failed. "
                f"Build time = {build_ms:.2f} ms"
            )
    
            if not all_commands:
                self.preview_output.append("[ERROR] No valid servo commands generated for routine.")
                return
    
            # --- Step 2: Show confirmation ---
            cmds_text = "\n".join(c.strip() for c, _ in all_commands)
    
            t_confirm_start = time.perf_counter()
            confirmed = self.confirm_large_text(
                "Confirm Send Routine",
                f"Send the following {len(all_commands)} servo commands to Arduino?",
                cmds_text
            )
            t_confirm_end = time.perf_counter()
    
            confirm_ms = (t_confirm_end - t_confirm_start) * 1000
            self.preview_output.append(f"[PROFILE] Confirm dialog time = {confirm_ms:.2f} ms")
    
            if not confirmed:
                self.preview_output.append("[INFO] Routine send canceled by user.")
                return
    
            # --- Step 3: Disable manual controls ---
            for slider in self.sliders.values():
                slider.setEnabled(False)
    
            self.send_button.setEnabled(False)
            self.cancel_routine_btn.setEnabled(True)
    
            # --- Step 4: Send routine step by step ---
            self._routine_send_index = 0
            self.routine_timer.stop()
    
            try:
                self.routine_timer.timeout.disconnect(self._routine_step)
            except TypeError:
                pass
    
            # --- PROFILING STATE ---
            self._profile_last_tick = None
            self._profile_total_time = 0.0
            self._profile_step_count = 0
    
            def send_next_step():
                import time
    
                step_total_start = time.perf_counter()
    
                # --- TIMER JITTER CHECK ---
                tick_now = time.perf_counter()
                if self._profile_last_tick is not None:
                    dt_ms = (tick_now - self._profile_last_tick) * 1000
                else:
                    dt_ms = None
                self._profile_last_tick = tick_now
    
                if self._routine_send_index >= len(all_commands):
                    self.preview_output.append("[INFO] Routine sent successfully.")
    
                    # Final stats
                    if self._profile_step_count > 0:
                        avg_step_ms = (self._profile_total_time / self._profile_step_count) * 1000
                        hz = 1000.0 / avg_step_ms if avg_step_ms > 0 else 0
                        self.preview_output.append(
                            f"[PROFILE] Routine complete. Avg step time = {avg_step_ms:.2f} ms (~{hz:.1f} Hz)"
                        )
    
                    for slider in self.sliders.values():
                        slider.setEnabled(True)
    
                    self.send_button.setEnabled(True)
                    self.cancel_routine_btn.setEnabled(self.preview_mode)
    
                    if self.preview_mode:
                        try:
                            self.routine_timer.timeout.disconnect(send_next_step)
                        except TypeError:
                            pass
    
                        self.routine_timer.timeout.connect(self._routine_step)
                        self.routine_timer.start(20)
    
                    return
    
                cmd, pose = all_commands[self._routine_send_index]
    
                # --- SERIAL SEND TIMING ---
                t_serial_start = time.perf_counter()
                try:
                    self.serial.send_command(cmd.encode())
                except Exception as e:
                    self.preview_output.append(f"[SERIAL ERROR] Send failed: {e}")
                t_serial_end = time.perf_counter()
    
                # --- SLIDER UPDATE TIMING ---
                t_slider_start = time.perf_counter()
    
                for ax in ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]:
                    self.sliders[ax].blockSignals(True)
    
                for ax, key in zip(["X", "Y", "Z", "Roll", "Pitch", "Yaw"],
                                   ["x", "y", "z", "roll", "pitch", "yaw"]):
                    self.sliders[ax].setValue(int(pose[key]))
    
                for ax in ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]:
                    self.sliders[ax].blockSignals(False)
    
                t_slider_end = time.perf_counter()
    
                # --- VISUALIZER UPDATE TIMING ---
                t_vis_start = time.perf_counter()
    
                did_vis_update = False
                if self._routine_send_index % self.visualizer_update_skip == 0:
                    did_vis_update = True
                    self.visualizer.update_platform({
                        "x": pose["x"],
                        "y": pose["y"],
                        "z": pose["z"],
                        "roll": pose["roll"],
                        "pitch": pose["pitch"],
                        "yaw": pose["yaw"]
                    })
    
                t_vis_end = time.perf_counter()
    
                self._routine_send_index += 1
    
                step_total_end = time.perf_counter()
    
                # --- STEP TIMINGS ---
                serial_ms = (t_serial_end - t_serial_start) * 1000
                slider_ms = (t_slider_end - t_slider_start) * 1000
                vis_ms = (t_vis_end - t_vis_start) * 1000
                step_total_ms = (step_total_end - step_total_start) * 1000
    
                self._profile_total_time += (step_total_end - step_total_start)
                self._profile_step_count += 1
    
                # --- PRINT EVERY N STEPS ---
                if self._routine_send_index % 10 == 0:
                    hz = 1000.0 / step_total_ms if step_total_ms > 0 else 0
    
                    if dt_ms is None:
                        dt_str = "N/A"
                    else:
                        dt_str = f"{dt_ms:.2f}ms"
    
                    self.preview_output.append(
                        f"[PROFILE] step {self._routine_send_index}/{len(all_commands)} | "
                        f"tick_dt={dt_str} | "
                        f"total={step_total_ms:.2f}ms (~{hz:.1f}Hz) | "
                        f"serial={serial_ms:.2f}ms | "
                        f"sliders={slider_ms:.2f}ms | "
                        f"vis={vis_ms:.2f}ms (updated={did_vis_update})"
                    )
    
            # connect and start
            try:
                self.routine_timer.timeout.disconnect()
            except TypeError:
                pass
    
            self.routine_timer.timeout.connect(send_next_step)
            self.routine_timer.start(20)
    
        else:
            # --- Slider-driven send ---
            pose = {
                'x': self.sliders['X'].value(),
                'y': self.sliders['Y'].value(),
                'z': self.sliders['Z'].value(),
                'roll': self.sliders['Roll'].value(),
                'pitch': self.sliders['Pitch'].value(),
                'yaw': self.sliders['Yaw'].value()
            }
    
            try:
                t_ik_start = time.perf_counter()
    
                ik_result = self.ik_solver.solve_pose(
                    pose['x'], pose['y'], pose['z'],
                    pose['roll'], pose['pitch'], pose['yaw'],
                    prev_arm_points=self.visualizer.prev_arm_points
                )
    
                t_ik_end = time.perf_counter()
    
                if not ik_result["success"]:
                    self.preview_output.append("[WARN] IK solver failed.")
                    return
    
                angles = [int(round(a)) for a in ik_result["servo_angles_deg"]]

                safe_angles, clipped = self.safety_clip_servos(angles)
                
                cmd = "S," + ",".join(str(a) for a in safe_angles) + ",0\n"
                
                if clipped:
                    for idx, original, new in clipped:
                        self.preview_output.append(
                            f'<span style="color:red;">[SAFETY CLIP] Servo {idx}: {original} → {new}</span>'
                        )

    
                ik_ms = (t_ik_end - t_ik_start) * 1000
                self.preview_output.append(f"[PROFILE] IK solve time = {ik_ms:.2f} ms")
    
            except Exception as e:
                self.preview_output.append(f"[ERROR] IK solver failed: {e}")
                return
    
            confirm = QtWidgets.QMessageBox.question(
                self,
                "Confirm Send",
                f"Send the following command to Arduino?\n{cmd.strip()}",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No
            )
    
            if confirm != QtWidgets.QMessageBox.Yes:
                self.preview_output.append("[INFO] Send canceled by user.")
                return
    
            self.preview_output.append(f"[SEND] {cmd.strip()}")
    
            try:
                t_serial_start = time.perf_counter()
                self.serial.send_command(cmd.encode())
                t_serial_end = time.perf_counter()
    
                serial_ms = (t_serial_end - t_serial_start) * 1000
                self.preview_output.append(f"[PROFILE] Serial send time = {serial_ms:.2f} ms")
    
            except Exception as e:
                self.preview_output.append(f"[SERIAL ERROR] Send failed: {e}")
                return
    
            t_vis_start = time.perf_counter()
            self.visualizer.update_platform(pose)
            t_vis_end = time.perf_counter()
    
            vis_ms = (t_vis_end - t_vis_start) * 1000
            self.preview_output.append(f"[PROFILE] Visualizer update time = {vis_ms:.2f} ms")



    def confirm_large_text(self, title, message, text):
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle(title)
        dialog.resize(700, 500)
    
        layout = QtWidgets.QVBoxLayout(dialog)
    
        label = QtWidgets.QLabel(message)
        layout.addWidget(label)
    
        text_box = QtWidgets.QTextEdit()
        text_box.setHtml(text)
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
    
        def confirm():
            result["confirmed"] = True
            dialog.accept()
    
        def cancel():
            result["confirmed"] = False
            dialog.reject()
    
        yes_btn.clicked.connect(confirm)
        no_btn.clicked.connect(cancel)
    
        dialog.exec_()
        return result["confirmed"]

    
    def closeEvent(self, event):
         # Stop routine timer if it's running
         if hasattr(self, 'routine_timer') and self.routine_timer.isActive():
             self.routine_timer.stop()
     
         # Optional: disconnect signals to be extra safe
         try:
             self.routine_timer.timeout.disconnect()
         except:
             pass
     
         # Close serial if needed
         if hasattr(self, 'serial'):
             self.serial.disconnect()  # or implement proper cleanup
     
         event.accept()
     
    def safety_clip_servos(self, angles):
        """
        Final safety clipping before sending to Arduino.
        Returns (clipped_angles, clipped_info)
        clipped_info = list of (index, original, clipped)
        """
    
        clipped = []
        safe_angles = angles.copy()
    
        for i, angle in enumerate(angles):
            original = angle
    
            # Servos 0,2,4 max 150
            if i in [0, 2, 4]:
                if angle > 170:
                    safe_angles[i] = 170
                    clipped.append((i, original, 170))
    
            # Servos 1,3,5 min 30
            if i in [1, 3, 5]:
                if angle < 10:
                    safe_angles[i] = 10
                    clipped.append((i, original, 10))
    
        return safe_angles, clipped

    def update_pd_gains(self):
        kp = self.kp_slider.value() / 1000.0
        kd = self.kd_slider.value() / 1000.0
    
        self.kp_label.setText(f"Kp: {kp:.3f}")
        self.kd_label.setText(f"Kd: {kd:.3f}")
    
        self.ball_controller.set_gains(kp, kd)
        
    def toggle_vision_mode(self):
        if not self.vision_enabled:
            self.enable_vision_mode()
        else:
            self.disable_vision_mode()
    
    
    def enable_vision_mode(self):
        self.preview_output.append("[INFO] Vision mode enabled.")
        self.vision_enabled = True
    
        self.vision_button.setText("Vision Mode ACTIVE")
        self.vision_button.setStyleSheet("background-color: orange; color: black;")
    
        self.cancel_vision_btn.setEnabled(True)
    
        # Disable manual sliders
        for slider in self.sliders.values():
            slider.setEnabled(False)
    
        # Start control loop (50 Hz)
        self.vision_timer.start(20)
    
    
    def disable_vision_mode(self):
        self.preview_output.append("[INFO] Vision mode disabled.")
        self.vision_enabled = False
    
        self.vision_timer.stop()
    
        self.vision_button.setText("Enable Vision Mode")
        self.vision_button.setStyleSheet("background-color: darkgreen; color: white;")
    
        self.cancel_vision_btn.setEnabled(False)
    
        # Re-enable sliders
        for slider in self.sliders.values():
            slider.setEnabled(True)
            
    def _vision_control_step(self):
        if not self.vision_enabled:
            return
    
        try:
            # --- 1. Get ball state ---
            ball_state = self.ball_tracker.update()
    
            if ball_state is None:
                return
    
            # --- 2. Compute desired tilt ---
            roll_deg, pitch_deg = self.ball_controller.compute(ball_state)
    
            # --- 3. Build pose (only roll/pitch controlled) ---
            pose = {
                "x": 0,
                "y": 0,
                "z": self.sliders["Z"].value(),  # keep current heiqght
                "roll": roll_deg,
                "pitch": pitch_deg,
                "yaw": 0
            }
    
            # --- 4. Solve IK ---
            ik_result = self.ik_solver.solve_pose(
                pose['x'], pose['y'], pose['z'],
                pose['roll'], pose['pitch'], pose['yaw'],
                prev_arm_points=self.visualizer.prev_arm_points
            )
    
            if not ik_result["success"]:
                self.preview_output.append("[WARN] IK failed in vision loop.")
                return
    
            angles = [int(round(a)) for a in ik_result["servo_angles_deg"]]
    
            safe_angles, clipped = self.safety_clip_servos(angles)
    
            cmd = "S," + ",".join(str(a) for a in safe_angles) + ",0\n"
    
            # --- 5. Send to hardware ---
            self.serial.send_command(cmd.encode())
    
            # --- 6. Update visualizer ---
            self.visualizer.update_platform(pose)
    
        except Exception as e:
            self.preview_output.append(f"[VISION ERROR] {e}")