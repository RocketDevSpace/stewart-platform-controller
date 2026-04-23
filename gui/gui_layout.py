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
from control.routine_runner import RoutineRunner
from core.ik_engine import IKEngine
from hardware.servo_driver import ServoDriver
import time
import threading


class _LegacySerialAdapter:
    """Adapts legacy SerialSender to the SerialManager.send() interface
    expected by hardware/servo_driver.py. Only used so RoutineRunner can
    dispatch via ServoDriver while gui_layout still owns a SerialSender."""
    def __init__(self, legacy_serial):
        self._s = legacy_serial
    def send(self, data: bytes) -> bool:
        try:
            self._s.send_command(data)
            return True
        except Exception:
            return False


class StewartGUILayout(QWidget):
    serial_line_received = pyqtSignal(str)

    def __init__(self, ik_solver=None):
        super().__init__()
    
        # --- Internal flags and data structures ---
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

        # --- Routine runner (Qt-free state machine) ---
        self._ik_engine = IKEngine()
        self._servo_driver = ServoDriver(_LegacySerialAdapter(self.serial))
        self.routine_runner = RoutineRunner(
            ik_engine=self._ik_engine,
            serial_driver=self._servo_driver,
            on_pose_update=self._on_routine_pose,
        )
        
        self.profile_enabled = True
        self.profile_every_n = 10  # print every 10 steps
        self._profile_step_counter = 0
        
        # --- Ball balancing system ---
        self.ball_tracker = BallTracker()
        self.ball_controller = BallController(
            kp=0.005,
            kd=0.010,
            max_tilt_deg=8.0
        )
        
        self.vision_enabled = False
        
        # Timer for control loop (separate from routine timer)
        self.vision_timer = QTimer()
        self.vision_timer.setTimerType(QtCore.Qt.PreciseTimer)
        self.vision_timer.timeout.connect(self._vision_control_step)
        
        self._latest_pose = None
        self._pose_lock = threading.Lock()
        self._vis_thread_running = True
        
        self._vis_thread = threading.Thread(
            target=self._visualizer_loop,
            daemon=True
        )
        self._vis_thread.start()

        
    
        
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
        self.kp_label = QLabel("Kp: 0.05")
        self.kp_slider = QSlider(QtCore.Qt.Horizontal)
        self.kp_slider.setMinimum(0)
        self.kp_slider.setMaximum(100)
        self.kp_slider.setValue(5)  # 0.005 scaled by 1000
        self.kp_slider.valueChanged.connect(self.update_pd_gains)
        
        control_layout.addWidget(self.kp_label)
        control_layout.addWidget(self.kp_slider)
        
        # --- Kd Slider ---
        self.kd_label = QLabel("Kd: 0.010")
        self.kd_slider = QSlider(QtCore.Qt.Horizontal)
        self.kd_slider.setMinimum(0)
        self.kd_slider.setMaximum(100)
        self.kd_slider.setValue(10)  # 0.010 scaled by 1000
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

        
    def _on_routine_pose(self, pose):
        """Callback from RoutineRunner: mirror pose to sliders and visualizer."""
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
        self.visualizer.update_platform(pose)

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

        # Seed prev_arm_points from current slider pose for smooth visualization
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
        self.visualizer.prev_arm_points = ik_seed.get("arm_points", None)

        if not self.routine_runner.load(routine_name):
            self.preview_output.append("[ERROR] Routine not found.")
            return
        if self.routine_runner.total_steps < 2:
            self.preview_output.append("[ERROR] Routine has no poses.")
            return

        self.routine_runner.start_preview()

        for slider in self.sliders.values():
            slider.setEnabled(False)
        self.cancel_routine_btn.setEnabled(True)

        self.routine_timer.start(20)

    def cancel_routine_preview(self):
        if self.routine_runner.is_running:
            self.preview_output.append("[INFO] Routine preview cancelled.")

        self.routine_runner.cancel()
        self.routine_timer.stop()

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
        still_going = self.routine_runner.tick()
        if not still_going and not self.routine_runner.is_running:
            self.routine_timer.stop()
            for slider in self.sliders.values():
                slider.setEnabled(True)
            self.send_button.setEnabled(True)
            self.cancel_routine_btn.setEnabled(False)
    
    
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
        """Send either current slider pose or full routine to Arduino."""

        routine_name = self.demo_list.currentText()
        if routine_name != "(Choose a routine...)":
            if not self.routine_runner.load(routine_name):
                self.preview_output.append("[ERROR] Routine not found.")
                return
            self.routine_runner.start_send()

            for slider in self.sliders.values():
                slider.setEnabled(False)
            self.send_button.setEnabled(False)
            self.cancel_routine_btn.setEnabled(True)

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
             
         self._vis_thread_running = False
     
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
            
    def _visualizer_loop(self):
        import time
    
        while self._vis_thread_running:
            pose = None
    
            with self._pose_lock:
                if self._latest_pose is not None:
                    pose = self._latest_pose.copy()
    
            if pose is not None:
                self.visualizer.update_platform(pose)
    
            time.sleep(0.03)  # ~30 Hz visual update
            
    def _vision_control_step(self):
        loop_start = time.perf_counter()
    
        if not self.vision_enabled:
            return
    
        try:
            # --- 1. Get ball state ---
            t0 = time.perf_counter()
            ball_state = self.ball_tracker.update()
            t1 = time.perf_counter()
    
            if ball_state is None:
                return
    
            # --- 2. Compute desired tilt ---
            roll_deg, pitch_deg = self.ball_controller.compute(ball_state)
            t2 = time.perf_counter()
    
            # --- 3. Build pose ---
            pose = {
                "x": 0,
                "y": 0,
                "z": self.sliders["Z"].value(),
                "roll": roll_deg,
                "pitch": pitch_deg,
                "yaw": 0
            }
    
            # --- 4. Solve IK ---
            t3 = time.perf_counter()
            ik_result = self.ik_solver.solve_pose(
                pose['x'], pose['y'], pose['z'],
                pose['roll'], pose['pitch'], pose['yaw'],
                prev_arm_points=self.visualizer.prev_arm_points
            )
            t4 = time.perf_counter()
    
            if not ik_result["success"]:
                self.preview_output.append("[WARN] IK failed in vision loop.")
                return
    
            angles = [int(round(a)) for a in ik_result["servo_angles_deg"]]
            safe_angles, clipped = self.safety_clip_servos(angles)
    
            cmd = "S," + ",".join(str(a) for a in safe_angles) + ",0\n"
    
            # --- 5. Send to hardware ---
            t5 = time.perf_counter()
            self.serial.send_command(cmd.encode())
            t6 = time.perf_counter()
    
            # --- 6. Update visualizer ---
            t7 = time.perf_counter()
            with self._pose_lock:
                self._latest_pose = pose
            t8 = time.perf_counter()
    
        except Exception as e:
            self.preview_output.append(f"[VISION ERROR] {e}")
    
        loop_end = time.perf_counter()
    
        print(f"""
    Ball update: {(t1-t0)*1000:.2f} ms
    PD compute: {(t2-t1)*1000:.3f} ms
    IK solve: {(t4-t3)*1000:.2f} ms
    Serial send: {(t6-t5)*1000:.2f} ms
    Visualizer: {(t8-t7)*1000:.2f} ms
    Total loop: {(loop_end-loop_start)*1000:.2f} ms
    """)