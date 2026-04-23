"""
gui/main_window.py

Top-level application window. Wires the clean modules together:
SerialManager + ServoDriver + IKEngine + RoutineRunner + StewartVisualizer +
ControlPanel + SerialMonitor + BallTracker + BallController.

This file owns:
  - QTimer for routine playback (calls routine_runner.tick())
  - QTimer for vision control loop
  - background visualizer thread (decouples slow matplotlib draws from
    the vision loop)
  - closeEvent cleanup (timers, threads, serial)

All control logic lives in the on_* signal handlers below. The view
widgets (ControlPanel, SerialMonitor) emit signals; this class translates
them into IK calls, serial dispatches, and visualizer updates.
"""

from __future__ import annotations

import threading
import time

from PyQt5 import QtCore
from PyQt5.QtWidgets import QHBoxLayout, QWidget
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from control.routine_runner import RoutineRunner
from core.ik_engine import IKEngine
from core.platform_state import Pose
from cv.ball_controller import BallController
from cv.ball_tracker import BallTracker
from gui.control_panel import ControlPanel, ROUTINE_PLACEHOLDER
from gui.serial_monitor import SerialMonitor
from hardware.serial_manager import SerialManager
from hardware.servo_driver import ServoDriver
from settings import (
    CONTROL_LOOP_INTERVAL_MS,
    SERIAL_BAUD,
    SERIAL_PORT,
    VISION_LOOP_INTERVAL_MS,
)
from visualization.visualizer3d import StewartVisualizer

VISUALIZER_THREAD_INTERVAL_S = 0.03


class MainWindow(QWidget):
    serial_line_received = QtCore.pyqtSignal(str)

    def __init__(self, ik_solver: object = None) -> None:  # noqa: ARG002
        # ik_solver accepted for API compatibility but ignored —
        # MainWindow instantiates IKEngine directly. Remove in M6.
        super().__init__()

        # --- Core engines ---
        self._ik = IKEngine()
        self._serial = SerialManager(SERIAL_PORT, SERIAL_BAUD)
        self._serial.connect()
        self._serial.set_receive_callback(
            lambda line: self.serial_line_received.emit(line)
        )
        self._servo = ServoDriver(self._serial)

        # --- Visualizer (needs canvas) ---
        self._figure = Figure()
        self._canvas = FigureCanvas(self._figure)
        self._canvas.setFixedSize(500, 500)
        self.visualizer = StewartVisualizer(self._canvas)

        # --- View widgets ---
        self.control_panel = ControlPanel()
        self.serial_monitor = SerialMonitor()
        self.control_panel.attach_below(self.serial_monitor)

        # --- Routine runner (Qt-free state machine) ---
        self.routine_runner = RoutineRunner(
            ik_engine=self._ik,
            serial_driver=self._servo,
            on_pose_update=self._on_routine_pose,
        )

        # --- Ball balancing ---
        self.ball_tracker = BallTracker()
        self.ball_controller = BallController(
            kp=0.005, kd=0.010, max_tilt_deg=8.0
        )
        self._vision_enabled = False

        # --- Timers ---
        self._routine_timer = QtCore.QTimer()
        self._routine_timer.setTimerType(QtCore.Qt.PreciseTimer)
        self._routine_timer.timeout.connect(self._routine_tick)

        self._vision_timer = QtCore.QTimer()
        self._vision_timer.setTimerType(QtCore.Qt.PreciseTimer)
        self._vision_timer.timeout.connect(self._vision_control_step)

        # --- Background visualizer thread (decouples vision loop from draw) ---
        self._latest_pose: dict | None = None
        self._pose_lock = threading.Lock()
        self._vis_thread_running = True
        self._vis_thread = threading.Thread(
            target=self._visualizer_loop, daemon=True
        )
        self._vis_thread.start()

        # --- Wire signals ---
        self.serial_line_received.connect(self.serial_monitor.append_line)
        self.control_panel.slider_changed.connect(self._on_slider_changed)
        self.control_panel.routine_selected.connect(self._on_routine_selected)
        self.control_panel.routine_cancelled.connect(self._on_routine_cancelled)
        self.control_panel.send_clicked.connect(self._on_send_clicked)
        self.control_panel.vision_toggled.connect(self._on_vision_toggled)
        self.control_panel.kp_changed.connect(self._on_kp_changed)
        self.control_panel.kd_changed.connect(self._on_kd_changed)
        self.control_panel.raw_command_sent.connect(self._on_raw_command_sent)

        # --- Layout ---
        layout = QHBoxLayout()
        layout.addWidget(self.control_panel)
        layout.addWidget(self._canvas)
        self.setLayout(layout)

    # ------------------------------------------------------------------
    # Routine playback
    # ------------------------------------------------------------------

    def _on_routine_pose(self, pose: dict) -> None:
        """Callback from RoutineRunner: mirror pose to sliders + visualizer."""
        self.control_panel.set_slider_values(pose)
        self.visualizer.update_platform(pose)

    def _routine_tick(self) -> None:
        still_going = self.routine_runner.tick()
        if not still_going and not self.routine_runner.is_running:
            self._routine_timer.stop()
            self.control_panel.set_sliders_enabled(True)
            self.control_panel.set_send_enabled(True)
            self.control_panel.set_cancel_enabled(False)

    def _on_routine_selected(self, name: str) -> None:
        self.control_panel.append_preview(f"[INFO] Previewing routine: {name}")

        # Seed prev_arm_points from current slider pose for visual continuity
        current = self.control_panel.get_slider_values()
        seed = self._ik.solve(
            Pose(**{k: float(v) for k, v in current.items()}),
            self.visualizer.prev_arm_points,
        )
        if seed.get("success"):
            self.visualizer.prev_arm_points = seed.get("arm_points")

        if not self.routine_runner.load(name):
            self.control_panel.append_preview("[ERROR] Routine not found.")
            return
        if self.routine_runner.total_steps < 2:
            self.control_panel.append_preview("[ERROR] Routine has no poses.")
            return

        self.routine_runner.start_preview()
        self.control_panel.set_sliders_enabled(False)
        self.control_panel.set_cancel_enabled(True)
        self._routine_timer.start(CONTROL_LOOP_INTERVAL_MS)

    def _on_routine_cancelled(self) -> None:
        if self.routine_runner.is_running:
            self.control_panel.append_preview(
                "[INFO] Routine preview cancelled."
            )
        self.routine_runner.cancel()
        self._routine_timer.stop()
        self.control_panel.set_sliders_enabled(True)
        self.control_panel.set_send_enabled(True)
        self.control_panel.set_cancel_enabled(False)
        self.control_panel.reset_routine_selector()

        pose = self.control_panel.get_slider_values()
        self.visualizer.update_platform(pose)

    # ------------------------------------------------------------------
    # Slider + send handlers
    # ------------------------------------------------------------------

    def _on_slider_changed(self, axis: str, value: int) -> None:  # noqa: ARG002
        pose = self.control_panel.get_slider_values()
        self.visualizer.update_platform(pose)

    def _on_send_clicked(self) -> None:
        routine_name = self.control_panel.current_routine()
        if routine_name != ROUTINE_PLACEHOLDER:
            if not self.routine_runner.load(routine_name):
                self.control_panel.append_preview(
                    "[ERROR] Routine not found."
                )
                return
            self.routine_runner.start_send()
            self.control_panel.set_sliders_enabled(False)
            self.control_panel.set_send_enabled(False)
            self.control_panel.set_cancel_enabled(True)
            self._routine_timer.start(CONTROL_LOOP_INTERVAL_MS)
            return

        # Slider-driven send
        pose_dict = self.control_panel.get_slider_values()
        pose = Pose(**{k: float(v) for k, v in pose_dict.items()})

        t0 = time.perf_counter()
        ik_result = self._ik.solve(pose, self.visualizer.prev_arm_points)
        ik_ms = (time.perf_counter() - t0) * 1000
        self.control_panel.append_preview(
            f"[PROFILE] IK solve time = {ik_ms:.2f} ms"
        )

        if not ik_result.get("success"):
            self.control_panel.append_preview("[WARN] IK solver failed.")
            return

        angles = list(ik_result["servo_angles_deg"])
        cmd_preview = self._servo.format_command(angles).strip()

        if not self.control_panel.confirm(
            "Confirm Send",
            f"Send the following command to Arduino?\n{cmd_preview}",
        ):
            self.control_panel.append_preview("[INFO] Send canceled by user.")
            return

        self.control_panel.append_preview(f"[SEND] {cmd_preview}")
        t1 = time.perf_counter()
        ok = self._servo.send_angles(angles)
        send_ms = (time.perf_counter() - t1) * 1000
        if not ok:
            self.control_panel.append_preview(
                "[SERIAL ERROR] Send failed."
            )
            return
        self.control_panel.append_preview(
            f"[PROFILE] Serial send time = {send_ms:.2f} ms"
        )

        self.visualizer.update_platform(pose_dict, ik_result=ik_result)
        self.visualizer.prev_arm_points = ik_result.get("arm_points")

    def _on_raw_command_sent(self, cmd: str) -> None:
        if not self.control_panel.confirm(
            "Confirm Send",
            f"Send the following command to Arduino?\n{cmd}",
        ):
            self.control_panel.append_preview(
                "[INFO] Send canceled by user."
            )
            return
        self._serial.send(cmd.encode())
        self.serial_monitor.append_command(cmd)
        self.control_panel.append_preview(f"[INFO] Command sent: {cmd}")
        self.control_panel.clear_raw_input()

    # ------------------------------------------------------------------
    # Vision mode
    # ------------------------------------------------------------------

    def _on_vision_toggled(self, enable: bool) -> None:
        if enable and not self._vision_enabled:
            self._enable_vision()
        elif not enable and self._vision_enabled:
            self._disable_vision()

    def _enable_vision(self) -> None:
        self.control_panel.append_preview("[INFO] Vision mode enabled.")
        self._vision_enabled = True
        self.control_panel.set_vision_active(True)
        self.control_panel.set_sliders_enabled(False)
        self._vision_timer.start(VISION_LOOP_INTERVAL_MS)

    def _disable_vision(self) -> None:
        self.control_panel.append_preview("[INFO] Vision mode disabled.")
        self._vision_enabled = False
        self._vision_timer.stop()
        self.control_panel.set_vision_active(False)
        self.control_panel.set_sliders_enabled(True)

    def _on_kp_changed(self, kp: float) -> None:
        self.ball_controller.set_gains(kp, self.ball_controller.kd)

    def _on_kd_changed(self, kd: float) -> None:
        self.ball_controller.set_gains(self.ball_controller.kp, kd)

    def _vision_control_step(self) -> None:
        if not self._vision_enabled:
            return
        try:
            ball_state = self.ball_tracker.update()
            if ball_state is None:
                return

            roll_deg, pitch_deg = self.ball_controller.compute(ball_state)

            slider_z = self.control_panel.get_slider_values()["z"]
            pose_dict = {
                "x": 0,
                "y": 0,
                "z": slider_z,
                "roll": roll_deg,
                "pitch": pitch_deg,
                "yaw": 0,
            }
            pose = Pose(**{k: float(v) for k, v in pose_dict.items()})

            ik_result = self._ik.solve(pose, self.visualizer.prev_arm_points)
            if not ik_result.get("success"):
                self.control_panel.append_preview(
                    "[WARN] IK failed in vision loop."
                )
                return

            self.visualizer.prev_arm_points = ik_result.get("arm_points")
            self._servo.send_angles(list(ik_result["servo_angles_deg"]))

            with self._pose_lock:
                self._latest_pose = pose_dict
        except Exception as exc:
            self.control_panel.append_preview(f"[VISION ERROR] {exc}")

    # ------------------------------------------------------------------
    # Background visualizer thread
    # ------------------------------------------------------------------

    def _visualizer_loop(self) -> None:
        """Background draw loop fed by the vision control step.

        Decouples slow matplotlib redraws from the realtime vision/IK
        path. Routine playback drives the visualizer synchronously via
        the on_pose callback and bypasses this loop.
        """
        while self._vis_thread_running:
            pose = None
            with self._pose_lock:
                if self._latest_pose is not None:
                    pose = self._latest_pose.copy()
            if pose is not None:
                self.visualizer.update_platform(pose)
            time.sleep(VISUALIZER_THREAD_INTERVAL_S)

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def closeEvent(self, event: object) -> None:  # noqa: N802
        if self._routine_timer.isActive():
            self._routine_timer.stop()
        if self._vision_timer.isActive():
            self._vision_timer.stop()

        try:
            self._routine_timer.timeout.disconnect()
        except TypeError:
            pass
        try:
            self._vision_timer.timeout.disconnect()
        except TypeError:
            pass

        self._vis_thread_running = False
        self._serial.disconnect()

        event.accept()  # type: ignore[attr-defined]
