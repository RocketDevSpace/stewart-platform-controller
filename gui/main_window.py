"""
gui/main_window.py

Top-level application window.  Wires the clean modules together:
  SerialManager + ServoDriver + IKEngine + RoutineRunner +
  StewartVisualizer + ControlPanel + SerialMonitor +
  VisionControlWorker (in its own QThread) + VisionMonitorWindow.

Vision loop ownership: VisionControlWorker runs in _vision_thread.
Snapshots arrive via snapshot_ready signal → _on_control_snapshot().
The worker owns BallTracker and BallController; this file owns the
thread lifecycle, GUI sync, timing plot, and neutral-pose fallback.
"""

from __future__ import annotations

import pathlib
import time
from collections import deque

from PyQt5 import QtCore, QtGui
from PyQt5.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QVBoxLayout,
    QWidget,
)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from control.routine_runner import RoutineRunner
from core.ik_engine import IKEngine
from core.platform_state import Pose
from cv.vision_control_worker import ControlSnapshot, VisionControlWorker
from gui.control_panel import ControlPanel, DARK_QSS, ROUTINE_PLACEHOLDER
from gui.serial_monitor import SerialMonitor
from gui.vision_monitor import VisionMonitorWindow
from hardware.serial_manager import SerialManager
from hardware.servo_driver import ServoDriver
from settings import (
    AUTO_TRIM_ENABLED,
    BALL_TARGET_DEFAULT_X_MM,
    BALL_TARGET_DEFAULT_Y_MM,
    CAMERA_INDEX,
    CONTROL_LOOP_INTERVAL_MS,
    GUI_LOG_MAX_LINES,
    GUI_SNAPSHOT_HZ,
    LOG_EVERY_N,
    MANUAL_PITCH_TRIM_DEG,
    MANUAL_ROLL_TRIM_DEG,
    MAX_TILT_DEG,
    PD_DEFAULT_KD,
    PD_DEFAULT_KP,
    SERIAL_BAUD,
    SERIAL_PORT,
    TIMING_PLOT_POINTS,
    TRACKER_REACQUIRE_VALID_FRAMES,
    VISION_LOOP_HZ,
    VISUALIZER_HZ,
)
from visualization.visualizer3d import StewartVisualizer

_TIMING_KEYS = [
    "ball_update",
    "pd_compute",
    "ik_solve",
    "serial_enqueue",
    "visualizer_gui",
    "frame_to_worker_ms",
    "worker_to_gui_ms",
    "frame_to_cmd",
    "total",
]
_TIMING_COLORS = {
    "ball_update":       "#38bdf8",
    "pd_compute":        "#34d399",
    "ik_solve":          "#f43f5e",
    "serial_enqueue":    "#a78bfa",
    "visualizer_gui":    "#f59e0b",
    "frame_to_worker_ms": "#0ea5e9",
    "worker_to_gui_ms":  "#64748b",
    "frame_to_cmd":      "#22d3ee",
    "total":             "#e2e8f0",
}
_TIMING_WINDOW_S = 30.0
_PLOT_UPDATE_EVERY = 5


def _settings_path() -> pathlib.Path:
    return pathlib.Path(__file__).parent.parent / "settings.py"


class MainWindow(QWidget):
    serial_line_received = QtCore.pyqtSignal(str)

    # Routed to VisionControlWorker (connected when worker starts)
    vision_gains_updated = QtCore.pyqtSignal(float, float)
    vision_hsv_updated = QtCore.pyqtSignal(int, int, int, int, int, int)
    vision_target_updated = QtCore.pyqtSignal(float, float)
    vision_trim_updated = QtCore.pyqtSignal(float, float)
    vision_auto_trim_enabled = QtCore.pyqtSignal(bool)
    vision_trim_reset_requested = QtCore.pyqtSignal()
    vision_calibrate_home_set = QtCore.pyqtSignal(bool)
    vision_pd_autotune_enabled = QtCore.pyqtSignal(bool)
    vision_pd_autotune_auto_apply = QtCore.pyqtSignal(bool)
    vision_pd_autotune_apply = QtCore.pyqtSignal()
    vision_snapshot_consumed = QtCore.pyqtSignal()

    def __init__(self, ik_solver: object = None) -> None:  # noqa: ARG002
        super().__init__()
        self.setStyleSheet(DARK_QSS)  # app-wide dark theme; cascades to all children

        # --- Core engines ---
        self._ik = IKEngine()
        self._serial = SerialManager(SERIAL_PORT, SERIAL_BAUD)
        self._serial.connect()
        self._serial.set_receive_callback(
            lambda line: self.serial_line_received.emit(line)
        )
        self._servo = ServoDriver(self._serial)

        # --- Visualizer ---
        self._figure = Figure(facecolor="#0f1726")
        self._canvas = FigureCanvas(self._figure)
        self._canvas.setMinimumSize(380, 350)
        self._canvas.setStyleSheet("background: #0f1726;")
        self.visualizer = StewartVisualizer(self._canvas)

        # --- Timing plot ---
        self._timing_figure = Figure(figsize=(5, 2.2))
        self._timing_figure.patch.set_facecolor("#0f1726")
        self._timing_ax = self._timing_figure.add_subplot(111)
        self._timing_canvas = FigureCanvas(self._timing_figure)
        self._timing_canvas.setFixedHeight(210)
        self._init_timing_plot_style()

        self._timing_summary_label = QLabel(
            "Vision Timing Avg (ms): waiting for data..."
        )
        self._timing_summary_label.setMaximumHeight(22)

        timing_capacity = max(TIMING_PLOT_POINTS, int(VISION_LOOP_HZ * 35))
        self._timing_history: dict[str, deque] = {
            k: deque(maxlen=timing_capacity) for k in _TIMING_KEYS
        }
        self._timing_timestamps: deque = deque(maxlen=timing_capacity)
        self._y_zoom = 1.0

        # --- View widgets ---
        self.control_panel = ControlPanel()
        self.serial_monitor = SerialMonitor()
        self.control_panel.attach_below(self.serial_monitor)
        self._vision_monitor = VisionMonitorWindow()

        # --- Routine runner ---
        self.routine_runner = RoutineRunner(
            ik_engine=self._ik,
            serial_driver=self._servo,
            on_pose_update=self._on_routine_pose,
        )

        # --- Vision worker state ---
        self._vision_thread: QtCore.QThread | None = None
        self._vision_worker: VisionControlWorker | None = None
        self._vision_enabled = False
        self._vision_starting = False
        self._vision_counter = 0
        self._valid_streak = 0
        self._vision_z_setpoint = 0.0
        self._last_visualizer_update = 0.0
        self._last_neutral_send = 0.0

        # Mirror of control_panel settings (for routing to worker)
        self._kp = PD_DEFAULT_KP
        self._kd = PD_DEFAULT_KD
        self._target_x_mm = float(BALL_TARGET_DEFAULT_X_MM)
        self._target_y_mm = float(BALL_TARGET_DEFAULT_Y_MM)
        self._trim_roll_deg = float(MANUAL_ROLL_TRIM_DEG)
        self._trim_pitch_deg = float(MANUAL_PITCH_TRIM_DEG)
        self._auto_trim_enabled = bool(AUTO_TRIM_ENABLED)
        self._home_calibration_active = False
        self._last_home_calib_diag_ts = 0.0
        self._pd_autotune_enabled = False
        self._pd_autotune_auto_apply = False
        self._pd_autotune_has_suggestion = False

        # --- Routine timer ---
        self._routine_timer = QtCore.QTimer()
        self._routine_timer.setTimerType(QtCore.Qt.PreciseTimer)
        self._routine_timer.timeout.connect(self._routine_tick)

        # --- Wire signals ---
        self.serial_line_received.connect(self.serial_monitor.append_line)

        # ControlPanel — original signals
        self.control_panel.slider_changed.connect(self._on_slider_changed)
        self.control_panel.routine_selected.connect(self._on_routine_selected)
        self.control_panel.routine_cancelled.connect(self._on_routine_cancelled)
        self.control_panel.send_clicked.connect(self._on_send_clicked)
        self.control_panel.vision_toggled.connect(self._on_vision_toggled)
        self.control_panel.kp_changed.connect(self._on_kp_changed)
        self.control_panel.kd_changed.connect(self._on_kd_changed)
        self.control_panel.raw_command_sent.connect(self._on_raw_command_sent)

        # ControlPanel — new vision-control signals
        self.control_panel.target_changed.connect(self._on_target_changed)
        self.control_panel.trim_changed.connect(self._on_trim_changed)
        self.control_panel.auto_trim_toggled.connect(self._on_auto_trim_toggled)
        self.control_panel.calibrate_home_clicked.connect(
            self._on_calibrate_home_clicked
        )
        self.control_panel.reset_trim_clicked.connect(self._on_reset_trim_clicked)
        self.control_panel.save_trim_as_default_clicked.connect(
            self._on_save_trim_as_default
        )
        self.control_panel.autotune_enable_clicked.connect(
            self._on_autotune_enable_clicked
        )
        self.control_panel.autotune_apply_clicked.connect(
            self._on_autotune_apply_clicked
        )
        self.control_panel.autotune_auto_apply_clicked.connect(
            self._on_autotune_auto_apply_clicked
        )
        self.control_panel.hsv_changed.connect(self._on_hsv_changed)
        self.control_panel.open_vision_monitor_clicked.connect(
            self._vision_monitor.show
        )

        # Timing canvas scroll → y-zoom
        self._timing_canvas.mpl_connect("scroll_event", self._on_timing_scroll)

        # --- Layout ---
        right_layout = QVBoxLayout()
        right_layout.addWidget(self._canvas, stretch=3)
        right_layout.addWidget(self._timing_summary_label, stretch=0)
        right_layout.addWidget(self._timing_canvas, stretch=1)

        layout = QHBoxLayout()
        layout.addWidget(self.control_panel, stretch=0)
        layout.addLayout(right_layout, stretch=1)
        self.setLayout(layout)

    # ------------------------------------------------------------------
    # Timing plot
    # ------------------------------------------------------------------

    def _init_timing_plot_style(self) -> None:
        self._timing_ax.set_facecolor("#0f1726")
        self._timing_ax.set_title(
            "Vision Loop Timings (30s)", color="#d6e2ff", fontsize=8
        )
        self._timing_ax.tick_params(colors="#9fb4d9", labelsize=7)
        self._timing_ax.grid(True, alpha=0.2, color="#2a3b59")

    def _on_timing_scroll(self, event: object) -> None:
        btn = getattr(event, "button", None)
        if btn == "up":
            self._y_zoom = max(0.2, self._y_zoom * 0.9)
        elif btn == "down":
            self._y_zoom = min(10.0, self._y_zoom * 1.1)
        self._update_timing_diagnostics(force_redraw=True)

    # ------------------------------------------------------------------
    # Routine playback
    # ------------------------------------------------------------------

    def _on_routine_pose(self, pose: dict) -> None:
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
            self.control_panel.append_preview("[INFO] Routine preview cancelled.")
        self.routine_runner.cancel()
        self._routine_timer.stop()
        self.control_panel.set_sliders_enabled(True)
        self.control_panel.set_send_enabled(True)
        self.control_panel.set_cancel_enabled(False)
        self.control_panel.reset_routine_selector()
        self.visualizer.update_platform(self.control_panel.get_slider_values())

    # ------------------------------------------------------------------
    # Slider + send handlers
    # ------------------------------------------------------------------

    def _on_slider_changed(self, axis: str, value: int) -> None:  # noqa: ARG002
        pose = self.control_panel.get_slider_values()
        self._vision_z_setpoint = float(pose["z"])
        self.visualizer.update_platform(pose)

    def _on_send_clicked(self) -> None:
        routine_name = self.control_panel.current_routine()
        if routine_name != ROUTINE_PLACEHOLDER:
            if not self.routine_runner.load(routine_name):
                self.control_panel.append_preview("[ERROR] Routine not found.")
                return
            self.routine_runner.start_send()
            self.control_panel.set_sliders_enabled(False)
            self.control_panel.set_send_enabled(False)
            self.control_panel.set_cancel_enabled(True)
            self._routine_timer.start(CONTROL_LOOP_INTERVAL_MS)
            return

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
            self.control_panel.append_preview("[SERIAL ERROR] Send failed.")
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
            self.control_panel.append_preview("[INFO] Send canceled by user.")
            return
        self._serial.send(cmd.encode())
        self.serial_monitor.append_command(cmd)
        self.control_panel.append_preview(f"[INFO] Command sent: {cmd}")
        self.control_panel.clear_raw_input()

    # ------------------------------------------------------------------
    # Vision control panel — new signal handlers
    # ------------------------------------------------------------------

    def _on_kp_changed(self, kp: float) -> None:
        self._kp = kp
        self.vision_gains_updated.emit(self._kp, self._kd)

    def _on_kd_changed(self, kd: float) -> None:
        self._kd = kd
        self.vision_gains_updated.emit(self._kp, self._kd)

    def _on_target_changed(self, x_mm: float, y_mm: float) -> None:
        self._target_x_mm = x_mm
        self._target_y_mm = y_mm
        if self._home_calibration_active and (
            abs(x_mm - BALL_TARGET_DEFAULT_X_MM) > 1e-9
            or abs(y_mm - BALL_TARGET_DEFAULT_Y_MM) > 1e-9
        ):
            self._home_calibration_active = False
            self.control_panel.sync_calibrate_button(False)
            self.vision_calibrate_home_set.emit(False)
            self.control_panel.append_preview(
                "[AUTO HOME] calibration cancelled (target moved)"
            )
        self.vision_target_updated.emit(x_mm, y_mm)

    def _on_trim_changed(self, roll_deg: float, pitch_deg: float) -> None:
        self._trim_roll_deg = roll_deg
        self._trim_pitch_deg = pitch_deg
        self.vision_trim_updated.emit(roll_deg, pitch_deg)

    def _on_auto_trim_toggled(self, enabled: bool) -> None:
        self._auto_trim_enabled = enabled
        self.vision_auto_trim_enabled.emit(enabled)
        self.control_panel.append_preview(
            f"[AUTO HOME] auto-trim {'enabled' if enabled else 'disabled'}"
        )

    def _on_calibrate_home_clicked(self) -> None:
        if not self._vision_enabled or self._vision_worker is None:
            self.control_panel.append_preview(
                "[AUTO HOME] start vision mode first"
            )
            self.control_panel.sync_calibrate_button(False)
            return

        if self._home_calibration_active:
            self._home_calibration_active = False
            self.control_panel.sync_calibrate_button(False)
            self.vision_calibrate_home_set.emit(False)
            self.control_panel.append_preview("[AUTO HOME] calibration cancelled")
            return

        # Start calibration — reset autotune, center target, enable auto-trim
        self._pd_autotune_enabled = False
        self._pd_autotune_auto_apply = False
        self._pd_autotune_has_suggestion = False
        self.control_panel.sync_autotune_buttons(False, False)
        self.vision_pd_autotune_enabled.emit(False)
        self.vision_pd_autotune_auto_apply.emit(False)

        self._auto_trim_enabled = True
        self.control_panel.sync_auto_trim_button(True)
        self.vision_auto_trim_enabled.emit(True)

        cx = int(round(BALL_TARGET_DEFAULT_X_MM))
        cy = int(round(BALL_TARGET_DEFAULT_Y_MM))
        self.control_panel.sync_target(float(cx), float(cy))
        self.vision_target_updated.emit(float(cx), float(cy))
        self._target_x_mm = float(cx)
        self._target_y_mm = float(cy)

        self._home_calibration_active = True
        self._last_home_calib_diag_ts = 0.0
        self.control_panel.sync_calibrate_button(True)
        self.vision_calibrate_home_set.emit(True)
        self.control_panel.append_preview(
            "[AUTO HOME] calibration started at center target (0,0)"
        )

    def _on_reset_trim_clicked(self) -> None:
        self.vision_trim_reset_requested.emit()
        self.control_panel.append_preview(
            "[AUTO HOME] trim reset requested (config defaults)"
        )

    def _on_save_trim_as_default(self) -> None:
        roll = self._trim_roll_deg
        pitch = self._trim_pitch_deg
        settings_path = _settings_path()
        try:
            text = settings_path.read_text(encoding="utf-8")
            import re
            text = re.sub(
                r"^(MANUAL_ROLL_TRIM_DEG\s*=\s*)[\-0-9.]+",
                rf"\g<1>{roll}",
                text, flags=re.MULTILINE,
            )
            text = re.sub(
                r"^(MANUAL_PITCH_TRIM_DEG\s*=\s*)[\-0-9.]+",
                rf"\g<1>{pitch}",
                text, flags=re.MULTILINE,
            )
            settings_path.write_text(text, encoding="utf-8")
            self.control_panel.append_preview(
                f"[TRIM] saved roll={roll:.2f}° pitch={pitch:.2f}° to settings.py"
            )
        except Exception as exc:
            self.control_panel.append_preview(f"[TRIM] save failed: {exc}")

    def _on_autotune_enable_clicked(self, enabled: bool) -> None:
        self._pd_autotune_enabled = enabled
        if not enabled:
            self._pd_autotune_auto_apply = False
            self._pd_autotune_has_suggestion = False
            self.control_panel.sync_autotune_buttons(False, False)
            self.control_panel.append_preview("[PD TUNE] disabled")
        else:
            self.control_panel.append_preview("[PD TUNE] enabled")
        self.vision_pd_autotune_enabled.emit(enabled)
        self.vision_pd_autotune_auto_apply.emit(self._pd_autotune_auto_apply)

    def _on_autotune_apply_clicked(self) -> None:
        if not self._pd_autotune_has_suggestion:
            self.control_panel.append_preview(
                "[PD TUNE] no recommendation available yet"
            )
            return
        self.vision_pd_autotune_apply.emit()
        self._pd_autotune_has_suggestion = False

    def _on_autotune_auto_apply_clicked(self, enabled: bool) -> None:
        self._pd_autotune_auto_apply = enabled
        if enabled:
            self._pd_autotune_enabled = True
            self._pd_autotune_has_suggestion = False
            self.control_panel.sync_autotune_buttons(True, True)
            self.control_panel.append_preview("[PD TUNE] auto-apply started")
        else:
            self._pd_autotune_enabled = False
            self._pd_autotune_has_suggestion = False
            self.control_panel.sync_autotune_buttons(False, False)
            self.control_panel.append_preview("[PD TUNE] auto-apply stopped")
        self.vision_pd_autotune_enabled.emit(self._pd_autotune_enabled)
        self.vision_pd_autotune_auto_apply.emit(enabled)

    def _on_hsv_changed(
        self, hmin: int, hmax: int, smin: int, smax: int, vmin: int, vmax: int
    ) -> None:
        self.vision_hsv_updated.emit(hmin, hmax, smin, smax, vmin, vmax)

    # ------------------------------------------------------------------
    # Vision mode enable / disable
    # ------------------------------------------------------------------

    def _on_vision_toggled(self, enable: bool) -> None:
        if enable and not self._vision_enabled:
            self._enable_vision_mode()
        elif not enable and self._vision_enabled:
            self._disable_vision_mode()

    def _enable_vision_mode(self) -> None:
        if self._vision_enabled or self._vision_starting:
            return
        self._vision_enabled = True
        self._vision_starting = True
        self._vision_counter = 0
        self._valid_streak = 0
        self._last_visualizer_update = 0.0
        for k in _TIMING_KEYS:
            self._timing_history[k].clear()
        self._timing_timestamps.clear()
        self.control_panel.set_vision_active(True)
        self.control_panel.set_sliders_enabled(False)
        self._vision_monitor.show()
        self._start_vision_worker()
        self.control_panel.append_preview("[INFO] Vision mode enabled.")

    def _disable_vision_mode(self) -> None:
        if not self._vision_enabled and self._vision_worker is None:
            return
        self._vision_enabled = False
        self._home_calibration_active = False
        self.control_panel.sync_calibrate_button(False)
        self._stop_vision_worker_async()
        self._timing_summary_label.setText(
            "Vision Timing Avg (ms): waiting for data..."
        )

    # ------------------------------------------------------------------
    # Worker thread lifecycle
    # ------------------------------------------------------------------

    def _start_vision_worker(self) -> None:
        if self._vision_thread is not None:
            return
        self._vision_thread = QtCore.QThread(self)
        self._vision_worker = VisionControlWorker(
            ik_solver=self._ik,
            z_provider=lambda: self._vision_z_setpoint,
            kp=self._kp,
            kd=self._kd,
            max_tilt_deg=MAX_TILT_DEG,
            camera_index=CAMERA_INDEX,
            loop_hz=VISION_LOOP_HZ,
            snapshot_hz=GUI_SNAPSHOT_HZ,
            command_sender=self._servo.send_angles,
            roll_offset=self._trim_roll_deg,
            pitch_offset=self._trim_pitch_deg,
            auto_trim_enabled=self._auto_trim_enabled,
        )
        self._vision_worker.moveToThread(self._vision_thread)
        self._vision_thread.started.connect(self._vision_worker.start)
        self._vision_thread.finished.connect(self._on_vision_thread_finished)
        self._vision_worker.snapshot_ready.connect(self._on_control_snapshot)
        self._vision_worker.camera_ready.connect(self._on_vision_camera_ready)
        self._vision_worker.error.connect(self._on_vision_error)
        self._vision_worker.stopped.connect(self._on_vision_worker_stopped)

        # Route GUI signals to worker slots
        self.vision_gains_updated.connect(self._vision_worker.set_gains)
        self.vision_hsv_updated.connect(self._vision_worker.set_hsv)
        self.vision_target_updated.connect(self._vision_worker.set_target)
        self.vision_trim_updated.connect(self._vision_worker.set_trim)
        self.vision_auto_trim_enabled.connect(
            self._vision_worker.set_auto_trim_enabled
        )
        self.vision_trim_reset_requested.connect(self._vision_worker.reset_trim)
        self.vision_calibrate_home_set.connect(
            self._vision_worker.set_home_calibration
        )
        self.vision_pd_autotune_enabled.connect(
            self._vision_worker.set_pd_autotune_enabled
        )
        self.vision_pd_autotune_auto_apply.connect(
            self._vision_worker.set_pd_autotune_auto_apply
        )
        self.vision_pd_autotune_apply.connect(
            self._vision_worker.apply_pd_autotune_recommendation
        )
        self.vision_snapshot_consumed.connect(
            self._vision_worker.mark_snapshot_consumed
        )

        self._vision_thread.start()

        # Push current GUI state to worker
        self.vision_gains_updated.emit(self._kp, self._kd)
        self.vision_target_updated.emit(self._target_x_mm, self._target_y_mm)
        self.vision_trim_updated.emit(self._trim_roll_deg, self._trim_pitch_deg)
        self.vision_auto_trim_enabled.emit(self._auto_trim_enabled)
        self.vision_calibrate_home_set.emit(self._home_calibration_active)
        self.vision_pd_autotune_enabled.emit(self._pd_autotune_enabled)
        self.vision_pd_autotune_auto_apply.emit(self._pd_autotune_auto_apply)

        # Push current HSV sliders
        sld = self.control_panel
        self.vision_hsv_updated.emit(
            sld._hsv_h_min_slider.value(),
            sld._hsv_h_max_slider.value(),
            sld._hsv_s_min_slider.value(),
            sld._hsv_s_max_slider.value(),
            sld._hsv_v_min_slider.value(),
            sld._hsv_v_max_slider.value(),
        )

    def _stop_vision_worker_async(self) -> None:
        if self._vision_worker is None:
            self._on_vision_worker_stopped()
            return

        # Disconnect routed signals to avoid stale calls after stop
        for sig, slot in [
            (self.vision_gains_updated, self._vision_worker.set_gains),
            (self.vision_hsv_updated, self._vision_worker.set_hsv),
            (self.vision_target_updated, self._vision_worker.set_target),
            (self.vision_trim_updated, self._vision_worker.set_trim),
            (self.vision_auto_trim_enabled,
             self._vision_worker.set_auto_trim_enabled),
            (self.vision_trim_reset_requested, self._vision_worker.reset_trim),
            (self.vision_calibrate_home_set,
             self._vision_worker.set_home_calibration),
            (self.vision_pd_autotune_enabled,
             self._vision_worker.set_pd_autotune_enabled),
            (self.vision_pd_autotune_auto_apply,
             self._vision_worker.set_pd_autotune_auto_apply),
            (self.vision_pd_autotune_apply,
             self._vision_worker.apply_pd_autotune_recommendation),
            (self.vision_snapshot_consumed,
             self._vision_worker.mark_snapshot_consumed),
        ]:
            try:
                sig.disconnect(slot)
            except TypeError:
                pass

        QtCore.QMetaObject.invokeMethod(
            self._vision_worker, "stop", QtCore.Qt.QueuedConnection
        )

    def _on_vision_worker_stopped(self) -> None:
        if self._vision_thread is not None:
            self._vision_thread.quit()
        else:
            self._on_vision_thread_finished()

    def _on_vision_thread_finished(self) -> None:
        if self._vision_thread is not None:
            self._vision_thread.deleteLater()
        self._vision_thread = None
        self._vision_worker = None
        self._vision_enabled = False
        self._vision_starting = False
        self.control_panel.set_vision_active(False)
        self.control_panel.set_sliders_enabled(True)
        self.control_panel.sync_calibrate_button(False)
        self.control_panel.append_preview("[INFO] Vision mode disabled.")

    def _on_vision_error(self, msg: str) -> None:
        self.control_panel.append_preview(f"[VISION ERROR] {msg}")
        if self._vision_starting:
            self._vision_starting = False

    def _on_vision_camera_ready(self, info: dict) -> None:
        try:
            lower = info.get("hsv_lower", [0, 0, 0])
            upper = info.get("hsv_upper", [179, 255, 255])
            hmin, smin, vmin = int(lower[0]), int(lower[1]), int(lower[2])
            hmax, smax, vmax = int(upper[0]), int(upper[1]), int(upper[2])
            self.control_panel.sync_hsv(hmin, hmax, smin, smax, vmin, vmax)
            period = float(info.get("period_ms", 0.0))
            fps = (1000.0 / period) if period > 0 else 0.0
            self.control_panel.append_preview(
                f"[CAMERA] backend={info.get('backend', 'unknown')} "
                f"mode={info.get('mode', '')} "
                f"period={period:.1f}ms (~{fps:.1f} FPS) "
                f"gray={float(info.get('gray', 0.0)):.1f}"
            )
        except Exception:
            pass
        self._vision_starting = False

    # ------------------------------------------------------------------
    # Snapshot handler
    # ------------------------------------------------------------------

    def _on_control_snapshot(self, snapshot: ControlSnapshot) -> None:
        try:
            if not self._vision_enabled:
                return

            self._update_camera_views(snapshot)

            if not snapshot.tracking_valid:
                if snapshot.reason == "stale_frame_repeat":
                    self._vision_counter += 1
                    if self._vision_counter % LOG_EVERY_N == 0:
                        self.control_panel.append_preview(
                            "[TRACK] stale frame repeat; waiting for fresh frame"
                        )
                    return

                self._valid_streak = 0
                now = time.perf_counter()
                miss = snapshot.miss_count
                if miss >= 20 and (now - self._last_neutral_send) > 0.5:
                    self._send_neutral_pose()
                    self._last_neutral_send = now

                self._vision_counter += 1
                if self._vision_counter % LOG_EVERY_N == 0:
                    self.control_panel.append_preview(
                        f"[TRACK] no ball (miss={miss}) reason={snapshot.reason}"
                    )
                return

            self._valid_streak += 1
            if self._valid_streak < TRACKER_REACQUIRE_VALID_FRAMES:
                if self._vision_counter % LOG_EVERY_N == 0:
                    self.control_panel.append_preview(
                        f"[TRACK] reacquire gating "
                        f"{self._valid_streak}/{TRACKER_REACQUIRE_VALID_FRAMES}"
                    )
                return

            if not snapshot.ik_success:
                self._vision_counter += 1
                if self._vision_counter % LOG_EVERY_N == 0:
                    self.control_panel.append_preview(
                        "[WARN] IK failed in vision loop."
                    )
                return

            safe_angles = [max(0, min(180, int(a))) for a in snapshot.servo_angles]

            # Visualizer at VISUALIZER_HZ
            now_perf = time.perf_counter()
            vis_ms = 0.0
            if now_perf - self._last_visualizer_update >= 1.0 / max(1, VISUALIZER_HZ):
                self._last_visualizer_update = now_perf
                t0 = time.perf_counter()
                self.visualizer.update_platform(
                    snapshot.pose, ik_result=snapshot.ik_result
                )
                vis_ms = (time.perf_counter() - t0) * 1000.0

            # Sync GUI from worker control_terms
            terms = snapshot.control_terms
            self._sync_gain_sliders_from_terms(terms)
            self._sync_autotune_from_terms(terms)
            self._sync_target_from_terms(terms)
            self._sync_trim_from_terms(terms)
            self._sync_auto_trim_from_terms(terms)
            self._sync_home_calibration_from_terms(terms)

            # Timing history
            timings = dict(snapshot.timings_ms)
            timings["serial_enqueue"] = float(timings.get("cmd_enqueue_worker", 0.0))
            timings["visualizer_gui"] = vis_ms
            wets = snapshot.worker_emit_perf_ts
            timings["worker_to_gui_ms"] = (
                max(0.0, (now_perf - wets) * 1000.0) if wets > 0 else 0.0
            )
            timings["frame_to_cmd"] = float(timings.get("frame_to_cmd_worker", 0.0))

            now_wall = time.time()
            self._timing_timestamps.append(now_wall)
            for key in _TIMING_KEYS:
                self._timing_history[key].append(float(timings.get(key, 0.0)))
            self._trim_timing_history(now_wall)
            self._update_timing_diagnostics()
            self._vision_counter += 1

            # Autotune event log
            tune_evt = terms.get("pd_autotune_event")
            if tune_evt:
                self.control_panel.append_preview(
                    f"[PD TUNE] {tune_evt.get('message', '')}"
                )

            # Home calibration diagnostics every 0.5 s
            if self._home_calibration_active:
                now_diag = time.perf_counter()
                if (now_diag - self._last_home_calib_diag_ts) >= 0.5:
                    self._last_home_calib_diag_ts = now_diag
                    self.control_panel.append_preview(
                        "[AUTO HOME] "
                        f"state={terms.get('auto_trim_state', 'n/a')} "
                        f"gate={terms.get('auto_trim_gate_reason', 'n/a')} "
                        f"elapsed={terms.get('home_calibration_elapsed_s', 0.0):.1f}s "
                        f"settled={terms.get('trim_settled_s', 0.0):.2f}/"
                        f"{terms.get('auto_trim_hold_s', 0.0):.2f}s "
                        f"trim=(r{terms.get('roll_offset', 0.0):+.3f},"
                        f"p{terms.get('pitch_offset', 0.0):+.3f})"
                    )

            # Periodic control trace
            if self._vision_counter % LOG_EVERY_N == 0:
                bs = snapshot.ball_state
                if bs is not None:
                    self.control_panel.append_preview(
                        "[CONTROL TRACE] "
                        f"ball=({bs.x_mm:.2f},{bs.y_mm:.2f})mm "
                        f"vel=({bs.vx_mm_s:.2f},{bs.vy_mm_s:.2f})mm/s "
                        f"pd=({terms.get('pd_vec', [0, 0])[0]:.4f},"
                        f"{terms.get('pd_vec', [0, 0])[1]:.4f}) "
                        f"cmd=(r{terms.get('roll_cmd', 0.0):.3f},"
                        f"p{terms.get('pitch_cmd', 0.0):.3f},"
                        f"z{snapshot.pose.get('z', 0.0):.1f}) "
                        f"kp={terms.get('kp', 0.0):.4f} "
                        f"kd={terms.get('kd', 0.0):.4f} "
                        f"w2g={timings.get('worker_to_gui_ms', 0.0):.1f}ms "
                        f"angles={safe_angles}"
                    )

            self._trim_preview_log()
        finally:
            self.vision_snapshot_consumed.emit()

    def _send_neutral_pose(self) -> None:
        neutral = Pose(x=0.0, y=0.0, z=self._vision_z_setpoint,
                       roll=0.0, pitch=0.0, yaw=0.0)
        ik = self._ik.solve(neutral, None)
        if ik.get("success"):
            safe = [
                float(max(0, min(180, int(round(a)))))
                for a in ik["servo_angles_deg"]
            ]
            self._servo.send_angles(safe)

    # ------------------------------------------------------------------
    # Sync methods — push worker state back to GUI without feedback loops
    # ------------------------------------------------------------------

    def _sync_gain_sliders_from_terms(self, terms: dict) -> None:
        kp = float(terms.get("kp", self._kp))
        kd = float(terms.get("kd", self._kd))
        if abs(kp - self._kp) > 1e-6 or abs(kd - self._kd) > 1e-6:
            self._kp = kp
            self._kd = kd
            self.control_panel.sync_kp_kd(kp, kd)

    def _sync_target_from_terms(self, terms: dict) -> None:
        tx = float(terms.get("target_x_mm", self._target_x_mm))
        ty = float(terms.get("target_y_mm", self._target_y_mm))
        if abs(tx - self._target_x_mm) > 0.5 or abs(ty - self._target_y_mm) > 0.5:
            self._target_x_mm = tx
            self._target_y_mm = ty
            self.control_panel.sync_target(tx, ty)

    def _sync_trim_from_terms(self, terms: dict) -> None:
        roll = float(terms.get("roll_offset", self._trim_roll_deg))
        pitch = float(terms.get("pitch_offset", self._trim_pitch_deg))
        if abs(roll - self._trim_roll_deg) > 0.005 or abs(pitch - self._trim_pitch_deg) > 0.005:
            self._trim_roll_deg = roll
            self._trim_pitch_deg = pitch
            self.control_panel.sync_trim(roll, pitch)

    def _sync_auto_trim_from_terms(self, terms: dict) -> None:
        enabled = bool(terms.get("auto_trim_enabled", self._auto_trim_enabled))
        if enabled != self._auto_trim_enabled:
            self._auto_trim_enabled = enabled
            self.control_panel.sync_auto_trim_button(enabled)

    def _sync_home_calibration_from_terms(self, terms: dict) -> None:
        active = bool(
            terms.get("home_calibration_active", self._home_calibration_active)
        )
        if active != self._home_calibration_active:
            self._home_calibration_active = active
            self.control_panel.sync_calibrate_button(active)

    def _sync_autotune_from_terms(self, terms: dict) -> None:
        enabled = bool(
            terms.get("pd_autotune_enabled", self._pd_autotune_enabled)
        )
        auto_apply = bool(
            terms.get("pd_autotune_auto_apply", self._pd_autotune_auto_apply)
        )
        has_suggestion = bool(
            terms.get("pd_autotune_has_suggestion", self._pd_autotune_has_suggestion)
        )
        changed = (
            enabled != self._pd_autotune_enabled
            or auto_apply != self._pd_autotune_auto_apply
            or has_suggestion != self._pd_autotune_has_suggestion
        )
        self._pd_autotune_enabled = enabled
        self._pd_autotune_auto_apply = auto_apply
        self._pd_autotune_has_suggestion = has_suggestion
        if changed:
            self.control_panel.sync_autotune_buttons(enabled, auto_apply)

    # ------------------------------------------------------------------
    # Camera views
    # ------------------------------------------------------------------

    def _update_camera_views(self, snapshot: ControlSnapshot) -> None:
        self._vision_monitor.update_warped(
            getattr(snapshot, "warped_bgr", None)
        )
        self._vision_monitor.update_camera(
            getattr(snapshot, "camera_bgr", None)
        )
        self._vision_monitor.update_mask(
            getattr(snapshot, "mask_gray", None)
        )

    # ------------------------------------------------------------------
    # Timing diagnostics
    # ------------------------------------------------------------------

    def _trim_timing_history(self, now: float) -> None:
        while (
            self._timing_timestamps
            and (now - self._timing_timestamps[0]) > _TIMING_WINDOW_S
        ):
            self._timing_timestamps.popleft()
            for key in _TIMING_KEYS:
                if self._timing_history[key]:
                    self._timing_history[key].popleft()

    def _update_timing_diagnostics(self, force_redraw: bool = False) -> None:
        if not self._timing_history["total"]:
            return
        avgs = {}
        for key in _TIMING_KEYS:
            vals = self._timing_history[key]
            avgs[key] = (sum(vals) / len(vals)) if vals else 0.0
        self._timing_summary_label.setText(
            "Vision Timing Avg (ms):  "
            f"ball={avgs['ball_update']:.2f}  "
            f"pd={avgs['pd_compute']:.2f}  "
            f"ik={avgs['ik_solve']:.2f}  "
            f"serQ={avgs['serial_enqueue']:.2f}  "
            f"vis={avgs['visualizer_gui']:.2f}  "
            f"f2w={avgs['frame_to_worker_ms']:.2f}  "
            f"w2g={avgs['worker_to_gui_ms']:.2f}  "
            f"f2c={avgs['frame_to_cmd']:.2f}  "
            f"total={avgs['total']:.2f}  "
            f"zoom={self._y_zoom:.2f}x"
        )

        if not force_redraw and (self._vision_counter % _PLOT_UPDATE_EVERY != 0):
            return
        if not self._timing_timestamps:
            return

        now_ref = self._timing_timestamps[-1]
        x = [t - now_ref for t in self._timing_timestamps]
        self._timing_ax.cla()
        self._timing_ax.set_facecolor("#0f1726")
        self._timing_ax.set_title(
            "Vision Loop Timings (30s)", color="#d6e2ff", fontsize=8
        )
        self._timing_ax.tick_params(colors="#9fb4d9", labelsize=7)
        self._timing_ax.grid(True, alpha=0.2, color="#2a3b59")
        self._timing_ax.set_xlim(-_TIMING_WINDOW_S, 0.0)
        self._timing_ax.set_xlabel("Time (s)", color="#9fb4d9", fontsize=7)
        self._timing_ax.set_ylabel("ms", color="#9fb4d9", fontsize=7)

        y_max = 1.0
        for key in _TIMING_KEYS:
            y = list(self._timing_history[key])
            if y:
                self._timing_ax.plot(
                    x, y, label=key, linewidth=1.2, color=_TIMING_COLORS[key]
                )
                y_max = max(y_max, max(y))
        self._timing_ax.set_ylim(0.0, y_max * self._y_zoom)
        self._timing_ax.legend(loc="upper right", fontsize=6)
        self._timing_canvas.draw_idle()

    # ------------------------------------------------------------------
    # Log trimming
    # ------------------------------------------------------------------

    def _trim_preview_log(self) -> None:
        doc = self.control_panel._preview_output.document()
        cursor = QtGui.QTextCursor(doc)
        while doc.blockCount() > GUI_LOG_MAX_LINES:
            cursor.movePosition(QtGui.QTextCursor.Start)
            cursor.select(QtGui.QTextCursor.BlockUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def closeEvent(self, event: object) -> None:  # noqa: N802
        if self._routine_timer.isActive():
            self._routine_timer.stop()
        try:
            self._routine_timer.timeout.disconnect()
        except TypeError:
            pass

        if self._vision_worker is not None:
            self._disable_vision_mode()
            if self._vision_thread is not None:
                if not self._vision_thread.wait(3000):
                    self.control_panel.append_preview(
                        "[WARN] Vision thread did not stop within 3 s."
                    )

        self._vision_monitor.close()
        self._serial.disconnect()
        event.accept()  # type: ignore[attr-defined]
