"""
gui/main_window.py

Top-level application window.  Wires the clean modules together:
  SerialManager + ServoDriver + IKEngine + PoseCommander + RoutineRunner +
  StewartVisualizer + ControlPanel + SerialMonitor + TimingPlotWidget +
  VisionControlWorker (in its own QThread) + VisionMonitorWindow.

Vision loop ownership: VisionControlWorker runs in _vision_thread.
Snapshots arrive via snapshot_ready signal → _on_control_snapshot().
The worker owns BallTracker + BallController AND the neutral-pose
fallback policy; this file owns the thread lifecycle and GUI sync only.
All IK solves and servo sends go through control/pose_commander.py —
gui/ files contain no control logic (hard constraint 5).
"""

from __future__ import annotations

import threading
import time

import numpy as np
from PyQt5 import QtCore
from PyQt5.QtWidgets import (
    QHBoxLayout,
    QVBoxLayout,
    QWidget,
)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import settings_store
from control.patterns import PATTERNS
from control.pose_commander import PoseCommander
from control.routine_runner import RoutineRunner
from core.ik_engine import IKEngine
from core.platform_state import Pose
from cv.vision_control_worker import ControlSnapshot, VisionControlWorker
from gui.control_panel import ControlPanel, DARK_QSS, ROUTINE_PLACEHOLDER
from gui.serial_monitor import SerialMonitor
from gui.timing_plot import TimingPlotWidget
from gui.vision_monitor import VisionMonitorWindow
from hardware.serial_manager import SerialManager
from hardware.servo_driver import ServoDriver
from settings import (
    PD_I_ENABLED,
    BALL_TARGET_DEFAULT_X_MM,
    BALL_TARGET_DEFAULT_Y_MM,
    CAMERA_INDEX,
    CONTROL_LOOP_INTERVAL_MS,
    GUI_SNAPSHOT_HZ,
    LOG_EVERY_N,
    MANUAL_PITCH_TRIM_DEG,
    MANUAL_ROLL_TRIM_DEG,
    MAX_TILT_DEG,
    PATH_SPEED_MM_S,
    PD_DEFAULT_KD,
    PD_DEFAULT_KP,
    SERIAL_BAUD,
    SERIAL_PORT,
    VISION_LOOP_HZ,
    VISION_POSITION_LOG_PATH,
    VISUALIZER_HZ,
)
from visualization.visualizer3d import StewartVisualizer


class MainWindow(QWidget):
    serial_line_received = QtCore.pyqtSignal(str)
    serial_link_lost = QtCore.pyqtSignal(str)
    # Emitted from the background connect thread → handled on the GUI thread
    serial_connect_finished = QtCore.pyqtSignal(bool)

    # Routed to VisionControlWorker (connected when worker starts)
    vision_gains_updated = QtCore.pyqtSignal(float, float)
    vision_hsv_updated = QtCore.pyqtSignal(int, int, int, int, int, int)
    vision_target_updated = QtCore.pyqtSignal(float, float)
    vision_trim_updated = QtCore.pyqtSignal(float, float)
    vision_z_updated = QtCore.pyqtSignal(float)
    vision_auto_trim_enabled = QtCore.pyqtSignal(bool)
    vision_trim_reset_requested = QtCore.pyqtSignal()
    vision_calibrate_home_set = QtCore.pyqtSignal(bool)
    vision_pd_autotune_enabled = QtCore.pyqtSignal(bool)
    vision_pd_autotune_auto_apply = QtCore.pyqtSignal(bool)
    vision_pd_autotune_apply = QtCore.pyqtSignal()
    vision_path_pattern_selected = QtCore.pyqtSignal(str)
    vision_path_following_set = QtCore.pyqtSignal(bool)
    vision_path_speed_updated = QtCore.pyqtSignal(float)
    vision_snapshot_consumed = QtCore.pyqtSignal()

    def __init__(self) -> None:
        super().__init__()
        self.setStyleSheet(DARK_QSS)  # app-wide dark theme; cascades to all children

        # --- Core engines ---
        # Constructed DISCONNECTED: SerialManager.connect() blocks ~2 s on
        # the Arduino boot handshake, so it runs on a background thread
        # after the window is built (see end of __init__).
        self._ik = IKEngine()
        self._serial = SerialManager(SERIAL_PORT, SERIAL_BAUD)
        self._serial.set_receive_callback(
            lambda line: self.serial_line_received.emit(line)
        )
        # Bridge unexpected link loss (USB unplug, I/O error) onto the GUI
        # thread — the callback fires on a background serial thread.
        self._serial.set_disconnect_callback(
            lambda reason: self.serial_link_lost.emit(reason)
        )
        self._servo = ServoDriver(self._serial)
        self._commander = PoseCommander(self._ik, self._servo)

        # --- Visualizer ---
        self._figure = Figure(facecolor="#0f1726")
        self._canvas = FigureCanvas(self._figure)
        self._canvas.setMinimumSize(380, 350)
        self._canvas.setStyleSheet("background: #0f1726;")
        self.visualizer = StewartVisualizer(self._canvas)

        # --- Timing plot ---
        self._timing_plot = TimingPlotWidget()

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
        self._vision_counter = 0
        self._vision_z_setpoint = 0.0
        self._last_visualizer_update = 0.0

        # Mirror of control_panel settings (for routing to worker)
        self._kp = PD_DEFAULT_KP
        self._kd = PD_DEFAULT_KD
        self._target_x_mm = float(BALL_TARGET_DEFAULT_X_MM)
        self._target_y_mm = float(BALL_TARGET_DEFAULT_Y_MM)
        self._trim_roll_deg = float(MANUAL_ROLL_TRIM_DEG)
        self._trim_pitch_deg = float(MANUAL_PITCH_TRIM_DEG)
        self._auto_trim_enabled = bool(PD_I_ENABLED)
        self._home_calibration_active = False
        self._last_home_calib_diag_ts = 0.0
        self._pd_autotune_enabled = False
        self._pd_autotune_auto_apply = False
        self._pd_autotune_has_suggestion = False
        # Worker-ack guard: after Apply Now, ignore has_suggestion=True from
        # stale snapshots until the worker reports has_suggestion False.
        self._autotune_apply_pending = False
        # Path following mirrors (worker owns the truth via control_terms)
        self._path_following_active = False
        self._path_speed_mm_s = float(PATH_SPEED_MM_S)
        self._last_path_status = ""

        # --- Routine timer ---
        self._routine_timer = QtCore.QTimer()
        self._routine_timer.setTimerType(QtCore.Qt.TimerType.PreciseTimer)
        self._routine_timer.timeout.connect(self._routine_tick)

        # --- Wire signals ---
        self.serial_line_received.connect(self.serial_monitor.append_line)
        self.serial_link_lost.connect(self._on_serial_link_lost)
        self.serial_connect_finished.connect(self._on_serial_connect_finished)

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
        self.control_panel.path_pattern_selected.connect(
            self._on_path_pattern_selected
        )
        self.control_panel.path_toggled.connect(self._on_path_toggled)
        self.control_panel.path_speed_changed.connect(
            self._on_path_speed_changed
        )
        self.control_panel.open_vision_monitor_clicked.connect(
            self._vision_monitor.show
        )

        # --- Layout ---
        right_layout = QVBoxLayout()
        right_layout.addWidget(self._canvas, stretch=3)
        right_layout.addWidget(self._timing_plot, stretch=1)

        layout = QHBoxLayout()
        layout.addWidget(self.control_panel, stretch=0)
        layout.addLayout(right_layout, stretch=1)
        self.setLayout(layout)

        # --- Non-blocking serial connect ---
        # SerialManager stays the owner of the connect/boot-wait logic; the
        # GUI only moves the blocking call off the GUI thread so the window
        # shows immediately. ServoDriver sends fail gracefully (return
        # False) until the link is up.
        self._serial_connect_thread = threading.Thread(
            target=self._connect_serial_blocking,
            daemon=True,
            name="serial-connect",
        )
        self._serial_connect_thread.start()

    # ------------------------------------------------------------------
    # Serial connect (background thread + GUI-thread completion slot)
    # ------------------------------------------------------------------

    def _connect_serial_blocking(self) -> None:
        """Runs on the serial-connect thread. Never raises."""
        try:
            ok = self._serial.connect()
        except Exception:
            ok = False
        self.serial_connect_finished.emit(ok)

    def _on_serial_connect_finished(self, ok: bool) -> None:
        if ok:
            msg = f"[SERIAL] Connected on {SERIAL_PORT}"
        else:
            msg = (
                f"[SERIAL] Connection on {SERIAL_PORT} FAILED — check the "
                "cable and the port in settings.py, then restart the app."
            )
        self.control_panel.append_preview(msg)
        self.serial_monitor.append_line(msg)

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
            if self.routine_runner.ik_failure_count:
                self.control_panel.append_preview(
                    f"[WARN] Routine skipped "
                    f"{self.routine_runner.ik_failure_count} steps "
                    "(IK failures) — platform held previous pose there."
                )
            # Never re-enable manual controls that vision mode has locked.
            if not self._vision_enabled:
                self.control_panel.set_sliders_enabled(True)
                self.control_panel.set_send_enabled(True)
            self.control_panel.set_cancel_enabled(False)

    def _on_routine_selected(self, name: str) -> None:
        if self._vision_enabled:
            self.control_panel.append_preview(
                "[MODE] Routines disabled while vision mode is active."
            )
            self.control_panel.reset_routine_selector()
            return
        self.control_panel.append_preview(f"[INFO] Previewing routine: {name}")

        current = self.control_panel.get_slider_values()
        seed = self._commander.solve(
            Pose(**{k: float(v) for k, v in current.items()}),
            self.visualizer.prev_arm_points,
        )
        if seed.success:
            self.visualizer.prev_arm_points = np.asarray(seed.arm_points)

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
            self.control_panel.append_preview("[INFO] Routine cancelled.")
        winding_down = self.routine_runner.cancel()
        if winding_down:
            # Send-mode cancel: the runner is easing the platform back to
            # neutral; keep the timer ticking until it lands (the
            # end-of-routine branch in _routine_tick restores the UI).
            self.control_panel.append_preview(
                "[INFO] Returning platform to neutral..."
            )
            return
        self._routine_timer.stop()
        if not self._vision_enabled:
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
        self.vision_z_updated.emit(self._vision_z_setpoint)
        self.visualizer.update_platform(pose)

    def _on_send_clicked(self) -> None:
        if self._vision_enabled:
            self.control_panel.append_preview(
                "[MODE] Manual send disabled while vision mode is active."
            )
            return
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
        ik_result = self._commander.solve(pose, self.visualizer.prev_arm_points)
        ik_ms = (time.perf_counter() - t0) * 1000
        self.control_panel.append_preview(
            f"[PROFILE] IK solve time = {ik_ms:.2f} ms"
        )

        if not ik_result.success:
            self.control_panel.append_preview(
                f"[WARN] IK solver failed: {ik_result.servo_status}"
            )
            return

        cmd_preview = self._servo.format_command(
            list(ik_result.servo_angles_deg)
        ).strip()

        if not self.control_panel.confirm(
            "Confirm Send",
            f"Send the following command to Arduino?\n{cmd_preview}",
        ):
            self.control_panel.append_preview("[INFO] Send canceled by user.")
            return

        self.control_panel.append_preview(f"[SEND] {cmd_preview}")
        t1 = time.perf_counter()
        ok = self._commander.send(ik_result)
        send_ms = (time.perf_counter() - t1) * 1000
        if not ok:
            self.control_panel.append_preview("[SERIAL ERROR] Send failed.")
            return
        self.control_panel.append_preview(
            f"[PROFILE] Serial send time = {send_ms:.2f} ms"
        )
        self.visualizer.update_platform(pose_dict, ik_result=ik_result)
        self.visualizer.prev_arm_points = np.asarray(ik_result.arm_points)

    def _on_raw_command_sent(self, cmd: str) -> None:
        if self._vision_enabled:
            self.control_panel.append_preview(
                "[MODE] Raw send disabled while vision mode is active."
            )
            return
        if not self.control_panel.confirm(
            "Confirm Send",
            f"Send the following command to Arduino?\n{cmd}",
        ):
            self.control_panel.append_preview("[INFO] Send canceled by user.")
            return
        # ServoDriver validates, safety-clips, ramps large jumps, and appends
        # the newline terminator — raw text must not bypass those rails.
        if self._servo.send_raw(cmd):
            self.serial_monitor.append_command(cmd)
            self.control_panel.append_preview(f"[INFO] Command sent: {cmd}")
        else:
            self.control_panel.append_preview(
                f"[ERROR] Raw command rejected or send failed: {cmd}"
            )
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
        if self._path_following_active:
            self._stop_path_following("[PATH] stopped (target moved)")
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

        # Path following released too — home calibration owns the target
        # (the controller enforces this anyway — belt and braces).
        self._path_following_active = False
        self.control_panel.sync_path_button(False)
        self.vision_path_following_set.emit(False)
        self._vision_monitor.set_path_overlay(None)

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
        try:
            settings_store.save_user_overrides({
                "MANUAL_ROLL_TRIM_DEG": float(roll),
                "MANUAL_PITCH_TRIM_DEG": float(pitch),
            })
            self.control_panel.append_preview(
                f"[TRIM] saved roll={roll:.2f}° pitch={pitch:.2f}° to user_settings.json"
            )
        except OSError as exc:
            self.control_panel.append_preview(f"[TRIM] save failed: {exc}")

    def _on_autotune_enable_clicked(self, enabled: bool) -> None:
        self._pd_autotune_enabled = enabled
        if not enabled:
            self._pd_autotune_auto_apply = False
            self._pd_autotune_has_suggestion = False
            self.control_panel.sync_autotune_buttons(False, False)
            self.control_panel.append_preview("[PD TUNE] disabled")
        else:
            # Path following released — autotune owns the target while it
            # steps (the controller enforces this anyway — belt and braces).
            self._path_following_active = False
            self.control_panel.sync_path_button(False)
            self.vision_path_following_set.emit(False)
            self._vision_monitor.set_path_overlay(None)
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
        # Snapshots emitted before the worker processed the apply still
        # carry has_suggestion=True; ignore them until the worker acks
        # with has_suggestion False (see _sync_autotune_from_terms).
        self._autotune_apply_pending = True

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
    # Path following handlers
    # ------------------------------------------------------------------

    def _stop_path_following(self, message: str) -> None:
        """Shared stop path: worker signal, button sync, overlay clear."""
        self._path_following_active = False
        self.control_panel.sync_path_button(False)
        self.vision_path_following_set.emit(False)
        self._vision_monitor.set_path_overlay(None)
        self.control_panel.append_preview(message)

    def _on_path_pattern_selected(self, label: str) -> None:
        if self._path_following_active:
            self._stop_path_following("[PATH] stopped (pattern changed)")
        self.vision_path_pattern_selected.emit(label)
        # Preview the selected pattern on the vision monitor (pure
        # geometry for display — playback stays in the worker/controller).
        if label in PATTERNS:
            path = PATTERNS[label]()
            self._vision_monitor.set_path_overlay(path.points, path.closed)
        else:
            self._vision_monitor.set_path_overlay(None)

    def _on_path_toggled(self, enabled: bool) -> None:
        if not enabled:
            self._stop_path_following("[PATH] stopped")
            return

        if not self._vision_enabled or self._vision_worker is None:
            self.control_panel.append_preview("[PATH] start vision mode first")
            self.control_panel.sync_path_button(False)
            return

        label = self.control_panel.current_pattern()
        if not label:
            self.control_panel.append_preview("[PATH] choose a pattern first")
            self.control_panel.sync_path_button(False)
            return

        # Mutual exclusion: autotune and home calibration must release the
        # target before the follower takes it over (the controller enforces
        # this too — belt and braces).
        self._pd_autotune_enabled = False
        self._pd_autotune_auto_apply = False
        self._pd_autotune_has_suggestion = False
        self.control_panel.sync_autotune_buttons(False, False)
        self.vision_pd_autotune_enabled.emit(False)
        self.vision_pd_autotune_auto_apply.emit(False)
        self._home_calibration_active = False
        self.control_panel.sync_calibrate_button(False)
        self.vision_calibrate_home_set.emit(False)

        if label in PATTERNS:
            path = PATTERNS[label]()
            self._vision_monitor.set_path_overlay(path.points, path.closed)
        self._path_following_active = True
        self.vision_path_following_set.emit(True)
        self.control_panel.append_preview(f"[PATH] following {label}")

    def _on_path_speed_changed(self, mm_s: float) -> None:
        self._path_speed_mm_s = float(mm_s)
        self.vision_path_speed_updated.emit(self._path_speed_mm_s)

    # ------------------------------------------------------------------
    # Vision mode enable / disable
    # ------------------------------------------------------------------

    def _on_vision_toggled(self, enable: bool) -> None:
        if enable and not self._vision_enabled:
            self._enable_vision_mode()
        elif not enable and self._vision_enabled:
            self._disable_vision_mode()

    def _enable_vision_mode(self) -> None:
        if self._vision_enabled or self._vision_thread is not None:
            # Already enabled, or the previous worker thread is still
            # winding down — never run two workers.
            return
        # Mode mutual exclusion: exactly one command source may own the
        # hardware. Stop any running routine IMMEDIATELY (no wind-down —
        # the vision loop takes over the hardware right away), then lock
        # out every manual command path (sliders, routines, SEND, raw box).
        if self.routine_runner.is_running:
            self.routine_runner.cancel(wind_down=False)
            self._routine_timer.stop()
            self.control_panel.set_cancel_enabled(False)
            self.control_panel.reset_routine_selector()
        self._vision_enabled = True
        self._vision_counter = 0
        self._last_visualizer_update = 0.0
        self._timing_plot.reset()
        self.control_panel.set_vision_active(True)
        self.control_panel.set_sliders_enabled(False)
        self.control_panel.set_manual_controls_enabled(False)
        self._vision_monitor.show()
        self._start_vision_worker()
        self.control_panel.append_preview("[INFO] Vision mode enabled.")

    def _disable_vision_mode(self) -> None:
        if not self._vision_enabled and self._vision_worker is None:
            return
        self._vision_enabled = False
        self._stop_vision_worker_async()
        # The remaining UI reset (buttons, monitor window, timing plot)
        # happens once the worker thread has actually finished — see
        # _on_vision_thread_finished → _reset_vision_ui.

    # ------------------------------------------------------------------
    # Worker thread lifecycle
    # ------------------------------------------------------------------

    def _start_vision_worker(self) -> None:
        if self._vision_thread is not None:
            return
        self._vision_thread = QtCore.QThread(self)
        self._vision_worker = VisionControlWorker(
            ik_solver=self._ik,
            kp=self._kp,
            kd=self._kd,
            max_tilt_deg=MAX_TILT_DEG,
            camera_index=CAMERA_INDEX,
            loop_hz=VISION_LOOP_HZ,
            snapshot_hz=GUI_SNAPSHOT_HZ,
            # Streaming path: latest-wins depth-1 queue, never blocks the
            # vision loop on serial I/O, stale setpoints are coalesced away.
            command_sender=lambda angles: self._servo.send_angles(
                angles, streaming=True
            ),
            roll_offset=self._trim_roll_deg,
            pitch_offset=self._trim_pitch_deg,
            auto_trim_enabled=self._auto_trim_enabled,
            # Serial RTT telemetry (EMA) for the timing plot — a plain
            # callable, like command_sender.
            rtt_provider=self._servo.rtt_stats,
            # Optional "t,x,y" per-frame log for jitter_bench --csv replay.
            position_log_path=VISION_POSITION_LOG_PATH or None,
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
        self.vision_z_updated.connect(self._vision_worker.set_z)
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
        self.vision_path_pattern_selected.connect(
            self._vision_worker.set_path_pattern
        )
        self.vision_path_following_set.connect(
            self._vision_worker.set_path_following
        )
        self.vision_path_speed_updated.connect(
            self._vision_worker.set_path_speed
        )
        self.vision_snapshot_consumed.connect(
            self._vision_worker.mark_snapshot_consumed
        )

        self._vision_thread.start()

        # Push current GUI state to worker
        self.vision_gains_updated.emit(self._kp, self._kd)
        self.vision_target_updated.emit(self._target_x_mm, self._target_y_mm)
        self.vision_trim_updated.emit(self._trim_roll_deg, self._trim_pitch_deg)
        self.vision_z_updated.emit(self._vision_z_setpoint)
        self.vision_auto_trim_enabled.emit(self._auto_trim_enabled)
        self.vision_calibrate_home_set.emit(self._home_calibration_active)
        self.vision_pd_autotune_enabled.emit(self._pd_autotune_enabled)
        self.vision_pd_autotune_auto_apply.emit(self._pd_autotune_auto_apply)
        self.vision_path_pattern_selected.emit(
            self.control_panel.current_pattern()
        )
        self.vision_path_speed_updated.emit(self._path_speed_mm_s)
        self.vision_hsv_updated.emit(*self.control_panel.get_hsv())

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
            (self.vision_z_updated, self._vision_worker.set_z),
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
            (self.vision_path_pattern_selected,
             self._vision_worker.set_path_pattern),
            (self.vision_path_following_set,
             self._vision_worker.set_path_following),
            (self.vision_path_speed_updated,
             self._vision_worker.set_path_speed),
            (self.vision_snapshot_consumed,
             self._vision_worker.mark_snapshot_consumed),
        ]:
            try:
                # PyQt5 stubs type slots narrowly; runtime accepts any callable.
                sig.disconnect(slot)  # type: ignore[call-overload]
            except TypeError:
                pass

        QtCore.QMetaObject.invokeMethod(
            self._vision_worker, "stop", QtCore.Qt.ConnectionType.QueuedConnection
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
        self.control_panel.set_vision_active(False)
        self.control_panel.set_sliders_enabled(True)
        self.control_panel.set_manual_controls_enabled(True)
        self._reset_vision_ui()
        self.control_panel.append_preview("[INFO] Vision mode disabled.")

    def _reset_vision_ui(self) -> None:
        """Return every piece of vision-mode UI state to its idle state.
        Called exactly once per vision session, from
        _on_vision_thread_finished."""
        self._pd_autotune_enabled = False
        self._pd_autotune_auto_apply = False
        self._pd_autotune_has_suggestion = False
        self._autotune_apply_pending = False
        self.control_panel.sync_autotune_buttons(False, False)
        self._home_calibration_active = False
        self.control_panel.sync_calibrate_button(False)
        self._path_following_active = False
        self.control_panel.sync_path_button(False)
        self.control_panel.set_path_status("path: idle")
        self._last_path_status = ""
        self._vision_monitor.set_path_overlay(None)
        # Close (hide) the monitor so it doesn't sit around showing the
        # last frozen frames; reopening starts fresh.
        self._vision_monitor.close()
        self._timing_plot.reset()

    def _on_serial_link_lost(self, reason: str) -> None:
        self.control_panel.append_preview(
            f"[SERIAL] Link lost ({reason}). Reconnect the Arduino and "
            "restart the app."
        )
        self.serial_monitor.append_line(f"[LINK LOST] {reason}")
        if self._vision_enabled:
            self._disable_vision_mode()

    def _on_vision_error(self, msg: str) -> None:
        self.control_panel.append_preview(f"[VISION ERROR] {msg}")

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
        except Exception as exc:
            self.control_panel.append_preview(
                f"[CAMERA] camera-ready info malformed: {exc!r}"
            )

    # ------------------------------------------------------------------
    # Snapshot handler
    # ------------------------------------------------------------------

    def _on_control_snapshot(self, snapshot: ControlSnapshot) -> None:
        try:
            if not self._vision_enabled:
                return

            # Exactly one increment per processed snapshot, regardless of
            # which branch below returns — keeps the LOG_EVERY_N cadence
            # regular across miss/gate/fail/ok transitions.
            self._vision_counter += 1

            self._update_camera_views(snapshot)

            if not snapshot.tracking_valid:
                # Neutral-pose fallback on sustained misses is the WORKER's
                # job (cv/vision_control_worker.py) — display only here.
                if self._vision_counter % LOG_EVERY_N == 0:
                    self.control_panel.append_preview(
                        f"[TRACK] no ball (miss={snapshot.miss_count}) "
                        f"reason={snapshot.reason}"
                    )
                return

            # Reacquire gating is enforced in the WORKER's command path
            # (before any servo send); the snapshot reason is display-only.
            if snapshot.reason == "reacquire_gating":
                if self._vision_counter % LOG_EVERY_N == 0:
                    self.control_panel.append_preview(
                        "[TRACK] reacquire gating (commands held)"
                    )
                return

            if not snapshot.ik_success:
                if self._vision_counter % LOG_EVERY_N == 0:
                    self.control_panel.append_preview(
                        "[WARN] IK failed in vision loop."
                    )
                return

            # For the trace log only — clamping happens in core/safety via
            # ServoDriver on the actual send path.
            safe_angles = [int(round(a)) for a in snapshot.servo_angles]

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
            self._sync_path_from_terms(terms)

            # Timing plot — derived metrics measured HERE are injected at
            # the call site; the widget owns history/trim/redraw cadence.
            timings = dict(snapshot.timings_ms)
            timings["visualizer_gui"] = vis_ms
            wets = snapshot.worker_emit_perf_ts
            timings["worker_to_gui_ms"] = (
                max(0.0, (now_perf - wets) * 1000.0) if wets > 0 else 0.0
            )
            # "frame_to_cmd" arrives measured from the worker — it used to
            # be overwritten here with a nonexistent "frame_to_cmd_worker"
            # key, so that plot line had always rendered 0.
            self._timing_plot.ingest(timings)

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
        finally:
            self.vision_snapshot_consumed.emit()

    # ------------------------------------------------------------------
    # Sync methods — push worker state back to GUI without feedback loops.
    # Each slider sync is skipped while the user is dragging that slider
    # group (isSliderDown) so stale worker echoes don't fight the drag.
    # ------------------------------------------------------------------

    def _sync_gain_sliders_from_terms(self, terms: dict) -> None:
        if self.control_panel.any_slider_down("gains"):
            return
        kp = float(terms.get("kp", self._kp))
        kd = float(terms.get("kd", self._kd))
        if abs(kp - self._kp) > 1e-6 or abs(kd - self._kd) > 1e-6:
            self._kp = kp
            self._kd = kd
            self.control_panel.sync_kp_kd(kp, kd)

    def _sync_target_from_terms(self, terms: dict) -> None:
        if self.control_panel.any_slider_down("target"):
            return
        tx = float(terms.get("target_x_mm", self._target_x_mm))
        ty = float(terms.get("target_y_mm", self._target_y_mm))
        if abs(tx - self._target_x_mm) > 0.5 or abs(ty - self._target_y_mm) > 0.5:
            self._target_x_mm = tx
            self._target_y_mm = ty
            self.control_panel.sync_target(tx, ty)

    def _sync_trim_from_terms(self, terms: dict) -> None:
        if self.control_panel.any_slider_down("trim"):
            return
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

    def _sync_path_from_terms(self, terms: dict) -> None:
        if "path_active" not in terms:
            return
        active = bool(terms["path_active"])
        if active != self._path_following_active:
            self._path_following_active = active
            self.control_panel.sync_path_button(active)
        status = (
            f"{terms.get('path_state', 'idle')} · "
            f"lap {int(terms.get('path_lap', 0))} · "
            f"{float(terms.get('path_progress', 0.0)) * 100:.0f}% · "
            f"err {float(terms.get('path_error_mm', 0.0)):.0f}mm"
        )
        # Update only when the formatted text changed — snapshots arrive at
        # GUI_SNAPSHOT_HZ and repainting an unchanged label is pure churn.
        if status != self._last_path_status:
            self._last_path_status = status
            self.control_panel.set_path_status(status)

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
        if self._autotune_apply_pending:
            if has_suggestion:
                # Stale snapshot emitted before the worker applied the
                # suggestion — do not re-adopt it.
                has_suggestion = False
            else:
                # Worker ack: the apply went through.
                self._autotune_apply_pending = False
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
            getattr(snapshot, "warped_bgr", None),
            ball_state=snapshot.ball_state,
            target_x_mm=self._target_x_mm,
            target_y_mm=self._target_y_mm,
            control_terms=snapshot.control_terms if snapshot.tracking_valid else None,
            # Matches BallTracker(platform_size_mm=240.0) in the worker;
            # see config.PLATFORM_SIZE for the physical measurement.
            platform_size_mm=240.0,
        )
        self._vision_monitor.update_camera(
            getattr(snapshot, "camera_bgr", None)
        )
        self._vision_monitor.update_mask(
            getattr(snapshot, "mask_gray", None)
        )

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

        if self._vision_worker is not None and self._vision_thread is not None:
            # The worker's stop handshake is a chain of QUEUED cross-thread
            # signals (stop → stopped → quit → finished), all of which need
            # the GUI event loop to dispatch. QThread.wait() here would block
            # that loop and deadlock until timeout — so instead spin a local
            # QEventLoop until the thread finishes (3 s guard).
            waiter = QtCore.QEventLoop()
            self._vision_thread.finished.connect(waiter.quit)
            timeout_guard = QtCore.QTimer(self)
            timeout_guard.setSingleShot(True)
            timeout_guard.timeout.connect(waiter.quit)
            timeout_guard.start(3000)
            self._disable_vision_mode()
            waiter.exec_()
            timeout_guard.stop()
            thread = self._vision_thread
            if thread is not None and thread.isRunning():
                # Timed out: force the event loop down and give it a moment.
                thread.quit()
                thread.wait(1000)

        self._vision_monitor.close()
        self._serial.disconnect()
        event.accept()  # type: ignore[attr-defined]

    def emergency_shutdown(self) -> None:
        """Crash-path cleanup (called by main.py's excepthook): stop timers,
        ramp the platform to neutral, and drop the serial link. Must never
        raise."""
        try:
            self._routine_timer.stop()
        except Exception:
            pass
        try:
            # Neutral servo pose == firmware boot pose; the large-move policy
            # in ServoDriver turns this into a gentle hardware ramp.
            self._servo.send_angles([90.0] * 6)
        except Exception:
            pass
        try:
            self._serial.disconnect()
        except Exception:
            pass
