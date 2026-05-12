"""
gui/control_panel.py

Slider/button widget. Pure view: emits signals on user actions and
exposes setter methods for state changes. Holds no control logic, no
IK calls, no serial command building. MainWindow connects the signals
to handlers that own the logic.
"""

from __future__ import annotations

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import (
    QComboBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSlider,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from routines.routines import ROUTINES
from settings import (
    BALL_TARGET_DEFAULT_X_MM,
    BALL_TARGET_DEFAULT_Y_MM,
    MANUAL_PITCH_TRIM_DEG,
    MANUAL_ROLL_TRIM_DEG,
    PD_DEFAULT_KD,
    PD_DEFAULT_KP,
    TRACKER_HSV_H_MAX,
    TRACKER_HSV_H_MIN,
    TRACKER_HSV_S_MAX,
    TRACKER_HSV_S_MIN,
    TRACKER_HSV_V_MAX,
    TRACKER_HSV_V_MIN,
)

ROUTINE_PLACEHOLDER = "(Choose a routine...)"
AXES = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
AXIS_KEYS = {"X": "x", "Y": "y", "Z": "z",
             "Roll": "roll", "Pitch": "pitch", "Yaw": "yaw"}

DARK_QSS = (
    "QWidget { background: #0b0f16; color: #d6e2ff;"
    " font-family: 'Segoe UI'; font-size: 10pt; }"
    " QGroupBox { border: 1px solid #1f2a3a; margin-top: 10px;"
    " border-radius: 6px; padding: 8px; }"
    " QGroupBox::title { subcontrol-origin: margin; left: 8px;"
    " padding: 0 4px; color: #7dd3fc; font-weight: 600; }"
    " QPushButton { background: #162033; border: 1px solid #2a3b59;"
    " border-radius: 5px; padding: 6px 10px; color: #d6e2ff; }"
    " QPushButton:hover { background: #1e2c45; }"
    " QPushButton:pressed { background: #0f1726; }"
    " QPushButton:disabled { color: #6b7a90; border-color: #2a3342; }"
    " QTextEdit, QLineEdit, QComboBox { background: #0f1726;"
    " border: 1px solid #27364d; border-radius: 4px; color: #d6e2ff; }"
    " QSlider::groove:horizontal { border: 1px solid #2a3b59; height: 6px;"
    " background: #121a2a; border-radius: 3px; }"
    " QSlider::handle:horizontal { background: #7dd3fc;"
    " border: 1px solid #38bdf8; width: 14px; margin: -6px 0;"
    " border-radius: 7px; }"
    " QLabel { color: #d6e2ff; }"
)


class ControlPanel(QWidget):
    slider_changed = pyqtSignal(str, int)
    routine_selected = pyqtSignal(str)
    routine_cancelled = pyqtSignal()
    send_clicked = pyqtSignal()
    vision_toggled = pyqtSignal(bool)
    kp_changed = pyqtSignal(float)
    kd_changed = pyqtSignal(float)
    raw_command_sent = pyqtSignal(str)
    target_changed = pyqtSignal(float, float)        # x_mm, y_mm
    trim_changed = pyqtSignal(float, float)          # roll_deg, pitch_deg
    auto_trim_toggled = pyqtSignal(bool)
    calibrate_home_clicked = pyqtSignal()
    reset_trim_clicked = pyqtSignal()
    autotune_enable_clicked = pyqtSignal(bool)
    autotune_apply_clicked = pyqtSignal()
    autotune_auto_apply_clicked = pyqtSignal(bool)
    hsv_changed = pyqtSignal(int, int, int, int, int, int)  # hmin,hmax,smin,smax,vmin,vmax
    open_vision_monitor_clicked = pyqtSignal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        self._sliders: dict[str, QSlider] = {}
        self._slider_labels: dict[str, QLabel] = {}
        self._vision_active = False
        self._auto_trim_active = False
        self._calibrating_home = False
        self._autotune_enabled = False
        self._autotune_auto_apply = False

        main_layout = QHBoxLayout()
        main_layout.addLayout(self._build_slider_column())
        main_layout.addLayout(self._build_ball_pd_column())
        main_layout.addLayout(self._build_command_column())
        self.setLayout(main_layout)

        self.setStyleSheet(DARK_QSS)

    # ------------------------------------------------------------------
    # Layout builders
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # Column 1: compact position sliders + HSV thresholds
    # ------------------------------------------------------------------

    def _build_slider_column(self) -> QVBoxLayout:
        layout = QVBoxLayout()
        layout.setSpacing(4)

        axis_group = QGroupBox("Position Controls")
        ag = QVBoxLayout()
        ag.setSpacing(2)
        ag.setContentsMargins(6, 10, 6, 6)
        for ax in AXES:
            lbl = QLabel(f"{ax}: 0")
            sld = QSlider(QtCore.Qt.Horizontal)
            sld.setMinimum(-100)
            sld.setMaximum(100)
            sld.setValue(0)
            sld.setMaximumHeight(20)
            sld.valueChanged.connect(
                lambda val, a=ax, lab=lbl: self._on_slider_value(a, val, lab)
            )
            ag.addWidget(lbl)
            ag.addWidget(sld)
            self._sliders[ax] = sld
            self._slider_labels[ax] = lbl
        axis_group.setLayout(ag)
        layout.addWidget(axis_group)

        layout.addWidget(self._build_hsv_group())
        layout.addStretch()
        return layout

    def _build_hsv_group(self) -> QGroupBox:
        hsv_group = QGroupBox("HSV Thresholds")
        hsv = QVBoxLayout()
        hsv.setSpacing(2)
        hsv.setContentsMargins(6, 10, 6, 6)

        _hsv_defs = [
            ("H Min", "h_min", 0, 179, TRACKER_HSV_H_MIN),
            ("H Max", "h_max", 0, 179, TRACKER_HSV_H_MAX),
            ("S Min", "s_min", 0, 255, TRACKER_HSV_S_MIN),
            ("S Max", "s_max", 0, 255, TRACKER_HSV_S_MAX),
            ("V Min", "v_min", 0, 255, TRACKER_HSV_V_MIN),
            ("V Max", "v_max", 0, 255, TRACKER_HSV_V_MAX),
        ]
        for display, attr, lo, hi, default in _hsv_defs:
            lbl = QLabel(f"{display}: {default}")
            sld = QSlider(QtCore.Qt.Horizontal)
            sld.setRange(lo, hi)
            sld.setValue(default)
            sld.setMaximumHeight(20)
            sld.valueChanged.connect(self._on_hsv_changed)
            setattr(self, f"_hsv_{attr}_label", lbl)
            setattr(self, f"_hsv_{attr}_slider", sld)
            hsv.addWidget(lbl)
            hsv.addWidget(sld)

        hsv_group.setLayout(hsv)
        return hsv_group

    # ------------------------------------------------------------------
    # Column 2: ball target, trim, PD control (Kp/Kd + autotune)
    # ------------------------------------------------------------------

    def _build_ball_pd_column(self) -> QVBoxLayout:
        layout = QVBoxLayout()
        layout.setSpacing(4)

        # --- Ball Target ---
        target_group = QGroupBox("Ball Target")
        tg = QVBoxLayout()
        tg.setSpacing(2)
        tg.setContentsMargins(6, 10, 6, 6)

        self._target_x_label = QLabel(
            f"Target X: {BALL_TARGET_DEFAULT_X_MM:.0f} mm"
        )
        self._target_x_slider = QSlider(QtCore.Qt.Horizontal)
        self._target_x_slider.setMinimum(-120)
        self._target_x_slider.setMaximum(120)
        self._target_x_slider.setValue(int(BALL_TARGET_DEFAULT_X_MM))
        self._target_x_slider.setMaximumHeight(20)
        self._target_x_slider.valueChanged.connect(self._on_target_changed)
        tg.addWidget(self._target_x_label)
        tg.addWidget(self._target_x_slider)

        self._target_y_label = QLabel(
            f"Target Y: {BALL_TARGET_DEFAULT_Y_MM:.0f} mm"
        )
        self._target_y_slider = QSlider(QtCore.Qt.Horizontal)
        self._target_y_slider.setMinimum(-120)
        self._target_y_slider.setMaximum(120)
        self._target_y_slider.setValue(int(BALL_TARGET_DEFAULT_Y_MM))
        self._target_y_slider.setMaximumHeight(20)
        self._target_y_slider.valueChanged.connect(self._on_target_changed)
        tg.addWidget(self._target_y_label)
        tg.addWidget(self._target_y_slider)

        target_group.setLayout(tg)
        layout.addWidget(target_group)

        # --- Platform Trim ---
        trim_group = QGroupBox("Platform Trim")
        tr = QVBoxLayout()
        tr.setSpacing(2)
        tr.setContentsMargins(6, 10, 6, 6)

        self._trim_roll_label = QLabel(f"Roll Trim: {MANUAL_ROLL_TRIM_DEG:.2f}°")
        self._trim_roll_slider = QSlider(QtCore.Qt.Horizontal)
        self._trim_roll_slider.setMinimum(-1000)
        self._trim_roll_slider.setMaximum(1000)
        self._trim_roll_slider.setValue(int(MANUAL_ROLL_TRIM_DEG * 100))
        self._trim_roll_slider.setMaximumHeight(20)
        self._trim_roll_slider.valueChanged.connect(self._on_trim_changed)
        tr.addWidget(self._trim_roll_label)
        tr.addWidget(self._trim_roll_slider)

        self._trim_pitch_label = QLabel(
            f"Pitch Trim: {MANUAL_PITCH_TRIM_DEG:.2f}°"
        )
        self._trim_pitch_slider = QSlider(QtCore.Qt.Horizontal)
        self._trim_pitch_slider.setMinimum(-1000)
        self._trim_pitch_slider.setMaximum(1000)
        self._trim_pitch_slider.setValue(int(MANUAL_PITCH_TRIM_DEG * 100))
        self._trim_pitch_slider.setMaximumHeight(20)
        self._trim_pitch_slider.valueChanged.connect(self._on_trim_changed)
        tr.addWidget(self._trim_pitch_label)
        tr.addWidget(self._trim_pitch_slider)

        self._auto_trim_btn = QPushButton("Auto-Trim: OFF")
        self._auto_trim_btn.clicked.connect(self._on_auto_trim_toggled)
        tr.addWidget(self._auto_trim_btn)

        self._calibrate_home_btn = QPushButton("Calibrate Home")
        self._calibrate_home_btn.clicked.connect(self._on_calibrate_home)
        tr.addWidget(self._calibrate_home_btn)

        self._reset_trim_btn = QPushButton("Reset Trim to Config")
        self._reset_trim_btn.clicked.connect(self.reset_trim_clicked.emit)
        tr.addWidget(self._reset_trim_btn)

        trim_group.setLayout(tr)
        layout.addWidget(trim_group)

        # --- PD Control (sliders + autotune) ---
        tune_group = QGroupBox("PD Control")
        tune = QVBoxLayout()
        tune.setSpacing(2)
        tune.setContentsMargins(6, 10, 6, 6)

        self._kp_label = QLabel(f"Kp: {PD_DEFAULT_KP:.3f}")
        self._kp_slider = QSlider(QtCore.Qt.Horizontal)
        self._kp_slider.setMinimum(0)
        self._kp_slider.setMaximum(300)  # supports PD_AUTOTUNE_MAX_KP = 0.250
        self._kp_slider.setValue(int(PD_DEFAULT_KP * 1000))
        self._kp_slider.setMaximumHeight(20)
        self._kp_slider.valueChanged.connect(self._on_kp_changed)
        tune.addWidget(self._kp_label)
        tune.addWidget(self._kp_slider)

        self._kd_label = QLabel(f"Kd: {PD_DEFAULT_KD:.3f}")
        self._kd_slider = QSlider(QtCore.Qt.Horizontal)
        self._kd_slider.setMinimum(0)
        self._kd_slider.setMaximum(100)
        self._kd_slider.setValue(int(PD_DEFAULT_KD * 1000))
        self._kd_slider.setMaximumHeight(20)
        self._kd_slider.valueChanged.connect(self._on_kd_changed)
        tune.addWidget(self._kd_label)
        tune.addWidget(self._kd_slider)

        self._autotune_enable_btn = QPushButton("AutoTune: OFF")
        self._autotune_enable_btn.clicked.connect(self._on_autotune_enable)
        tune.addWidget(self._autotune_enable_btn)

        self._autotune_apply_btn = QPushButton("Apply Now")
        self._autotune_apply_btn.clicked.connect(self.autotune_apply_clicked.emit)
        tune.addWidget(self._autotune_apply_btn)

        self._autotune_auto_apply_btn = QPushButton("Auto-Apply: OFF")
        self._autotune_auto_apply_btn.clicked.connect(self._on_autotune_auto_apply)
        tune.addWidget(self._autotune_auto_apply_btn)

        tune_group.setLayout(tune)
        layout.addWidget(tune_group)

        layout.addStretch()
        return layout

    # ------------------------------------------------------------------
    # Column 3: routines, vision mode, command I/O, serial monitor
    # ------------------------------------------------------------------

    def _build_command_column(self) -> QVBoxLayout:
        layout = QVBoxLayout()
        layout.setSpacing(4)

        # --- Demo Routines ---
        routine_group = QGroupBox("Demo Routines")
        rg = QVBoxLayout()
        rg.setSpacing(4)
        rg.setContentsMargins(6, 10, 6, 6)

        self._demo_list = QComboBox()
        self._demo_list.addItem(ROUTINE_PLACEHOLDER)
        for name in ROUTINES.keys():
            self._demo_list.addItem(name)
        self._demo_list.currentIndexChanged.connect(self._on_routine_changed)
        rg.addWidget(self._demo_list)

        self._cancel_routine_btn = QPushButton("❌ Cancel Routine")
        self._cancel_routine_btn.clicked.connect(self.routine_cancelled.emit)
        self._cancel_routine_btn.setEnabled(False)
        rg.addWidget(self._cancel_routine_btn)

        routine_group.setLayout(rg)
        layout.addWidget(routine_group)

        # --- Ball Balancing (vision mode + monitor) ---
        vision_group = QGroupBox("Ball Balancing Control")
        vg = QVBoxLayout()
        vg.setSpacing(4)
        vg.setContentsMargins(6, 10, 6, 6)

        self._vision_button = QPushButton("Enable Vision Mode")
        self._vision_button.clicked.connect(self._on_vision_button)
        vg.addWidget(self._vision_button)

        self._cancel_vision_btn = QPushButton("\U0001f6d1 Cancel Vision Mode")
        self._cancel_vision_btn.clicked.connect(
            lambda: self.vision_toggled.emit(False)
        )
        self._cancel_vision_btn.setEnabled(False)
        vg.addWidget(self._cancel_vision_btn)

        self._open_vision_monitor_btn = QPushButton("Open Vision Monitor")
        self._open_vision_monitor_btn.clicked.connect(
            self.open_vision_monitor_clicked.emit
        )
        vg.addWidget(self._open_vision_monitor_btn)

        vision_group.setLayout(vg)
        layout.addWidget(vision_group)

        # --- Command I/O ---
        io_group = QGroupBox("Command I/O")
        ig = QVBoxLayout()
        ig.setSpacing(4)
        ig.setContentsMargins(6, 10, 6, 6)

        self._send_button = QPushButton("SEND TO ARDUINO")
        self._send_button.clicked.connect(self.send_clicked.emit)
        ig.addWidget(self._send_button)

        ig.addWidget(QLabel("Command Preview:"))
        self._preview_output = QTextEdit()
        self._preview_output.setReadOnly(True)
        self._preview_output.setPlaceholderText(
            "Command preview will appear here..."
        )
        self._preview_output.setMaximumHeight(80)
        ig.addWidget(self._preview_output)

        ig.addWidget(QLabel("Direct Serial Command:"))
        self._raw_serial_input = QLineEdit()
        self._raw_serial_input.setPlaceholderText(
            "e.g. S,90,90,90,90,90,90"
        )
        ig.addWidget(self._raw_serial_input)

        self._raw_serial_send_btn = QPushButton("Send Command")
        self._raw_serial_send_btn.clicked.connect(self._on_raw_send)
        ig.addWidget(self._raw_serial_send_btn)

        io_group.setLayout(ig)
        layout.addWidget(io_group)

        self._aux_layout = QVBoxLayout()
        layout.addLayout(self._aux_layout)

        return layout

    # ------------------------------------------------------------------
    # Internal slot adapters (original)
    # ------------------------------------------------------------------

    def _on_slider_value(self, axis: str, value: int, label: QLabel) -> None:
        label.setText(f"{axis}: {value}")
        self.slider_changed.emit(axis, value)

    def _on_routine_changed(self) -> None:
        name = self._demo_list.currentText()
        if name == ROUTINE_PLACEHOLDER:
            self.routine_cancelled.emit()
        else:
            self.routine_selected.emit(name)

    def _on_vision_button(self) -> None:
        self.vision_toggled.emit(not self._vision_active)

    def _on_kp_changed(self, raw: int) -> None:
        kp = raw / 1000.0
        self._kp_label.setText(f"Kp: {kp:.3f}")
        self.kp_changed.emit(kp)

    def _on_kd_changed(self, raw: int) -> None:
        kd = raw / 1000.0
        self._kd_label.setText(f"Kd: {kd:.3f}")
        self.kd_changed.emit(kd)

    def _on_raw_send(self) -> None:
        cmd = self._raw_serial_input.text().strip()
        if cmd:
            self.raw_command_sent.emit(cmd)

    # ------------------------------------------------------------------
    # Internal slot adapters (new vision controls)
    # ------------------------------------------------------------------

    def _on_target_changed(self) -> None:
        x = float(self._target_x_slider.value())
        y = float(self._target_y_slider.value())
        self._target_x_label.setText(f"Target X: {x:.0f} mm")
        self._target_y_label.setText(f"Target Y: {y:.0f} mm")
        self.target_changed.emit(x, y)

    def _on_trim_changed(self) -> None:
        roll = self._trim_roll_slider.value() / 100.0
        pitch = self._trim_pitch_slider.value() / 100.0
        self._trim_roll_label.setText(f"Roll Trim: {roll:.2f}°")
        self._trim_pitch_label.setText(f"Pitch Trim: {pitch:.2f}°")
        self.trim_changed.emit(roll, pitch)

    def _on_auto_trim_toggled(self) -> None:
        self._auto_trim_active = not self._auto_trim_active
        self._auto_trim_btn.setText(
            "Auto-Trim: ON" if self._auto_trim_active else "Auto-Trim: OFF"
        )
        self.auto_trim_toggled.emit(self._auto_trim_active)

    def _on_calibrate_home(self) -> None:
        self._calibrating_home = not self._calibrating_home
        self._calibrate_home_btn.setText(
            "Cancel Calibration" if self._calibrating_home else "Calibrate Home"
        )
        self.calibrate_home_clicked.emit()

    def _on_autotune_enable(self) -> None:
        self._autotune_enabled = not self._autotune_enabled
        self._autotune_enable_btn.setText(
            "AutoTune: ON" if self._autotune_enabled else "AutoTune: OFF"
        )
        self.autotune_enable_clicked.emit(self._autotune_enabled)

    def _on_autotune_auto_apply(self) -> None:
        self._autotune_auto_apply = not self._autotune_auto_apply
        self._autotune_auto_apply_btn.setText(
            "Auto-Apply: ON" if self._autotune_auto_apply else "Auto-Apply: OFF"
        )
        self.autotune_auto_apply_clicked.emit(self._autotune_auto_apply)

    def _on_hsv_changed(self) -> None:
        hmin = self._hsv_h_min_slider.value()
        hmax = self._hsv_h_max_slider.value()
        smin = self._hsv_s_min_slider.value()
        smax = self._hsv_s_max_slider.value()
        vmin = self._hsv_v_min_slider.value()
        vmax = self._hsv_v_max_slider.value()
        self._hsv_h_min_label.setText(f"H Min: {hmin}")
        self._hsv_h_max_label.setText(f"H Max: {hmax}")
        self._hsv_s_min_label.setText(f"S Min: {smin}")
        self._hsv_s_max_label.setText(f"S Max: {smax}")
        self._hsv_v_min_label.setText(f"V Min: {vmin}")
        self._hsv_v_max_label.setText(f"V Max: {vmax}")
        self.hsv_changed.emit(hmin, hmax, smin, smax, vmin, vmax)

    # ------------------------------------------------------------------
    # Public interface (original)
    # ------------------------------------------------------------------

    def set_sliders_enabled(self, enabled: bool) -> None:
        for sld in self._sliders.values():
            sld.setEnabled(enabled)

    def set_send_enabled(self, enabled: bool) -> None:
        self._send_button.setEnabled(enabled)

    def set_cancel_enabled(self, enabled: bool) -> None:
        self._cancel_routine_btn.setEnabled(enabled)

    def set_vision_active(self, active: bool) -> None:
        self._vision_active = active
        if active:
            self._vision_button.setText("Vision Mode ACTIVE")
            self._cancel_vision_btn.setEnabled(True)
        else:
            self._vision_button.setText("Enable Vision Mode")
            self._cancel_vision_btn.setEnabled(False)

    def get_slider_values(self) -> dict:
        return {
            AXIS_KEYS[ax]: self._sliders[ax].value() for ax in AXES
        }

    def set_slider_values(self, pose: dict) -> None:
        """Mirror a pose to the sliders without re-emitting valueChanged."""
        for ax in AXES:
            sld = self._sliders[ax]
            sld.blockSignals(True)
            sld.setValue(int(pose[AXIS_KEYS[ax]]))
            self._slider_labels[ax].setText(f"{ax}: {sld.value()}")
            sld.blockSignals(False)

    def reset_routine_selector(self) -> None:
        if self._demo_list.currentText() != ROUTINE_PLACEHOLDER:
            self._demo_list.blockSignals(True)
            self._demo_list.setCurrentIndex(0)
            self._demo_list.blockSignals(False)

    def current_routine(self) -> str:
        """Return the current dropdown selection (placeholder if none)."""
        return str(self._demo_list.currentText())

    def append_preview(self, text: str) -> None:
        self._preview_output.append(text)

    def clear_raw_input(self) -> None:
        self._raw_serial_input.clear()

    def attach_below(self, widget: QWidget) -> None:
        """Place an extra widget at the bottom of the middle column."""
        self._aux_layout.addWidget(widget)

    def confirm(self, title: str, message: str) -> bool:
        """Modal Yes/No dialog. Used for raw-command and slider-send confirms."""
        reply = QtWidgets.QMessageBox.question(
            self,
            title,
            message,
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
        )
        return bool(reply == QtWidgets.QMessageBox.Yes)

    # ------------------------------------------------------------------
    # Sync methods — called from MainWindow to push state into the GUI
    # without triggering re-emission (blockSignals pattern).
    # ------------------------------------------------------------------

    def sync_kp_kd(self, kp: float, kd: float) -> None:
        self._kp_slider.blockSignals(True)
        self._kp_slider.setValue(int(kp * 1000))
        self._kp_label.setText(f"Kp: {kp:.3f}")
        self._kp_slider.blockSignals(False)

        self._kd_slider.blockSignals(True)
        self._kd_slider.setValue(int(kd * 1000))
        self._kd_label.setText(f"Kd: {kd:.3f}")
        self._kd_slider.blockSignals(False)

    def sync_target(self, x_mm: float, y_mm: float) -> None:
        self._target_x_slider.blockSignals(True)
        self._target_x_slider.setValue(int(x_mm))
        self._target_x_label.setText(f"Target X: {x_mm:.0f} mm")
        self._target_x_slider.blockSignals(False)

        self._target_y_slider.blockSignals(True)
        self._target_y_slider.setValue(int(y_mm))
        self._target_y_label.setText(f"Target Y: {y_mm:.0f} mm")
        self._target_y_slider.blockSignals(False)

    def sync_trim(self, roll_deg: float, pitch_deg: float) -> None:
        self._trim_roll_slider.blockSignals(True)
        self._trim_roll_slider.setValue(int(roll_deg * 100))
        self._trim_roll_label.setText(f"Roll Trim: {roll_deg:.2f}°")
        self._trim_roll_slider.blockSignals(False)

        self._trim_pitch_slider.blockSignals(True)
        self._trim_pitch_slider.setValue(int(pitch_deg * 100))
        self._trim_pitch_label.setText(f"Pitch Trim: {pitch_deg:.2f}°")
        self._trim_pitch_slider.blockSignals(False)

    def sync_auto_trim_button(self, active: bool) -> None:
        self._auto_trim_active = active
        self._auto_trim_btn.setText(
            "Auto-Trim: ON" if active else "Auto-Trim: OFF"
        )

    def sync_calibrate_button(self, calibrating: bool) -> None:
        self._calibrating_home = calibrating
        self._calibrate_home_btn.setText(
            "Cancel Calibration" if calibrating else "Calibrate Home"
        )

    def sync_autotune_buttons(self, enabled: bool, auto_apply: bool) -> None:
        self._autotune_enabled = enabled
        self._autotune_enable_btn.setText(
            "AutoTune: ON" if enabled else "AutoTune: OFF"
        )
        self._autotune_auto_apply = auto_apply
        self._autotune_auto_apply_btn.setText(
            "Auto-Apply: ON" if auto_apply else "Auto-Apply: OFF"
        )

    def sync_hsv(
        self,
        hmin: int, hmax: int,
        smin: int, smax: int,
        vmin: int, vmax: int,
    ) -> None:
        for sld, val, lbl, name in (
            (self._hsv_h_min_slider, hmin, self._hsv_h_min_label, "H Min"),
            (self._hsv_h_max_slider, hmax, self._hsv_h_max_label, "H Max"),
            (self._hsv_s_min_slider, smin, self._hsv_s_min_label, "S Min"),
            (self._hsv_s_max_slider, smax, self._hsv_s_max_label, "S Max"),
            (self._hsv_v_min_slider, vmin, self._hsv_v_min_label, "V Min"),
            (self._hsv_v_max_slider, vmax, self._hsv_v_max_label, "V Max"),
        ):
            sld.blockSignals(True)
            sld.setValue(val)
            lbl.setText(f"{name}: {val}")
            sld.blockSignals(False)
