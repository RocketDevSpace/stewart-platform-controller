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

ROUTINE_PLACEHOLDER = "(Choose a routine...)"
AXES = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
AXIS_KEYS = {"X": "x", "Y": "y", "Z": "z",
             "Roll": "roll", "Pitch": "pitch", "Yaw": "yaw"}


class ControlPanel(QWidget):
    slider_changed = pyqtSignal(str, int)
    routine_selected = pyqtSignal(str)
    routine_cancelled = pyqtSignal()
    send_clicked = pyqtSignal()
    vision_toggled = pyqtSignal(bool)
    kp_changed = pyqtSignal(float)
    kd_changed = pyqtSignal(float)
    raw_command_sent = pyqtSignal(str)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        self._sliders: dict[str, QSlider] = {}
        self._slider_labels: dict[str, QLabel] = {}
        self._vision_active = False

        main_layout = QHBoxLayout()
        main_layout.addLayout(self._build_slider_column())
        main_layout.addLayout(self._build_middle_column())
        self.setLayout(main_layout)

    # ------------------------------------------------------------------
    # Layout builders
    # ------------------------------------------------------------------

    def _build_slider_column(self) -> QVBoxLayout:
        layout = QVBoxLayout()
        for ax in AXES:
            lbl = QLabel(f"{ax}: 0")
            sld = QSlider(QtCore.Qt.Horizontal)
            sld.setMinimum(-100)
            sld.setMaximum(100)
            sld.setValue(0)
            sld.valueChanged.connect(
                lambda val, a=ax, lab=lbl: self._on_slider_value(a, val, lab)
            )
            layout.addWidget(lbl)
            layout.addWidget(sld)
            self._sliders[ax] = sld
            self._slider_labels[ax] = lbl
        return layout

    def _build_middle_column(self) -> QVBoxLayout:
        layout = QVBoxLayout()

        self._demo_list = QComboBox()
        self._demo_list.addItem(ROUTINE_PLACEHOLDER)
        for name in ROUTINES.keys():
            self._demo_list.addItem(name)
        self._demo_list.currentIndexChanged.connect(self._on_routine_changed)
        layout.addWidget(QLabel("Demo Routines:"))
        layout.addWidget(self._demo_list)

        self._cancel_routine_btn = QPushButton("\u274c Cancel Routine")
        self._cancel_routine_btn.setStyleSheet(
            "background-color: darkred; color: white; font-weight: bold;"
        )
        self._cancel_routine_btn.clicked.connect(self.routine_cancelled.emit)
        self._cancel_routine_btn.setEnabled(False)
        layout.addWidget(self._cancel_routine_btn)

        layout.addWidget(QLabel("Ball Balancing Control"))

        self._vision_button = QPushButton("Enable Vision Mode")
        self._vision_button.setStyleSheet(
            "background-color: darkgreen; color: white;"
        )
        self._vision_button.clicked.connect(self._on_vision_button)
        layout.addWidget(self._vision_button)

        self._cancel_vision_btn = QPushButton("\U0001f6d1 Cancel Vision Mode")
        self._cancel_vision_btn.setStyleSheet(
            "background-color: darkred; color: white;"
        )
        self._cancel_vision_btn.clicked.connect(
            lambda: self.vision_toggled.emit(False)
        )
        self._cancel_vision_btn.setEnabled(False)
        layout.addWidget(self._cancel_vision_btn)

        self._kp_label = QLabel("Kp: 0.005")
        self._kp_slider = QSlider(QtCore.Qt.Horizontal)
        self._kp_slider.setMinimum(0)
        self._kp_slider.setMaximum(100)
        self._kp_slider.setValue(5)
        self._kp_slider.valueChanged.connect(self._on_kp_changed)
        layout.addWidget(self._kp_label)
        layout.addWidget(self._kp_slider)

        self._kd_label = QLabel("Kd: 0.010")
        self._kd_slider = QSlider(QtCore.Qt.Horizontal)
        self._kd_slider.setMinimum(0)
        self._kd_slider.setMaximum(100)
        self._kd_slider.setValue(10)
        self._kd_slider.valueChanged.connect(self._on_kd_changed)
        layout.addWidget(self._kd_label)
        layout.addWidget(self._kd_slider)

        self._send_button = QPushButton("SEND TO ARDUINO")
        self._send_button.setStyleSheet(
            "background-color: red; color: white; font-size: 16pt;"
        )
        self._send_button.clicked.connect(self.send_clicked.emit)
        layout.addWidget(self._send_button)

        self._preview_output = QTextEdit()
        self._preview_output.setReadOnly(True)
        self._preview_output.setPlaceholderText(
            "Command preview will appear here..."
        )
        layout.addWidget(QLabel("Command Preview:"))
        layout.addWidget(self._preview_output)

        self._raw_serial_input = QLineEdit()
        self._raw_serial_input.setPlaceholderText(
            "Type raw serial command here, e.g. S,90,90,90,90,90,90"
        )
        layout.addWidget(QLabel("Direct Serial Command:"))
        layout.addWidget(self._raw_serial_input)

        self._raw_serial_send_btn = QPushButton("Send Command")
        self._raw_serial_send_btn.clicked.connect(self._on_raw_send)
        layout.addWidget(self._raw_serial_send_btn)

        self._aux_layout = QVBoxLayout()
        layout.addLayout(self._aux_layout)

        return layout

    # ------------------------------------------------------------------
    # Internal slot adapters
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
    # Public interface
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
            self._vision_button.setStyleSheet(
                "background-color: orange; color: black;"
            )
            self._cancel_vision_btn.setEnabled(True)
        else:
            self._vision_button.setText("Enable Vision Mode")
            self._vision_button.setStyleSheet(
                "background-color: darkgreen; color: white;"
            )
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
        """Place an extra widget at the bottom of the middle column.

        Used by MainWindow to embed SerialMonitor in the same column as
        the existing controls (matches legacy gui_layout layout).
        """
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
