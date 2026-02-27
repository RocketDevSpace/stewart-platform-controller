from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import (
    QGridLayout,
    QGroupBox,
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
from stewart_control.config import (
    BALL_TARGET_DEFAULT_X_MM,
    BALL_TARGET_DEFAULT_Y_MM,
    PD_DEFAULT_KD,
    PD_DEFAULT_KP,
    TRACKER_HSV_H_MAX,
    TRACKER_HSV_H_MIN,
    TRACKER_HSV_S_MAX,
    TRACKER_HSV_S_MIN,
    TRACKER_HSV_V_MAX,
    TRACKER_HSV_V_MIN,
)


class VisionMonitorWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Stewart Vision + Visualizer")
        self._build_ui()
        self._apply_theme()

    def _build_ui(self):
        root = QGridLayout()
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        self.camera_view_label = QLabel("Camera View (Disabled)")
        self.warped_view_label = QLabel("Warped Vector View (Disabled)")
        self.hsv_view_label = QLabel("HSV/Mask View (Disabled)")

        for lbl in [self.camera_view_label, self.warped_view_label, self.hsv_view_label]:
            lbl.setAlignment(QtCore.Qt.AlignCenter)
            lbl.setMinimumSize(480, 320)
            lbl.setFrameShape(QtWidgets.QFrame.Box)
            lbl.setStyleSheet("border:1px solid #2a3b59; background:#0f1726;")

        root.addWidget(self.warped_view_label, 0, 0, 2, 1)
        root.addWidget(self.camera_view_label, 0, 1)
        root.addWidget(self.hsv_view_label, 1, 1)
        root.setRowStretch(0, 1)
        root.setRowStretch(1, 1)
        root.setColumnStretch(0, 3)
        root.setColumnStretch(1, 2)
        self.setLayout(root)
        self.resize(1400, 900)

    def _apply_theme(self):
        self.setStyleSheet(
            """
            QWidget { background: #0b0f16; color: #d6e2ff; }
            QLabel { border: 1px solid #2a3b59; background: #0f1726; color: #d6e2ff; }
            """
        )


class StewartGUIView(QWidget):
    def __init__(self):
        super().__init__()
        self.monitor_window = VisionMonitorWindow()
        self._build_ui()
        self._apply_theme()

    def _build_ui(self):
        root = QGridLayout()
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        control_panel = self._build_controls_panel()
        telemetry_panel = self._build_telemetry_panel()
        comms_panel = self._build_comms_panel()

        root.addWidget(control_panel, 0, 0)
        root.addWidget(telemetry_panel, 0, 1)
        root.addWidget(comms_panel, 0, 2)

        root.setColumnStretch(0, 2)
        root.setColumnStretch(1, 2)
        root.setColumnStretch(2, 2)
        self.setLayout(root)
        self.setWindowTitle("Stewart Platform Mission Console")
        self.resize(1600, 920)

    def _build_controls_panel(self):
        box = QGroupBox("Control Blocks")
        layout = QVBoxLayout()

        self.sliders = {}
        pose = QGroupBox("Pose Command")
        pose_layout = QVBoxLayout()
        for ax in ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]:
            lbl = QLabel(f"{ax}: 0")
            sld = QSlider(QtCore.Qt.Horizontal)
            sld.setMinimum(-100)
            sld.setMaximum(100)
            sld.setValue(0)
            pose_layout.addWidget(lbl)
            pose_layout.addWidget(sld)
            self.sliders[ax] = (lbl, sld)
        pose.setLayout(pose_layout)
        layout.addWidget(pose)

        pd = QGroupBox("PD Gains")
        pd_layout = QVBoxLayout()
        self.kp_label = QLabel(f"Kp: {PD_DEFAULT_KP:.3f}")
        self.kp_slider = QSlider(QtCore.Qt.Horizontal)
        self.kp_slider.setMinimum(0)
        self.kp_slider.setMaximum(300)
        self.kp_slider.setValue(int(round(PD_DEFAULT_KP * 1000.0)))
        self.kd_label = QLabel(f"Kd: {PD_DEFAULT_KD:.3f}")
        self.kd_slider = QSlider(QtCore.Qt.Horizontal)
        self.kd_slider.setMinimum(0)
        self.kd_slider.setMaximum(100)
        self.kd_slider.setValue(int(round(PD_DEFAULT_KD * 1000.0)))
        self.target_x_label = QLabel(f"Target X (mm): {int(BALL_TARGET_DEFAULT_X_MM)}")
        self.target_x_slider = QSlider(QtCore.Qt.Horizontal)
        self.target_x_slider.setMinimum(-120)
        self.target_x_slider.setMaximum(120)
        self.target_x_slider.setValue(int(round(BALL_TARGET_DEFAULT_X_MM)))
        self.target_y_label = QLabel(f"Target Y (mm): {int(BALL_TARGET_DEFAULT_Y_MM)}")
        self.target_y_slider = QSlider(QtCore.Qt.Horizontal)
        self.target_y_slider.setMinimum(-120)
        self.target_y_slider.setMaximum(120)
        self.target_y_slider.setValue(int(round(BALL_TARGET_DEFAULT_Y_MM)))
        pd_layout.addWidget(self.kp_label)
        pd_layout.addWidget(self.kp_slider)
        pd_layout.addWidget(self.kd_label)
        pd_layout.addWidget(self.kd_slider)
        pd_layout.addWidget(self.target_x_label)
        pd_layout.addWidget(self.target_x_slider)
        pd_layout.addWidget(self.target_y_label)
        pd_layout.addWidget(self.target_y_slider)
        self.autotune_enable_btn = QPushButton("Enable PD AutoTune")
        self.autotune_apply_btn = QPushButton("Apply AutoTune Recommendation")
        self.autotune_auto_apply_btn = QPushButton("Start Auto-Apply AutoTune")
        self.autotune_apply_btn.setEnabled(False)
        pd_layout.addWidget(self.autotune_enable_btn)
        pd_layout.addWidget(self.autotune_apply_btn)
        pd_layout.addWidget(self.autotune_auto_apply_btn)
        pd.setLayout(pd_layout)
        layout.addWidget(pd)

        hsv = QGroupBox("HSV Thresholds")
        hsv_layout = QVBoxLayout()
        self.hsv_controls = {}
        defaults = [
            ("H Min", TRACKER_HSV_H_MIN, 179),
            ("H Max", TRACKER_HSV_H_MAX, 179),
            ("S Min", TRACKER_HSV_S_MIN, 255),
            ("S Max", TRACKER_HSV_S_MAX, 255),
            ("V Min", TRACKER_HSV_V_MIN, 255),
            ("V Max", TRACKER_HSV_V_MAX, 255),
        ]
        for name, val, maxv in defaults:
            lbl = QLabel(f"{name}: {val}")
            sld = QSlider(QtCore.Qt.Horizontal)
            sld.setMinimum(0)
            sld.setMaximum(maxv)
            sld.setValue(val)
            hsv_layout.addWidget(lbl)
            hsv_layout.addWidget(sld)
            self.hsv_controls[name] = (lbl, sld)
        hsv.setLayout(hsv_layout)
        layout.addWidget(hsv)

        box.setLayout(layout)
        return box

    def _build_telemetry_panel(self):
        box = QGroupBox("Telemetry")
        layout = QVBoxLayout()

        self.visualizer_figure = Figure(figsize=(6, 3.8))
        self.visualizer_canvas = FigureCanvas(self.visualizer_figure)
        self.visualizer_canvas.setMinimumHeight(360)
        layout.addWidget(self.visualizer_canvas)

        self.timing_summary_label = QLabel("Vision Timing Avg (ms): waiting for data...")
        self.timing_summary_label.setWordWrap(True)
        layout.addWidget(self.timing_summary_label)

        self.timing_figure = Figure(figsize=(6, 5))
        self.timing_canvas = FigureCanvas(self.timing_figure)
        self.timing_ax = self.timing_figure.add_subplot(111)
        self.timing_canvas.setMinimumHeight(500)
        layout.addWidget(self.timing_canvas)

        box.setLayout(layout)
        return box

    def _build_comms_panel(self):
        box = QGroupBox("Routines + Commands + Serial")
        layout = QVBoxLayout()

        self.open_monitor_button = QPushButton("Open Vision Monitor Window")
        layout.addWidget(self.open_monitor_button)

        self.demo_list = QtWidgets.QComboBox()
        self.demo_list.addItem("(Choose a routine...)")
        layout.addWidget(QLabel("Demo Routines:"))
        layout.addWidget(self.demo_list)

        btn_row = QHBoxLayout()
        self.send_button = QPushButton("SEND TO ARDUINO")
        self.vision_button = QPushButton("Enable Vision Mode")
        self.cancel_vision_btn = QPushButton("Cancel Vision Mode")
        self.cancel_routine_btn = QPushButton("Cancel Routine")
        self.cancel_vision_btn.setEnabled(False)
        self.cancel_routine_btn.setEnabled(False)
        btn_row.addWidget(self.send_button)
        btn_row.addWidget(self.vision_button)
        btn_row.addWidget(self.cancel_vision_btn)
        btn_row.addWidget(self.cancel_routine_btn)
        layout.addLayout(btn_row)

        self.raw_serial_input = QtWidgets.QLineEdit()
        self.raw_serial_input.setPlaceholderText("Type raw serial command, e.g. S,90,90,90,90,90,90")
        self.raw_serial_send_btn = QPushButton("Send Raw Command")
        layout.addWidget(self.raw_serial_input)
        layout.addWidget(self.raw_serial_send_btn)

        self.preview_output = QTextEdit()
        self.preview_output.setReadOnly(True)
        self.preview_output.setPlaceholderText("Command preview / diagnostics")
        layout.addWidget(QLabel("Preview Log:"))
        layout.addWidget(self.preview_output)

        self.serial_monitor = QTextEdit()
        self.serial_monitor.setReadOnly(True)
        self.serial_monitor.setPlaceholderText("Serial monitor")
        layout.addWidget(QLabel("Serial Monitor:"))
        layout.addWidget(self.serial_monitor)

        box.setLayout(layout)
        return box

    def _apply_theme(self):
        self.setStyleSheet(
            """
            QWidget { background: #0b0f16; color: #d6e2ff; font-family: 'Segoe UI'; font-size: 10pt; }
            QGroupBox { border: 1px solid #1f2a3a; margin-top: 10px; border-radius: 6px; padding: 8px; }
            QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; color: #7dd3fc; font-weight: 600; }
            QPushButton { background: #162033; border: 1px solid #2a3b59; border-radius: 5px; padding: 6px 10px; color: #d6e2ff; }
            QPushButton:hover { background: #1e2c45; }
            QPushButton:pressed { background: #0f1726; }
            QPushButton:disabled { color: #6b7a90; border-color: #2a3342; }
            QTextEdit, QLineEdit, QComboBox { background: #0f1726; border: 1px solid #27364d; border-radius: 4px; color: #d6e2ff; }
            QSlider::groove:horizontal { border: 1px solid #2a3b59; height: 6px; background: #121a2a; border-radius: 3px; }
            QSlider::handle:horizontal { background: #7dd3fc; border: 1px solid #38bdf8; width: 14px; margin: -6px 0; border-radius: 7px; }
            QLabel { color: #d6e2ff; }
            """
        )
