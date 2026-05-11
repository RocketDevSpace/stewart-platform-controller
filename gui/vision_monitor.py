"""
gui/vision_monitor.py

Floating debug window showing three camera views from VisionControlWorker:
  - warped (perspective-corrected) frame — left column, spans 2 rows
  - raw camera frame — top-right
  - HSV mask — bottom-right

All views start as "Disabled" placeholders; MainWindow pushes numpy BGR/gray
arrays via update_warped(), update_camera(), and update_mask() from the
snapshot handler.
"""

from __future__ import annotations

import numpy as np

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QGridLayout, QLabel, QWidget

_DARK_BORDER = "border: 1px solid #2a3b59; background: #0f1726;"
_MIN_W = 480
_MIN_H = 320


def _bgr_to_pixmap(bgr: np.ndarray, w: int, h: int) -> QPixmap:
    rgb = bgr[..., ::-1].copy()
    qimg = QImage(
        rgb.data,
        rgb.shape[1],
        rgb.shape[0],
        rgb.strides[0],
        QImage.Format_RGB888,
    )
    return QPixmap.fromImage(qimg).scaled(
        w, h, Qt.KeepAspectRatio, Qt.SmoothTransformation
    )


def _gray_to_pixmap(gray: np.ndarray, w: int, h: int) -> QPixmap:
    if gray.ndim == 2:
        gray3 = np.stack([gray, gray, gray], axis=2)
    else:
        gray3 = gray
    qimg = QImage(
        gray3.data,
        gray3.shape[1],
        gray3.shape[0],
        gray3.strides[0],
        QImage.Format_RGB888,
    )
    return QPixmap.fromImage(qimg).scaled(
        w, h, Qt.KeepAspectRatio, Qt.SmoothTransformation
    )


class VisionMonitorWindow(QWidget):
    """Floating window with three camera-view QLabels."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent, Qt.Window)
        self.setWindowTitle("Vision Monitor")
        self.setStyleSheet("background: #0b0f16;")

        self._warped_label = QLabel("Warped: Disabled")
        self._warped_label.setAlignment(Qt.AlignCenter)
        self._warped_label.setMinimumSize(_MIN_W, _MIN_H * 2)
        self._warped_label.setStyleSheet(_DARK_BORDER)

        self._camera_label = QLabel("Camera: Disabled")
        self._camera_label.setAlignment(Qt.AlignCenter)
        self._camera_label.setMinimumSize(_MIN_W, _MIN_H)
        self._camera_label.setStyleSheet(_DARK_BORDER)

        self._mask_label = QLabel("HSV Mask: Disabled")
        self._mask_label.setAlignment(Qt.AlignCenter)
        self._mask_label.setMinimumSize(_MIN_W, _MIN_H)
        self._mask_label.setStyleSheet(_DARK_BORDER)

        grid = QGridLayout()
        # Warped view occupies rows 0–1 in column 0
        grid.addWidget(self._warped_label, 0, 0, 2, 1)
        grid.addWidget(self._camera_label, 0, 1)
        grid.addWidget(self._mask_label, 1, 1)
        self.setLayout(grid)

    # ------------------------------------------------------------------
    # Public update methods (called from MainWindow's snapshot handler)
    # ------------------------------------------------------------------

    def update_warped(self, bgr: np.ndarray | None) -> None:
        if bgr is None:
            self._warped_label.setText("Warped: Disabled")
            return
        w = self._warped_label.width()
        h = self._warped_label.height()
        self._warped_label.setPixmap(_bgr_to_pixmap(bgr, w, h))

    def update_camera(self, bgr: np.ndarray | None) -> None:
        if bgr is None:
            self._camera_label.setText("Camera: Disabled")
            return
        w = self._camera_label.width()
        h = self._camera_label.height()
        self._camera_label.setPixmap(_bgr_to_pixmap(bgr, w, h))

    def update_mask(self, gray: np.ndarray | None) -> None:
        if gray is None:
            self._mask_label.setText("HSV Mask: Disabled")
            return
        w = self._mask_label.width()
        h = self._mask_label.height()
        self._mask_label.setPixmap(_gray_to_pixmap(gray, w, h))
