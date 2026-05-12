"""
gui/vision_monitor.py

Floating debug window showing three camera views from VisionControlWorker:
  - warped (perspective-corrected) frame — left column, spans 2 rows
  - raw camera frame — top-right
  - HSV mask — bottom-right

All views start as "Disabled" placeholders; MainWindow pushes numpy BGR/gray
arrays via update_warped(), update_camera(), and update_mask() from the
snapshot handler.

Overlays drawn with OpenCV before display:
  - Section label in top-left of every frame
  - Warped: ball circle, velocity arrow, position/velocity text, target cross
"""

from __future__ import annotations

import cv2
import numpy as np

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QGridLayout, QLabel, QWidget

from core.platform_state import BallState

_DARK_BORDER = "border: 1px solid #2a3b59; background: #0f1726;"
_MIN_W = 480
_MIN_H = 320

# Assumed platform diameter — matches BallTracker default (240 mm).
# px_per_mm is derived per-frame from the actual frame height.
_PLATFORM_DIAMETER_MM = 240.0

# Velocity arrow scale: 1 px per N mm/s
_VEL_ARROW_SCALE = 8.0

_LABEL_COLOR = (0, 210, 255)     # cyan-yellow
_BALL_COLOR = (0, 255, 80)       # green
_VEL_COLOR = (0, 165, 255)       # orange
_TARGET_COLOR = (80, 80, 255)    # red-ish
_TEXT_COLOR = (210, 255, 210)    # pale green


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


def _put_label(frame: np.ndarray, text: str) -> None:
    cv2.putText(
        frame, text, (8, 26),
        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA,
    )
    cv2.putText(
        frame, text, (8, 26),
        cv2.FONT_HERSHEY_SIMPLEX, 0.7, _LABEL_COLOR, 2, cv2.LINE_AA,
    )


def _draw_warped_overlays(
    bgr: np.ndarray,
    ball_state: BallState | None,
    target_x_mm: float = 0.0,
    target_y_mm: float = 0.0,
) -> np.ndarray:
    frame = bgr.copy()
    h, w = frame.shape[:2]
    px_per_mm = h / _PLATFORM_DIAMETER_MM
    cx = w // 2
    cy = h // 2

    _put_label(frame, "WARPED")

    # Target crosshair
    tx = int(cx + target_x_mm * px_per_mm)
    ty = int(cy - target_y_mm * px_per_mm)
    cv2.line(frame, (tx - 12, ty), (tx + 12, ty), _TARGET_COLOR, 1, cv2.LINE_AA)
    cv2.line(frame, (tx, ty - 12), (tx, ty + 12), _TARGET_COLOR, 1, cv2.LINE_AA)

    if ball_state is not None:
        bx = int(cx + ball_state.x_mm * px_per_mm)
        by = int(cy - ball_state.y_mm * px_per_mm)

        # Ball circle
        cv2.circle(frame, (bx, by), 10, _BALL_COLOR, 2, cv2.LINE_AA)
        cv2.circle(frame, (bx, by), 3, _BALL_COLOR, -1)

        # Velocity arrow
        vx_px = int(ball_state.vx_mm_s * px_per_mm / _VEL_ARROW_SCALE)
        vy_px = int(-ball_state.vy_mm_s * px_per_mm / _VEL_ARROW_SCALE)
        if abs(vx_px) + abs(vy_px) > 3:
            ex = max(0, min(w - 1, bx + vx_px))
            ey = max(0, min(h - 1, by + vy_px))
            cv2.arrowedLine(
                frame, (bx, by), (ex, ey),
                _VEL_COLOR, 2, cv2.LINE_AA, tipLength=0.35,
            )

        # Position and velocity text
        pos_txt = f"pos ({ball_state.x_mm:+.1f}, {ball_state.y_mm:+.1f}) mm"
        vel_txt = f"vel ({ball_state.vx_mm_s:+.0f}, {ball_state.vy_mm_s:+.0f}) mm/s"
        for i, txt in enumerate((pos_txt, vel_txt)):
            y_txt = h - 28 + i * 18
            cv2.putText(frame, txt, (8, y_txt),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                        (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, txt, (8, y_txt),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                        _TEXT_COLOR, 1, cv2.LINE_AA)

    return frame


def _draw_camera_overlays(bgr: np.ndarray) -> np.ndarray:
    frame = bgr.copy()
    _put_label(frame, "CAMERA")
    return frame


def _draw_mask_overlays(gray3: np.ndarray) -> np.ndarray:
    frame = gray3.copy()
    _put_label(frame, "MASK")
    return frame


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
        grid.addWidget(self._warped_label, 0, 0, 2, 1)
        grid.addWidget(self._camera_label, 0, 1)
        grid.addWidget(self._mask_label, 1, 1)
        self.setLayout(grid)

    # ------------------------------------------------------------------
    # Public update methods (called from MainWindow's snapshot handler)
    # ------------------------------------------------------------------

    def update_warped(
        self,
        bgr: np.ndarray | None,
        ball_state: BallState | None = None,
        target_x_mm: float = 0.0,
        target_y_mm: float = 0.0,
    ) -> None:
        if bgr is None:
            self._warped_label.setText("Warped: Disabled")
            return
        frame = _draw_warped_overlays(bgr, ball_state, target_x_mm, target_y_mm)
        w = self._warped_label.width()
        h = self._warped_label.height()
        self._warped_label.setPixmap(_bgr_to_pixmap(frame, w, h))

    def update_camera(self, bgr: np.ndarray | None) -> None:
        if bgr is None:
            self._camera_label.setText("Camera: Disabled")
            return
        frame = _draw_camera_overlays(bgr)
        w = self._camera_label.width()
        h = self._camera_label.height()
        self._camera_label.setPixmap(_bgr_to_pixmap(frame, w, h))

    def update_mask(self, gray: np.ndarray | None) -> None:
        if gray is None:
            self._mask_label.setText("HSV Mask: Disabled")
            return
        if gray.ndim == 2:
            gray3: np.ndarray = np.stack([gray, gray, gray], axis=2)
        else:
            gray3 = gray
        frame = _draw_mask_overlays(gray3)
        w = self._mask_label.width()
        h = self._mask_label.height()
        self._mask_label.setPixmap(_gray_to_pixmap(frame, w, h))
