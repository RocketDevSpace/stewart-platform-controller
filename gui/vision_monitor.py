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

# Velocity arrow scale: 1 px per N mm/s
_VEL_ARROW_SCALE = 8.0

_LABEL_COLOR = (0, 210, 255)     # cyan-yellow
_BALL_COLOR = (0, 255, 80)       # green
_VEL_COLOR = (0, 165, 255)       # orange
_DISP_COLOR = (255, 255, 255)    # white — displacement to target
_PD_COLOR = (220, 80, 220)       # magenta — PD restoration command
_TARGET_COLOR = (80, 80, 255)    # red-ish
_TEXT_COLOR = (210, 255, 210)    # pale green
_PATH_COLOR = (200, 200, 0)      # teal — path polyline + carrot

# PD vec scale: px per degree of computed tilt command
_PD_VEC_SCALE_PX_PER_DEG = 18.0


def _bgr_to_pixmap(bgr: np.ndarray, w: int, h: int) -> QPixmap:
    rgb = np.ascontiguousarray(bgr[..., ::-1])
    # tobytes() copies, which also guarantees the buffer outlives the QImage.
    qimg = QImage(
        rgb.tobytes(),
        rgb.shape[1],
        rgb.shape[0],
        rgb.strides[0],
        QImage.Format_RGB888,
    )
    return QPixmap.fromImage(qimg).scaled(
        w, h, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation
    )


def _gray_to_bgr(gray: np.ndarray) -> np.ndarray:
    """Stack a single-channel image to 3-channel; pass 3-channel through
    (contiguous)."""
    if gray.ndim == 2:
        return np.stack([gray, gray, gray], axis=2)
    return np.ascontiguousarray(gray)


def _gray_to_pixmap(gray3: np.ndarray, w: int, h: int) -> QPixmap:
    """gray3 must already be 3-channel (see _gray_to_bgr)."""
    gray3 = np.ascontiguousarray(gray3)
    qimg = QImage(
        gray3.tobytes(),
        gray3.shape[1],
        gray3.shape[0],
        gray3.strides[0],
        QImage.Format_RGB888,
    )
    return QPixmap.fromImage(qimg).scaled(
        w, h, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation
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


def _arrow(
    frame: np.ndarray,
    p1: tuple[int, int],
    p2: tuple[int, int],
    color: tuple[int, int, int],
    thickness: int = 2,
) -> None:
    """Draw a clamped arrowed line; skips if too short."""
    h, w = frame.shape[:2]
    x2 = max(0, min(w - 1, p2[0]))
    y2 = max(0, min(h - 1, p2[1]))
    if abs(x2 - p1[0]) + abs(y2 - p1[1]) < 3:
        return
    cv2.arrowedLine(frame, p1, (x2, y2), color, thickness, cv2.LINE_AA, tipLength=0.25)


def _draw_warped_overlays(
    bgr: np.ndarray,
    ball_state: BallState | None,
    target_x_mm: float = 0.0,
    target_y_mm: float = 0.0,
    control_terms: dict | None = None,
    platform_size_mm: float = 240.0,
    path_points_mm: np.ndarray | None = None,
    path_closed: bool = True,
) -> np.ndarray:
    frame = bgr.copy()
    h, w = frame.shape[:2]
    px_per_mm = h / platform_size_mm
    cx = w // 2
    cy = h // 2

    _put_label(frame, "WARPED")

    # Path polyline (teal) — drawn first, beneath the other overlays.
    if path_points_mm is not None and len(path_points_mm) >= 2:
        px = (cx + path_points_mm[:, 0] * px_per_mm).astype(np.int32)
        py = (cy - path_points_mm[:, 1] * px_per_mm).astype(np.int32)
        poly = np.column_stack([px, py]).reshape(-1, 1, 2)
        cv2.polylines(
            frame, [poly], isClosed=bool(path_closed),
            color=_PATH_COLOR, thickness=1, lineType=cv2.LINE_AA,
        )

    # Target crosshair — prefer the frame-accurate target from the worker's
    # control terms: moving path/autotune targets are frame-accurate in
    # terms; the mirror args remain the fallback for the terms-less
    # invalid-tracking case.
    if (
        control_terms is not None
        and "target_x_mm" in control_terms
        and "target_y_mm" in control_terms
    ):
        target_x_mm = float(control_terms["target_x_mm"])
        target_y_mm = float(control_terms["target_y_mm"])
    tx = int(cx + target_x_mm * px_per_mm)
    ty = int(cy - target_y_mm * px_per_mm)
    cv2.line(frame, (tx - 14, ty), (tx + 14, ty), _TARGET_COLOR, 1, cv2.LINE_AA)
    cv2.line(frame, (tx, ty - 14), (tx, ty + 14), _TARGET_COLOR, 1, cv2.LINE_AA)

    # Path following: moving carrot at the follower's current target +
    # progress text bottom-right.
    if control_terms is not None and bool(control_terms.get("path_active")):
        cv2.circle(frame, (tx, ty), 5, _PATH_COLOR, -1, cv2.LINE_AA)
        path_txt = (
            f"PATH {control_terms.get('path_state', '')} "
            f"lap {int(control_terms.get('path_lap', 0))} "
            f"{float(control_terms.get('path_progress', 0.0)):.0%}"
        )
        size, _base = cv2.getTextSize(
            path_txt, cv2.FONT_HERSHEY_SIMPLEX, 0.46, 1
        )
        org = (w - int(size[0]) - 8, h - 8)
        cv2.putText(frame, path_txt, org,
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, path_txt, org,
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, _PATH_COLOR, 1, cv2.LINE_AA)

    if ball_state is not None:
        bx = int(cx + ball_state.x_mm * px_per_mm)
        by = int(cy - ball_state.y_mm * px_per_mm)
        bp = (bx, by)

        # Ball circle
        cv2.circle(frame, bp, 10, _BALL_COLOR, 2, cv2.LINE_AA)
        cv2.circle(frame, bp, 3, _BALL_COLOR, -1)

        # --- Vector 1: displacement to target (white) ---
        if control_terms is not None:
            dv = control_terms.get("position_vec_mm", None)
        else:
            dv = (target_x_mm - ball_state.x_mm, target_y_mm - ball_state.y_mm)
        if dv is not None:
            dx_px = int(dv[0] * px_per_mm)
            dy_px = int(-dv[1] * px_per_mm)
            _arrow(frame, bp, (bx + dx_px, by + dy_px), _DISP_COLOR, 2)

        # --- Vector 2: velocity (orange) ---
        if control_terms is not None:
            vv = control_terms.get("velocity_vec_mm_s", None)
        else:
            vv = (ball_state.vx_mm_s, ball_state.vy_mm_s)
        if vv is not None:
            vx_px = int(vv[0] * px_per_mm / _VEL_ARROW_SCALE)
            vy_px = int(-vv[1] * px_per_mm / _VEL_ARROW_SCALE)
            _arrow(frame, bp, (bx + vx_px, by + vy_px), _VEL_COLOR, 2)

        # --- Vector 3: PD restoration command (magenta) ---
        if control_terms is not None:
            pd = control_terms.get("pd_vec", None)
            if pd is not None:
                pd_x_px = int(pd[0] * _PD_VEC_SCALE_PX_PER_DEG)
                pd_y_px = int(-pd[1] * _PD_VEC_SCALE_PX_PER_DEG)
                _arrow(frame, bp, (bx + pd_x_px, by + pd_y_px), _PD_COLOR, 2)

        # Legend (top-right)
        legend = [
            ("disp", _DISP_COLOR),
            ("vel", _VEL_COLOR),
            ("PD", _PD_COLOR),
        ]
        for i, (lbl, col) in enumerate(legend):
            lx = w - 70
            ly = 22 + i * 18
            cv2.line(frame, (lx, ly), (lx + 20, ly), col, 2, cv2.LINE_AA)
            cv2.putText(frame, lbl, (lx + 24, ly + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, lbl, (lx + 24, ly + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1, cv2.LINE_AA)

        # Status text (bottom-left)
        pd_txt = ""
        if control_terms is not None:
            pd = control_terms.get("pd_vec")
            if pd is not None:
                pd_txt = f"PD ({pd[0]:+.2f}, {pd[1]:+.2f}) deg"
        lines = [
            f"pos ({ball_state.x_mm:+.1f}, {ball_state.y_mm:+.1f}) mm",
            f"vel ({ball_state.vx_mm_s:+.0f}, {ball_state.vy_mm_s:+.0f}) mm/s",
        ]
        if pd_txt:
            lines.append(pd_txt)
        for i, txt in enumerate(lines):
            y_txt = h - (len(lines) - i) * 18 - 4
            cv2.putText(frame, txt, (8, y_txt),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.46, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, txt, (8, y_txt),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.46, _TEXT_COLOR, 1, cv2.LINE_AA)

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
        super().__init__(parent, Qt.WindowType.Window)
        self.setWindowTitle("Vision Monitor")
        self.setStyleSheet("background: #0b0f16;")

        self._warped_label = QLabel("Warped: Disabled")
        self._warped_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._warped_label.setMinimumSize(_MIN_W, _MIN_H * 2)
        self._warped_label.setStyleSheet(_DARK_BORDER)

        self._camera_label = QLabel("Camera: Disabled")
        self._camera_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._camera_label.setMinimumSize(_MIN_W, _MIN_H)
        self._camera_label.setStyleSheet(_DARK_BORDER)

        self._mask_label = QLabel("HSV Mask: Disabled")
        self._mask_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._mask_label.setMinimumSize(_MIN_W, _MIN_H)
        self._mask_label.setStyleSheet(_DARK_BORDER)

        grid = QGridLayout()
        grid.addWidget(self._warped_label, 0, 0, 2, 1)
        grid.addWidget(self._camera_label, 0, 1)
        grid.addWidget(self._mask_label, 1, 1)
        self.setLayout(grid)

        # Path polyline (mm) drawn on the warped view; set/cleared by
        # MainWindow via set_path_overlay(), read per-frame by
        # update_warped().
        self._path_points_mm: np.ndarray | None = None
        self._path_closed = True

    def set_path_overlay(
        self, points_mm: np.ndarray | None, closed: bool = True
    ) -> None:
        """Store the path polyline ((N, 2) platform mm) or clear with
        None."""
        self._path_points_mm = (
            None if points_mm is None
            else np.asarray(points_mm, dtype=np.float64)
        )
        self._path_closed = bool(closed)

    # ------------------------------------------------------------------
    # Public update methods (called from MainWindow's snapshot handler)
    # ------------------------------------------------------------------

    def update_warped(
        self,
        bgr: np.ndarray | None,
        ball_state: BallState | None = None,
        target_x_mm: float = 0.0,
        target_y_mm: float = 0.0,
        control_terms: dict | None = None,
        platform_size_mm: float = 240.0,
    ) -> None:
        if bgr is None:
            self._warped_label.setText("Warped: Disabled")
            return
        frame = _draw_warped_overlays(
            bgr, ball_state, target_x_mm, target_y_mm, control_terms,
            platform_size_mm,
            path_points_mm=self._path_points_mm,
            path_closed=self._path_closed,
        )
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
        frame = _draw_mask_overlays(_gray_to_bgr(gray))
        w = self._mask_label.width()
        h = self._mask_label.height()
        self._mask_label.setPixmap(_gray_to_pixmap(frame, w, h))
