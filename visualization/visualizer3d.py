from __future__ import annotations

from typing import Any

import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

import logging

from config import SERVO_SHAFTS, SERVO_AXES, PLATFORM_SIZE
from core.ik_engine import IKEngine
from core.platform_state import IKResult, Pose

logger = logging.getLogger(__name__)


class StewartVisualizer:
    """3D platform view with persistent artists.

    All scatters/lines and the legend are created ONCE in __init__ and
    updated in place via set_data_3d / _offsets3d on every update — no
    per-call cla()/replot (the old approach rebuilt the whole axes each
    frame, which dominated GUI-thread time at VISUALIZER_HZ). The dark
    style, axis limits, and view are static and applied once.

    Does NOT call IK for pre-solved geometry (hard constraint 9): callers
    pass ik_result; IKEngine is used only as a fallback when they don't.
    """

    def __init__(self, canvas: FigureCanvas) -> None:
        self.canvas = canvas
        self.prev_arm_points: np.ndarray | None = None
        self._ik = IKEngine()

        self.fig = canvas.figure
        self.ax: Any = self.fig.add_subplot(111, projection="3d")
        self.ax.set_box_aspect([1, 1, 0.8])
        self._apply_dark_style()

        # --- Static artists (never change) ---
        self.ax.scatter(
            SERVO_SHAFTS[:, 0], SERVO_SHAFTS[:, 1], SERVO_SHAFTS[:, 2],
            s=50, c='r', label='Shafts',
        )
        for i in range(6):
            p = SERVO_SHAFTS[i]
            axis = SERVO_AXES[i]
            line = np.array([p - axis * 10, p + axis * 10])
            self.ax.plot(line[:, 0], line[:, 1], line[:, 2], c='r')

        # --- Dynamic artists (updated in place each frame) ---
        self._platform_scatter = self.ax.scatter(
            [], [], [], s=60, c='b', label='Platform Points',
        )
        self._arm_scatter = self.ax.scatter(
            [], [], [], s=60, c='g', label='Arm Points',
        )
        self._rod_lines = [
            self.ax.plot([], [], [], linewidth=2, c='k')[0] for _ in range(6)
        ]
        self._arm_lines = [
            self.ax.plot([], [], [], linewidth=2, c='orange')[0]
            for _ in range(6)
        ]
        # Platform square outline: one closed 5-point loop.
        self._square_line = self.ax.plot([], [], [], linewidth=2, c='b')[0]

        # --- Static axes limits / view / legend (set once) ---
        self.ax.set_xlim(-120, 120)
        self.ax.set_ylim(-120, 120)
        self.ax.set_zlim(0, 200)
        self.ax.view_init(elev=20, azim=30)
        self.ax.legend(
            facecolor="#111827", edgecolor="#2a3b59",
            labelcolor="#d6e2ff", fontsize=7,
        )

    def update_platform(
        self,
        pos_dict: dict,
        ik_result: IKResult | None = None,
    ) -> None:
        if ik_result is not None and ik_result.success:
            solution = ik_result
        else:
            pose = Pose(
                x=pos_dict['x'],
                y=pos_dict['y'],
                z=pos_dict['z'],
                roll=pos_dict['roll'],
                pitch=pos_dict['pitch'],
                yaw=pos_dict['yaw'],
            )
            solution = self._ik.solve(pose, self.prev_arm_points)

        if not solution.success:
            # Keep the previously drawn geometry (platform holds its last
            # commanded pose on an IK failure) — just report it.
            logger.warning("IK failed in visualizer: %s", solution.servo_status)
            return

        platform_points = solution.platform_points
        arm_points = solution.arm_points

        self._update_square_platform(
            solution.platform_center, solution.platform_R
        )
        self._platform_scatter._offsets3d = (
            platform_points[:, 0], platform_points[:, 1], platform_points[:, 2],
        )
        self._arm_scatter._offsets3d = (
            arm_points[:, 0], arm_points[:, 1], arm_points[:, 2],
        )
        for i in range(6):
            rod = np.array([arm_points[i], platform_points[i]])
            self._rod_lines[i].set_data_3d(rod[:, 0], rod[:, 1], rod[:, 2])
            arm = np.array([SERVO_SHAFTS[i], arm_points[i]])
            self._arm_lines[i].set_data_3d(arm[:, 0], arm[:, 1], arm[:, 2])

        self.prev_arm_points = arm_points.copy()
        self.canvas.draw_idle()

    def _apply_dark_style(self) -> None:
        pane_bg = "#111827"
        grid_c = "#2a3b59"
        text_c = "#d6e2ff"
        dim_c = "#9fb4d9"

        self.fig.patch.set_facecolor("#0f1726")
        self.ax.set_facecolor(pane_bg)

        self.ax.set_xlabel("X", color=dim_c)
        self.ax.set_ylabel("Y", color=dim_c)
        self.ax.set_zlabel("Z", color=dim_c)
        self.ax.set_title("Stewart Platform", color=text_c, fontsize=9)
        self.ax.tick_params(colors=dim_c, labelsize=6)

        for axis in (self.ax.xaxis, self.ax.yaxis, self.ax.zaxis):
            axis.pane.set_facecolor(pane_bg)
            axis.pane.set_edgecolor(grid_c)
            axis.pane.set_alpha(0.9)
            axis._axinfo["grid"]["color"] = grid_c  # type: ignore[attr-defined]

    def _update_square_platform(self, center: np.ndarray, R: np.ndarray) -> None:
        half = PLATFORM_SIZE / 2
        corners_local = np.array([
            [-half, -half, 0],
            [half, -half, 0],
            [half, half, 0],
            [-half, half, 0],
            [-half, -half, 0],
        ])
        corners_world = (R @ corners_local.T).T + center
        self._square_line.set_data_3d(
            corners_world[:, 0], corners_world[:, 1], corners_world[:, 2],
        )
