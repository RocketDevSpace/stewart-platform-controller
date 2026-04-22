"""
control/routine_runner.py

Routine playback state machine. Has NO Qt imports — the GUI owns the
QTimer and calls tick() on each interval. RoutineRunner owns all state:
the loaded step list, the current index, the running flag, and the
preview/send mode flag.

Preview mode: tick() just forwards the current pose to the GUI via the
on_pose_update callback, advances the index, and wraps at the end.

Send mode: tick() solves IK via the injected IKEngine, sends the
resulting servo angles via the injected ServoDriver, forwards the pose
to the GUI, and returns False once all steps have been consumed.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from core.ik_engine import IKEngine
from core.platform_state import Pose
from hardware.servo_driver import ServoDriver
from routines.routines import ROUTINES


class RoutineRunner:
    def __init__(
        self,
        ik_engine: IKEngine,
        serial_driver: ServoDriver,
        on_pose_update: Callable[[dict], None],
    ) -> None:
        self._ik = ik_engine
        self._servo = serial_driver
        self._on_pose = on_pose_update
        self._steps: list[dict] = []
        self._index: int = 0
        self._running: bool = False
        self._send_mode: bool = False
        self._prev_arm_points: Any = None

    def load(self, routine_name: str) -> bool:
        if routine_name not in ROUTINES:
            return False
        self._steps = list(ROUTINES[routine_name]())  # type: ignore[operator]
        self._index = 0
        self._running = False
        self._send_mode = False
        self._prev_arm_points = None
        return True

    def start_preview(self) -> None:
        self._running = True
        self._send_mode = False

    def start_send(self) -> None:
        self._running = True
        self._send_mode = True

    def cancel(self) -> None:
        self._running = False
        self._send_mode = False
        self._steps = []
        self._index = 0
        self._prev_arm_points = None

    def tick(self) -> bool:
        if not self._running or not self._steps:
            return False

        pose_dict = self._steps[self._index]

        if self._send_mode:
            pose = Pose(
                x=float(pose_dict["x"]),
                y=float(pose_dict["y"]),
                z=float(pose_dict["z"]),
                roll=float(pose_dict["roll"]),
                pitch=float(pose_dict["pitch"]),
                yaw=float(pose_dict["yaw"]),
            )
            ik_result = self._ik.solve(pose, self._prev_arm_points)
            if ik_result.get("success"):
                self._prev_arm_points = ik_result.get("arm_points")
                angles = list(ik_result["servo_angles_deg"])
                self._servo.send_angles(angles)

            self._on_pose(pose_dict)
            self._index += 1
            if self._index >= len(self._steps):
                self._running = False
                return False
            return True

        # preview mode: wrap at end, never stops on its own
        self._on_pose(pose_dict)
        self._index += 1
        if self._index >= len(self._steps):
            self._index = 0
        return True

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def is_send_mode(self) -> bool:
        return self._send_mode

    @property
    def current_index(self) -> int:
        return self._index

    @property
    def total_steps(self) -> int:
        return len(self._steps)
