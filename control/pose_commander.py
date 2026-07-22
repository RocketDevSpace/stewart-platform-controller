"""
control/pose_commander.py

PoseCommander: the control-layer facade the GUI uses to turn a Pose into
servo motion. Wraps IKEngine (solve) + ServoDriver (send) so gui/ files
contain zero IK call sites and zero command dispatch logic (hard
constraint 5). No Qt imports — plain Python, unit-testable.
"""

from __future__ import annotations

import numpy as np

from core.ik_engine import IKEngine
from core.platform_state import IKResult, Pose
from hardware.servo_driver import ServoDriver


class PoseCommander:
    """Solve a Pose to servo angles and dispatch them to the hardware."""

    def __init__(self, ik_engine: IKEngine, servo_driver: ServoDriver) -> None:
        self._ik = ik_engine
        self._servo = servo_driver

    def solve(
        self, pose: Pose, prev_arm_points: np.ndarray | None = None
    ) -> IKResult:
        """Run IK for the pose. prev_arm_points carries elbow continuity."""
        return self._ik.solve(pose, prev_arm_points)

    def send(self, ik_result: IKResult) -> bool:
        """Send a solved result's angles through the servo driver.

        Safety clipping and the large-move ramp happen inside
        ServoDriver.send_angles. Returns False on serial failure.
        """
        return self._servo.send_angles(list(ik_result.servo_angles_deg))

    def solve_and_send(
        self, pose: Pose, prev_arm_points: np.ndarray | None = None
    ) -> tuple[IKResult, bool]:
        """Convenience: solve, then send if the solve succeeded.

        Returns (ik_result, sent) — sent is False when the solve failed
        or the serial send failed.
        """
        ik_result = self.solve(pose, prev_arm_points)
        sent = self.send(ik_result) if ik_result.success else False
        return ik_result, sent
