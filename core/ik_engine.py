from __future__ import annotations

import numpy as np

from core.platform_state import Pose


class IKEngine:
    def solve(self, pose: Pose, prev_arm_points: np.ndarray | None = None) -> dict:
        from kinematics.ik_solver import solve_pose
        try:
            return solve_pose(  # type: ignore[no-any-return]
                pose.x, pose.y, pose.z,
                pose.roll, pose.pitch, pose.yaw,
                prev_arm_points,
            )
        except Exception as e:
            return {"success": False, "debug": {"failures": [("exception", str(e))]}}


def solve_ik(pose: Pose, prev_arm_points: np.ndarray | None = None) -> dict:
    return IKEngine().solve(pose, prev_arm_points)
