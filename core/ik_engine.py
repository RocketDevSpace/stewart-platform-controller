from __future__ import annotations

import logging

import numpy as np

from core.platform_state import IKResult, Pose

logger = logging.getLogger(__name__)


class IKEngine:
    """The single call site for kinematics.ik_solver.solve_pose
    (hard constraint 3). Stateless: continuity is the caller's
    responsibility via prev_arm_points."""

    def solve(
        self, pose: Pose, prev_arm_points: np.ndarray | None = None
    ) -> IKResult:
        from kinematics.ik_solver import solve_pose
        try:
            return solve_pose(
                pose.x, pose.y, pose.z,
                pose.roll, pose.pitch, pose.yaw,
                prev_arm_points,
            )
        except Exception as exc:
            # Programming errors (e.g. malformed prev_arm_points) must not
            # masquerade as ordinary workspace failures — log the traceback.
            logger.exception("IK solve raised for pose=%s", pose)
            return IKResult.failed(str(exc))
