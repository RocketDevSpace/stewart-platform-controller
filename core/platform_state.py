"""Shared data contracts. Dataclasses only — no logic."""

from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass
class Pose:
    """Platform pose. x/y/z in mm, roll/pitch/yaw in degrees.

    z is an OFFSET from config.Z_NEUTRAL (the solver adds Z_NEUTRAL itself);
    z=0.0 is the neutral platform height, not the base plane.
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class IKResult:
    """Result of one IK solve. The typed contract for every consumer.

    servo_angles_deg holds SERVO_NEUTRAL_DEG placeholders (not zeros — 0 is
    a valid extreme command) for any servo whose status is not None. Never
    send angles from a result whose success is False.
    """

    success: bool
    servo_angles_deg: tuple[float, ...]        # 6 angles, degrees
    platform_points: np.ndarray                # (6, 3) world coords
    arm_points: np.ndarray                     # (6, 3) world coords
    platform_center: np.ndarray                # (3,)
    platform_R: np.ndarray                     # (3, 3) rotation matrix
    servo_status: tuple[Optional[str], ...]    # per-servo failure reason; None = OK
    debug: dict = field(default_factory=dict)

    @staticmethod
    def failed(reason: str, neutral_deg: float = 90.0) -> "IKResult":
        """A fully-populated failure result (e.g. for exception paths) so
        consumers never KeyError/AttributeError on the failure shape."""
        return IKResult(
            success=False,
            servo_angles_deg=(neutral_deg,) * 6,
            platform_points=np.zeros((6, 3)),
            arm_points=np.zeros((6, 3)),
            platform_center=np.zeros(3),
            platform_R=np.eye(3),
            servo_status=(reason,) * 6,
            debug={"failures": [("exception", reason)]},
        )


@dataclass
class BallState:
    x_mm: float
    y_mm: float
    vx_mm_s: float
    vy_mm_s: float
    z_mm: Optional[float] = None
    vz_mm_s: Optional[float] = None
