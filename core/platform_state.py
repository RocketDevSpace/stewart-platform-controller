from dataclasses import dataclass, field
from typing import Optional


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass
class ServoAngles:
    angles: list[float] = field(default_factory=lambda: [0.0] * 6)


@dataclass
class BallState:
    x_mm: float
    y_mm: float
    vx_mm_s: float
    vy_mm_s: float
    z_mm: Optional[float] = None
    vz_mm_s: Optional[float] = None
