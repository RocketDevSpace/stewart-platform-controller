# -*- coding: utf-8 -*-
"""Stewart Platform IK solver.

Called ONLY through core/ik_engine.py (hard constraint 3).

Branch selection design (rewritten 2026-07-22; replaces the seed-by-ball-
joint-height + asymmetric-alternate scheme that caused the yaw-snap logged
at M5): for every servo BOTH elbow branches are computed symmetrically —
ball-joint point AND servo angle per branch — then exactly one branch is
chosen, and the returned angle, arm point, and continuity state all come
from that single choice:

- With prev_arm_points: the branch whose ball joint is closest to the
  previous one (continuity).
- Without: the branch whose servo angle is inside [0, 180]; if both
  qualify, the one closest to SERVO_NEUTRAL_DEG. (Ball-joint height is NOT
  a valid criterion — at large yaw it selects a branch whose servo angle
  is ~260 deg, which the old code silently clamped to 180.)

A chosen angle outside [0, 180] beyond a small tolerance is a PER-SERVO
FAILURE (recorded in servo_status), never a silent clamp: clamping detaches
the commanded angle from the returned geometry and, when the underlying
angle drifts through the +/-180 wrap, turns continuous motion into a
commanded 180-deg snap on real hardware.
"""

import numpy as np

from config import (
    ARM_LENGTH,
    PLATFORM_POINTS_LOCAL,
    ROD_LENGTH,
    SERVO_AXES,
    SERVO_MATH_HORIZONTAL,
    SERVO_NEUTRAL_DEG,
    SERVO_SHAFTS,
    SERVO_SIGN,
    Z_NEUTRAL,
)
from core.platform_state import IKResult

# Numerical slack for the workspace-boundary test: |ratio| in (1, 1+EPS]
# is float noise on a legally reachable edge pose, not "no solution".
_RATIO_EPS = 1e-9
# Servo angles this far outside [0, 180] are clamped as numeric noise;
# anything further out is a per-servo failure.
_ANGLE_TOL_DEG = 0.5


# -----------------------
# Rotation helpers
# -----------------------
def rot_x(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def build_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """roll/pitch/yaw in radians."""
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def wrap_deg(angle: float) -> float:
    """Wrap any angle into [-180, 180) degrees."""
    return (angle + 180.0) % 360.0 - 180.0


def _servo_angle_from_theta(theta_rad: float, i: int) -> float:
    """Map a math-frame arm angle to the physical servo command angle."""
    delta = wrap_deg(np.rad2deg(theta_rad) - SERVO_MATH_HORIZONTAL[i])
    if SERVO_MATH_HORIZONTAL[i] == 0:
        return float(SERVO_NEUTRAL_DEG + delta * SERVO_SIGN[i])
    return float(SERVO_NEUTRAL_DEG - delta * SERVO_SIGN[i])


# -----------------------
# Main IK solver
# -----------------------
def solve_pose(
    x: float,
    y: float,
    z: float,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    prev_arm_points: np.ndarray | None = None,
) -> IKResult:
    """Solve Stewart platform IK for one pose (z is an offset from Z_NEUTRAL).

    Returns core.platform_state.IKResult. On any per-servo failure the
    result's success is False, servo_status[i] carries the reason, and
    that servo's angle slot holds the SERVO_NEUTRAL_DEG placeholder.
    """
    roll = np.deg2rad(roll_deg)
    pitch = np.deg2rad(pitch_deg)
    yaw = np.deg2rad(yaw_deg)

    R = build_rotation_matrix(roll, pitch, yaw)
    platform_center = np.array([x, y, Z_NEUTRAL + z])
    platform_points_world = (R @ PLATFORM_POINTS_LOCAL.T).T + platform_center

    arm_points_world = np.zeros((6, 3))
    servo_angles: list[float] = [float(SERVO_NEUTRAL_DEG)] * 6
    servo_status: list[str | None] = [None] * 6
    debug: dict = {"failures": []}

    for i in range(6):
        shaft = SERVO_SHAFTS[i]
        axis = SERVO_AXES[i] / np.linalg.norm(SERVO_AXES[i])
        target = platform_points_world[i]

        # Basis for the arm-rotation plane (perpendicular to the axis).
        tmp = np.array([0, 0, 1.0])
        if abs(np.dot(tmp, axis)) > 0.9:  # defensive; all axes are horizontal
            tmp = np.array([0, 1.0, 0])
        u = np.cross(axis, tmp)
        u /= np.linalg.norm(u)
        v = np.cross(axis, u)
        v /= np.linalg.norm(v)

        # Target position relative to the shaft, in plane coordinates.
        d = target - shaft
        du = np.dot(d, u)
        dv = np.dot(d, v)
        da = np.dot(d, axis)
        proj_dist = np.sqrt(du**2 + dv**2)

        if proj_dist < 1e-9:
            servo_status[i] = "target on servo axis (proj_dist ~ 0)"
            debug["failures"].append((i, servo_status[i]))
            continue

        # Circle/sphere intersection.
        rhs = (
            ARM_LENGTH**2 + du**2 + dv**2 + da**2 - ROD_LENGTH**2
        ) / (2 * ARM_LENGTH)
        ratio = rhs / proj_dist
        if abs(ratio) > 1.0 + _RATIO_EPS:
            servo_status[i] = f"no real solution ratio={ratio:.4f}"
            debug["failures"].append((i, servo_status[i]))
            continue
        ratio = float(np.clip(ratio, -1.0, 1.0))
        phi = np.arctan2(dv, du)

        spread = np.arccos(ratio)
        thetas = (phi + spread, phi - spread)

        # BOTH branches, symmetrically: ball point + servo angle each.
        points = tuple(
            shaft + ARM_LENGTH * (np.cos(t) * u + np.sin(t) * v)
            for t in thetas
        )
        angles = tuple(_servo_angle_from_theta(t, i) for t in thetas)

        # Choose exactly one branch.
        if prev_arm_points is not None:
            prev_p = prev_arm_points[i]
            k = 0 if (
                np.linalg.norm(points[0] - prev_p)
                <= np.linalg.norm(points[1] - prev_p)
            ) else 1
        else:
            in_range = [
                -_ANGLE_TOL_DEG <= a <= 180.0 + _ANGLE_TOL_DEG for a in angles
            ]
            if in_range[0] and in_range[1]:
                k = 0 if (
                    abs(angles[0] - SERVO_NEUTRAL_DEG)
                    <= abs(angles[1] - SERVO_NEUTRAL_DEG)
                ) else 1
            elif in_range[0] or in_range[1]:
                k = 0 if in_range[0] else 1
            else:
                servo_status[i] = (
                    f"both branches out of range "
                    f"({angles[0]:.1f}, {angles[1]:.1f})"
                )
                debug["failures"].append((i, servo_status[i]))
                continue

        chosen_angle = angles[k]
        if not (-_ANGLE_TOL_DEG <= chosen_angle <= 180.0 + _ANGLE_TOL_DEG):
            servo_status[i] = f"servo angle out of range ({chosen_angle:.1f})"
            debug["failures"].append((i, servo_status[i]))
            continue

        arm_points_world[i] = points[k]
        servo_angles[i] = float(np.clip(chosen_angle, 0.0, 180.0))

    return IKResult(
        success=all(s is None for s in servo_status),
        servo_angles_deg=tuple(servo_angles),
        platform_points=platform_points_world,
        arm_points=arm_points_world,
        platform_center=platform_center,
        platform_R=R,
        servo_status=tuple(servo_status),
        debug=debug,
    )
