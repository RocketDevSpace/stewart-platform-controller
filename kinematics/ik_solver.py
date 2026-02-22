# -*- coding: utf-8 -*-
"""
Stewart Platform IK Solver
Author: hudso
Updated: 2026-02-16
"""

import numpy as np
from stewart_control.config import (
    SERVO_SHAFTS,
    SERVO_AXES,
    SERVO_SIGN,
    SERVO_NEUTRAL_DEG,
    PLATFORM_POINTS_LOCAL,
    ARM_LENGTH,
    ROD_LENGTH,
    Z_NEUTRAL,
)

# -----------------------
# Rotation helper functions
# -----------------------
def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]])

def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])

def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])

def build_rotation_matrix(roll, pitch, yaw):
    """roll=pitch=yaw in radians"""
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)

def wrap_deg(angle):
    """Wrap any angle into [-180, 180] degrees."""
    return (angle + 180) % 360 - 180


# -----------------------
# Main IK solver
# -----------------------
def solve_pose(x, y, z, roll_deg, pitch_deg, yaw_deg, prev_arm_points=None):
    """
    Solve Stewart Platform IK.
    Returns:
      - platform_points: 6x3 array
      - arm_points: 6x3 array
      - servo_angles_deg: array of 6
      - success: bool
      - debug: dict
    """

    roll = np.deg2rad(roll_deg)
    pitch = np.deg2rad(pitch_deg)
    yaw = np.deg2rad(yaw_deg)

    R = build_rotation_matrix(roll, pitch, yaw)
    platform_center = np.array([x, y, Z_NEUTRAL + z])
    platform_points_world = (R @ PLATFORM_POINTS_LOCAL.T).T + platform_center

    arm_points_world = np.zeros((6,3))
    servo_angles_deg = np.zeros(6)
    debug = {"failures": []}
    success = True

    # Define math horizontal for each servo (degrees)
    # 0 = horizontal in math frame, 180 = horizontal reversed
    # Update this array based on your physical layout
    SERVO_MATH_HORIZONTAL = np.array([0, 180, 0, 180, 0, 180])

    for i in range(6):
        shaft = SERVO_SHAFTS[i]
        axis = SERVO_AXES[i] / np.linalg.norm(SERVO_AXES[i])
        target = platform_points_world[i]

        # Build basis vectors for plane perpendicular to axis
        tmp = np.array([0,0,1.0])
        if abs(np.dot(tmp, axis)) > 0.9:
            tmp = np.array([0,1.0,0])
        u = np.cross(axis, tmp)
        u /= np.linalg.norm(u)
        v = np.cross(axis, u)
        v /= np.linalg.norm(v)

        # Coordinates relative to shaft
        d = target - shaft
        du = np.dot(d, u)
        dv = np.dot(d, v)
        da = np.dot(d, axis)
        proj_dist = np.sqrt(du**2 + dv**2)

        # Solve circle/sphere intersection
        rhs = (ARM_LENGTH**2 + du**2 + dv**2 + da**2 - ROD_LENGTH**2) / (2*ARM_LENGTH)
        phi = np.arctan2(dv, du)

        if proj_dist < 1e-9:
            success = False
            debug["failures"].append((i, "proj_dist too small"))
            continue

        ratio = rhs / proj_dist
        if abs(ratio) > 1.0:
            success = False
            debug["failures"].append((i, f"no real solution ratio={ratio:.4f}"))
            continue

        theta1 = phi + np.arccos(ratio)
        theta2 = phi - np.arccos(ratio)

        # Convert to actual ball joint points
        P1 = shaft + ARM_LENGTH * (np.cos(theta1)*u + np.sin(theta1)*v)
        P2 = shaft + ARM_LENGTH * (np.cos(theta2)*u + np.sin(theta2)*v)

        # Smooth choice using prev_arm_points
        if prev_arm_points is not None:
            prevP = prev_arm_points[i]
            if np.linalg.norm(P1 - prevP) < np.linalg.norm(P2 - prevP):
                P = P1
                theta = theta1
            else:
                P = P2
                theta = theta2
        else:
            P = P1 if P1[2] > P2[2] else P2
            theta = theta1 if P1[2] > P2[2] else theta2

        arm_points_world[i] = P

        # ----------- Servo angle calculation -----------
        theta_deg = np.rad2deg(theta)
        theta_alt_deg = np.rad2deg(theta2)
        
        # Wrap raw theta into [-180, 180]
        theta_deg_wrapped = wrap_deg(theta_deg)
        theta_alt_wrapped = wrap_deg(theta_alt_deg)
        
        # Compute delta relative to horizontal reference, also wrapped
        delta_theta_deg = wrap_deg(theta_deg_wrapped - SERVO_MATH_HORIZONTAL[i])
        delta_theta_alt_deg = wrap_deg(theta_alt_wrapped - SERVO_MATH_HORIZONTAL[i])


        # Map to servo physical position
        if SERVO_MATH_HORIZONTAL[i] == 0:
            servo_candidate1 = SERVO_NEUTRAL_DEG + delta_theta_deg * SERVO_SIGN[i]
            servo_candidate2 = SERVO_NEUTRAL_DEG + delta_theta_alt_deg * SERVO_SIGN[i]
        else:  # horizontal = 180
            servo_candidate1 = SERVO_NEUTRAL_DEG - delta_theta_deg * SERVO_SIGN[i]
            servo_candidate2 = SERVO_NEUTRAL_DEG - delta_theta_alt_deg * SERVO_SIGN[i]

        # Pick candidate closest to neutral
        if abs(servo_candidate1 - SERVO_NEUTRAL_DEG) < abs(servo_candidate2 - SERVO_NEUTRAL_DEG):
            chosen_theta = theta_deg
            servo_angle = servo_candidate1
        else:
            chosen_theta = theta_alt_deg
            servo_angle = servo_candidate2

        # Clamp to 0-180
        servo_angle = np.clip(servo_angle, 0, 180)
        servo_angles_deg[i] = servo_angle

        # # Debug logging
        # print(f"[DEBUG] Servo {i} raw theta (deg): {theta_deg:.3f}, alt theta (deg): {theta_alt_deg:.3f}")
        # print(f"[DEBUG] Servo {i} delta theta (deg): {delta_theta_deg:.3f}, alt delta (deg): {delta_theta_alt_deg:.3f}")
        # print(f"[INFO] Servo {i} final angle (deg): {servo_angle:.3f} (chosen theta deg: {chosen_theta:.3f})")

    return {
        "success": success,
        "platform_points": platform_points_world,
        "arm_points": arm_points_world,
        "servo_angles_deg": servo_angles_deg,
        "debug": debug,
        "platform_center": platform_center,
        "platform_R": R
    }
