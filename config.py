# -*- coding: utf-8 -*-
"""
Created on Mon Feb 16 12:25:39 2026

@author: hudso
"""

import numpy as np

# =========================
# PLATFORM / SERVO GEOMETRY
# =========================

# Neutral platform height (center of platform plane)
Z_NEUTRAL = 151.5

# Servo arm length (shaft center -> arm ball joint center)
ARM_LENGTH = 26.5   # <-- change if needed

# Rod length (arm ball joint -> platform ball joint)
ROD_LENGTH = 163.0  # <-- your measured U-joint/balljoint distance

# Platform size (square side length)
PLATFORM_SIZE = 240.0

# =========================
# SERVO SHAFT LOCATIONS (XY)
# =========================
# Numbering order you specified:
# 1: (66.0, 37.5)
# 2: (-0.524, 75.908)
# 3: (-65.476, 38.408)
# 4: (-65.476, -38.408)
# 5: (-0.524, -75.908)
# 6: (66.0, -37.5)

SERVO_SHAFTS_XY = np.array([
    (93.0, 37.5),
    (-14.024, 99.290),
    (-78.976, 61.790),
    (-78.976, -61.790),
    (-14.024, -99.920),
    (93.0, -37.5),
], dtype=float)

# Full 3D shaft centers (z=0 always)
SERVO_SHAFTS = np.column_stack((SERVO_SHAFTS_XY, np.zeros(6)))


# =========================
# PLATFORM BALL JOINT POINTS
# =========================
# Your mapping (platform point XY):
# 1: (43.7, 41.7)
# 2: (14.3, 58.7)
# 3: (-58, 17)
# 4: (-58, -17)
# 5: (14.3, -58.7)
# 6: (43.7, -41.7)

PLATFORM_POINTS_LOCAL_XY = np.array([
    (55.502, 60.332),
    (24.498, 78.232),
    (-80, 17.9),
    (-80, -17.9),
    (24.498, -78.232),
    (55.502, -60.332),
], dtype=float)

# Local platform points in its own coordinate frame (z=0 plane)
PLATFORM_POINTS_LOCAL = np.column_stack((PLATFORM_POINTS_LOCAL_XY, np.zeros(6)))


# =========================
# SERVO ARM ROTATION AXES
# =========================
# Each servo arm rotates in a vertical plane, meaning its axis is horizontal.
# We define the axis direction vector (unit) for each servo shaft.

# Face 1/6 shafts are on x=+ side, axis points +x (as you described)
# Other faces rotated +/-120 deg around Z.
# We can infer axis by angle of servo position around origin.

def compute_servo_axes(shafts_xy):
    axes = []
    for (x, y) in shafts_xy:
        angle = np.arctan2(y, x)  # angle around origin

        # Face axis direction is perpendicular to radius vector (approx)
        # But based on your earlier definition, axis is along outward normal of each face.
        # We approximate by snapping to the nearest of 3 directions.

        # Three face normals at 0, 120, -120 degrees
        candidates = [
            np.array([1.0, 0.0, 0.0]),
            np.array([np.cos(2*np.pi/3), np.sin(2*np.pi/3), 0.0]),
            np.array([np.cos(-2*np.pi/3), np.sin(-2*np.pi/3), 0.0]),
        ]

        # Choose closest by dot product with (x,y)
        radial = np.array([x, y, 0.0])
        radial /= np.linalg.norm(radial)

        best = max(candidates, key=lambda c: np.dot(c, radial))
        axes.append(best)

    return np.array(axes)

SERVO_AXES = compute_servo_axes(SERVO_SHAFTS_XY)


# =========================
# SERVO ROTATION DIRECTIONS
# =========================
# You stated:
# - servos 1,3,5 rotate same sense
# - servos 2,4,6 rotate opposite sense

SERVO_SIGN = np.array([+1, -1, +1, -1, +1, -1], dtype=float)


# =========================
# DEFAULT SERVO NEUTRAL ANGLES
# =========================
SERVO_NEUTRAL_DEG = 90.0

# Optional safety limits
SERVO_MIN_DEG = 0.0
SERVO_MAX_DEG = 180.0


# =========================
# RUNTIME / CONTROL CONFIG
# =========================

# Logging levels:
# 0 = silent
# 1 = errors/warnings
# 2 = periodic diagnostics
DEBUG_LEVEL = 1
LOG_EVERY_N = 25

# IO defaults
SERIAL_PORT = "COM4"
CAMERA_INDEX = 0
CAMERA_DEBUG_WINDOWS = True

# Loop rates
VISION_LOOP_HZ = 50
VISUALIZER_HZ = 25

# GUI buffers
GUI_LOG_MAX_LINES = 500

# Serial queue behavior
SERIAL_QUEUE_MAX = 256

# Timing plot config
TIMING_PLOT_POINTS = 300
