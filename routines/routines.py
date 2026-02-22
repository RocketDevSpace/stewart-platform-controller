# -*- coding: utf-8 -*-
"""
Created on Mon Feb 16 16:07:57 2026

@author: hudso
"""

# stewart_control/routines.py

import numpy as np
import math


def cube_routine_smooth(steps_per_edge=8):
    # 8 cube vertices, centered at 0
    verts = np.array([
        [+14, +14, +14],
        [+14, +14, -14],
        [+14, -14, +14],
        [+14, -14, -14],
        [-14, +14, +14],
        [-14, +14, -14],
        [-14, -14, +14],
        [-14, -14, -14],
    ])
    
    # Define a path through the cube vertices
    path_indices = [0, 1, 3, 2, 6, 7, 5, 4, 0]  # just one continuous loop

    poses = []
    for i in range(len(path_indices)-1):
        start = verts[path_indices[i]]
        end = verts[path_indices[i+1]]
        
        # interpolate linearly along the edge
        for t in np.linspace(0, 1, steps_per_edge, endpoint=False):
            interp = start*(1-t) + end*t
            pose = {
                "x": float(interp[0]),
                "y": float(interp[1]),
                "z": float(interp[2]),
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0
            }
            poses.append(pose)
    # include final vertex
    poses.append({"x": float(verts[path_indices[-1]][0]),
                  "y": float(verts[path_indices[-1]][1]),
                  "z": float(verts[path_indices[-1]][2]),
                  "roll": 0.0, "pitch": 0.0, "yaw": 0.0})
    return poses


def xy_circle(radius=20, num_points=100):
    poses = []

    for i in range(num_points):
        theta = 2 * math.pi * (i / num_points)

        x = radius * math.cos(theta)
        y = radius * math.sin(theta)

        poses.append({
            "x": x,
            "y": y,
            "z": 0,
            "roll": 0,
            "pitch": 0,
            "yaw": 0
        })

    return poses

def generate_cone_orbit_routine(radius=8, tilt_deg=15, z_height=0, steps=120):
    """
    Generates a circular XY motion while rolling/pitching inward toward origin,
    creating a cone-like sweep.

    radius   : circle radius in XY
    tilt_deg : max inward tilt angle in degrees
    z_height : constant Z
    steps    : number of points in the circle
    """

    routine = []

    for i in range(steps):
        t = (2 * math.pi) * (i / steps)

        x = radius * math.cos(t)
        y = radius * math.sin(t)

        # inward tilt proportional to x/y direction
        pitch = tilt_deg * (x / radius)
        roll  = -tilt_deg * (y / radius)

        routine.append({
            "x": x,
            "y": y,
            "z": z_height,
            "roll": roll,
            "pitch": pitch,
            "yaw": 0
        })

    return routine


def generate_parabola_plane_dance(
    max_xy=15,          # max +/- X or Y excursion (mm)
    z_top=5,            # top Z height
    z_drop=5,          # how far it drops at edges (mm)
    max_tilt_deg=10,    # max pitch/roll angle at edges
    steps_per_half=40   # resolution
):
    """
    Generates:
    XZ parabola sweep → YZ parabola sweep
    Repeated twice.

    Returns list of pose dictionaries.
    """

    routine = []

    def parabola_z(pos):
        return z_top - z_drop * (pos / max_xy) ** 2

    def sweep_plane(plane="xz"):
        # center → +edge → center → -edge → center
        segments = [
            (0, max_xy),
            (max_xy, 0),
            (0, -max_xy),
            (-max_xy, 0),
        ]

        for start, end in segments:
            for i in range(steps_per_half):
                t = i / (steps_per_half - 1)
                pos = start + (end - start) * t
                z = parabola_z(pos)

                if plane == "xz":
                    x = pos
                    y = 0
                    pitch = -max_tilt_deg * (pos / max_xy)
                    roll = 0

                elif plane == "yz":
                    x = 0
                    y = pos
                    roll = max_tilt_deg * (pos / max_xy)
                    pitch = 0

                routine.append({
                    "x": x,
                    "y": y,
                    "z": z,
                    "roll": roll,
                    "pitch": pitch,
                    "yaw": 0
                })

    # Entire dance twice:
    for _ in range(2):
        sweep_plane("xz")
        sweep_plane("yz")

    return routine


def generate_screw_motion(
    z_min=-12,
    z_max=8,
    yaw_min=-35,
    yaw_max=35,
    steps_per_half=80,
    cycles=2
):
    """
    Generates a screw motion:
    Up in Z while yaw rotates one way,
    Down in Z while yaw rotates back.

    Adjustable:
    - z_min, z_max
    - yaw_min, yaw_max
    - steps_per_half (resolution)
    - cycles (number of full up/down repeats)
    """

    routine = []

    def smooth_interp(a, b, t):
        # cosine easing for smooth acceleration
        t_smooth = (1 - math.cos(math.pi * t)) / 2
        return a + (b - a) * t_smooth

    for _ in range(cycles):

        # Up phase
        for i in range(steps_per_half):
            t = i / (steps_per_half - 1)

            z = smooth_interp(z_min, z_max, t)
            yaw = smooth_interp(yaw_min, yaw_max, t)

            routine.append({
                "x": 0,
                "y": 0,
                "z": z,
                "roll": 0,
                "pitch": 0,
                "yaw": yaw
            })

        # Down phase
        for i in range(steps_per_half):
            t = i / (steps_per_half - 1)

            z = smooth_interp(z_max, z_min, t)
            yaw = smooth_interp(yaw_max, yaw_min, t)

            routine.append({
                "x": 0,
                "y": 0,
                "z": z,
                "roll": 0,
                "pitch": 0,
                "yaw": yaw
            })

    return routine



ROUTINES = {
    "Cube Vertices (32x32x32)": cube_routine_smooth,
    "XY Circle (r=8)": xy_circle,
    "Cone Tracing": generate_cone_orbit_routine,
    "Parabola Dance": generate_parabola_plane_dance,
    "Screw": generate_screw_motion
}
