# -*- coding: utf-8 -*-
"""
Created on Mon Feb 16 13:21:27 2026

@author: hudso
"""

from typing import List

# Safe servo limits (you can adjust based on your servos)
SERVO_MIN = 0
SERVO_MAX = 180

def validate_angles(angles: List[float]) -> bool:
    """
    Validate that each servo angle is within safe bounds.
    Returns True if all angles are safe.
    """
    for i, a in enumerate(angles):
        if a < SERVO_MIN or a > SERVO_MAX:
            print(f"[VALIDATION ERROR] Servo {i+1} angle {a} out of bounds!")
            return False
    return True

def format_command(angles):
    """
    angles: iterable of 6 servo angles (degrees)
    Returns Arduino serial command string.
    """
    if len(angles) != 6:
        raise ValueError("Expected 6 servo angles.")

    angles_int = [int(round(a)) for a in angles]

    for a in angles_int:
        if a < 0 or a > 180:
            raise ValueError(f"Servo angle out of range: {a}")

    return "S," + ",".join(map(str, angles_int)) + "\n"

