from typing import TYPE_CHECKING

if TYPE_CHECKING:
    pass

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from settings import SAFETY_LIMITS  # noqa: E402


def clip_servo_angles(
    angles: list[float],
) -> tuple[list[float], list[tuple[int, float, float]]]:
    """
    Clip servo angles to hardware-safe ranges.

    Rules:
    - Servos 0, 2, 4 (odd physical mount): max capped at SAFETY_LIMITS["odd_servo_max"]
    - Servos 1, 3, 5 (even physical mount): min capped at SAFETY_LIMITS["even_servo_min"]

    Returns:
        (clipped_angles, [(index, original_value, clipped_value), ...])
        The clip list is empty when no clipping occurred.

    Matches the behaviour of safety_clip_servos() in gui/gui_layout.py exactly.
    """
    safe_angles = list(angles)
    clipped: list[tuple[int, float, float]] = []

    for i, angle in enumerate(angles):
        if i in (0, 2, 4):
            limit = SAFETY_LIMITS["odd_servo_max"]
            if angle > limit:
                safe_angles[i] = float(limit)
                clipped.append((i, angle, float(limit)))
        elif i in (1, 3, 5):
            limit = SAFETY_LIMITS["even_servo_min"]
            if angle < limit:
                safe_angles[i] = float(limit)
                clipped.append((i, angle, float(limit)))

    return safe_angles, clipped
