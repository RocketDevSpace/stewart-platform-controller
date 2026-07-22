from settings import SAFETY_LIMITS


def clip_servo_angles(
    angles: list[float],
) -> tuple[list[float], list[tuple[int, float, float]]]:
    """
    Clip servo angles to hardware-safe ranges.

    Rules (all indices 0-based):
    - Indices 0, 2, 4 (mirror-mounted one way): max capped at SAFETY_LIMITS["odd_servo_max"]
    - Indices 1, 3, 5 (mirror-mounted the other way): min capped at SAFETY_LIMITS["even_servo_min"]

    Returns:
        (clipped_angles, [(index, original_value, clipped_value), ...])
        The clip list is empty when no clipping occurred.
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
