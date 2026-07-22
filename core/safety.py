"""
core/safety.py

Single home for servo-angle safety policy:

- clip_servo_angles(): the ONLY angle clamp in the codebase. Global
  min/max rails for every servo plus the per-mount asymmetric limits.
- select_speed_delay(): large-move ramp policy. The Arduino firmware ramps
  all servos concurrently at 1 degree per `speedDelay` ms (clamped 0-20 by
  the firmware; see firmware/README.md). Commands whose largest per-servo
  jump exceeds SERVO_SLEW_INSTANT_MAX_DEG are sent with a non-zero
  speedDelay so the RAMP HAPPENS IN HARDWARE instead of the platform
  snapping. Note the firmware acks only after the ramp completes, so a
  ramped command occupies the firmware command loop for
  (max_delta * speedDelay) ms — fine for one-shot manual sends, which is
  the only path that should ever produce large jumps.
"""

import math

from settings import (
    SAFETY_LIMITS,
    SERVO_LARGE_MOVE_SPEED_DELAY_MS,
    SERVO_SLEW_INSTANT_MAX_DEG,
)

NUM_SERVOS = 6


def clip_servo_angles(
    angles: list[float],
) -> tuple[list[float], list[tuple[int, float, float]]]:
    """
    Clip servo angles to hardware-safe ranges.

    Rules (all indices 0-based):
    - Every servo: clamped into [SAFETY_LIMITS["min_angle"], SAFETY_LIMITS["max_angle"]].
    - Indices 0, 2, 4 (mirror-mounted one way): additionally capped at
      SAFETY_LIMITS["odd_servo_max"].
    - Indices 1, 3, 5 (mirror-mounted the other way): additionally floored at
      SAFETY_LIMITS["even_servo_min"].

    Raises ValueError unless exactly NUM_SERVOS angles are given — a wrong
    length means a caller bug and must fail loudly, not reach hardware.

    Returns:
        (clipped_angles, [(index, original_value, clipped_value), ...])
        The clip list is empty when no clipping occurred.
    """
    if len(angles) != NUM_SERVOS:
        raise ValueError(
            f"expected {NUM_SERVOS} servo angles, got {len(angles)}"
        )
    if not all(math.isfinite(a) for a in angles):
        raise ValueError(f"non-finite servo angle in {angles!r}")

    global_min = float(SAFETY_LIMITS["min_angle"])
    global_max = float(SAFETY_LIMITS["max_angle"])

    safe_angles = list(angles)
    clipped: list[tuple[int, float, float]] = []

    for i, angle in enumerate(angles):
        low = global_min
        high = global_max
        if i in (0, 2, 4):
            high = min(high, float(SAFETY_LIMITS["odd_servo_max"]))
        else:
            low = max(low, float(SAFETY_LIMITS["even_servo_min"]))

        new_angle = min(max(float(angle), low), high)
        if new_angle != angle:
            safe_angles[i] = new_angle
            clipped.append((i, float(angle), new_angle))

    return safe_angles, clipped


def select_speed_delay(
    prev_angles: list[float] | None,
    target_angles: list[float],
) -> int:
    """Choose the firmware speedDelay for a send, given the last sent angles.

    Returns 0 (instant write) when every per-servo jump is within
    SERVO_SLEW_INSTANT_MAX_DEG, else SERVO_LARGE_MOVE_SPEED_DELAY_MS so the
    firmware ramps the move. With prev_angles None (nothing sent yet) the
    previous pose is unknown; the firmware boots at 90 deg on all servos, so
    callers should pass [90.0] * 6 for the first send rather than None —
    None is treated conservatively as "ramp".
    """
    if prev_angles is None:
        return int(SERVO_LARGE_MOVE_SPEED_DELAY_MS)
    max_delta = max(
        abs(t - p) for t, p in zip(target_angles, prev_angles)
    )
    if max_delta <= SERVO_SLEW_INSTANT_MAX_DEG:
        return 0
    return int(SERVO_LARGE_MOVE_SPEED_DELAY_MS)
