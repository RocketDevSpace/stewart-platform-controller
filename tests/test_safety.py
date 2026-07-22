"""
Unit tests for core/safety.py — clip_servo_angles() + select_speed_delay()

Rules under test:
- Every servo: global clamp into [min_angle, max_angle] = [0, 180]
- Indices 0, 2, 4: max additionally capped at 170 (odd_servo_max)
- Indices 1, 3, 5: min additionally floored at 10 (even_servo_min)
- Exactly 6 finite angles or ValueError
- Large-move ramp policy (select_speed_delay)
"""
# sys.path is set by repo root conftest.py
import pytest

from core.safety import clip_servo_angles, select_speed_delay
from settings import (
    SERVO_LARGE_MOVE_SPEED_DELAY_MS,
    SERVO_SLEW_INSTANT_MAX_DEG,
)


def _all_within_bounds() -> list[float]:
    return [170.0, 10.0, 170.0, 10.0, 170.0, 10.0]


class TestNoClipping:
    def test_all_within_bounds_returns_unchanged(self) -> None:
        angles = [90.0, 90.0, 90.0, 90.0, 90.0, 90.0]
        result, clips = clip_servo_angles(angles)
        assert result == angles
        assert clips == []

    def test_returns_a_copy_not_the_same_list(self) -> None:
        angles = [90.0, 90.0, 90.0, 90.0, 90.0, 90.0]
        result, _ = clip_servo_angles(angles)
        assert result is not angles

    def test_boundary_exactly_170_not_clipped(self) -> None:
        angles = [170.0, 90.0, 90.0, 90.0, 90.0, 90.0]
        result, clips = clip_servo_angles(angles)
        assert result[0] == 170.0
        assert clips == []

    def test_boundary_exactly_10_not_clipped(self) -> None:
        angles = [90.0, 10.0, 90.0, 90.0, 90.0, 90.0]
        result, clips = clip_servo_angles(angles)
        assert result[1] == 10.0
        assert clips == []

    def test_all_boundary_values_not_clipped(self) -> None:
        angles = _all_within_bounds()
        result, clips = clip_servo_angles(angles)
        assert result == angles
        assert clips == []


class TestOddServoMaxClip:
    def test_servo_0_above_max(self) -> None:
        angles = [180.0, 90.0, 90.0, 90.0, 90.0, 90.0]
        result, clips = clip_servo_angles(angles)
        assert result[0] == 170.0
        assert (0, 180.0, 170.0) in clips

    def test_servo_2_above_max(self) -> None:
        angles = [90.0, 90.0, 175.0, 90.0, 90.0, 90.0]
        result, clips = clip_servo_angles(angles)
        assert result[2] == 170.0
        assert (2, 175.0, 170.0) in clips

    def test_servo_4_above_max(self) -> None:
        angles = [90.0, 90.0, 90.0, 90.0, 200.0, 90.0]
        result, clips = clip_servo_angles(angles)
        assert result[4] == 170.0
        assert (4, 200.0, 170.0) in clips

    def test_odd_servo_below_max_not_clipped(self) -> None:
        angles = [0.0, 90.0, 0.0, 90.0, 0.0, 90.0]
        result, clips = clip_servo_angles(angles)
        assert result[0] == 0.0
        assert result[2] == 0.0
        assert result[4] == 0.0
        assert clips == []


class TestEvenServoMinClip:
    def test_servo_1_below_min(self) -> None:
        angles = [90.0, 5.0, 90.0, 90.0, 90.0, 90.0]
        result, clips = clip_servo_angles(angles)
        assert result[1] == 10.0
        assert (1, 5.0, 10.0) in clips

    def test_servo_3_below_min(self) -> None:
        angles = [90.0, 90.0, 90.0, 0.0, 90.0, 90.0]
        result, clips = clip_servo_angles(angles)
        assert result[3] == 10.0
        assert (3, 0.0, 10.0) in clips

    def test_servo_5_below_min(self) -> None:
        angles = [90.0, 90.0, 90.0, 90.0, 90.0, -5.0]
        result, clips = clip_servo_angles(angles)
        assert result[5] == 10.0
        assert (5, -5.0, 10.0) in clips

    def test_even_servo_above_min_not_clipped(self) -> None:
        angles = [90.0, 180.0, 90.0, 180.0, 90.0, 180.0]
        result, clips = clip_servo_angles(angles)
        assert result[1] == 180.0
        assert result[3] == 180.0
        assert result[5] == 180.0
        assert clips == []


class TestMultipleClips:
    def test_all_six_servos_need_clipping(self) -> None:
        angles = [180.0, 0.0, 180.0, 0.0, 180.0, 0.0]
        result, clips = clip_servo_angles(angles)
        assert result == [170.0, 10.0, 170.0, 10.0, 170.0, 10.0]
        assert len(clips) == 6

    def test_two_clips_reported_in_index_order(self) -> None:
        angles = [180.0, 5.0, 90.0, 90.0, 90.0, 90.0]
        result, clips = clip_servo_angles(angles)
        assert result[0] == 170.0
        assert result[1] == 10.0
        assert clips[0][0] == 0
        assert clips[1][0] == 1

    def test_clip_info_tuple_structure(self) -> None:
        angles = [171.0, 90.0, 90.0, 90.0, 90.0, 90.0]
        _, clips = clip_servo_angles(angles)
        assert len(clips) == 1
        idx, original, clipped = clips[0]
        assert idx == 0
        assert original == 171.0
        assert clipped == 170.0

    def test_unaffected_servos_unchanged_during_multi_clip(self) -> None:
        angles = [180.0, 0.0, 90.0, 90.0, 90.0, 90.0]
        result, _ = clip_servo_angles(angles)
        assert result[2] == 90.0
        assert result[3] == 90.0
        assert result[4] == 90.0
        assert result[5] == 90.0


class TestGlobalRange:
    """The 0-180 rail must exist in Python, not only in the firmware."""

    def test_negative_angle_on_even_index_clamped_to_zero(self) -> None:
        angles = [-50.0, 90.0, 90.0, 90.0, 90.0, 90.0]
        result, clips = clip_servo_angles(angles)
        assert result[0] == 0.0
        assert (0, -50.0, 0.0) in clips

    def test_above_180_on_odd_index_clamped(self) -> None:
        # index 1 has no odd_servo_max cap; global max_angle=180 must bind
        angles = [90.0, 300.0, 90.0, 90.0, 90.0, 90.0]
        result, clips = clip_servo_angles(angles)
        assert result[1] == 180.0
        assert (1, 300.0, 180.0) in clips

    def test_negative_on_odd_index_hits_even_servo_min(self) -> None:
        angles = [90.0, -20.0, 90.0, 90.0, 90.0, 90.0]
        result, _ = clip_servo_angles(angles)
        assert result[1] == 10.0


class TestInputValidation:
    def test_wrong_length_short_raises(self) -> None:
        with pytest.raises(ValueError):
            clip_servo_angles([90.0] * 5)

    def test_wrong_length_long_raises(self) -> None:
        with pytest.raises(ValueError):
            clip_servo_angles([90.0] * 7)

    def test_nan_raises(self) -> None:
        with pytest.raises(ValueError):
            clip_servo_angles([float("nan"), 90.0, 90.0, 90.0, 90.0, 90.0])

    def test_inf_raises(self) -> None:
        with pytest.raises(ValueError):
            clip_servo_angles([90.0, float("inf"), 90.0, 90.0, 90.0, 90.0])


class TestSelectSpeedDelay:
    def test_small_move_is_instant(self) -> None:
        prev = [90.0] * 6
        target = [90.0 + SERVO_SLEW_INSTANT_MAX_DEG, 90.0, 90.0, 90.0, 90.0, 90.0]
        assert select_speed_delay(prev, target) == 0

    def test_large_move_gets_ramp_delay(self) -> None:
        prev = [90.0] * 6
        target = [90.0 + SERVO_SLEW_INSTANT_MAX_DEG + 1.0,
                  90.0, 90.0, 90.0, 90.0, 90.0]
        assert select_speed_delay(prev, target) == SERVO_LARGE_MOVE_SPEED_DELAY_MS

    def test_largest_delta_governs(self) -> None:
        prev = [90.0] * 6
        target = [91.0, 92.0, 90.0, 90.0, 90.0, 20.0]  # servo 5 jumps 70
        assert select_speed_delay(prev, target) == SERVO_LARGE_MOVE_SPEED_DELAY_MS

    def test_unknown_prev_is_conservative(self) -> None:
        assert (
            select_speed_delay(None, [90.0] * 6)
            == SERVO_LARGE_MOVE_SPEED_DELAY_MS
        )
