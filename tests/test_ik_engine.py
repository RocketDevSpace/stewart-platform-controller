from typing import Any

import numpy as np

from core.ik_engine import IKEngine
from core.platform_state import IKResult, Pose

NEUTRAL = Pose(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0)


class TestIKEngineSolve:
    def test_neutral_pose_succeeds(self) -> None:
        result = IKEngine().solve(NEUTRAL)
        assert result.success is True

    def test_result_is_typed_contract(self) -> None:
        result = IKEngine().solve(NEUTRAL)
        assert isinstance(result, IKResult)

    def test_servo_angles_has_six_elements(self) -> None:
        result = IKEngine().solve(NEUTRAL)
        assert len(result.servo_angles_deg) == 6

    def test_servo_status_all_none_on_success(self) -> None:
        result = IKEngine().solve(NEUTRAL)
        assert result.servo_status == (None,) * 6

    def test_servo_angles_in_range(self) -> None:
        result = IKEngine().solve(NEUTRAL)
        assert all(0 <= a <= 180 for a in result.servo_angles_deg)

    def test_same_input_returns_consistent_output(self) -> None:
        pose = Pose(x=5.0, y=-3.0, z=10.0, roll=2.0, pitch=-1.5, yaw=0.5)
        r1 = IKEngine().solve(pose)
        r2 = IKEngine().solve(pose)
        assert r1.success == r2.success
        np.testing.assert_array_equal(r1.servo_angles_deg, r2.servo_angles_deg)
        np.testing.assert_array_equal(r1.platform_points, r2.platform_points)

    def test_prev_arm_points_none_does_not_raise(self) -> None:
        result = IKEngine().solve(NEUTRAL, prev_arm_points=None)
        assert isinstance(result.success, bool)

    def test_exception_returns_complete_failure_result(self) -> None:
        engine = IKEngine()
        import kinematics.ik_solver as ik_mod
        original = ik_mod.solve_pose

        def exploding(*args: Any, **kwargs: Any) -> None:
            raise RuntimeError("boom")

        ik_mod.solve_pose = exploding  # type: ignore[assignment]
        try:
            result = engine.solve(NEUTRAL)
        finally:
            ik_mod.solve_pose = original

        assert result.success is False
        assert all(s is not None and "boom" in s for s in result.servo_status)
        # The failure shape is COMPLETE — consumers can touch every field.
        assert len(result.servo_angles_deg) == 6
        assert np.asarray(result.platform_points).shape == (6, 3)
        assert np.asarray(result.arm_points).shape == (6, 3)
        assert result.platform_R.shape == (3, 3)
        assert result.debug["failures"][0][0] == "exception"

    def test_failure_placeholder_angles_are_neutral_not_zero(self) -> None:
        """0 deg is a valid extreme command — failure slots must hold the
        neutral placeholder instead."""
        result = IKResult.failed("test")
        assert all(a == 90.0 for a in result.servo_angles_deg)
