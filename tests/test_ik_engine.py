import numpy as np
import pytest

from core.ik_engine import IKEngine, solve_ik
from core.platform_state import Pose

EXPECTED_KEYS = {
    "success",
    "platform_points",
    "arm_points",
    "servo_angles_deg",
    "platform_center",
    "platform_R",
    "debug",
}

NEUTRAL = Pose(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0)


class TestIKEngineSolve:
    def test_neutral_pose_succeeds(self):
        result = IKEngine().solve(NEUTRAL)
        assert result["success"] is True

    def test_result_has_all_expected_keys(self):
        result = IKEngine().solve(NEUTRAL)
        assert EXPECTED_KEYS == set(result.keys())

    def test_servo_angles_has_six_elements(self):
        result = IKEngine().solve(NEUTRAL)
        assert len(result["servo_angles_deg"]) == 6

    def test_servo_angles_in_range(self):
        result = IKEngine().solve(NEUTRAL)
        angles = result["servo_angles_deg"]
        assert all(0 <= a <= 180 for a in angles)

    def test_same_input_returns_consistent_output(self):
        pose = Pose(x=5.0, y=-3.0, z=10.0, roll=2.0, pitch=-1.5, yaw=0.5)
        r1 = IKEngine().solve(pose)
        r2 = IKEngine().solve(pose)
        assert r1["success"] == r2["success"]
        np.testing.assert_array_equal(r1["servo_angles_deg"], r2["servo_angles_deg"])
        np.testing.assert_array_equal(r1["platform_points"], r2["platform_points"])

    def test_prev_arm_points_none_does_not_raise(self):
        result = IKEngine().solve(NEUTRAL, prev_arm_points=None)
        assert "success" in result

    def test_exception_returns_failure_dict(self):
        engine = IKEngine()
        # Pass a pose whose solve_pose will be intercepted by monkeypatching
        import kinematics.ik_solver as ik_mod
        original = ik_mod.solve_pose

        def exploding(*args, **kwargs):
            raise RuntimeError("boom")

        ik_mod.solve_pose = exploding
        try:
            result = engine.solve(NEUTRAL)
        finally:
            ik_mod.solve_pose = original

        assert result["success"] is False
        assert result["debug"]["failures"][0][0] == "exception"
        assert "boom" in result["debug"]["failures"][0][1]


class TestSolveIkConvenienceFunction:
    def test_neutral_pose_succeeds(self):
        result = solve_ik(NEUTRAL)
        assert result["success"] is True

    def test_matches_engine_solve(self):
        pose = Pose(x=2.0, y=1.0, z=5.0, roll=0.0, pitch=0.0, yaw=0.0)
        r_fn = solve_ik(pose)
        r_engine = IKEngine().solve(pose)
        assert r_fn["success"] == r_engine["success"]
        np.testing.assert_array_equal(r_fn["servo_angles_deg"], r_engine["servo_angles_deg"])
