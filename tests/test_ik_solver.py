"""
Kinematics regression tests for kinematics/ik_solver.py (via IKEngine).

These are the tests that would have caught both the PR #15 geometry
transposition (golden values) and the M5/M9 yaw-snap (sweep continuity):
- golden servo angles at neutral
- rod-length invariant |arm_point - platform_point| == ROD_LENGTH
- unreachable poses fail rather than fabricate
- branch seeding at the screw routine's extreme first pose
- per-step servo-angle continuity across every routine
"""

import numpy as np
import pytest

from config import ROD_LENGTH
from core.ik_engine import IKEngine
from core.platform_state import IKResult, Pose
from routines.routines import ROUTINES

ENGINE = IKEngine()
NEUTRAL = Pose()

# Computed from the validated geometry (config.py, post-PR #15) at neutral.
GOLDEN_NEUTRAL_ANGLES = [88.412, 91.588, 88.412, 91.588, 88.412, 91.588]


def _solve(pose: Pose, prev: np.ndarray | None = None) -> "IKResult":
    return ENGINE.solve(pose, prev)


class TestGoldenValues:
    def test_neutral_angles(self) -> None:
        res = _solve(NEUTRAL)
        assert res.success
        np.testing.assert_allclose(
            res.servo_angles_deg, GOLDEN_NEUTRAL_ANGLES, atol=0.01
        )

    def test_neutral_symmetry(self) -> None:
        """Mirror-pair symmetry: even and odd indices mirror around 90."""
        res = _solve(NEUTRAL)
        a = res.servo_angles_deg
        for even_i, odd_i in ((0, 1), (2, 3), (4, 5)):
            assert a[even_i] + a[odd_i] == pytest.approx(180.0, abs=0.01)


class TestGeometricInvariants:
    @pytest.mark.parametrize(
        "pose",
        [
            Pose(),
            Pose(z=10.0),
            Pose(x=8.0, y=-5.0, z=-6.0),
            Pose(roll=4.0, pitch=-3.0),
            Pose(z=-12.0, yaw=-35.0),  # screw start
        ],
    )
    def test_rod_length_invariant(self, pose: Pose) -> None:
        res = _solve(pose)
        assert res.success, res.servo_status
        rods = np.linalg.norm(
            np.asarray(res.arm_points) - np.asarray(res.platform_points), axis=1
        )
        np.testing.assert_allclose(rods, ROD_LENGTH, atol=1e-6)

    def test_unreachable_pose_fails_honestly(self) -> None:
        res = _solve(Pose(z=200.0))
        assert res.success is False
        assert any(s is not None for s in res.servo_status)

    def test_failure_result_shape_is_complete(self) -> None:
        res = _solve(Pose(z=200.0))
        # No KeyError/AttributeError landmines on the failure path.
        assert len(res.servo_angles_deg) == 6
        assert np.asarray(res.platform_points).shape == (6, 3)
        assert np.asarray(res.arm_points).shape == (6, 3)
        assert res.platform_R.shape == (3, 3)


class TestBranchSelection:
    def test_screw_start_pose_seeds_in_range(self) -> None:
        """The old seed (ball-joint height) picked a ~260-deg branch here and
        silently clamped it to 180; the redesigned seed must produce genuine
        in-range angles with headroom from the rails."""
        res = _solve(Pose(z=-12.0, yaw=-35.0))
        assert res.success, res.servo_status
        for a in res.servo_angles_deg:
            assert 5.0 < a < 175.0

    def test_continuity_follows_prev_branch(self) -> None:
        first = _solve(Pose(z=-12.0, yaw=-35.0))
        second = _solve(
            Pose(z=-11.8, yaw=-34.5), np.asarray(first.arm_points)
        )
        assert second.success
        deltas = np.abs(
            np.array(second.servo_angles_deg) - np.array(first.servo_angles_deg)
        )
        assert float(np.max(deltas)) < 3.0


class TestRoutineSweepContinuity:
    """The M9 regression test: threading continuity through every routine,
    no adjacent-step servo command may jump more than the bound. The old
    solver produced a commanded 180-deg snap on the screw routine."""

    # Cube has real vertex kinks (~14 deg); cone exits the workspace on ~20
    # steps by design (ratio ~ -1.005 — pre-existing, now visible) which
    # inflates the step measured across the gap.
    BOUNDS = {
        "Cube Vertices (28x28x28)": 20.0,
        "XY Circle (r=20)": 6.0,
        "Cone Tracing (r=8, tilt 15)": 25.0,
        "Parabola Dance": 6.0,
        "Screw (z -12..8, yaw +/-35)": 6.0,
    }

    @pytest.mark.parametrize("name", list(ROUTINES.keys()))
    def test_sweep_continuity(self, name: str) -> None:
        steps = ROUTINES[name]()
        prev_points: np.ndarray | None = None
        prev_angles: np.ndarray | None = None
        max_delta = 0.0
        for p in steps:
            res = _solve(Pose(**{k: float(v) for k, v in p.items()}), prev_points)
            if not res.success:
                continue  # runner skips these; continuity measured across gap
            prev_points = np.asarray(res.arm_points)
            angles = np.array(res.servo_angles_deg)
            if prev_angles is not None:
                max_delta = max(
                    max_delta, float(np.max(np.abs(angles - prev_angles)))
                )
            prev_angles = angles
        assert max_delta < self.BOUNDS[name], (
            f"{name}: max per-step servo delta {max_delta:.2f} deg"
        )

    def test_screw_routine_solves_every_step(self) -> None:
        steps = ROUTINES["Screw (z -12..8, yaw +/-35)"]()
        prev: np.ndarray | None = None
        for i, p in enumerate(steps):
            res = _solve(Pose(**{k: float(v) for k, v in p.items()}), prev)
            assert res.success, f"step {i}: {res.servo_status}"
            prev = np.asarray(res.arm_points)

    def test_cone_failure_count_pinned(self) -> None:
        """Cone tracing exits the workspace on exactly these steps today —
        pin the count so a geometry/solver change that widens or narrows
        the reachable envelope is noticed."""
        steps = ROUTINES["Cone Tracing (r=8, tilt 15)"]()
        prev: np.ndarray | None = None
        fails = 0
        for p in steps:
            res = _solve(Pose(**{k: float(v) for k, v in p.items()}), prev)
            if res.success:
                prev = np.asarray(res.arm_points)
            else:
                fails += 1
        assert fails == 20
