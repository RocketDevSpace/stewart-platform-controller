"""
Integration tests: PathFollower wired into BallController.

Pins the contracts that make path following safe alongside the other
target owners: the override channel, the motion-free stop transfer, both
exclusion directions (autotune / home-cal), rest suppression while
following (lifted when an open path completes), and the I-term-rework
trim contract: the persistent trim never moves during a run, while the
PDCore integral stays active throughout — including during a stall,
where its ramp toward the standing error IS the recovery mechanism
(the old gate-based freeze/thaw choreography is gone).
"""

from control.ball_controller import BallController
from control.patterns import circle, from_points
from core.platform_state import BallState
from settings import PATH_SPEED_MM_S


class FakeClock:
    def __init__(self, start: float = 0.0) -> None:
        self.t = start

    def __call__(self) -> float:
        return self.t

    def advance(self, dt_s: float) -> None:
        self.t += dt_s


def _ball(x: float, y: float, vx: float = 0.0, vy: float = 0.0) -> BallState:
    return BallState(x_mm=x, y_mm=y, vx_mm_s=vx, vy_mm_s=vy)


def _make(clock: FakeClock) -> BallController:
    ctrl = BallController(kp=0.045, kd=0.022, clock=clock)
    ctrl.set_path(circle())
    return ctrl


class TestOverrideChannel:
    def test_follower_drives_override_not_manual(self) -> None:
        clock = FakeClock()
        ctrl = _make(clock)
        ctrl.set_target(5.0, -5.0)          # manual target
        assert ctrl.start_path()
        clock.advance(1 / 30)
        # Ball near the circle's rightmost point (65, 0)
        _, _, terms = ctrl.compute_with_terms(_ball(64.0, 1.0))
        # Active target is now a path point (near the ball), not manual.
        assert abs(terms["target_x_mm"] - 65.0) < 3.0
        assert abs(terms["target_y_mm"]) < 4.0
        # Manual target untouched underneath.
        assert ctrl._arbiter.manual == (5.0, -5.0)

    def test_target_advances_along_path(self) -> None:
        clock = FakeClock()
        ctrl = _make(clock)
        ctrl.start_path()
        clock.advance(1 / 30)
        ctrl.compute_with_terms(_ball(65.0, 0.0))       # seed
        targets = []
        for _ in range(30):                              # 1 s on-target
            clock.advance(1 / 30)
            _, _, terms = ctrl.compute_with_terms(_ball(65.0, 0.0))
            targets.append((terms["target_x_mm"], terms["target_y_mm"]))
        # Full-rate advance for most of the second: ~PATH_SPEED_MM_S of arc.
        # (Error grows as the target walks away from the parked ball, so the
        # taper engages late in the window; expect > half the full advance.)
        assert terms["path_s_mm"] > PATH_SPEED_MM_S * 0.5
        assert targets[-1] != targets[0]


class TestStopTransfer:
    def test_stop_freezes_target_in_place(self) -> None:
        clock = FakeClock()
        ctrl = _make(clock)
        ctrl.start_path()
        clock.advance(1 / 30)
        ctrl.compute_with_terms(_ball(65.0, 0.0))
        clock.advance(1 / 30)
        _, _, terms = ctrl.compute_with_terms(_ball(65.0, 0.0))
        before = (terms["target_x_mm"], terms["target_y_mm"])
        ctrl.stop_path()
        assert not ctrl.path_following_active
        # Active target bit-identical across the transition, now manual.
        assert ctrl._arbiter.active == before
        assert not ctrl._arbiter.override_active
        clock.advance(1 / 30)
        _, _, terms2 = ctrl.compute_with_terms(_ball(65.0, 0.0))
        assert (terms2["target_x_mm"], terms2["target_y_mm"]) == before


class TestExclusion:
    def test_start_path_disables_autotune_and_homecal(self) -> None:
        clock = FakeClock()
        ctrl = _make(clock)
        ctrl.set_pd_autotune(True)
        ctrl.start_home_calibration()
        ctrl.start_path()
        assert not ctrl.pd_autotune_enabled
        assert not ctrl.home_calibration_active
        assert ctrl.path_following_active

    def test_autotune_start_stops_path(self) -> None:
        clock = FakeClock()
        ctrl = _make(clock)
        ctrl.start_path()
        ctrl.set_pd_autotune(True)
        assert not ctrl.path_following_active
        assert ctrl.pd_autotune_enabled

    def test_home_calibration_stops_path(self) -> None:
        clock = FakeClock()
        ctrl = _make(clock)
        ctrl.start_path()
        ctrl.start_home_calibration()
        assert not ctrl.path_following_active
        assert ctrl.home_calibration_active

    def test_set_path_midrun_stops_cleanly(self) -> None:
        clock = FakeClock()
        ctrl = _make(clock)
        ctrl.start_path()
        clock.advance(1 / 30)
        ctrl.compute_with_terms(_ball(65.0, 0.0))
        ctrl.set_path(circle(radius_mm=50.0))
        assert not ctrl.path_following_active
        assert not ctrl._arbiter.override_active


class TestRestSuppression:
    def test_never_rests_while_following(self) -> None:
        clock = FakeClock()
        ctrl = _make(clock)
        ctrl.set_path_speed(10.0)
        ctrl.start_path()
        clock.advance(1 / 30)
        ctrl.compute_with_terms(_ball(65.0, 0.0))
        # Ball glued near the (slow) target for 3 s — prime rest conditions.
        for _ in range(90):
            clock.advance(1 / 30)
            _, _, terms = ctrl.compute_with_terms(_ball(65.0, 0.0))
        assert terms["rest_mode_active"] is False
        assert terms["rest_state"] == "active"

    def test_rest_reengages_after_open_path_done(self) -> None:
        clock = FakeClock()
        ctrl = BallController(kp=0.045, kd=0.022, clock=clock)
        # Short open segment: done quickly.
        ctrl.set_path(from_points([(0.0, 0.0), (10.0, 0.0)], closed=False))
        ctrl.set_path_speed(80.0)
        ctrl.start_path()
        clock.advance(1 / 30)
        ctrl.compute_with_terms(_ball(0.0, 0.0))
        # Ride the target to the end.
        for _ in range(60):
            clock.advance(1 / 30)
            _, _, terms = ctrl.compute_with_terms(_ball(10.0, 0.0))
        assert terms["path_state"] == "done"
        # Ball parked at the endpoint, slow: rest may now engage.
        for _ in range(60):
            clock.advance(1 / 30)
            _, _, terms = ctrl.compute_with_terms(_ball(10.0, 0.2))
        assert terms["rest_mode_active"] is True


class TestTrimAndIntegralDuringPath:
    def test_trim_store_untouched_and_integral_active_during_run(self) -> None:
        """I-term rework: the persistent trim NEVER moves during a run
        (it is a store), while the integral stays ACTIVE the whole time
        — pursuit error is inside its taper band and the rotating warp
        field is exactly what it exists to track. No freeze/thaw
        choreography remains."""
        clock = FakeClock()
        ctrl = BallController(
            kp=0.045, kd=0.022, auto_trim_enabled=True, clock=clock
        )
        ctrl.set_path(circle())
        ctrl.start_path()
        clock.advance(1 / 30)
        ctrl.compute_with_terms(_ball(65.0, 0.0))
        before = (ctrl.roll_offset, ctrl.pitch_offset)
        terms: dict = {}
        for _ in range(150):
            clock.advance(1 / 30)
            _, _, terms = ctrl.compute_with_terms(_ball(64.0, 1.0))
        assert (ctrl.roll_offset, ctrl.pitch_offset) == before
        assert terms["i_frozen"] is False
        assert terms["i_atten"] > 0.9        # small pursuit error: full ki

    def test_stall_keeps_integrating_toward_recovery(self) -> None:
        """The rig deadlock's controller-level pin: with the ball parked
        30 mm short of the frozen target (path stalled), the integral
        keeps ramping toward the standing error — the recovery mechanism
        is structural now, not a gate special-case. (The closed-loop
        recovery itself — ball pulled back in, laps resume — is pinned
        over the warp field in tests/test_path_feasibility.py.)"""
        clock = FakeClock()
        ctrl = BallController(
            kp=0.045, kd=0.022, auto_trim_enabled=True, clock=clock
        )
        ctrl.set_path(circle())
        ctrl.start_path()
        clock.advance(1 / 30)
        ctrl.compute_with_terms(_ball(35.0, 0.0))       # seed at (65, 0)
        terms: dict = {}
        for _ in range(150):
            clock.advance(1 / 30)
            _, _, terms = ctrl.compute_with_terms(_ball(35.0, 0.0))
        assert terms["path_state"] == "stalled"
        # Target still frozen at the seed point (no advance while stalled).
        assert abs(terms["target_x_mm"] - 65.0) < 3.0
        # Integral ramping toward the +x error (ball short of target).
        assert terms["i_term"][0] > 0.05
        # Persistent trim untouched.
        assert (ctrl.roll_offset, ctrl.pitch_offset) == (0.0, 0.0)


class TestTermsKeys:
    def test_path_keys_present_in_all_states(self) -> None:
        clock = FakeClock()
        ctrl = _make(clock)
        expected = {
            "path_active", "path_state", "path_name", "path_progress",
            "path_lap", "path_s_mm", "path_error_mm", "path_speed_mm_s",
        }
        _, _, terms = ctrl.compute_with_terms(_ball(0.0, 0.0))
        assert expected <= set(terms.keys())      # idle
        ctrl.start_path()
        clock.advance(1 / 30)
        _, _, terms = ctrl.compute_with_terms(_ball(65.0, 0.0))
        assert expected <= set(terms.keys())      # following
        assert terms["path_active"] is True
