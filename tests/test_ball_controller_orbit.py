"""
BallController <-> HarmonicOrbit integration (mirrors
test_ball_controller_paths.py): override-channel discipline, the
motion-free stop transfer, all pairwise mode exclusions, rest
suppression, the scaled-feedback pass-through, the TRACK integral
freeze, and orbit terms presence in idle and active states.
"""
import math

import pytest

from control.ball_controller import BallController
from control.orbit import STATE_ENTRAIN, STATE_TRACK
from control.patterns import circle
from core.platform_state import BallState


class FakeClock:
    def __init__(self, start: float = 100.0) -> None:
        self.t = start

    def __call__(self) -> float:
        return self.t

    def advance(self, dt_s: float) -> None:
        self.t += dt_s


def _ball(x: float, y: float, vx: float = 0.0, vy: float = 0.0) -> BallState:
    return BallState(x_mm=x, y_mm=y, vx_mm_s=vx, vy_mm_s=vy)


def _make(clock: FakeClock) -> BallController:
    # Hermetic reference gains (the overlay-independence rule).
    ctrl = BallController(kp=0.045, kd=0.022, ki=0.030, clock=clock)
    return ctrl


class TestOverrideChannel:
    def test_orbit_drives_override_not_manual(self) -> None:
        clock = FakeClock()
        ctrl = _make(clock)
        ctrl.set_target(3.0, 4.0)
        assert ctrl.start_orbit()
        clock.advance(1 / 30)
        ctrl.compute_with_terms(_ball(30.0, 10.0))
        assert ctrl._arbiter.override_active
        assert ctrl._arbiter.manual == (3.0, 4.0)     # untouched

    def test_seed_starts_at_ball_no_jump(self) -> None:
        clock = FakeClock()
        ctrl = _make(clock)
        ctrl.start_orbit()
        clock.advance(1 / 30)
        _, _, terms = ctrl.compute_with_terms(_ball(25.0, 25.0))
        tx, ty = terms["target_x_mm"], terms["target_y_mm"]
        assert tx == pytest.approx(25.0, abs=1e-6)
        assert ty == pytest.approx(25.0, abs=1e-6)
        assert terms["orbit_state"] == STATE_ENTRAIN


class TestStopTransfer:
    def test_stop_orbit_freezes_target_in_place(self) -> None:
        clock = FakeClock()
        ctrl = _make(clock)
        ctrl.start_orbit()
        for _ in range(30):
            clock.advance(1 / 30)
            ctrl.compute_with_terms(_ball(40.0, 5.0))
        active_before = ctrl._arbiter.active
        ctrl.stop_orbit()
        assert ctrl._arbiter.active == active_before  # bit-identical
        assert not ctrl._arbiter.override_active
        assert not ctrl.orbit_active

    def test_stop_when_idle_is_noop(self) -> None:
        clock = FakeClock()
        ctrl = _make(clock)
        ctrl.set_target(7.0, -2.0)
        ctrl.stop_orbit()                             # must not clobber manual
        assert ctrl._arbiter.manual == (7.0, -2.0)
        assert not ctrl._arbiter.override_active


class TestExclusion:
    def test_start_orbit_stops_path(self) -> None:
        clock = FakeClock()
        ctrl = _make(clock)
        ctrl.set_path(circle())
        assert ctrl.start_path()
        assert ctrl.path_following_active
        ctrl.start_orbit()
        assert not ctrl.path_following_active
        assert ctrl.orbit_active

    def test_start_path_stops_orbit(self) -> None:
        clock = FakeClock()
        ctrl = _make(clock)
        ctrl.start_orbit()
        ctrl.set_path(circle())
        assert ctrl.start_path()
        assert not ctrl.orbit_active
        assert ctrl.path_following_active

    def test_autotune_stops_orbit(self) -> None:
        clock = FakeClock()
        ctrl = _make(clock)
        ctrl.start_orbit()
        ctrl.set_pd_autotune(True)
        assert not ctrl.orbit_active
        assert ctrl.pd_autotune_enabled

    def test_start_orbit_stops_autotune(self) -> None:
        clock = FakeClock()
        ctrl = _make(clock)
        ctrl.set_pd_autotune(True)
        ctrl.start_orbit()
        assert not ctrl.pd_autotune_enabled
        assert ctrl.orbit_active

    def test_home_cal_stops_orbit(self) -> None:
        clock = FakeClock()
        ctrl = _make(clock)
        ctrl.start_orbit()
        ctrl.start_home_calibration()
        assert not ctrl.orbit_active
        assert ctrl.home_calibration_active

    def test_start_orbit_cancels_home_cal(self) -> None:
        clock = FakeClock()
        ctrl = _make(clock)
        ctrl.start_home_calibration()
        ctrl.start_orbit()
        assert not ctrl.home_calibration_active
        assert ctrl.orbit_active


class TestRestSuppression:
    def test_rest_suppressed_while_orbiting(self) -> None:
        clock = FakeClock()
        ctrl = _make(clock)
        ctrl.start_orbit()
        # Ball parked dead-on the (slow-moving) reference for a while:
        # without suppression this satisfies every rest gate.
        _, _, terms = ctrl.compute_with_terms(_ball(30.0, 0.0))
        for _ in range(90):
            clock.advance(1 / 30)
            tx, ty = terms["target_x_mm"], terms["target_y_mm"]
            _, _, terms = ctrl.compute_with_terms(_ball(tx, ty))
        assert terms["rest_mode_active"] is False
        assert terms["rest_state"] == "active"


class TestScaledFeedbackAndFreeze:
    def test_p_term_reflects_orbit_gain_scale(self) -> None:
        clock = FakeClock()
        ctrl = _make(clock)
        ctrl.start_orbit()
        clock.advance(1 / 30)
        ctrl.compute_with_terms(_ball(30.0, 0.0))      # seed
        clock.advance(1 / 30)
        _, _, terms = ctrl.compute_with_terms(_ball(30.0, 10.0))
        ex, ey = terms["position_vec_mm"]
        assert terms["p_term"][0] == pytest.approx(0.045 * 0.5 * ex)
        assert terms["p_term"][1] == pytest.approx(0.045 * 0.5 * ey)

    def test_integral_runs_in_entrain_freezes_in_track(self) -> None:
        clock = FakeClock()
        ctrl = _make(clock)
        ctrl.start_orbit()
        clock.advance(1 / 30)
        _, _, terms = ctrl.compute_with_terms(_ball(30.0, 0.0))
        assert terms["orbit_state"] == STATE_ENTRAIN
        assert terms["i_frozen"] is False              # DC learning allowed
        # Ride the reference until TRACK.
        for _ in range(240):
            clock.advance(1 / 30)
            tx, ty = terms["target_x_mm"], terms["target_y_mm"]
            _, _, terms = ctrl.compute_with_terms(_ball(tx, ty))
        assert terms["orbit_state"] == STATE_TRACK
        assert terms["i_frozen"] is True               # table owns periodic


class TestTermsKeys:
    ORBIT_KEYS = {
        "orbit_active", "orbit_state", "orbit_phase", "orbit_omega",
        "orbit_r_mm", "orbit_err_mm", "orbit_ff_deg", "orbit_ilc_deg",
        "orbit_lap", "orbit_recover_count", "orbit_learning",
    }

    def test_orbit_keys_present_idle_and_active(self) -> None:
        clock = FakeClock()
        ctrl = _make(clock)
        _, _, idle_terms = ctrl.compute_with_terms(_ball(0.0, 0.0))
        assert self.ORBIT_KEYS <= set(idle_terms.keys())
        assert idle_terms["orbit_active"] is False
        ctrl.start_orbit()
        clock.advance(1 / 30)
        _, _, terms = ctrl.compute_with_terms(_ball(30.0, 0.0))
        assert self.ORBIT_KEYS <= set(terms.keys())
        assert terms["orbit_active"] is True
        assert terms["orbit_r_mm"] == pytest.approx(30.0, abs=1e-6)

    def test_orbit_ff_feeds_ff_vec(self) -> None:
        # Once spun up, the PID's reported ff_vec is the orbit's tilt
        # (centripetal magnitude at least).
        clock = FakeClock()
        ctrl = _make(clock)
        ctrl.start_orbit()
        clock.advance(1 / 30)
        _, _, terms = ctrl.compute_with_terms(_ball(50.0, 0.0))
        for _ in range(240):
            clock.advance(1 / 30)
            tx, ty = terms["target_x_mm"], terms["target_y_mm"]
            _, _, terms = ctrl.compute_with_terms(_ball(tx, ty))
        assert terms["orbit_state"] == STATE_TRACK
        ff = terms["ff_vec"]
        assert math.hypot(*ff) == pytest.approx(terms["orbit_ff_deg"], abs=1e-9)
        assert terms["orbit_ff_deg"] > 0.05            # centripetal present
