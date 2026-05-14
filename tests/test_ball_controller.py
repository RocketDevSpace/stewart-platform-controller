"""
Unit tests for control/ball_controller.py — BallController.compute()

Rules under test:
- None input returns (0.0, 0.0)
- Disabled controller returns (0.0, 0.0) regardless of input
- Ball at positive x produces negative pitch (platform corrects toward center)
- Ball at positive y produces positive roll (platform corrects toward center)
- Output is clamped to [-max_tilt_deg, max_tilt_deg]
- Autotune disables auto-trim on enable
- Physics inversion produces correct kp/kd from step-response metrics
- Autotune state machine advances from wait_settle to measuring after settling
"""
import math

import pytest

from control.ball_controller import BallController, _PDLegEvaluator
from core.platform_state import BallState


def _state(x: float = 0.0, y: float = 0.0, vx: float = 0.0, vy: float = 0.0) -> BallState:
    return BallState(x_mm=x, y_mm=y, vx_mm_s=vx, vy_mm_s=vy)


class TestNoneInput:
    def test_none_returns_zero(self) -> None:
        ctrl = BallController()
        roll, pitch = ctrl.compute(None)
        assert roll == 0.0
        assert pitch == 0.0


class TestDisabled:
    def test_disabled_returns_zero(self) -> None:
        ctrl = BallController()
        ctrl.disable()
        roll, pitch = ctrl.compute(_state(x=50.0, y=50.0))
        assert roll == 0.0
        assert pitch == 0.0

    def test_re_enabled_produces_nonzero(self) -> None:
        ctrl = BallController()
        ctrl.disable()
        ctrl.enable()
        roll, pitch = ctrl.compute(_state(x=10.0))
        assert pitch != 0.0


class TestOutputDirection:
    def test_ball_positive_x_gives_negative_pitch(self) -> None:
        ctrl = BallController(kp=1.0, kd=0.0)
        _, pitch = ctrl.compute(_state(x=10.0))
        assert pitch < 0.0

    def test_ball_negative_x_gives_positive_pitch(self) -> None:
        ctrl = BallController(kp=1.0, kd=0.0)
        _, pitch = ctrl.compute(_state(x=-10.0))
        assert pitch > 0.0

    def test_ball_positive_y_gives_positive_roll(self) -> None:
        ctrl = BallController(kp=1.0, kd=0.0)
        roll, _ = ctrl.compute(_state(y=10.0))
        assert roll > 0.0

    def test_ball_negative_y_gives_negative_roll(self) -> None:
        ctrl = BallController(kp=1.0, kd=0.0)
        roll, _ = ctrl.compute(_state(y=-10.0))
        assert roll < 0.0

    def test_zero_position_zero_velocity_gives_zero(self) -> None:
        ctrl = BallController(kp=1.0, kd=1.0)
        roll, pitch = ctrl.compute(_state())
        assert roll == pytest.approx(0.0)
        assert pitch == pytest.approx(0.0)


class TestClamping:
    def test_large_x_clamps_pitch_to_max(self) -> None:
        ctrl = BallController(kp=1.0, kd=0.0, max_tilt_deg=5.0)
        _, pitch = ctrl.compute(_state(x=1000.0))
        assert pitch == pytest.approx(-5.0)

    def test_large_negative_x_clamps_pitch_to_min(self) -> None:
        ctrl = BallController(kp=1.0, kd=0.0, max_tilt_deg=5.0)
        _, pitch = ctrl.compute(_state(x=-1000.0))
        assert pitch == pytest.approx(5.0)

    def test_large_y_clamps_roll_to_max(self) -> None:
        ctrl = BallController(kp=1.0, kd=0.0, max_tilt_deg=5.0)
        roll, _ = ctrl.compute(_state(y=1000.0))
        assert roll == pytest.approx(5.0)

    def test_within_limit_not_clamped(self) -> None:
        ctrl = BallController(kp=0.01, kd=0.0, max_tilt_deg=5.0)
        _, pitch = ctrl.compute(_state(x=10.0))
        assert abs(pitch) < 5.0


class TestGains:
    def test_set_gains_updates_output(self) -> None:
        ctrl = BallController(kp=0.0, kd=0.0, max_tilt_deg=90.0)
        _, pitch_before = ctrl.compute(_state(x=10.0))
        assert pitch_before == pytest.approx(0.0)

        ctrl.set_gains(kp=1.0, kd=0.0)
        _, pitch_after = ctrl.compute(_state(x=10.0))
        assert pitch_after != pytest.approx(0.0)


class TestAutotuneDisablesAutoTrim:
    def test_autotune_enable_disables_auto_trim(self) -> None:
        ctrl = BallController(auto_trim_enabled=True)
        assert ctrl.auto_trim_enabled is True
        ctrl.set_pd_autotune(True)
        assert ctrl.auto_trim_enabled is False

    def test_autotune_disable_restores_auto_trim(self) -> None:
        ctrl = BallController(auto_trim_enabled=True)
        ctrl.set_pd_autotune(True)
        ctrl.set_pd_autotune(False)
        assert ctrl.auto_trim_enabled is True

    def test_autotune_enable_preserves_false_trim(self) -> None:
        ctrl = BallController(auto_trim_enabled=False)
        ctrl.set_pd_autotune(True)
        ctrl.set_pd_autotune(False)
        assert ctrl.auto_trim_enabled is False


class TestZetaFromOvershoot:
    def test_known_overshoot_gives_correct_zeta(self) -> None:
        # OS=0.163 → ζ≈0.5 by formula ζ = -ln(OS)/sqrt(π²+ln²(OS))
        os = 0.163
        ln_os = math.log(os)
        expected_zeta = -ln_os / math.sqrt(math.pi ** 2 + ln_os ** 2)
        assert expected_zeta == pytest.approx(0.5, abs=0.01)

    def test_large_overshoot_gives_low_zeta(self) -> None:
        os = 0.50
        ln_os = math.log(os)
        zeta = -ln_os / math.sqrt(math.pi ** 2 + ln_os ** 2)
        assert zeta < 0.3

    def test_small_overshoot_gives_high_zeta(self) -> None:
        os = 0.01
        ln_os = math.log(os)
        zeta = -ln_os / math.sqrt(math.pi ** 2 + ln_os ** 2)
        assert zeta > 0.8


class TestComputePDFromMetrics:
    def _make_ctrl(self) -> BallController:
        ctrl = BallController()
        ctrl.pd_autotune_g_eff = 171.0
        ctrl.pd_autotune_target_zeta = 0.70
        ctrl.pd_autotune_min_kp = 0.005
        ctrl.pd_autotune_max_kp = 0.250
        ctrl.pd_autotune_min_kd = 0.0
        ctrl.pd_autotune_max_kd = 0.100
        ctrl.pd_autotune_min_overshoot_ratio = 0.02
        return ctrl

    def test_underdamped_exact_inversion(self) -> None:
        ctrl = self._make_ctrl()
        # OS=0.163 → ζ_obs≈0.5; t_cross=0.877s → wd=π/0.877, wn≈4.135
        # kp=wn²/g≈0.1 (within default max_kp=0.250)
        t_cross = 0.877
        wd = math.pi / t_cross
        zeta_obs = 0.5
        wn = wd / math.sqrt(1.0 - zeta_obs ** 2)
        g = 171.0
        zeta_target = 0.70
        expected_kp = wn ** 2 / g
        expected_kd = 2.0 * zeta_target * wn / g

        metrics = {
            "overshoot_ratio": 0.163,
            "settle_time_s": 2.0,
            "first_crossing_elapsed_s": t_cross,
            "timed_out": 0.0,
        }
        kp, kd, rationale = ctrl._compute_pd_from_metrics(metrics)
        assert kp == pytest.approx(expected_kp, rel=0.02)
        assert kd == pytest.approx(expected_kd, rel=0.02)
        assert "crossing" in rationale

    def test_overdamped_fallback_uses_settle_time(self) -> None:
        ctrl = self._make_ctrl()
        settle_s = 2.0
        g = 171.0
        zeta_target = 0.70
        wn = 5.8 / settle_s
        expected_kp = wn ** 2 / g
        expected_kd = 2.0 * zeta_target * wn / g

        metrics = {
            "overshoot_ratio": 0.005,  # below min_overshoot_ratio → overdamped
            "settle_time_s": settle_s,
            "first_crossing_elapsed_s": None,
            "timed_out": 0.0,
        }
        kp, kd, rationale = ctrl._compute_pd_from_metrics(metrics)
        assert kp == pytest.approx(expected_kp, rel=0.02)
        assert kd == pytest.approx(expected_kd, rel=0.02)
        assert "fallback" in rationale

    def test_timeout_holds_current_gains(self) -> None:
        ctrl = self._make_ctrl()
        ctrl.kp = 0.045
        ctrl.kd = 0.022
        metrics = {
            "overshoot_ratio": 0.0,
            "settle_time_s": 10.0,
            "first_crossing_elapsed_s": None,
            "timed_out": 1.0,
        }
        kp, kd, rationale = ctrl._compute_pd_from_metrics(metrics)
        assert kp == pytest.approx(0.045)
        assert kd == pytest.approx(0.022)
        assert "timeout" in rationale

    def test_output_is_clamped_to_bounds(self) -> None:
        ctrl = self._make_ctrl()
        ctrl.pd_autotune_max_kp = 0.010
        ctrl.pd_autotune_max_kd = 0.005
        metrics = {
            "overshoot_ratio": 0.163,
            "settle_time_s": 0.1,  # very fast → huge wn
            "first_crossing_elapsed_s": 0.05,
            "timed_out": 0.0,
        }
        kp, kd, _ = ctrl._compute_pd_from_metrics(metrics)
        assert kp <= 0.010
        assert kd <= 0.005


class TestAutotuneStateMachine:
    def test_enable_enters_wait_settle(self) -> None:
        ctrl = BallController()
        ctrl.set_pd_autotune(True)
        # Prime the state machine with one call
        ctrl._update_pd_autotune(0.0, 0.0, 0.0, 0.0)
        assert ctrl._pd_autotune_state == "wait_settle"

    def test_wait_settle_advances_to_measuring_after_hold(self) -> None:
        from settings import PD_AUTOTUNE_WAIT_SETTLE_HOLD_S
        ctrl = BallController()
        ctrl.set_pd_autotune(True)
        ctrl._update_pd_autotune(0.0, 0.0, 0.0, 0.0)
        assert ctrl._pd_autotune_state == "wait_settle"

        # Force the timer past the threshold
        ctrl._pd_autotune_wait_settle_timer = PD_AUTOTUNE_WAIT_SETTLE_HOLD_S + 0.1
        ctrl._update_pd_autotune(0.0, 0.0, 0.0, 0.0)
        assert ctrl._pd_autotune_state == "measuring"

    def test_wait_settle_timer_resets_on_movement(self) -> None:
        ctrl = BallController()
        ctrl.set_pd_autotune(True)
        ctrl._update_pd_autotune(0.0, 0.0, 0.0, 0.0)
        ctrl._pd_autotune_wait_settle_timer = 0.5

        # Ball far from center → timer resets
        ctrl._update_pd_autotune(100.0, 100.0, 50.0, 50.0)
        assert ctrl._pd_autotune_wait_settle_timer == 0.0

    def test_leg_targets_are_axis_aligned(self) -> None:
        ctrl = BallController()
        ctrl._pd_autotune_center_x_mm = 0.0
        ctrl._pd_autotune_center_y_mm = 0.0
        targets = [ctrl._autotune_leg_target(i) for i in range(4)]
        # Each target should be on exactly one axis (one coord nonzero)
        for tx, ty in targets:
            assert (tx == 0.0) != (ty == 0.0), f"target ({tx},{ty}) not axis-aligned"

    def test_first_crossing_recorded(self) -> None:
        evaluator = _PDLegEvaluator()
        # Start with ball at x=40 (step of 40mm along x axis)
        evaluator.start(ts=0.0, x=0.0, y=0.0, tx=40.0, ty=0.0)
        assert evaluator.first_crossing_elapsed_s is None

        # Observe: ball still at positive projected error
        evaluator.observe(
            ts=0.5, x=20.0, y=0.0, vx=0.0, vy=0.0, tx=40.0, ty=0.0,
            settle_radius_mm=5.0, settle_speed_mm_s=10.0,
            settle_hold_s=0.3, min_trial_s=0.1, timeout_s=10.0,
        )
        assert evaluator.first_crossing_elapsed_s is None

        # Observe: ball overshoots past target (projected error goes negative)
        evaluator.observe(
            ts=1.0, x=50.0, y=0.0, vx=0.0, vy=0.0, tx=40.0, ty=0.0,
            settle_radius_mm=5.0, settle_speed_mm_s=10.0,
            settle_hold_s=0.3, min_trial_s=0.1, timeout_s=10.0,
        )
        assert evaluator.first_crossing_elapsed_s == pytest.approx(1.0)
