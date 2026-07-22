"""
Unit tests for the decomposed ball controller (M10):
BallController facade (compute_with_terms) + PDAutotuner.

Rules under test:
- None input returns (0.0, 0.0)
- Disabled controller returns (0.0, 0.0) regardless of input
- Ball at positive x produces negative pitch (platform corrects toward center)
- Ball at positive y produces positive roll (platform corrects toward center)
- Output is clamped to [-max_tilt_deg, max_tilt_deg]
- Autotune disables auto-trim on enable
- Physics inversion produces correct kp/kd from step-response metrics
- Autotune state machine advances from wait_settle to measuring after settling
- Autotune session log goes to the injected path, opened fresh per session
"""
import math
from pathlib import Path

import pytest

from control.autotune import PDAutotuner, _PDLegEvaluator
from control.ball_controller import BallController
from control.setpoint import SetpointArbiter
from core.platform_state import BallState


class _FakeClock:
    def __init__(self, start: float = 0.0) -> None:
        self.now = float(start)

    def advance(self, dt_s: float) -> None:
        self.now += float(dt_s)

    def __call__(self) -> float:
        return self.now


def _state(x: float = 0.0, y: float = 0.0, vx: float = 0.0, vy: float = 0.0) -> BallState:
    return BallState(x_mm=x, y_mm=y, vx_mm_s=vx, vy_mm_s=vy)


def _make_tuner(log_path: str, clock: _FakeClock | None = None) -> PDAutotuner:
    clk = clock if clock is not None else _FakeClock()
    arbiter = SetpointArbiter(0.0, 0.0, clk)
    return PDAutotuner(arbiter, clk, log_path=log_path)


class TestNoneInput:
    def test_none_returns_zero(self) -> None:
        ctrl = BallController()
        roll, pitch, _ = ctrl.compute_with_terms(None)
        assert roll == 0.0
        assert pitch == 0.0


class TestDisabled:
    def test_disabled_returns_zero(self) -> None:
        ctrl = BallController()
        ctrl.disable()
        roll, pitch, _ = ctrl.compute_with_terms(_state(x=50.0, y=50.0))
        assert roll == 0.0
        assert pitch == 0.0

    def test_re_enabled_produces_nonzero(self) -> None:
        ctrl = BallController()
        ctrl.disable()
        ctrl.enable()
        roll, pitch, _ = ctrl.compute_with_terms(_state(x=10.0))
        assert pitch != 0.0


class TestOutputDirection:
    def test_ball_positive_x_gives_negative_pitch(self) -> None:
        ctrl = BallController(kp=1.0, kd=0.0)
        _, pitch, _ = ctrl.compute_with_terms(_state(x=10.0))
        assert pitch < 0.0

    def test_ball_negative_x_gives_positive_pitch(self) -> None:
        ctrl = BallController(kp=1.0, kd=0.0)
        _, pitch, _ = ctrl.compute_with_terms(_state(x=-10.0))
        assert pitch > 0.0

    def test_ball_positive_y_gives_positive_roll(self) -> None:
        ctrl = BallController(kp=1.0, kd=0.0)
        roll, _, _ = ctrl.compute_with_terms(_state(y=10.0))
        assert roll > 0.0

    def test_ball_negative_y_gives_negative_roll(self) -> None:
        ctrl = BallController(kp=1.0, kd=0.0)
        roll, _, _ = ctrl.compute_with_terms(_state(y=-10.0))
        assert roll < 0.0

    def test_zero_position_zero_velocity_gives_zero(self) -> None:
        ctrl = BallController(kp=1.0, kd=1.0)
        roll, pitch, _ = ctrl.compute_with_terms(_state())
        assert roll == pytest.approx(0.0)
        assert pitch == pytest.approx(0.0)


class TestClamping:
    def test_large_x_clamps_pitch_to_max(self) -> None:
        ctrl = BallController(kp=1.0, kd=0.0, max_tilt_deg=5.0)
        _, pitch, _ = ctrl.compute_with_terms(_state(x=1000.0))
        assert pitch == pytest.approx(-5.0)

    def test_large_negative_x_clamps_pitch_to_min(self) -> None:
        ctrl = BallController(kp=1.0, kd=0.0, max_tilt_deg=5.0)
        _, pitch, _ = ctrl.compute_with_terms(_state(x=-1000.0))
        assert pitch == pytest.approx(5.0)

    def test_large_y_clamps_roll_to_max(self) -> None:
        ctrl = BallController(kp=1.0, kd=0.0, max_tilt_deg=5.0)
        roll, _, _ = ctrl.compute_with_terms(_state(y=1000.0))
        assert roll == pytest.approx(5.0)

    def test_within_limit_not_clamped(self) -> None:
        ctrl = BallController(kp=0.01, kd=0.0, max_tilt_deg=5.0)
        _, pitch, _ = ctrl.compute_with_terms(_state(x=10.0))
        assert abs(pitch) < 5.0


class TestGains:
    def test_set_gains_updates_output(self) -> None:
        ctrl = BallController(kp=0.0, kd=0.0, max_tilt_deg=90.0)
        _, pitch_before, _ = ctrl.compute_with_terms(_state(x=10.0))
        assert pitch_before == pytest.approx(0.0)

        ctrl.set_gains(kp=1.0, kd=0.0)
        _, pitch_after, _ = ctrl.compute_with_terms(_state(x=10.0))
        assert pitch_after != pytest.approx(0.0)


class TestAutotuneDisablesAutoTrim:
    def test_autotune_enable_disables_auto_trim(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.chdir(tmp_path)
        ctrl = BallController(auto_trim_enabled=True)
        assert ctrl.auto_trim_enabled is True
        ctrl.set_pd_autotune(True)
        assert ctrl.auto_trim_enabled is False

    def test_autotune_disable_restores_auto_trim(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.chdir(tmp_path)
        ctrl = BallController(auto_trim_enabled=True)
        ctrl.set_pd_autotune(True)
        ctrl.set_pd_autotune(False)
        assert ctrl.auto_trim_enabled is True

    def test_autotune_enable_preserves_false_trim(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.chdir(tmp_path)
        ctrl = BallController(auto_trim_enabled=False)
        ctrl.set_pd_autotune(True)
        ctrl.set_pd_autotune(False)
        assert ctrl.auto_trim_enabled is False


class TestComputePDFromMetrics:
    KP = 0.075
    KD = 0.030

    def _make_tuner(self, tmp_path: Path) -> PDAutotuner:
        tuner = _make_tuner(log_path=str(tmp_path / "autotune_test.log"))
        tuner.g_eff = 171.0
        tuner.target_zeta = 0.70
        tuner.min_kp = 0.005
        tuner.max_kp = 0.250
        tuner.min_kd = 0.0
        tuner.max_kd = 0.100
        tuner.min_overshoot_ratio = 0.02
        # KP/KD chosen so neither the underdamped (~0.100) nor overdamped
        # (~0.049) expected outputs hit the MAX_GAIN_DELTA_FRAC=0.50 fence
        return tuner

    def test_underdamped_exact_inversion(self, tmp_path: Path) -> None:
        tuner = self._make_tuner(tmp_path)
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
        kp, kd, rationale = tuner._compute_pd_from_metrics(metrics, self.KP, self.KD)
        assert kp == pytest.approx(expected_kp, rel=0.02)
        assert kd == pytest.approx(expected_kd, rel=0.02)
        assert "crossing" in rationale

    def test_overdamped_fallback_uses_settle_time(self, tmp_path: Path) -> None:
        tuner = self._make_tuner(tmp_path)
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
        kp, kd, rationale = tuner._compute_pd_from_metrics(metrics, self.KP, self.KD)
        assert kp == pytest.approx(expected_kp, rel=0.02)
        assert kd == pytest.approx(expected_kd, rel=0.02)
        assert "fallback" in rationale

    def test_timeout_holds_current_gains(self, tmp_path: Path) -> None:
        tuner = self._make_tuner(tmp_path)
        metrics = {
            "overshoot_ratio": 0.0,
            "settle_time_s": 10.0,
            "first_crossing_elapsed_s": None,
            "timed_out": 1.0,
        }
        kp, kd, rationale = tuner._compute_pd_from_metrics(metrics, 0.045, 0.022)
        assert kp == pytest.approx(0.045)
        assert kd == pytest.approx(0.022)
        assert "timeout" in rationale

    def test_output_is_clamped_to_bounds(self, tmp_path: Path) -> None:
        tuner = self._make_tuner(tmp_path)
        tuner.max_kp = 0.010
        tuner.max_kd = 0.005
        metrics = {
            "overshoot_ratio": 0.163,
            "settle_time_s": 0.1,  # very fast → huge wn
            "first_crossing_elapsed_s": 0.05,
            "timed_out": 0.0,
        }
        kp, kd, _ = tuner._compute_pd_from_metrics(metrics, self.KP, self.KD)
        assert kp <= 0.010
        assert kd <= 0.005

    def test_short_crossing_uses_settle_fallback(self, tmp_path: Path) -> None:
        tuner = self._make_tuner(tmp_path)
        # t_cross below MIN_CROSS_S threshold — should fall through to settle fallback
        metrics = {
            "overshoot_ratio": 0.163,
            "settle_time_s": 2.0,
            "first_crossing_elapsed_s": 0.10,   # too short — ignored
            "timed_out": 0.0,
        }
        kp, kd, rationale = tuner._compute_pd_from_metrics(metrics, self.KP, self.KD)
        assert "fallback" in rationale

    def test_gain_delta_is_capped(self, tmp_path: Path) -> None:
        tuner = self._make_tuner(tmp_path)
        kp0, kd0 = 0.045, 0.022
        # t_cross=1.0 ≥ MIN_CROSS_S; OS=0.163 → wn≈3.63 → kp_raw≈0.077 > 0.045*1.5=0.0675
        metrics = {
            "overshoot_ratio": 0.163,
            "settle_time_s": 2.0,
            "first_crossing_elapsed_s": 1.0,   # above MIN_CROSS_S
            "timed_out": 0.0,
        }
        kp, kd, _ = tuner._compute_pd_from_metrics(metrics, kp0, kd0)
        from settings import PD_AUTOTUNE_MAX_GAIN_DELTA_FRAC
        assert kp <= kp0 * (1.0 + PD_AUTOTUNE_MAX_GAIN_DELTA_FRAC) + 1e-9
        assert kp >= kp0 * (1.0 - PD_AUTOTUNE_MAX_GAIN_DELTA_FRAC) - 1e-9

    def test_gain_delta_cap_kd_zero_can_move(self, tmp_path: Path) -> None:
        # Regression: when kd == 0 the multiplicative window collapsed to
        # [0, 0], locking kd at zero forever. The absolute floor (0.001) must
        # allow kd to move off zero.
        tuner = self._make_tuner(tmp_path)
        metrics = {
            "overshoot_ratio": 0.163,
            "settle_time_s": 2.0,
            "first_crossing_elapsed_s": 1.0,
            "timed_out": 0.0,
        }
        _, kd, _ = tuner._compute_pd_from_metrics(metrics, 0.045, 0.0)
        assert kd > 0.0, "kd must be able to move off zero"


class TestAutotuneStateMachine:
    def test_enable_enters_wait_settle(self, tmp_path: Path) -> None:
        tuner = _make_tuner(log_path=str(tmp_path / "autotune_test.log"))
        tuner.set_enabled(True, None, 0.045, 0.022)
        # Prime the state machine with one call
        tuner.update(0.0, 0.0, 0.0, 0.0, 0.045, 0.022)
        assert tuner._state == "wait_settle"

    def test_wait_settle_advances_to_measuring_after_hold(self, tmp_path: Path) -> None:
        from settings import PD_AUTOTUNE_WAIT_SETTLE_HOLD_S
        tuner = _make_tuner(log_path=str(tmp_path / "autotune_test.log"))
        tuner.set_enabled(True, None, 0.045, 0.022)
        tuner.update(0.0, 0.0, 0.0, 0.0, 0.045, 0.022)
        assert tuner._state == "wait_settle"

        # Force the timer past the threshold
        tuner._wait_settle_timer = PD_AUTOTUNE_WAIT_SETTLE_HOLD_S + 0.1
        tuner.update(0.0, 0.0, 0.0, 0.0, 0.045, 0.022)
        assert tuner._state == "measuring"

    def test_wait_settle_timer_resets_on_movement(self, tmp_path: Path) -> None:
        tuner = _make_tuner(log_path=str(tmp_path / "autotune_test.log"))
        tuner.set_enabled(True, None, 0.045, 0.022)
        tuner.update(0.0, 0.0, 0.0, 0.0, 0.045, 0.022)
        tuner._wait_settle_timer = 0.5

        # Ball far from center → timer resets
        tuner.update(100.0, 100.0, 50.0, 50.0, 0.045, 0.022)
        assert tuner._wait_settle_timer == 0.0

    def test_leg_targets_are_axis_aligned(self, tmp_path: Path) -> None:
        tuner = _make_tuner(log_path=str(tmp_path / "autotune_test.log"))
        tuner.set_center(0.0, 0.0)
        targets = [tuner._autotune_leg_target(i) for i in range(4)]
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


class TestAutotuneSessionLog:
    def test_session_log_written_to_injected_path(self, tmp_path: Path) -> None:
        log_file = tmp_path / "autotune_session_test.log"
        tuner = _make_tuner(log_path=str(log_file))
        tuner.set_enabled(True, None, 0.045, 0.022)
        assert log_file.exists()
        assert "SESSION START" in log_file.read_text(encoding="utf-8")

        tuner.set_enabled(False, None, 0.045, 0.022)
        text = log_file.read_text(encoding="utf-8")
        assert "SESSION END" in text

    def test_re_enable_swaps_handler_and_truncates(self, tmp_path: Path) -> None:
        log_file = tmp_path / "autotune_session_test.log"
        tuner = _make_tuner(log_path=str(log_file))
        tuner.set_enabled(True, None, 0.045, 0.022)
        tuner.set_enabled(False, None, 0.045, 0.022)
        assert "SESSION END" in log_file.read_text(encoding="utf-8")

        # New session opens the file fresh (mode="w"): old lines are gone.
        tuner.set_enabled(True, None, 0.050, 0.020)
        text = log_file.read_text(encoding="utf-8")
        assert "SESSION START" in text
        assert "SESSION END" not in text
