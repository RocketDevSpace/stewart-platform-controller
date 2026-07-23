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
- Rest mode: resting output is exactly level + trim; auto-trim keeps
  integrating during rest and the resting output follows it; a fast
  disturbance regains near-full PD authority on the same call
- D-term pins: no derivative kick on target steps; d-cap clamps
"""
import math
from pathlib import Path

import pytest

from control.autotune import PDAutotuner, _PDLegEvaluator
from control.ball_controller import BallController
from control.setpoint import SetpointArbiter
from core.platform_state import BallState
from settings import PD_D_TERM_LIMIT_DEG


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


class TestAutotuneFreezesIntegral:
    """I-term rework: autotune no longer touches the auto_trim_enabled
    flag (the old stash/restore is gone). The integral freeze during a
    session is DERIVED from autotuner.enabled inside compute — the leg
    evaluator assumes a pure PD step response."""

    def test_autotune_does_not_flip_the_flag(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.chdir(tmp_path)
        ctrl = BallController(auto_trim_enabled=True)
        ctrl.set_pd_autotune(True)
        assert ctrl.auto_trim_enabled is True
        ctrl.set_pd_autotune(False)
        assert ctrl.auto_trim_enabled is True

    def test_integral_frozen_during_session_thaws_after(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.chdir(tmp_path)
        clock = _FakeClock()
        ctrl = BallController(auto_trim_enabled=True, clock=clock)
        ctrl.set_pd_autotune(True)
        clock.advance(1 / 30)
        _, _, terms = ctrl.compute_with_terms(_state(x=10.0))
        assert terms["i_frozen"] is True
        ctrl.set_pd_autotune(False)
        clock.advance(1 / 30)
        _, _, terms = ctrl.compute_with_terms(_state(x=10.0))
        assert terms["i_frozen"] is False

    def test_flag_off_stays_off_through_a_session(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
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


class TestRestModeIntegration:
    """Near-target rest mode wired through BallController (RestGate)."""

    def _rest_controller(
        self,
        clock: _FakeClock,
        kp: float = 0.045,
        kd: float = 0.022,
        roll_offset: float = 0.0,
        pitch_offset: float = 0.0,
        auto_trim_enabled: bool = False,
        rest_mode_enabled: bool = True,
    ) -> BallController:
        return BallController(
            kp=kp, kd=kd,
            roll_offset=roll_offset, pitch_offset=pitch_offset,
            auto_trim_enabled=auto_trim_enabled,
            rest_mode_enabled=rest_mode_enabled,
            clock=clock,
        )

    def test_rest_terms_keys_present(self) -> None:
        clock = _FakeClock()
        ctrl = self._rest_controller(clock)
        _, _, terms = ctrl.compute_with_terms(_state(x=5.0))
        for key in (
            "rest_state", "rest_mode_active", "rest_radius_mm",
            "rest_speed_lpf_mm_s", "rest_hold_elapsed_s",
        ):
            assert key in terms, f"missing terms key {key!r}"
        assert terms["rest_state"] in ("active", "holding", "resting")
        assert terms["rest_radius_mm"] == pytest.approx(5.0)

    def test_resting_output_is_exactly_trim_offsets(self) -> None:
        clock = _FakeClock(100.0)
        ctrl = self._rest_controller(
            clock, roll_offset=0.5, pitch_offset=-0.5)
        state = _state(x=3.0)  # 3 mm off target: inside the enter radius

        # Active PD: output carries the P contribution on top of the trim.
        _, pitch, terms = ctrl.compute_with_terms(state)
        assert terms["rest_state"] == "holding"
        assert pitch == pytest.approx(0.045 * (-3.0) - 0.5)

        # Dwell past the hold; slew (300 deg/s * 0.1 s = 30 deg/step) has
        # long since settled the tiny hop to level + trim.
        for _ in range(10):
            clock.advance(0.1)
            roll, pitch, terms = ctrl.compute_with_terms(state)
        assert terms["rest_state"] == "resting"
        assert terms["rest_mode_active"] is True
        assert roll == pytest.approx(0.5)
        assert pitch == pytest.approx(-0.5)
        assert terms["roll_cmd"] == pytest.approx(0.5)
        assert terms["pitch_cmd"] == pytest.approx(-0.5)

    def test_rest_waits_for_integral_settle_then_serves_trim(self) -> None:
        # I-term rework composition: resting parks the output at
        # trim + I with P and D dropped, so rest must WAIT until the
        # integral is flat (|dI/dt| under REST_I_RATE_MAX_DEG_S) — a
        # still-converging I is not an equilibrium and limit-cycles
        # (sim-caught). Once flat, rest engages and its command is
        # trim + I exactly.
        clock = _FakeClock(50.0)
        ctrl = self._rest_controller(clock, kd=0.0, auto_trim_enabled=True)
        ctrl.set_target(0.0, 0.0)

        # Ball parked 5 mm off: the integral walks at ~0.14 deg/s —
        # far above the settle threshold. Rest must NOT engage no
        # matter how long the ball sits quiet.
        state = _state(x=5.0)
        terms: dict = {}
        for _ in range(30):
            clock.advance(0.1)
            _, _, terms = ctrl.compute_with_terms(state)
        assert terms["i_term"][0] < 0.0
        assert terms["i_rate_deg_s"] > 0.02
        assert terms["rest_mode_active"] is False

        # Ball now essentially AT target (0.4 mm — the leak-equilibrium
        # zone): the integral rate falls under the threshold, rest
        # engages after its own hold, and the resting output is
        # trim + integral (pitch contribution = i_x).
        state = _state(x=0.4)
        for _ in range(30):
            clock.advance(0.1)
            _, pitch, terms = ctrl.compute_with_terms(state)
        assert terms["i_rate_deg_s"] <= 0.02
        assert terms["rest_mode_active"] is True
        # Trim store untouched; the leveling is all integral.
        assert terms["pitch_offset"] == 0.0
        assert pitch == pytest.approx(
            terms["pitch_offset"] + terms["i_term"][0], abs=0.02
        )

    def test_fast_disturbance_gets_full_pd_response_same_call(self) -> None:
        # END-TO-END: a resting controller hit by a fast disturbance must
        # answer on that same call with >= 85% of the command an identical
        # never-resting controller produces for the same input history.
        def drive(rest_mode_enabled: bool) -> tuple[float, float, dict]:
            clock = _FakeClock(10.0)
            ctrl = self._rest_controller(
                clock, rest_mode_enabled=rest_mode_enabled)
            terms: dict = {}
            for _ in range(10):
                _, _, terms = ctrl.compute_with_terms(_state())
                clock.advance(0.1)
            if rest_mode_enabled:
                assert terms["rest_mode_active"] is True  # was resting
            # One disturbance frame: radius 15 mm, speed 200 mm/s.
            roll, pitch, terms = ctrl.compute_with_terms(
                _state(x=15.0, vx=200.0))
            return roll, pitch, terms

        roll_r, pitch_r, terms_r = drive(rest_mode_enabled=True)
        roll_n, pitch_n, terms_n = drive(rest_mode_enabled=False)

        # Rest exits on the SAME call: full PD is computed this cycle.
        assert terms_r["rest_state"] == "active"
        assert terms_r["rest_mode_active"] is False
        assert pitch_n < 0.0  # sanity: disturbance demands a real command
        assert pitch_r * pitch_n > 0.0  # same direction
        assert abs(pitch_r) >= 0.85 * abs(pitch_n)

    def test_set_target_resets_rest(self) -> None:
        clock = _FakeClock()
        ctrl = self._rest_controller(clock)
        for _ in range(10):
            _, _, terms = ctrl.compute_with_terms(_state())
            clock.advance(0.1)
        assert terms["rest_mode_active"] is True

        ctrl.set_target(0.0, 0.0)  # same coordinates — still a reset event
        _, _, terms = ctrl.compute_with_terms(_state())
        assert terms["rest_state"] != "resting"

    def test_reset_motion_state_resets_rest(self) -> None:
        clock = _FakeClock()
        ctrl = self._rest_controller(clock)
        for _ in range(10):
            _, _, terms = ctrl.compute_with_terms(_state())
            clock.advance(0.1)
        assert terms["rest_mode_active"] is True

        ctrl.reset_motion_state()  # tracking loss / reacquire path
        _, _, terms = ctrl.compute_with_terms(_state())
        assert terms["rest_state"] != "resting"

    def test_rest_mode_disabled_never_rests(self) -> None:
        clock = _FakeClock()
        ctrl = self._rest_controller(clock, rest_mode_enabled=False)
        for _ in range(30):
            _, _, terms = ctrl.compute_with_terms(_state())
            clock.advance(0.1)
        assert terms["rest_state"] == "active"
        assert terms["rest_mode_active"] is False


class TestHomeCalConvergence:
    """I-term rework: home calibration auto-completes — the controller
    watches the integral, and once it is flat with the ball slow and
    near center, folds it into trim, cancels, and flags a transient
    home_cal_event the GUI persists."""

    def test_converges_folds_and_completes(self) -> None:
        clock = _FakeClock(100.0)   # nonzero: elapsed-timer guard
        ctrl = BallController(kp=0.045, kd=0.022, clock=clock)
        ctrl.start_home_calibration()
        # Ball essentially at center and still: the integral is flat
        # from the start; the 2 s window fills and convergence fires.
        event: dict = {}
        for _ in range(120):
            clock.advance(1.0 / 30.0)
            _, _, terms = ctrl.compute_with_terms(
                BallState(x_mm=0.2, y_mm=0.0, vx_mm_s=0.0, vy_mm_s=0.0)
            )
            if "home_cal_event" in terms:
                event = terms["home_cal_event"]
                break
        assert event.get("type") == "completed"
        assert ctrl.home_calibration_active is False
        # The event carries the freshly folded offsets (what the GUI
        # must persist) and they match the store.
        assert event["roll"] == ctrl.roll_offset
        assert event["pitch"] == ctrl.pitch_offset
        # Fold drained the integral (next frame re-integrates a tick).
        clock.advance(1.0 / 30.0)
        _, _, terms = ctrl.compute_with_terms(
            BallState(x_mm=0.2, y_mm=0.0, vx_mm_s=0.0, vy_mm_s=0.0)
        )
        assert abs(terms["i_term"][0]) < 0.01
        assert abs(terms["i_term"][1]) < 0.01

    def test_far_ball_saturated_integral_does_not_fake_converge(self) -> None:
        # A ball stuck 30 mm out drives the integral to the wide
        # home-cal limit, where it goes FLAT — the radius gate must
        # refuse to fold that windup; the run ends in timeout with the
        # integral discarded and trim untouched.
        clock = _FakeClock(100.0)   # nonzero: elapsed-timer guard
        ctrl = BallController(kp=0.045, kd=0.022, clock=clock)
        ctrl.start_home_calibration()
        event: dict = {}
        for _ in range(int(31.0 * 30)):
            clock.advance(1.0 / 30.0)
            _, _, terms = ctrl.compute_with_terms(
                BallState(x_mm=30.0, y_mm=0.0, vx_mm_s=0.0, vy_mm_s=0.0)
            )
            if "home_cal_event" in terms:
                event = terms["home_cal_event"]
                break
        assert event.get("type") == "timeout"
        assert ctrl.home_calibration_active is False
        assert (ctrl.roll_offset, ctrl.pitch_offset) == (0.0, 0.0)
        clock.advance(1.0 / 30.0)
        _, _, terms = ctrl.compute_with_terms(
            BallState(x_mm=30.0, y_mm=0.0, vx_mm_s=0.0, vy_mm_s=0.0)
        )
        # Windup discarded (next frame re-integrates a tick at most).
        assert abs(terms["i_term"][0]) < 0.05
        assert abs(terms["i_term"][1]) < 0.05

    def test_request_trim_fold_flags_saved_event(self) -> None:
        clock = _FakeClock()
        ctrl = BallController(kp=0.045, kd=0.022, clock=clock,
                              auto_trim_enabled=True)
        ctrl.compute_with_terms(
            BallState(x_mm=15.0, y_mm=0.0, vx_mm_s=0.0, vy_mm_s=0.0)
        )
        for _ in range(60):
            clock.advance(1.0 / 30.0)
            ctrl.compute_with_terms(
                BallState(x_mm=15.0, y_mm=0.0, vx_mm_s=0.0, vy_mm_s=0.0)
            )
        roll, pitch = ctrl.request_trim_fold()
        assert pitch < 0.0                     # absorbed the -x integral
        assert (ctrl.roll_offset, ctrl.pitch_offset) == (roll, pitch)
        clock.advance(1.0 / 30.0)
        _, _, terms = ctrl.compute_with_terms(
            BallState(x_mm=15.0, y_mm=0.0, vx_mm_s=0.0, vy_mm_s=0.0)
        )
        event = terms.get("home_cal_event", {})
        assert event.get("type") == "saved"
        assert event["roll"] == roll
        assert event["pitch"] == pitch


class TestRestHomeCalExclusion:
    def test_never_rests_during_home_calibration(self) -> None:
        """Home calibration needs ACTIVE PD centering the ball; rest mode
        parking at level+trim made calibration far less reliable
        (observed on the rig 2026-07-22)."""
        clock = _FakeClock()
        ctrl = BallController(kp=0.045, kd=0.022, clock=clock)
        ctrl.start_home_calibration()
        # Ball near center and still — prime rest-entry conditions.
        # (2 mm keeps the integral visibly moving so the convergence
        # watcher cannot auto-complete the calibration mid-test.)
        for _ in range(60):
            clock.advance(1.0 / 30.0)
            _, _, terms = ctrl.compute_with_terms(
                BallState(x_mm=2.0, y_mm=0.0, vx_mm_s=0.0, vy_mm_s=0.0)
            )
        assert terms["rest_state"] == "active"
        assert terms["rest_mode_active"] is False
        # After calibration ends, rest can engage again. Ball essentially
        # AT target (0.3 mm): the integral rate is flat (a synthetic ball
        # parked at 1 mm never converges — it can't move — which would
        # hold the I-settle entry gate open forever; a real ball closes
        # the loop).
        ctrl.cancel_home_calibration()
        for _ in range(60):
            clock.advance(1.0 / 30.0)
            _, _, terms = ctrl.compute_with_terms(
                BallState(x_mm=0.3, y_mm=0.0, vx_mm_s=0.0, vy_mm_s=0.0)
            )
        assert terms["rest_mode_active"] is True


class TestDTermPins:
    """Pinned d-term semantics (control code unchanged by the rest work)."""

    def test_target_step_does_not_kick_d_term(self) -> None:
        # D acts on measured ball velocity, not on the error derivative:
        # stepping the target must not change the d-term at all.
        clock = _FakeClock()
        ctrl = BallController(
            kp=0.05, kd=0.02, rest_mode_enabled=False, clock=clock)
        _, _, before = ctrl.compute_with_terms(_state(x=10.0, vx=40.0))

        clock.advance(0.02)
        ctrl.set_target(30.0, 0.0)  # 30 mm target step
        _, _, after = ctrl.compute_with_terms(_state(x=10.0, vx=40.0))

        assert after["d_term"] == pytest.approx(before["d_term"])
        assert after["d_term"] == pytest.approx((0.02 * -40.0, 0.0))
        # ...while the P term did see the step (the error jumped).
        assert after["p_term"] != pytest.approx(before["p_term"])

    def test_d_term_cap_clamps_extreme_velocity(self) -> None:
        ctrl = BallController(
            kp=0.0, kd=1.0, rest_mode_enabled=False, clock=_FakeClock())
        _, _, terms = ctrl.compute_with_terms(
            _state(vx=10000.0, vy=-10000.0))
        assert terms["d_term"] == pytest.approx(
            (-PD_D_TERM_LIMIT_DEG, PD_D_TERM_LIMIT_DEG))


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
