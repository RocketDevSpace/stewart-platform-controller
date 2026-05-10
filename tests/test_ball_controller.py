"""
Unit tests for control/ball_controller.py — BallController.compute()

Rules under test:
- None input returns (0.0, 0.0)
- Disabled controller returns (0.0, 0.0) regardless of input
- Ball at positive x produces negative pitch (platform corrects toward center)
- Ball at positive y produces positive roll (platform corrects toward center)
- Output is clamped to [-max_tilt_deg, max_tilt_deg]
"""
import pytest

from control.ball_controller import BallController
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
