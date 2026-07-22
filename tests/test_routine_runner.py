from typing import Any
from unittest.mock import MagicMock

import numpy as np

from control.routine_runner import RoutineRunner
from core.ik_engine import IKEngine
from core.platform_state import IKResult
from hardware.servo_driver import ServoDriver
from routines.routines import ROUTINES

VALID_ROUTINE = next(iter(ROUTINES.keys()))

_NEUTRAL = {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}


def _make_ik_success_result() -> IKResult:
    return IKResult(
        success=True,
        servo_angles_deg=(90.0,) * 6,
        platform_points=np.zeros((6, 3)),
        arm_points=np.zeros((6, 3)),
        platform_center=np.zeros(3),
        platform_R=np.eye(3),
        servo_status=(None,) * 6,
        debug={"failures": []},
    )


def _make_runner() -> tuple[RoutineRunner, Any, Any, MagicMock]:
    ik = MagicMock(spec=IKEngine)
    ik.solve.return_value = _make_ik_success_result()
    servo = MagicMock(spec=ServoDriver)
    servo.send_angles.return_value = True
    on_pose = MagicMock()
    runner = RoutineRunner(ik_engine=ik, serial_driver=servo, on_pose_update=on_pose)
    return runner, ik, servo, on_pose


class TestLoad:
    def test_valid_routine_returns_true_and_sets_total_steps(self) -> None:
        runner, _, _, _ = _make_runner()
        assert runner.load(VALID_ROUTINE) is True
        assert runner.total_steps > 0

    def test_invalid_routine_returns_false(self) -> None:
        runner, _, _, _ = _make_runner()
        assert runner.load("nonexistent_routine") is False
        assert runner.total_steps == 0

    def test_reload_resets_state_cleanly(self) -> None:
        runner, _, _, _ = _make_runner()
        runner.load(VALID_ROUTINE)
        runner.start_send()
        # advance a few steps
        for _ in range(3):
            runner.tick()
        assert runner.current_index > 0
        assert runner.is_running

        runner.load(VALID_ROUTINE)
        assert runner.current_index == 0
        assert runner.is_running is False
        assert runner.is_send_mode is False


class TestPreviewMode:
    def test_tick_returns_true_while_steps_remain(self) -> None:
        runner, _, _, _ = _make_runner()
        runner.load(VALID_ROUTINE)
        runner.start_preview()
        assert runner.tick() is True
        assert runner.tick() is True

    def test_index_wraps_at_end(self) -> None:
        runner, _, _, _ = _make_runner()
        runner.load(VALID_ROUTINE)
        runner.start_preview()
        n = runner.total_steps
        for _ in range(n):
            assert runner.tick() is True
        assert runner.current_index == 0  # wrapped back

    def test_on_pose_update_called_once_per_tick(self) -> None:
        runner, _, _, on_pose = _make_runner()
        runner.load(VALID_ROUTINE)
        runner.start_preview()
        runner.tick()
        runner.tick()
        runner.tick()
        assert on_pose.call_count == 3

    def test_ik_not_called_in_preview(self) -> None:
        runner, ik, _, _ = _make_runner()
        runner.load(VALID_ROUTINE)
        runner.start_preview()
        for _ in range(5):
            runner.tick()
        ik.solve.assert_not_called()

    def test_send_angles_not_called_in_preview(self) -> None:
        runner, _, servo, _ = _make_runner()
        runner.load(VALID_ROUTINE)
        runner.start_preview()
        for _ in range(5):
            runner.tick()
        servo.send_angles.assert_not_called()


class TestSendMode:
    def test_ik_called_once_per_tick(self) -> None:
        runner, ik, _, _ = _make_runner()
        runner.load(VALID_ROUTINE)
        runner.start_send()
        runner.tick()
        runner.tick()
        runner.tick()
        assert ik.solve.call_count == 3

    def test_send_angles_called_once_per_tick(self) -> None:
        runner, _, servo, _ = _make_runner()
        runner.load(VALID_ROUTINE)
        runner.start_send()
        runner.tick()
        runner.tick()
        runner.tick()
        assert servo.send_angles.call_count == 3

    def test_returns_false_when_exhausted(self) -> None:
        runner, _, _, _ = _make_runner()
        runner.load(VALID_ROUTINE)
        runner.start_send()
        n = runner.total_steps
        for i in range(n - 1):
            assert runner.tick() is True
        # the final tick should return False
        assert runner.tick() is False
        assert runner.is_running is False

    def test_on_pose_update_called_once_per_tick(self) -> None:
        runner, _, _, on_pose = _make_runner()
        runner.load(VALID_ROUTINE)
        runner.start_send()
        runner.tick()
        runner.tick()
        assert on_pose.call_count == 2


class TestCancel:
    def test_preview_cancel_stops_immediately(self) -> None:
        runner, _, _, _ = _make_runner()
        runner.load(VALID_ROUTINE)
        runner.start_preview()
        runner.tick()
        assert runner.cancel() is False
        assert runner.is_running is False

    def test_tick_returns_false_after_preview_cancel(self) -> None:
        runner, _, _, _ = _make_runner()
        runner.load(VALID_ROUTINE)
        runner.start_preview()
        runner.cancel()
        assert runner.tick() is False

    def test_send_cancel_winds_down_to_neutral(self) -> None:
        """Cancelling a send-mode routine mid-flight replaces the remaining
        steps with an ease to neutral instead of stopping dead."""
        runner, _, _, on_pose = _make_runner()
        runner.load(VALID_ROUTINE)
        runner.start_send()
        for _ in range(5):
            runner.tick()
        assert runner.cancel() is True          # winding down
        assert runner.is_running is True
        while runner.tick():
            pass
        last_pose = on_pose.call_args[0][0]
        assert last_pose == _NEUTRAL

    def test_send_cancel_without_wind_down_stops_dead(self) -> None:
        runner, _, servo, _ = _make_runner()
        runner.load(VALID_ROUTINE)
        runner.start_send()
        runner.tick()
        sends_before = servo.send_angles.call_count
        assert runner.cancel(wind_down=False) is False
        assert runner.is_running is False
        assert runner.tick() is False
        assert servo.send_angles.call_count == sends_before


class TestReturnHome:
    def test_send_mode_appends_ease_to_neutral(self) -> None:
        """A send-mode routine must never park at its last waypoint."""
        runner, _, _, on_pose = _make_runner()
        runner.load(VALID_ROUTINE)
        preview_steps = runner.total_steps
        runner.start_send()
        assert runner.total_steps > preview_steps  # ease appended
        while runner.tick():
            pass
        last_pose = on_pose.call_args[0][0]
        assert last_pose == _NEUTRAL


class TestRestartAfterExhaustion:
    def test_start_send_after_exhaustion_restarts_cleanly(self) -> None:
        """Restarting without load() used to IndexError on the stale index."""
        runner, _, _, _ = _make_runner()
        runner.load(VALID_ROUTINE)
        runner.start_send()
        while runner.tick():
            pass
        assert runner.is_running is False
        runner.start_send()               # no load() in between
        assert runner.tick() is True      # must not raise
        assert runner.current_index == 1

    def test_start_preview_after_exhaustion_restarts_cleanly(self) -> None:
        runner, _, _, _ = _make_runner()
        runner.load(VALID_ROUTINE)
        runner.start_send()
        while runner.tick():
            pass
        runner.start_preview()
        assert runner.tick() is True


class TestIKFailureHandling:
    def test_failed_steps_skip_send_but_forward_pose(self) -> None:
        runner, ik, servo, on_pose = _make_runner()
        ik.solve.return_value = IKResult.failed("unreachable")
        runner.load(VALID_ROUTINE)
        runner.start_send()
        runner.tick()
        runner.tick()
        servo.send_angles.assert_not_called()
        assert on_pose.call_count == 2
        assert runner.ik_failure_count == 2

    def test_failure_count_resets_on_new_send(self) -> None:
        runner, ik, _, _ = _make_runner()
        ik.solve.return_value = IKResult.failed("unreachable")
        runner.load(VALID_ROUTINE)
        runner.start_send()
        runner.tick()
        assert runner.ik_failure_count == 1
        runner.load(VALID_ROUTINE)
        runner.start_send()
        assert runner.ik_failure_count == 0

    def test_prev_arm_points_retained_across_failed_step(self) -> None:
        runner, ik, _, _ = _make_runner()
        good = _make_ik_success_result()
        runner.load(VALID_ROUTINE)
        runner.start_send()
        runner.tick()                                   # success
        ik.solve.return_value = IKResult.failed("edge")
        runner.tick()                                   # failure
        ik.solve.return_value = good
        runner.tick()                                   # success again
        # The third call must still thread the arm points from the FIRST
        # successful solve (continuity survives skipped steps).
        prev_arg = ik.solve.call_args[0][1]
        assert prev_arg is not None


class TestRampHold:
    def test_tick_holds_during_firmware_ramp(self) -> None:
        """After a ramped send (driver.last_ramp_ms > 0) the runner must
        hold the stream until the firmware's ramp window has passed —
        streaming into a ramping firmware overflows its 64-byte UART
        buffer (observed on hardware 2026-07-22)."""
        fake_now = [0.0]
        runner, ik, servo, _ = _make_runner()
        runner._clock = lambda: fake_now[0]
        servo.last_ramp_ms = 300.0  # driver reports a 300 ms firmware ramp
        runner.load(VALID_ROUTINE)
        runner.start_send()

        assert runner.tick() is True          # sends step 0, arms the hold
        assert servo.send_angles.call_count == 1
        idx_after_first = runner.current_index

        fake_now[0] = 0.1                     # still inside 300*1.25 ms
        assert runner.tick() is True          # held: no send, no advance
        assert servo.send_angles.call_count == 1
        assert runner.current_index == idx_after_first

        fake_now[0] = 0.5                     # past the hold window
        runner.tick()
        assert servo.send_angles.call_count == 2

    def test_no_hold_for_instant_sends(self) -> None:
        fake_now = [0.0]
        runner, _, servo, _ = _make_runner()
        runner._clock = lambda: fake_now[0]
        servo.last_ramp_ms = 0.0
        runner.load(VALID_ROUTINE)
        runner.start_send()
        runner.tick()
        runner.tick()
        assert servo.send_angles.call_count == 2


class TestRoutineGenerators:
    """Every ROUTINES entry must produce the six-key pose contract —
    previously only the first routine ever executed under test."""

    def test_all_routines_execute_and_honor_contract(self) -> None:
        for name, gen in ROUTINES.items():
            steps = gen()
            assert len(steps) >= 2, name
            for step in steps:
                assert set(step.keys()) == set(_NEUTRAL.keys()), name
                for v in step.values():
                    assert isinstance(v, (int, float)), name
