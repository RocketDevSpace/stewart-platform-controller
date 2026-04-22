from typing import Any
from unittest.mock import MagicMock

import numpy as np

from control.routine_runner import RoutineRunner
from core.ik_engine import IKEngine
from hardware.servo_driver import ServoDriver
from routines.routines import ROUTINES

VALID_ROUTINE = next(iter(ROUTINES.keys()))


def _make_ik_success_result() -> dict:
    return {
        "success": True,
        "platform_points": np.zeros((6, 3)),
        "arm_points": np.zeros((6, 3)),
        "servo_angles_deg": np.array([90.0] * 6),
        "platform_center": np.zeros(3),
        "platform_R": np.eye(3),
        "debug": {"failures": []},
    }


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
    def test_cancel_clears_running(self) -> None:
        runner, _, _, _ = _make_runner()
        runner.load(VALID_ROUTINE)
        runner.start_preview()
        runner.tick()
        runner.cancel()
        assert runner.is_running is False

    def test_tick_returns_false_after_cancel(self) -> None:
        runner, _, _, _ = _make_runner()
        runner.load(VALID_ROUTINE)
        runner.start_preview()
        runner.cancel()
        assert runner.tick() is False
