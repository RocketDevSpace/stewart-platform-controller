"""
Unit tests for hardware/servo_driver.py

Tests cover format_command() and send_angles() using a mock SerialManager.
SerialManager itself requires hardware and is not tested here — any test
that would need a real serial port is marked [HARDWARE] and skipped in CI.
"""

import pytest
from unittest.mock import MagicMock
from hardware.servo_driver import ServoDriver
from hardware.serial_manager import SerialManager


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_driver(send_return: bool = True) -> tuple[ServoDriver, MagicMock]:
    """Return (ServoDriver, mock_serial) with send() preset to send_return."""
    mock_serial = MagicMock(spec=SerialManager)
    mock_serial.send.return_value = send_return
    driver = ServoDriver(mock_serial)
    return driver, mock_serial


def all_90() -> list[float]:
    return [90.0, 90.0, 90.0, 90.0, 90.0, 90.0]


# ---------------------------------------------------------------------------
# format_command — pure formatting, no side effects
# ---------------------------------------------------------------------------

class TestFormatCommand:
    def test_all_90_returns_correct_string(self) -> None:
        driver, _ = make_driver()
        assert driver.format_command(all_90()) == "S,90,90,90,90,90,90,0\n"

    def test_float_angles_rounded_to_int(self) -> None:
        driver, _ = make_driver()
        angles = [90.6, 90.0, 90.0, 90.0, 90.0, 90.0]
        assert driver.format_command(angles) == "S,91,90,90,90,90,90,0\n"

    def test_format_command_does_not_call_send(self) -> None:
        driver, mock_serial = make_driver()
        driver.format_command(all_90())
        mock_serial.send.assert_not_called()

    def test_command_ends_with_0_newline(self) -> None:
        driver, _ = make_driver()
        assert driver.format_command(all_90()).endswith(",0\n")

    def test_format_does_not_clip_unsafe_angle(self) -> None:
        """format_command is pure — it does NOT apply safety clipping."""
        driver, _ = make_driver()
        angles = [180.0, 90.0, 90.0, 90.0, 90.0, 90.0]
        # servo 0 at 180 should appear as 180 in the raw formatted string
        assert driver.format_command(angles) == "S,180,90,90,90,90,90,0\n"


# ---------------------------------------------------------------------------
# send_angles — clipping + formatting + dispatch
# ---------------------------------------------------------------------------

class TestSendAngles:
    def test_successful_send_returns_true(self) -> None:
        driver, _ = make_driver(send_return=True)
        assert driver.send_angles(all_90()) is True

    def test_serial_failure_returns_false(self) -> None:
        driver, _ = make_driver(send_return=False)
        assert driver.send_angles(all_90()) is False

    def test_send_calls_serial_send_once(self) -> None:
        driver, mock_serial = make_driver()
        driver.send_angles(all_90())
        mock_serial.send.assert_called_once()

    def test_unclipped_angles_sent_unchanged(self) -> None:
        driver, mock_serial = make_driver()
        driver.send_angles(all_90())
        sent_bytes = mock_serial.send.call_args[0][0]
        assert sent_bytes == b"S,90,90,90,90,90,90,0\n"

    def test_servo_0_clipped_to_170_before_send(self) -> None:
        """Servo 0 at 180 must be clipped to 170 by send_angles.

        An 80 deg jump from the boot pose exceeds SERVO_SLEW_INSTANT_MAX_DEG,
        so the command carries the large-move speedDelay (firmware ramp).
        """
        driver, mock_serial = make_driver()
        angles = [180.0, 90.0, 90.0, 90.0, 90.0, 90.0]
        driver.send_angles(angles)
        sent_bytes = mock_serial.send.call_args[0][0]
        assert sent_bytes == b"S,170,90,90,90,90,90,5\n"

    def test_servo_1_clipped_to_10_before_send(self) -> None:
        """Servo 1 below 10 must be clipped to 10 by send_angles."""
        driver, mock_serial = make_driver()
        angles = [90.0, 0.0, 90.0, 90.0, 90.0, 90.0]
        driver.send_angles(angles)
        sent_bytes = mock_serial.send.call_args[0][0]
        assert sent_bytes == b"S,90,10,90,90,90,90,5\n"

    def test_all_servos_clipped_simultaneously(self) -> None:
        driver, mock_serial = make_driver()
        angles = [180.0, 0.0, 180.0, 0.0, 180.0, 0.0]
        driver.send_angles(angles)
        sent_bytes = mock_serial.send.call_args[0][0]
        assert sent_bytes == b"S,170,10,170,10,170,10,5\n"

    def test_serial_send_receives_bytes_not_str(self) -> None:
        driver, mock_serial = make_driver()
        driver.send_angles(all_90())
        sent_arg = mock_serial.send.call_args[0][0]
        assert isinstance(sent_arg, bytes)

    def test_small_move_sent_instant(self) -> None:
        """Jumps within SERVO_SLEW_INSTANT_MAX_DEG go out with speedDelay 0."""
        driver, mock_serial = make_driver()
        driver.send_angles([95.0, 90.0, 90.0, 90.0, 90.0, 90.0])
        assert mock_serial.send.call_args[0][0] == b"S,95,90,90,90,90,90,0\n"

    def test_slew_tracks_last_sent_pose(self) -> None:
        """After a successful send, deltas are measured from the NEW pose."""
        driver, mock_serial = make_driver()
        driver.send_angles([100.0, 90.0, 90.0, 90.0, 90.0, 90.0])  # 10 deg, instant
        driver.send_angles([110.0, 90.0, 90.0, 90.0, 90.0, 90.0])  # 10 more, instant
        assert mock_serial.send.call_args[0][0] == b"S,110,90,90,90,90,90,0\n"

    def test_failed_send_does_not_update_last_sent(self) -> None:
        driver, mock_serial = make_driver(send_return=False)
        driver.send_angles([100.0, 90.0, 90.0, 90.0, 90.0, 90.0])
        mock_serial.send.return_value = True
        # Previous send failed, so the reference pose is still boot (90s):
        # a 10 deg total move stays instant.
        driver.send_angles([100.0, 90.0, 90.0, 90.0, 90.0, 90.0])
        assert mock_serial.send.call_args[0][0] == b"S,100,90,90,90,90,90,0\n"

    def test_streaming_uses_send_latest(self) -> None:
        driver, mock_serial = make_driver()
        mock_serial.send_latest.return_value = True
        driver.send_angles(all_90(), streaming=True)
        mock_serial.send_latest.assert_called_once()
        mock_serial.send.assert_not_called()

    def test_wrong_length_raises(self) -> None:
        driver, mock_serial = make_driver()
        with pytest.raises(ValueError):
            driver.send_angles([90.0] * 5)
        mock_serial.send.assert_not_called()

    def test_non_finite_angle_raises(self) -> None:
        driver, mock_serial = make_driver()
        with pytest.raises(ValueError):
            driver.send_angles([float("nan"), 90.0, 90.0, 90.0, 90.0, 90.0])
        mock_serial.send.assert_not_called()


# ---------------------------------------------------------------------------
# send_raw — validated raw-command path (GUI raw box routes here)
# ---------------------------------------------------------------------------

class TestSendRaw:
    def test_valid_raw_command_sent_with_newline(self) -> None:
        driver, mock_serial = make_driver()
        assert driver.send_raw("S,95,90,90,90,90,90") is True
        assert mock_serial.send.call_args[0][0] == b"S,95,90,90,90,90,90,0\n"

    def test_raw_without_prefix_rejected(self) -> None:
        driver, mock_serial = make_driver()
        assert driver.send_raw("hello") is False
        mock_serial.send.assert_not_called()

    def test_raw_wrong_field_count_rejected(self) -> None:
        driver, mock_serial = make_driver()
        assert driver.send_raw("S,90,90") is False
        mock_serial.send.assert_not_called()

    def test_raw_non_numeric_rejected(self) -> None:
        driver, mock_serial = make_driver()
        assert driver.send_raw("S,a,b,c,d,e,f,0") is False
        mock_serial.send.assert_not_called()

    def test_raw_out_of_range_angle_clipped(self) -> None:
        driver, mock_serial = make_driver()
        assert driver.send_raw("S,300,90,90,90,90,90,0") is True
        # 300 -> 170 (odd_servo_max), and the 80 deg jump gets the ramp delay
        assert mock_serial.send.call_args[0][0] == b"S,170,90,90,90,90,90,5\n"

    def test_raw_explicit_speed_delay_respected(self) -> None:
        driver, mock_serial = make_driver()
        assert driver.send_raw("S,95,90,90,90,90,90,7") is True
        assert mock_serial.send.call_args[0][0] == b"S,95,90,90,90,90,90,7\n"


# ---------------------------------------------------------------------------
# Hardware-dependent tests — skipped in CI
# ---------------------------------------------------------------------------

@pytest.mark.skip(reason="requires hardware")
def test_HARDWARE_connect_and_send_real_command() -> None:  # type: ignore[misc]
    """[HARDWARE] Connects to the real Arduino and sends a neutral pose."""
    from settings import SERIAL_PORT, SERIAL_BAUD
    manager = SerialManager(SERIAL_PORT, SERIAL_BAUD)
    assert manager.connect(), "Could not connect to Arduino"
    driver = ServoDriver(manager)
    result = driver.send_angles([90.0, 90.0, 90.0, 90.0, 90.0, 90.0])
    manager.disconnect()
    assert result is True
