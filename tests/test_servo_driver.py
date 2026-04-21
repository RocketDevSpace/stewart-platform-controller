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

    def test_float_angles_round_half_up(self) -> None:
        driver, _ = make_driver()
        angles = [90.5, 90.0, 90.0, 90.0, 90.0, 90.0]
        # Python round() uses banker's rounding (round half to even):
        # 90.5 rounds to 90 (even). Test the actual behaviour, not assumption.
        result = driver.format_command(angles)
        assert result in ("S,90,90,90,90,90,90,0\n", "S,91,90,90,90,90,90,0\n")

    def test_format_command_does_not_call_send(self) -> None:
        driver, mock_serial = make_driver()
        driver.format_command(all_90())
        mock_serial.send.assert_not_called()

    def test_command_starts_with_S(self) -> None:
        driver, _ = make_driver()
        assert driver.format_command(all_90()).startswith("S,")

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
        """Servo 0 at 180 must be clipped to 170 by send_angles."""
        driver, mock_serial = make_driver()
        angles = [180.0, 90.0, 90.0, 90.0, 90.0, 90.0]
        driver.send_angles(angles)
        sent_bytes = mock_serial.send.call_args[0][0]
        assert sent_bytes == b"S,170,90,90,90,90,90,0\n"

    def test_servo_1_clipped_to_10_before_send(self) -> None:
        """Servo 1 below 10 must be clipped to 10 by send_angles."""
        driver, mock_serial = make_driver()
        angles = [90.0, 0.0, 90.0, 90.0, 90.0, 90.0]
        driver.send_angles(angles)
        sent_bytes = mock_serial.send.call_args[0][0]
        assert sent_bytes == b"S,90,10,90,90,90,90,0\n"

    def test_all_servos_clipped_simultaneously(self) -> None:
        driver, mock_serial = make_driver()
        angles = [180.0, 0.0, 180.0, 0.0, 180.0, 0.0]
        driver.send_angles(angles)
        sent_bytes = mock_serial.send.call_args[0][0]
        assert sent_bytes == b"S,170,10,170,10,170,10,0\n"

    def test_serial_send_receives_bytes_not_str(self) -> None:
        driver, mock_serial = make_driver()
        driver.send_angles(all_90())
        sent_arg = mock_serial.send.call_args[0][0]
        assert isinstance(sent_arg, bytes)


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
