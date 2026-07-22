"""
Unit tests for hardware/serial_manager.py using a fake pyserial layer.

Covers the failure modes that used to be silent:
- double-connect guard (no second reader thread / port leak)
- read-loop survival when the receive callback raises
- unexpected link death -> disconnect callback + is_connected() False
- latest-wins coalescing of streamed payloads
- clean disconnect joins the background threads
"""

import threading
import time
from collections.abc import Callable, Iterator
from types import SimpleNamespace

import pytest

import hardware.serial_manager as sm


class FakeSerial:
    """Minimal stand-in for serial.Serial."""

    instances: list["FakeSerial"] = []

    def __init__(self, port: str, baud: int, timeout: float = 0.1,
                 write_timeout: float | None = None) -> None:
        self.port = port
        self.baud = baud
        self.is_open = True
        self.written: list[bytes] = []
        self.write_gate: threading.Event | None = None
        # Serve the firmware boot banner so connect()'s ready poll
        # returns immediately instead of waiting out its timeout.
        self._rx: list[bytes] = [b"[READY]\n"]
        self._rx_lock = threading.Lock()
        self.read_error: Exception | None = None
        FakeSerial.instances.append(self)

    # -- test helpers -------------------------------------------------
    def feed(self, data: bytes) -> None:
        with self._rx_lock:
            self._rx.append(data)

    # -- serial.Serial API --------------------------------------------
    def read(self, n: int) -> bytes:
        if self.read_error is not None:
            err, self.read_error = self.read_error, None
            raise err
        with self._rx_lock:
            if self._rx:
                return self._rx.pop(0)
        time.sleep(0.005)
        return b""

    def write(self, data: bytes) -> int:
        if self.write_gate is not None:
            self.write_gate.wait(timeout=2.0)
        self.written.append(data)
        return len(data)

    def close(self) -> None:
        self.is_open = False


@pytest.fixture()
def manager(monkeypatch: pytest.MonkeyPatch) -> Iterator[sm.SerialManager]:
    FakeSerial.instances.clear()
    monkeypatch.setattr(sm, "serial", SimpleNamespace(Serial=FakeSerial))
    monkeypatch.setattr(sm, "_ARDUINO_BOOT_DELAY_S", 0.0)
    # Bound the ready-banner poll so a missing banner can't stall a test.
    monkeypatch.setattr(sm, "_READY_WAIT_S", 0.5)
    mgr = sm.SerialManager("FAKE", 115200)
    yield mgr
    mgr.disconnect()


def _wait_until(cond: Callable[[], bool], timeout: float = 2.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if cond():
            return True
        time.sleep(0.01)
    return False


class TestConnectionLifecycle:
    def test_connect_opens_port(self, manager: sm.SerialManager) -> None:
        assert manager.connect() is True
        assert manager.is_connected()
        assert len(FakeSerial.instances) == 1

    def test_double_connect_does_not_reopen(self, manager: sm.SerialManager) -> None:
        assert manager.connect() is True
        assert manager.connect() is True
        assert len(FakeSerial.instances) == 1

    def test_send_without_connect_fails(self, manager: sm.SerialManager) -> None:
        assert manager.send(b"x\n") is False

    def test_connect_consumes_ready_banner(self, manager: sm.SerialManager) -> None:
        received: list[str] = []
        manager.set_receive_callback(received.append)
        assert manager.connect() is True
        # The banner is eaten by the connect-time poll (before the reader
        # thread starts) and must NOT be re-delivered as a received line.
        time.sleep(0.05)
        assert received == []

    def test_connect_without_banner_still_succeeds(
        self, manager: sm.SerialManager, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        class NoBannerSerial(FakeSerial):
            def __init__(self, port: str, baud: int, timeout: float = 0.1,
                         write_timeout: float | None = None) -> None:
                super().__init__(port, baud, timeout, write_timeout)
                with self._rx_lock:
                    self._rx.clear()

        monkeypatch.setattr(sm, "serial", SimpleNamespace(Serial=NoBannerSerial))
        t0 = time.monotonic()
        assert manager.connect() is True
        # Old firmware: no banner — connect waits out the (patched) ready
        # window and still comes up connected.
        assert time.monotonic() - t0 >= 0.4
        assert manager.is_connected()

    def test_disconnect_joins_threads(self, manager: sm.SerialManager) -> None:
        manager.connect()
        read_thread = manager._read_thread
        write_thread = manager._write_thread
        manager.disconnect()
        assert read_thread is not None and not read_thread.is_alive()
        assert write_thread is not None and not write_thread.is_alive()
        assert not manager.is_connected()


class TestReadLoop:
    def test_lines_delivered_to_callback(self, manager: sm.SerialManager) -> None:
        received: list[str] = []
        manager.set_receive_callback(received.append)
        manager.connect()
        FakeSerial.instances[0].feed(b"[READY]\n[OK] hello\n")
        assert _wait_until(lambda: len(received) == 2)
        assert received == ["[READY]", "[OK] hello"]

    def test_callback_exception_does_not_kill_read_loop(
        self, manager: sm.SerialManager
    ) -> None:
        received: list[str] = []

        def flaky(line: str) -> None:
            if line == "boom":
                raise RuntimeError("callback bug")
            received.append(line)

        manager.set_receive_callback(flaky)
        manager.connect()
        FakeSerial.instances[0].feed(b"boom\nafter\n")
        assert _wait_until(lambda: "after" in received)
        assert manager.is_connected()

    def test_read_error_reports_disconnect(self, manager: sm.SerialManager) -> None:
        reasons: list[str] = []
        manager.set_disconnect_callback(reasons.append)
        manager.connect()
        FakeSerial.instances[0].read_error = OSError("device unplugged")
        assert _wait_until(lambda: len(reasons) == 1)
        assert "unplugged" in reasons[0]
        assert not manager.is_connected()


class TestSend:
    def test_send_writes_bytes(self, manager: sm.SerialManager) -> None:
        manager.connect()
        assert manager.send(b"S,90,90,90,90,90,90,0\n") is True
        assert FakeSerial.instances[0].written == [b"S,90,90,90,90,90,90,0\n"]

    def test_write_error_reports_disconnect(self, manager: sm.SerialManager) -> None:
        reasons: list[str] = []
        manager.set_disconnect_callback(reasons.append)
        manager.connect()
        fake = FakeSerial.instances[0]

        def bad_write(data: bytes) -> int:
            raise OSError("write failed")

        fake.write = bad_write  # type: ignore[method-assign]
        assert manager.send(b"x\n") is False
        assert _wait_until(lambda: len(reasons) == 1)
        assert not manager.is_connected()

    def test_send_latest_coalesces_to_newest(self, manager: sm.SerialManager) -> None:
        manager.connect()
        fake = FakeSerial.instances[0]
        gate = threading.Event()
        fake.write_gate = gate

        # First payload occupies the (gated) writer; the next two arrive
        # while it is blocked and must coalesce down to only the newest.
        assert manager.send_latest(b"A\n")
        assert _wait_until(lambda: manager._latest_data is None)  # writer took A
        assert manager.send_latest(b"B\n")
        assert manager.send_latest(b"C\n")
        gate.set()
        assert _wait_until(lambda: len(fake.written) == 2)
        time.sleep(0.05)  # give a wrong extra write a chance to appear
        assert fake.written == [b"A\n", b"C\n"]

    def test_send_latest_when_disconnected_returns_false(
        self, manager: sm.SerialManager
    ) -> None:
        assert manager.send_latest(b"x\n") is False
