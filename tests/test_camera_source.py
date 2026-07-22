"""
Unit tests for cv/camera_source.py using an injected fake VideoCapture.
"""

import time
from typing import Any

import numpy as np
import pytest

from cv.camera_source import CameraSource


class FakeCap:
    instances: list["FakeCap"] = []

    def __init__(self, index: int, backend: int) -> None:
        self.index = index
        self.backend = backend
        self.released = False
        self._counter = 0
        FakeCap.instances.append(self)

    def isOpened(self) -> bool:  # noqa: N802 (cv2 API name)
        return True

    def set(self, prop: int, value: Any) -> bool:
        return True

    def read(self) -> tuple[bool, np.ndarray]:
        self._counter += 1
        time.sleep(0.002)
        frame = np.full((48, 64, 3), self._counter % 255, dtype=np.uint8)
        return True, frame

    def release(self) -> None:
        self.released = True


@pytest.fixture()
def source() -> Any:
    FakeCap.instances.clear()
    src = CameraSource(camera_index=0, cap_factory=FakeCap)
    yield src
    src.close()


def _wait_until(cond: Any, timeout: float = 2.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if cond():
            return True
        time.sleep(0.01)
    return False


class TestLifecycle:
    def test_init_does_no_io(self) -> None:
        FakeCap.instances.clear()
        CameraSource(camera_index=0, cap_factory=FakeCap)
        assert FakeCap.instances == []      # factory untouched until open()

    def test_open_starts_capture(self, source: CameraSource) -> None:
        stats = source.open()
        assert stats.backend != "unopened"
        assert _wait_until(lambda: source.latest_frame_ts() > 0)

    def test_close_releases_cap(self, source: CameraSource) -> None:
        source.open()
        cap = source.cap
        source.close()
        assert cap.released is True
        assert source.cap is None

    def test_double_open_is_idempotent(self, source: CameraSource) -> None:
        source.open()
        n = len(FakeCap.instances)
        source.open()
        assert len(FakeCap.instances) == n


class TestFrameAccess:
    def test_read_latest_returns_copy(self, source: CameraSource) -> None:
        source.open()
        assert _wait_until(lambda: source.read_latest() is not None)
        grabbed = source.read_latest()
        assert grabbed is not None
        frame, ts = grabbed
        original = frame[0, 0, 0]
        frame[:] = 99                      # mutating the copy...
        again = source.read_latest()
        assert again is not None
        # ...must not corrupt the internal buffer (value evolves with the
        # counter but never becomes our poison value everywhere).
        assert not np.all(again[0] == 99) or original == 99

    def test_has_new_frame_monotonic(self, source: CameraSource) -> None:
        source.open()
        assert _wait_until(lambda: source.latest_frame_ts() > 0)
        ts = source.latest_frame_ts()
        assert source.has_new_frame(ts - 1.0)
        assert not source.has_new_frame(ts + 1e9)

    def test_stats_snapshot(self, source: CameraSource) -> None:
        source.open()
        assert _wait_until(lambda: source.stats().period_ms > 0)
        st = source.stats()
        assert st.software_gain >= 1.0
        assert st.policy_mode in ("bootstrap_auto", "manual")
