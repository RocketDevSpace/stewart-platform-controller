"""
Unit tests for cv/camera_source.py using an injected fake VideoCapture.
"""

import time
from typing import Any

import cv2
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


class AsymmetricFakeCap(FakeCap):
    """FakeCap serving a STATIC left/right-asymmetric frame, so flipped
    output can be compared across separate read calls."""

    def read(self) -> tuple[bool, np.ndarray]:
        self._counter += 1
        time.sleep(0.002)
        frame = np.zeros((48, 64, 3), dtype=np.uint8)
        frame[:, :32] = 200          # bright LEFT half — chiral pattern
        frame[10:20, 5:9, 2] = 77    # off-center patch
        return True, frame


@pytest.fixture()
def source() -> Any:
    FakeCap.instances.clear()
    src = CameraSource(camera_index=0, cap_factory=FakeCap)
    yield src
    src.close()


@pytest.fixture()
def asym_source() -> Any:
    FakeCap.instances.clear()
    src = CameraSource(camera_index=0, cap_factory=AsymmetricFakeCap)
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


class TestFrameCallback:
    def test_callback_fires_per_published_frame(
        self, source: CameraSource
    ) -> None:
        calls: list[float] = []
        source.set_frame_callback(lambda: calls.append(1.0))
        source.open()
        # Observe distinct published timestamps from the consumer side...
        observed: set[float] = set()
        deadline = time.time() + 2.0
        while time.time() < deadline and len(observed) < 5:
            ts = source.latest_frame_ts()
            if ts > 0:
                observed.add(ts)
            time.sleep(0.001)
        assert len(observed) >= 5
        # ...every one of them was published exactly once, and each publish
        # fired the callback (allow the in-flight one to land).
        assert _wait_until(lambda: len(calls) >= len(observed))

    def test_raising_callback_does_not_kill_capture(
        self, source: CameraSource
    ) -> None:
        def boom() -> None:
            raise RuntimeError("boom")

        source.set_frame_callback(boom)
        source.open()
        assert _wait_until(lambda: source.latest_frame_ts() > 0)
        ts = source.latest_frame_ts()
        # Frames keep flowing after callback exceptions.
        assert _wait_until(lambda: source.latest_frame_ts() > ts)

    def test_callback_clearable(self, source: CameraSource) -> None:
        calls: list[float] = []
        source.set_frame_callback(lambda: calls.append(1.0))
        source.open()
        assert _wait_until(lambda: len(calls) > 0)
        source.set_frame_callback(None)
        n = len(calls)
        ts = source.latest_frame_ts()
        assert _wait_until(lambda: source.latest_frame_ts() > ts)
        # At most one in-flight invocation may still land after clearing.
        assert len(calls) <= n + 1


class TestReadLatestFlipped:
    def test_returns_flipped_content(self, asym_source: CameraSource) -> None:
        asym_source.open()
        assert _wait_until(lambda: asym_source.read_latest() is not None)
        plain = asym_source.read_latest()
        flipped = asym_source.read_latest_flipped()
        assert plain is not None and flipped is not None
        # Static frame: flipped read equals cv2.flip of the plain read.
        assert np.array_equal(flipped[0], cv2.flip(plain[0], 1))
        assert not np.array_equal(flipped[0], plain[0])   # chiral pattern

    def test_dst_reused_when_shape_matches(
        self, asym_source: CameraSource
    ) -> None:
        asym_source.open()
        assert _wait_until(
            lambda: asym_source.read_latest_flipped() is not None
        )
        first = asym_source.read_latest_flipped()
        assert first is not None
        buf = first[0]
        again = asym_source.read_latest_flipped(buf)
        assert again is not None
        assert again[0] is buf                # same object, no reallocation

    def test_wrong_shape_dst_reallocated(
        self, asym_source: CameraSource
    ) -> None:
        asym_source.open()
        assert _wait_until(
            lambda: asym_source.read_latest_flipped() is not None
        )
        bad = np.zeros((8, 8, 3), dtype=np.uint8)
        grabbed = asym_source.read_latest_flipped(bad)
        assert grabbed is not None
        assert grabbed[0] is not bad
        assert grabbed[0].shape == (48, 64, 3)

    def test_returns_none_before_first_frame(self) -> None:
        FakeCap.instances.clear()
        src = CameraSource(camera_index=0, cap_factory=FakeCap)
        assert src.read_latest_flipped() is None
