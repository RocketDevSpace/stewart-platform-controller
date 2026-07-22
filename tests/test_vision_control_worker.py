"""
tests/test_vision_control_worker.py

Drives the worker's REAL _tick() with a fake CameraSource and fake tracker
(no camera needed) — replacing the old tautology tests that asserted their
own local variables. Covers: stale-frame skip, miss counting + filter
reset, reacquire gating on the command path, snapshot backpressure through
the real code path, and partial-init camera cleanup.
"""

import sys
from typing import Any

import numpy as np

from core.platform_state import BallState
from cv.camera_source import CaptureStats
from cv.vision_control_worker import ControlSnapshot, VisionControlWorker
from settings import TRACKER_REACQUIRE_VALID_FRAMES


# ---------------------------------------------------------------------------
# Fakes
# ---------------------------------------------------------------------------


class FakeCamera:
    def __init__(self) -> None:
        self.ts = 0.0
        self.frame = np.zeros((4, 4, 3), dtype=np.uint8)
        self.closed = False

    def advance(self) -> None:
        self.ts += 1.0 / 30.0

    def has_new_frame(self, since_ts: float) -> bool:
        return self.ts > since_ts

    def read_latest(self) -> tuple[np.ndarray, float] | None:
        return self.frame.copy(), self.ts

    def latest_frame_ts(self) -> float:
        return self.ts

    def stats(self) -> CaptureStats:
        return CaptureStats(
            backend="FAKE", period_ms=33.3, gray_mean=100.0,
            fail_streak=0, slow_streak=0, policy_mode="manual",
            software_gain=1.0,
        )

    def close(self) -> None:
        self.closed = True


class FakeTracker:
    """Returns a scripted sequence of BallState/None results."""

    def __init__(self, results: list[BallState | None]) -> None:
        self.results = list(results)
        self.hsv_lower = np.array([10, 83, 125], dtype=np.uint8)
        self.hsv_upper = np.array([28, 255, 255], dtype=np.uint8)

    def process(
        self, frame: np.ndarray, ts: float, brightness_gain: float = 1.0
    ) -> BallState | None:
        if self.results:
            return self.results.pop(0)
        return None

    def debug_views(self) -> tuple[None, None, None]:
        return None, None, None

    def set_hsv_thresholds(self, *args: int) -> None:
        pass


class FakeController:
    def __init__(self) -> None:
        self.resets = 0

    def compute_with_terms(
        self, ball_state: BallState
    ) -> tuple[float, float, dict]:
        return 1.0, -1.0, {"kp": 0.045, "kd": 0.022}

    def reset_motion_state(self) -> None:
        self.resets += 1


def _get_app() -> object:
    from PyQt5.QtWidgets import QApplication
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    return app


def _ball(x: float = 0.0, y: float = 0.0) -> BallState:
    return BallState(x_mm=x, y_mm=y, vx_mm_s=0.0, vy_mm_s=0.0)


def _make_worker(
    results: list[BallState | None],
    command_sender: Any = None,
) -> tuple[VisionControlWorker, FakeCamera, FakeController]:
    from core.ik_engine import IKEngine
    _get_app()
    worker = VisionControlWorker(
        ik_solver=IKEngine(),
        z_provider=lambda: 0.0,
        kp=0.045,
        kd=0.022,
        camera_index=0,
        command_sender=command_sender,
    )
    camera = FakeCamera()
    controller = FakeController()
    worker.camera = camera  # type: ignore[assignment]
    worker.ball_tracker = FakeTracker(results)  # type: ignore[assignment]
    worker.ball_controller = controller  # type: ignore[assignment]
    worker._running = True
    return worker, camera, controller


# ---------------------------------------------------------------------------
# ControlSnapshot construction (kept: cheap contract checks)
# ---------------------------------------------------------------------------


class TestControlSnapshot:
    def test_minimal_construction(self) -> None:
        snap = ControlSnapshot(
            timestamp=1.0,
            ball_state=None,
            pose={"x": 0, "y": 0, "z": 0, "roll": 0, "pitch": 0, "yaw": 0},
            servo_angles=[],
            ik_success=False,
            timings_ms={},
            ik_result=None,
            control_terms={},
            tracking_valid=False,
        )
        assert snap.tracking_valid is False
        assert snap.ball_state is None
        assert snap.miss_count == 0


# ---------------------------------------------------------------------------
# Real _tick() behavior with fakes
# ---------------------------------------------------------------------------


class TestTickStaleFrame:
    def test_stale_frame_skips_without_miss_or_reset(self) -> None:
        worker, camera, controller = _make_worker([_ball()])
        camera.advance()
        worker._tick()                      # processes frame 1
        misses_before = worker._miss_count
        worker._tick()                      # SAME frame ts → stale skip
        assert worker._miss_count == misses_before
        assert controller.resets == 0

    def test_new_frame_is_processed(self) -> None:
        worker, camera, _ = _make_worker([_ball(), _ball()])
        camera.advance()
        worker._tick()
        camera.advance()
        worker._tick()
        assert worker._valid_streak == 2


class TestTickMissHandling:
    def test_miss_increments_and_resets_streak(self) -> None:
        worker, camera, controller = _make_worker([None, None, _ball()])
        for _ in range(3):
            camera.advance()
            worker._tick()
        assert worker._miss_count == 0          # reset by the valid frame
        assert worker._valid_streak == 1
        assert controller.resets == 1           # reset once on reacquire


class TestReacquireGating:
    def test_first_detections_after_loss_do_not_send(self) -> None:
        sent: list = []
        results: list[BallState | None] = [None] + [_ball()] * (
            TRACKER_REACQUIRE_VALID_FRAMES + 2
        )
        worker, camera, _ = _make_worker(results, command_sender=sent.append)
        camera.advance()
        worker._tick()                          # miss
        for _ in range(TRACKER_REACQUIRE_VALID_FRAMES - 1):
            camera.advance()
            worker._tick()                      # gated valid frames
        assert sent == []                       # nothing reached hardware
        camera.advance()
        worker._tick()                          # streak reaches threshold
        assert len(sent) == 1

    def test_gated_snapshots_carry_reason(self) -> None:
        snaps: list[ControlSnapshot] = []
        results: list[BallState | None] = [None, _ball()]
        worker, camera, _ = _make_worker(results)
        worker.snapshot_ready.connect(snaps.append)
        camera.advance()
        worker._last_snapshot_emit_perf = -1e9
        worker._tick()
        worker.mark_snapshot_consumed()
        camera.advance()
        worker._last_snapshot_emit_perf = -1e9
        worker._tick()
        gated = [s for s in snaps if s.reason == "reacquire_gating"]
        assert len(gated) == 1


class TestBackpressure:
    def test_second_snapshot_blocked_until_consumed(self) -> None:
        snaps: list[ControlSnapshot] = []
        worker, camera, _ = _make_worker([None, None, None])
        worker.snapshot_ready.connect(snaps.append)
        worker._last_snapshot_emit_perf = -1e9  # force emit cadence open
        camera.advance()
        worker._tick()
        assert len(snaps) == 1                  # emitted, now inflight
        camera.advance()
        worker._last_snapshot_emit_perf = -1e9
        worker._tick()
        assert len(snaps) == 1                  # blocked while inflight
        worker.mark_snapshot_consumed()
        camera.advance()
        worker._last_snapshot_emit_perf = -1e9
        worker._tick()
        assert len(snaps) == 2                  # allowed again

    def test_timings_contain_only_measured_keys(self) -> None:
        snaps: list[ControlSnapshot] = []
        worker, camera, _ = _make_worker([None])
        worker.snapshot_ready.connect(snaps.append)
        worker._last_snapshot_emit_perf = -1e9
        camera.advance()
        worker._tick()
        assert snaps, "no snapshot emitted"
        # The 28 fabricated zero-filled trk_* keys are gone.
        fabricated = [k for k in snaps[0].timings_ms if k.startswith("trk_")]
        assert set(fabricated) <= {"trk_cap_period", "trk_gray_mean"}


class TestErrorRateLimit:
    def test_persistent_tick_errors_are_rate_limited(self) -> None:
        errors: list[str] = []
        worker, camera, _ = _make_worker([])
        worker.error.connect(errors.append)

        class ExplodingTracker:
            hsv_lower = np.array([0, 0, 0], dtype=np.uint8)
            hsv_upper = np.array([0, 0, 0], dtype=np.uint8)

            def process(self, *a: Any, **k: Any) -> None:
                raise RuntimeError("boom")

            def debug_views(self) -> tuple[None, None, None]:
                return None, None, None

        worker.ball_tracker = ExplodingTracker()  # type: ignore[assignment]
        for _ in range(5):
            camera.advance()
            worker._tick()
        assert len(errors) == 1                  # not 5


class TestStopClosesCamera:
    def test_stop_closes_camera_source(self) -> None:
        worker, camera, _ = _make_worker([])
        stopped: list[bool] = []
        worker.stopped.connect(lambda: stopped.append(True))
        worker.stop()
        assert camera.closed is True
        assert worker.camera is None
        assert stopped == [True]
