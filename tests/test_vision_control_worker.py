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
from settings import (
    TRACKER_REACQUIRE_VALID_FRAMES,
    VISION_MISS_NEUTRAL_AFTER_FRAMES,
)


# ---------------------------------------------------------------------------
# Fakes
# ---------------------------------------------------------------------------


class FakeCamera:
    def __init__(self) -> None:
        self.ts = 0.0
        self.frame = np.zeros((4, 4, 3), dtype=np.uint8)
        self.closed = False
        self.frame_callback: Any = None

    def advance(self) -> None:
        self.ts += 1.0 / 30.0

    def has_new_frame(self, since_ts: float) -> bool:
        return self.ts > since_ts

    def read_latest(self) -> tuple[np.ndarray, float] | None:
        return self.frame.copy(), self.ts

    def read_latest_flipped(
        self, dst: np.ndarray | None = None
    ) -> tuple[np.ndarray, float] | None:
        if (
            dst is None
            or dst.shape != self.frame.shape
            or dst.dtype != self.frame.dtype
        ):
            dst = np.empty_like(self.frame)
        np.copyto(dst, self.frame[:, ::-1])   # horizontal flip
        return dst, self.ts

    def set_frame_callback(self, cb: Any) -> None:
        self.frame_callback = cb

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
        self,
        frame: np.ndarray,
        ts: float,
        brightness_gain: float = 1.0,
        pre_flipped: bool = False,
    ) -> BallState | None:
        if self.results:
            return self.results.pop(0)
        return None

    def debug_views(self) -> tuple[None, None, None]:
        return None, None, None

    def set_hsv_thresholds(self, *args: int) -> None:
        pass


class ViewFakeTracker(FakeTracker):
    """FakeTracker that mimics the real tracker's debug-view behavior:
    stores a REFERENCE to the processed frame (no copy)."""

    def __init__(self, results: list[BallState | None]) -> None:
        super().__init__(results)
        self.last_frame: np.ndarray | None = None

    def process(
        self,
        frame: np.ndarray,
        ts: float,
        brightness_gain: float = 1.0,
        pre_flipped: bool = False,
    ) -> BallState | None:
        self.last_frame = frame
        return super().process(frame, ts, brightness_gain, pre_flipped)

    def debug_views(self) -> Any:
        return self.last_frame, None, None


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


class TestSetZ:
    def test_set_z_flows_into_commanded_pose(self) -> None:
        snaps: list[ControlSnapshot] = []
        worker, camera, _ = _make_worker([_ball(), None])
        worker.snapshot_ready.connect(snaps.append)
        worker.set_z(12.0)

        worker._last_snapshot_emit_perf = -1e9
        camera.advance()
        worker._tick()                          # valid frame
        worker.mark_snapshot_consumed()
        worker._last_snapshot_emit_perf = -1e9
        camera.advance()
        worker._tick()                          # miss frame
        assert len(snaps) == 2
        assert snaps[0].pose["z"] == 12.0       # valid-path pose
        assert snaps[1].pose["z"] == 12.0       # miss-path pose

    def test_z_defaults_to_zero(self) -> None:
        snaps: list[ControlSnapshot] = []
        worker, camera, _ = _make_worker([_ball()])
        worker.snapshot_ready.connect(snaps.append)
        worker._last_snapshot_emit_perf = -1e9
        camera.advance()
        worker._tick()
        assert snaps[0].pose["z"] == 0.0


class TestNeutralFallback:
    def test_sustained_misses_send_one_neutral_per_window(self) -> None:
        sent: list[list[float]] = []
        misses: list[BallState | None] = [None] * (
            VISION_MISS_NEUTRAL_AFTER_FRAMES + 5
        )
        worker, camera, _ = _make_worker(misses, command_sender=sent.append)
        for _ in range(VISION_MISS_NEUTRAL_AFTER_FRAMES - 1):
            camera.advance()
            worker._tick()
        assert sent == []                       # below the miss threshold
        for _ in range(6):
            camera.advance()
            worker._tick()
        # Threshold crossed: exactly ONE neutral send inside the resend
        # throttle window, no matter how many further misses arrive.
        assert len(sent) == 1
        assert len(sent[0]) == 6

        # A new throttle window (simulated by rewinding the stamp) allows
        # exactly one more.
        worker.ball_tracker = FakeTracker([None, None])  # type: ignore[assignment]
        worker._last_neutral_send = -1e9
        camera.advance()
        worker._tick()
        camera.advance()
        worker._tick()
        assert len(sent) == 2

    def test_neutral_send_ignores_reacquire_gate(self) -> None:
        # The neutral fallback is the safety action: it must fire even
        # though the reacquire gate is blocking normal command sends
        # (valid_streak is 0 throughout a miss run).
        sent: list[list[float]] = []
        misses: list[BallState | None] = [None] * (
            VISION_MISS_NEUTRAL_AFTER_FRAMES + 1
        )
        worker, camera, _ = _make_worker(misses, command_sender=sent.append)
        for _ in range(VISION_MISS_NEUTRAL_AFTER_FRAMES + 1):
            camera.advance()
            worker._tick()
        assert worker._valid_streak == 0
        assert len(sent) == 1

    def test_recovery_resets_miss_count_and_rearms(self) -> None:
        sent: list[list[float]] = []
        results: list[BallState | None] = (
            [None] * (VISION_MISS_NEUTRAL_AFTER_FRAMES + 1) + [_ball()]
        )
        worker, camera, _ = _make_worker(results, command_sender=sent.append)
        for _ in range(len(results)):
            camera.advance()
            worker._tick()
        neutral_sends = len(sent)
        assert worker._miss_count == 0          # cleared by the valid frame
        # A fresh short miss run (below threshold) must NOT send neutral
        # again even with the throttle window forced open.
        worker.ball_tracker = FakeTracker([None])  # type: ignore[assignment]
        worker._last_neutral_send = -1e9
        camera.advance()
        worker._tick()
        assert len(sent) == neutral_sends


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


class TestEventDrivenTick:
    def test_frame_callback_emits_bridge_signal(self) -> None:
        # _on_frame_available runs on the capture thread in production; it
        # must only emit _frame_arrived. In-test (same thread) the auto
        # connection delivers synchronously — spy with a connected list.
        worker, camera, _ = _make_worker([])
        fired: list[bool] = []
        worker._frame_arrived.connect(lambda: fired.append(True))
        worker._on_frame_available()
        assert fired == [True]

    def test_frame_callback_drives_tick(self) -> None:
        # Same-thread emission delivers _frame_arrived -> _tick directly:
        # the bridge alone (no QTimer) processes the new frame.
        worker, camera, _ = _make_worker([_ball()])
        camera.advance()
        worker._on_frame_available()
        assert worker._valid_streak == 1

    def test_start_sets_and_stop_clears_camera_callback(self) -> None:
        worker, camera, _ = _make_worker([])
        worker._running = False           # allow start() to run
        worker.start()
        assert camera.frame_callback == worker._on_frame_available
        worker.stop()
        assert camera.frame_callback is None
        assert camera.closed is True


class TestCopyTrim:
    def test_emitted_camera_view_is_a_copy(self) -> None:
        # The tracker stores a REFERENCE to the worker's reusable frame
        # buffer; the emitted snapshot must own its pixels — mutating the
        # worker buffers after emission must not alter the snapshot.
        snaps: list[ControlSnapshot] = []
        worker, camera, _ = _make_worker([])
        worker.ball_tracker = ViewFakeTracker([_ball()])  # type: ignore[assignment]
        worker.snapshot_ready.connect(snaps.append)
        worker._last_snapshot_emit_perf = -1e9
        camera.frame[:] = 7
        camera.advance()
        worker._tick()
        assert len(snaps) == 1
        view = snaps[0].camera_bgr
        assert isinstance(view, np.ndarray)
        assert np.all(view == 7)
        if worker._frame_buf is not None:
            worker._frame_buf[:] = 250
        if worker._frame_buf_alt is not None:
            worker._frame_buf_alt[:] = 250
        assert np.all(view == 7)          # snapshot unaffected

    def test_frame_buffers_alternate(self) -> None:
        # Two reusable buffers, alternated per tick: the tracker's stored
        # reference from tick N-1 stays intact while tick N runs.
        worker, camera, _ = _make_worker([_ball(), _ball(), _ball()])
        camera.advance()
        worker._tick()
        b1 = worker._frame_buf_alt
        camera.advance()
        worker._tick()
        b2 = worker._frame_buf_alt
        camera.advance()
        worker._tick()
        b3 = worker._frame_buf_alt
        assert b1 is not None and b2 is not None
        assert b1 is not b2
        assert b3 is b1                   # steady state: two buffers cycle

    def test_tick_passes_pre_flipped_frame(self) -> None:
        # The worker reads via read_latest_flipped and passes
        # pre_flipped=True, so the tracker sees the flipped content.
        worker, camera, _ = _make_worker([])
        tracker = ViewFakeTracker([_ball()])
        worker.ball_tracker = tracker  # type: ignore[assignment]
        camera.frame[:, 0] = 200          # asymmetric: first column bright
        camera.advance()
        worker._tick()
        assert tracker.last_frame is not None
        assert np.array_equal(tracker.last_frame, camera.frame[:, ::-1])
