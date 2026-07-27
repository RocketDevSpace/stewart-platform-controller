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


class FakeQuad:
    """Minimal stand-in for PlatformQuadTracker's telemetry surface."""

    def __init__(self) -> None:
        self.state = "unlocked"
        self.last_corners_cam: np.ndarray | None = None
        self.last_update_ms = 0.0
        self.last_diag: dict | None = None
        self.acq_progress = (0, 10)
        self.last_audit_px: float | None = None
        self.last_audit_raw_px: float | None = None
        self.audit_strikes = 0


class FakeTracker:
    """Returns a scripted sequence of BallState/None results."""

    def __init__(self, results: list[BallState | None]) -> None:
        self.results = list(results)
        self.hsv_lower = np.array([10, 83, 125], dtype=np.uint8)
        self.hsv_upper = np.array([28, 255, 255], dtype=np.uint8)
        # Boundary-quad telemetry surface (disabled by default so legacy
        # tests exercise the pre-quad timing/snapshot contract).
        self.quad_enabled = False
        self.quad = FakeQuad()
        self.homography_source = "aruco"

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
        self.ki = 0.0
        self.paths: list[Any] = []
        self.path_speeds: list[float] = []
        self.start_path_calls = 0
        self.stop_path_calls = 0
        self.orbit_radii: list[float] = []
        self.orbit_speeds: list[float] = []
        self.start_orbit_calls = 0
        self.stop_orbit_calls = 0

    def compute_with_terms(
        self, ball_state: BallState
    ) -> tuple[float, float, dict]:
        return 1.0, -1.0, {"kp": 0.045, "kd": 0.022}

    def reset_motion_state(self) -> None:
        self.resets += 1

    def set_path(self, path: Any) -> None:
        self.paths.append(path)

    def set_path_speed(self, mm_s: float) -> None:
        self.path_speeds.append(float(mm_s))

    def set_orbit_radius(self, radius_mm: float) -> None:
        self.orbit_radii.append(float(radius_mm))

    def set_orbit_speed(self, mm_s: float) -> None:
        self.orbit_speeds.append(float(mm_s))

    def start_orbit(self) -> bool:
        self.start_orbit_calls += 1
        return True

    def stop_orbit(self) -> None:
        self.stop_orbit_calls += 1

    def start_path(self) -> bool:
        self.start_path_calls += 1
        return True

    def stop_path(self) -> None:
        self.stop_path_calls += 1

    def request_trim_fold(self) -> tuple[float, float]:
        self.trim_fold_calls = getattr(self, "trim_fold_calls", 0) + 1
        return 0.0, 0.0


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


class TestQuadTelemetry:
    """Boundary-quad snapshot fields + the quad_fit timing key."""

    def test_snapshot_carries_source_on_both_branches(self) -> None:
        snaps: list[ControlSnapshot] = []
        worker, camera, _ = _make_worker([_ball(), None])
        worker.snapshot_ready.connect(snaps.append)
        worker._last_snapshot_emit_perf = -1e9
        camera.advance()
        worker._tick()                          # valid frame
        worker.mark_snapshot_consumed()
        worker._last_snapshot_emit_perf = -1e9
        camera.advance()
        worker._tick()                          # miss frame
        assert len(snaps) == 2
        assert snaps[0].homography_source == "aruco"
        assert snaps[1].homography_source == "aruco"   # miss branch too
        assert snaps[0].quad_corners_px is None

    def test_locked_quad_corners_are_an_owned_copy(self) -> None:
        snaps: list[ControlSnapshot] = []
        worker, camera, _ = _make_worker([_ball()])
        tracker = worker.ball_tracker
        assert tracker is not None
        tracker.quad_enabled = True             # type: ignore[attr-defined]
        tracker.homography_source = "quad"      # type: ignore[attr-defined]
        tracker.quad.state = "locked"           # type: ignore[attr-defined]
        corners = np.array(
            [[10.0, 10.0], [600.0, 12.0], [610.0, 400.0], [8.0, 390.0]]
        )
        tracker.quad.last_corners_cam = corners  # type: ignore[attr-defined]
        worker.snapshot_ready.connect(snaps.append)
        worker._last_snapshot_emit_perf = -1e9
        camera.advance()
        worker._tick()
        assert snaps
        got = snaps[0].quad_corners_px
        assert isinstance(got, np.ndarray)
        assert np.array_equal(got, corners)
        assert got is not corners               # crosses threads: owned copy
        assert snaps[0].homography_source == "quad"

    def test_acquisition_diag_propagates_while_not_locked(self) -> None:
        snaps: list[ControlSnapshot] = []
        worker, camera, _ = _make_worker([_ball()])
        tracker = worker.ball_tracker
        assert tracker is not None
        tracker.quad_enabled = True             # type: ignore[attr-defined]
        tracker.quad.state = "acquiring"        # type: ignore[attr-defined]
        tracker.quad.acq_progress = (3, 10)     # type: ignore[attr-defined, misc]
        pred = np.array([[10.0, 10.0], [600.0, 12.0], [610.0, 400.0], [8.0, 390.0]])
        tracker.quad.last_diag = {              # type: ignore[attr-defined]
            "pred_corners": pred,
            "fit_corners": None,
            "gate": None,
            "sides": [
                {"reason": "ok", "usable": 32, "edges": 30, "inliers": 28, "grad": 22.0},
                {"reason": "low-contrast", "usable": 32, "edges": 4, "inliers": 0, "grad": 2.3},
                {"reason": "ok", "usable": 32, "edges": 30, "inliers": 29, "grad": 21.0},
                {"reason": "clipped", "usable": 8, "edges": 0, "inliers": 0, "grad": 0.0},
            ],
        }
        worker.snapshot_ready.connect(snaps.append)
        worker._last_snapshot_emit_perf = -1e9
        camera.advance()
        worker._tick()
        assert snaps
        diag = snaps[0].quad_diag
        assert isinstance(diag, dict)
        assert diag["state"] == "acquiring"
        assert diag["acq_good"] == 3 and diag["acq_frames"] == 10
        assert isinstance(diag["pred_corners"], np.ndarray)
        assert diag["pred_corners"] is not pred          # owned copy
        assert diag["sides"][1]["reason"] == "low-contrast"
        assert snaps[0].quad_corners_px is None          # not locked

    def test_locked_quad_has_no_acq_diag(self) -> None:
        snaps: list[ControlSnapshot] = []
        worker, camera, _ = _make_worker([_ball()])
        tracker = worker.ball_tracker
        assert tracker is not None
        tracker.quad_enabled = True             # type: ignore[attr-defined]
        tracker.quad.state = "locked"           # type: ignore[attr-defined]
        tracker.quad.last_corners_cam = np.zeros((4, 2))  # type: ignore[attr-defined]
        tracker.quad.last_diag = {"sides": []}  # type: ignore[attr-defined]
        worker.snapshot_ready.connect(snaps.append)
        worker._last_snapshot_emit_perf = -1e9
        camera.advance()
        worker._tick()
        # No audit has run yet -> no diag at all; corners present.
        assert snaps[0].quad_diag is None
        assert snaps[0].quad_corners_px is not None

    def test_locked_quad_audit_numbers_propagate(self) -> None:
        snaps: list[ControlSnapshot] = []
        worker, camera, _ = _make_worker([_ball()])
        tracker = worker.ball_tracker
        assert tracker is not None
        tracker.quad_enabled = True             # type: ignore[attr-defined]
        tracker.quad.state = "locked"           # type: ignore[attr-defined]
        tracker.quad.last_corners_cam = np.zeros((4, 2))  # type: ignore[attr-defined]
        tracker.quad.last_audit_px = 1.7        # type: ignore[attr-defined]
        tracker.quad.last_audit_raw_px = 8.9    # type: ignore[attr-defined]
        tracker.quad.audit_strikes = 2          # type: ignore[attr-defined, misc]
        worker.snapshot_ready.connect(snaps.append)
        worker._last_snapshot_emit_perf = -1e9
        camera.advance()
        worker._tick()
        diag = snaps[0].quad_diag
        assert isinstance(diag, dict)
        assert diag["state"] == "locked"
        assert diag["audit_px"] == 1.7
        assert diag["audit_raw_px"] == 8.9
        assert diag["audit_strikes"] == 2

    def test_quad_fit_timing_key_only_when_enabled(self) -> None:
        # Disabled (the legacy default): no key — no fabricated zeros.
        snaps: list[ControlSnapshot] = []
        worker, camera, _ = _make_worker([_ball()])
        worker.snapshot_ready.connect(snaps.append)
        worker._last_snapshot_emit_perf = -1e9
        camera.advance()
        worker._tick()
        assert "quad_fit" not in snaps[0].timings_ms

        # Enabled with a genuine measurement: key present with the value.
        snaps2: list[ControlSnapshot] = []
        worker2, camera2, _ = _make_worker([_ball()])
        tracker2 = worker2.ball_tracker
        assert tracker2 is not None
        tracker2.quad_enabled = True            # type: ignore[attr-defined]
        tracker2.quad.last_update_ms = 0.42     # type: ignore[attr-defined]
        worker2.snapshot_ready.connect(snaps2.append)
        worker2._last_snapshot_emit_perf = -1e9
        camera2.advance()
        worker2._tick()
        assert snaps2[0].timings_ms["quad_fit"] == 0.42


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


class TestPathSlots:
    def test_slots_guard_none_controller(self) -> None:
        worker, _, _ = _make_worker([])
        worker.ball_controller = None
        # Must not raise with no controller (pre-start / post-stop).
        worker.set_path_pattern("Circle (r=65)")
        worker.set_path_following(True)
        worker.set_path_following(False)
        worker.set_path_speed(42.0)

    def test_pattern_and_speed_cached_prestart_applied_at_start(self) -> None:
        worker, camera, controller = _make_worker([])
        worker._running = False               # allow start() to run
        saved = worker.ball_controller
        worker.ball_controller = None         # pre-start: cache only
        worker.set_path_pattern("Circle (r=65)")
        worker.set_path_speed(55.0)
        worker.ball_controller = saved
        worker.start()
        assert len(controller.paths) == 1
        assert controller.paths[0].name.startswith("circle")
        assert controller.path_speeds[-1] == 55.0
        # Following must NEVER auto-start on a fresh session.
        assert controller.start_path_calls == 0
        worker.stop()

    def test_following_never_autostarts_at_start(self) -> None:
        worker, camera, controller = _make_worker([])
        worker._running = False
        worker.start()
        assert controller.start_path_calls == 0
        assert controller.stop_path_calls == 0
        worker.stop()

    def test_following_toggle_forwards_to_controller(self) -> None:
        worker, _, controller = _make_worker([])
        worker.set_path_following(True)
        worker.set_path_following(False)
        assert controller.start_path_calls == 1
        assert controller.stop_path_calls == 1

    def test_live_pattern_and_speed_forward_immediately(self) -> None:
        worker, _, controller = _make_worker([])
        worker.set_path_pattern("Circle (r=65)")
        worker.set_path_speed(20.0)
        assert len(controller.paths) == 1
        assert controller.path_speeds == [20.0]

    def test_unknown_or_empty_label_caches_without_set_path(self) -> None:
        worker, _, controller = _make_worker([])
        worker.set_path_pattern("Not A Pattern")
        worker.set_path_pattern("")
        assert controller.paths == []
        assert worker._path_pattern_init == ""


class TestOrbitSlots:
    def test_slots_guard_none_controller(self) -> None:
        worker, _, _ = _make_worker([])
        worker.ball_controller = None
        worker.set_orbit_enabled(True)
        worker.set_orbit_enabled(False)
        worker.set_orbit_radius(60.0)         # caches only, no raise

    def test_toggle_forwards_to_controller(self) -> None:
        worker, _, controller = _make_worker([])
        worker.set_orbit_enabled(True)
        worker.set_orbit_enabled(False)
        assert controller.start_orbit_calls == 1
        assert controller.stop_orbit_calls == 1

    def test_radius_cached_prestart_applied_at_start(self) -> None:
        worker, camera, controller = _make_worker([])
        worker._running = False
        saved = worker.ball_controller
        worker.ball_controller = None
        worker.set_orbit_radius(60.0)
        worker.ball_controller = saved
        worker.start()
        assert controller.orbit_radii[-1] == 60.0
        # Orbiting must NEVER auto-start on a fresh session.
        assert controller.start_orbit_calls == 0
        worker.stop()

    def test_path_speed_fans_out_to_orbit(self) -> None:
        # The one-slider-feeds-both contract.
        worker, _, controller = _make_worker([])
        worker.set_path_speed(35.0)
        assert controller.path_speeds[-1] == 35.0
        assert controller.orbit_speeds[-1] == 35.0


class TestTrimFoldSlot:
    def test_guards_none_controller(self) -> None:
        worker, _, _ = _make_worker([])
        worker.ball_controller = None
        worker.fold_trim()          # must not raise

    def test_forwards_to_controller(self) -> None:
        worker, _, controller = _make_worker([])
        worker.fold_trim()
        assert getattr(controller, "trim_fold_calls", 0) == 1


class TestKiSlot:
    def test_guards_none_controller_and_caches(self) -> None:
        worker, _, _ = _make_worker([])
        worker.ball_controller = None
        worker.set_ki(0.05)         # must not raise
        assert worker._ki_init == 0.05

    def test_forwards_live(self) -> None:
        worker, _, controller = _make_worker([])
        worker.set_ki(0.041)
        assert controller.ki == 0.041


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
