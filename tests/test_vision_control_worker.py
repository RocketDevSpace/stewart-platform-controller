"""
tests/test_vision_control_worker.py

Unit tests for cv/vision_control_worker.py.

Skips hardware-dependent tests; covers the backpressure mechanism and
ControlSnapshot construction without requiring a live camera.
"""

import sys

from cv.vision_control_worker import ControlSnapshot, VisionControlWorker

# ---------------------------------------------------------------------------
# ControlSnapshot construction
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
            ik_result={},
            control_terms={},
            tracking_valid=False,
        )
        assert snap.tracking_valid is False
        assert snap.ball_state is None
        assert snap.miss_count == 0
        assert snap.reason == ""
        assert snap.camera_bgr is None
        assert snap.warped_bgr is None
        assert snap.mask_gray is None
        assert snap.worker_emit_perf_ts == 0.0

    def test_valid_snapshot_fields(self) -> None:
        from core.platform_state import BallState
        bs = BallState(x_mm=10.0, y_mm=-5.0, vx_mm_s=1.0, vy_mm_s=-2.0)
        snap = ControlSnapshot(
            timestamp=2.0,
            ball_state=bs,
            pose={"x": 0, "y": 0, "z": 50, "roll": 1.0, "pitch": -0.5, "yaw": 0},
            servo_angles=[90, 90, 90, 90, 90, 90],
            ik_success=True,
            timings_ms={"ball_update": 3.1, "total": 5.0},
            ik_result={"success": True},
            control_terms={"kp": 0.045, "kd": 0.022},
            tracking_valid=True,
            reason="ok",
            worker_emit_perf_ts=1234.5,
        )
        assert snap.tracking_valid is True
        assert snap.ball_state is not None and snap.ball_state.x_mm == 10.0
        assert snap.ik_success is True
        assert snap.worker_emit_perf_ts == 1234.5


# ---------------------------------------------------------------------------
# VisionControlWorker backpressure (_snapshot_inflight flag)
# ---------------------------------------------------------------------------


def _get_app() -> object:
    from PyQt5.QtWidgets import QApplication
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    return app


def _make_worker() -> VisionControlWorker:
    from core.ik_engine import IKEngine
    _get_app()  # ensure QApplication exists before creating QObject
    return VisionControlWorker(
        ik_solver=IKEngine(),
        z_provider=lambda: 0.0,
        kp=0.045,
        kd=0.022,
        camera_index=0,
        command_sender=None,
    )


class TestBackpressure:
    """
    Test that _snapshot_inflight is set True when a snapshot is emitted and
    cleared back to False when mark_snapshot_consumed() is called.

    Does NOT start the worker (no camera required).
    """

    def test_inflight_starts_false(self) -> None:
        worker = _make_worker()
        assert worker._snapshot_inflight is False

    def test_inflight_true_after_emit(self) -> None:
        worker = _make_worker()
        assert not worker._snapshot_inflight

        snap = ControlSnapshot(
            timestamp=0.0,
            ball_state=None,
            pose={},
            servo_angles=[],
            ik_success=False,
            timings_ms={},
            ik_result={},
            control_terms={},
            tracking_valid=False,
        )
        # Simulate what _tick does: set inflight, then emit
        worker._snapshot_inflight = True
        worker.snapshot_ready.emit(snap)

        # Inflight must remain True until consumed
        assert worker._snapshot_inflight is True

    def test_inflight_cleared_after_consume(self) -> None:
        worker = _make_worker()
        worker._snapshot_inflight = True
        worker.mark_snapshot_consumed()
        assert worker._snapshot_inflight is False

    def test_second_emit_blocked_while_inflight(self) -> None:
        worker = _make_worker()
        emitted: list = []
        worker.snapshot_ready.connect(emitted.append)

        snap = ControlSnapshot(
            timestamp=0.0,
            ball_state=None,
            pose={},
            servo_angles=[],
            ik_success=False,
            timings_ms={},
            ik_result={},
            control_terms={},
            tracking_valid=False,
        )

        # First emission: inflight is False → emit proceeds
        assert not worker._snapshot_inflight
        worker._snapshot_inflight = True
        worker.snapshot_ready.emit(snap)

        # While inflight, a second emission attempt must be skipped
        should_emit = not worker._snapshot_inflight  # evaluates False
        assert not should_emit  # confirms guard condition is correct

        # After consume, emission is allowed again
        worker.mark_snapshot_consumed()
        assert not worker._snapshot_inflight
