"""
Synthetic-frame tests for the (now camera-free) cv/ball_tracker.py.

Frames are rendered with real ArUco marker bitmaps + an orange disc, so the
FULL pipeline runs: marker detection -> homography -> warp -> HSV mask ->
contour -> mm conversion -> velocity filter. No camera involved.
"""

import cv2
import numpy as np
import pytest

from cv.ball_tracker import BallTracker
from settings import BALL_VEL_FILTER_ALPHA

WARP = 480          # warp_size_px used in tests (matches production setting)
MARKER_PX = 60
ORANGE_BGR = (0, 128, 255)   # HSV ~ (15, 255, 255) — inside the default gate


def _marker_bitmap(marker_id: int) -> np.ndarray:
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    return cv2.aruco.generateImageMarker(d, marker_id, MARKER_PX)


def _scene(
    ball_mm: tuple[float, float] | None,
    include_markers: bool = True,
    skip_marker: int | None = None,
) -> np.ndarray:
    """Render the platform scene in warp-space geometry, then mirror it so
    BallTracker.process()'s internal flip restores it exactly."""
    canvas = np.full((WARP, WARP, 3), 255, dtype=np.uint8)

    marker_centers = {0: (360, 120), 1: (120, 120), 2: (120, 360), 3: (360, 360)}
    if include_markers:
        for mid, (cx, cy) in marker_centers.items():
            if mid == skip_marker:
                continue
            bmp = _marker_bitmap(mid)
            bgr = cv2.cvtColor(bmp, cv2.COLOR_GRAY2BGR)
            half = MARKER_PX // 2
            canvas[cy - half:cy + half, cx - half:cx + half] = bgr

    if ball_mm is not None:
        px_per_mm = WARP / 240.0
        bx = int(WARP // 2 + ball_mm[0] * px_per_mm)
        by = int(WARP // 2 - ball_mm[1] * px_per_mm)
        cv2.circle(canvas, (bx, by), 14, ORANGE_BGR, -1)

    return cv2.flip(canvas, 1)


def _tracker() -> BallTracker:
    return BallTracker(platform_size_mm=240.0, warp_size_px=WARP)


class TestDetection:
    def test_ball_at_center_maps_to_origin(self) -> None:
        t = _tracker()
        state = t.process(_scene((0.0, 0.0)), frame_ts=1.0)
        assert state is not None
        assert state.x_mm == pytest.approx(0.0, abs=2.0)
        assert state.y_mm == pytest.approx(0.0, abs=2.0)

    def test_ball_offset_maps_to_mm(self) -> None:
        t = _tracker()
        state = t.process(_scene((30.0, -20.0)), frame_ts=1.0)
        assert state is not None
        assert state.x_mm == pytest.approx(30.0, abs=2.0)
        assert state.y_mm == pytest.approx(-20.0, abs=2.0)

    def test_no_ball_returns_none(self) -> None:
        t = _tracker()
        assert t.process(_scene(None), frame_ts=1.0) is None

    def test_three_marker_parallelogram_completion(self) -> None:
        t = _tracker()
        state = t.process(_scene((0.0, 0.0), skip_marker=2), frame_ts=1.0)
        assert state is not None
        assert state.x_mm == pytest.approx(0.0, abs=3.0)

    def test_no_markers_ever_returns_none(self) -> None:
        t = _tracker()
        assert t.process(_scene((0.0, 0.0), include_markers=False), 1.0) is None


class TestVelocity:
    def test_first_frame_velocity_zero(self) -> None:
        t = _tracker()
        state = t.process(_scene((0.0, 0.0)), frame_ts=1.0)
        assert state is not None
        assert state.vx_mm_s == 0.0

    def test_velocity_low_pass(self) -> None:
        t = _tracker()
        t.process(_scene((0.0, 0.0)), frame_ts=1.0)
        dt = 1.0 / 30.0
        state = t.process(_scene((6.0, 0.0)), frame_ts=1.0 + dt)
        assert state is not None
        # raw vx ~ 6mm / dt = 180 mm/s; first filtered sample = alpha * raw
        expected = BALL_VEL_FILTER_ALPHA * (6.0 / dt)
        assert state.vx_mm_s == pytest.approx(expected, rel=0.25)

    def test_velocity_resets_on_loss(self) -> None:
        t = _tracker()
        t.process(_scene((0.0, 0.0)), frame_ts=1.0)
        t.process(_scene((6.0, 0.0)), frame_ts=1.0 + 1 / 30)
        t.process(_scene(None), frame_ts=1.0 + 2 / 30)      # loss
        state = t.process(_scene((10.0, 0.0)), frame_ts=1.0 + 3 / 30)
        assert state is not None
        assert t._vx_f == pytest.approx(state.vx_mm_s)


class TestPositionFilterReset:
    def test_reacquired_ball_not_blended_toward_stale_position(self) -> None:
        t = _tracker()
        t.pos_filter_enabled = True
        t.process(_scene((0.0, 0.0)), frame_ts=1.0)
        t.process(_scene(None), frame_ts=1.0 + 1 / 30)      # loss
        state = t.process(_scene((30.0, -20.0)), frame_ts=1.0 + 2 / 30)
        assert state is not None
        # With stale prev_ball_mm the filter would drag this toward (0,0).
        assert state.x_mm == pytest.approx(30.0, abs=2.0)
        assert state.y_mm == pytest.approx(-20.0, abs=2.0)


class TestStaleHomographyBound:
    def test_marker_loss_serves_cached_h_boundedly_then_reports_loss(
        self,
    ) -> None:
        t = _tracker()
        t.aruco_redetect_every_n = 1   # attempt detection every frame
        assert t.process(_scene((0.0, 0.0)), frame_ts=1.0) is not None

        lost_scene = _scene((0.0, 0.0), include_markers=False)
        served = 0
        result: object = True
        for i in range(t.max_aruco_hold_frames + 2):
            result = t.process(lost_scene, frame_ts=2.0 + i)
            if result is not None:
                served += 1
        # Bounded staleness: at most the hold budget, then loss.
        assert served == t.max_aruco_hold_frames
        assert result is None
        assert t._last_H is None       # cache invalidated, not stale forever

    def test_markers_return_after_loss_recovers(self) -> None:
        t = _tracker()
        t.aruco_redetect_every_n = 1
        t.process(_scene((0.0, 0.0)), frame_ts=1.0)
        lost = _scene((0.0, 0.0), include_markers=False)
        for i in range(t.max_aruco_hold_frames + 2):
            t.process(lost, frame_ts=2.0 + i)
        state = t.process(_scene((10.0, 10.0)), frame_ts=20.0)
        assert state is not None
        assert state.x_mm == pytest.approx(10.0, abs=2.0)


class TestFindBallCenter:
    def test_small_blob_rejected(self) -> None:
        t = _tracker()
        mask = np.zeros((WARP, WARP), dtype=np.uint8)
        cv2.circle(mask, (240, 240), 3, 255, -1)   # tiny: area < min, r < min
        assert t.find_ball_center(mask) is None

    def test_adequate_disc_accepted(self) -> None:
        t = _tracker()
        mask = np.zeros((WARP, WARP), dtype=np.uint8)
        cv2.circle(mask, (300, 200), 14, 255, -1)
        found = t.find_ball_center(mask)
        assert found is not None
        (x, y), radius = found
        assert (x, y) == pytest.approx((300, 200), abs=2)
        assert radius >= 12

    def test_empty_mask_rejected(self) -> None:
        t = _tracker()
        mask = np.zeros((WARP, WARP), dtype=np.uint8)
        assert t.find_ball_center(mask) is None


class TestMmToWarpPx:
    def test_center(self) -> None:
        t = _tracker()
        assert t.mm_to_warp_px(0.0, 0.0) == [WARP // 2, WARP // 2]

    def test_corners(self) -> None:
        t = _tracker()
        px = t.mm_to_warp_px(60.0, 60.0)
        assert px == [360.0, 120.0]
