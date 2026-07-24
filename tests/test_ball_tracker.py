"""
Synthetic-frame tests for the (now camera-free) cv/ball_tracker.py.

Frames are rendered with real ArUco marker bitmaps + an orange disc, so the
FULL pipeline runs: marker detection -> homography -> warp -> HSV mask ->
contour -> mm conversion -> velocity filter. No camera involved.
"""

import cv2
import numpy as np
import pytest

from core.platform_state import BallState
from cv.ball_tracker import BallTracker
from cv.quad_tracker import STATE_LOCKED, STATE_UNLOCKED
from settings import BALL_VEL_FILTER_ALPHA, TRACKER_AB_BETA_MAX

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


# --- Perspective scene (boundary-quad rework, 2026-07-24) ---
# Overhead "mm canvas": dark background, gray platform filling the full
# +/-120 mm square, real ArUco bitmaps at +/-60 mm, orange ball — then
# warped by a KNOWN canvas->camera homography to an oblique 640x480
# camera frame and mirrored (the existing pre-flip convention). Unlike
# _scene (flat warp-geometry, no visible boundary), this renders the
# platform BOUNDARY EDGES, so the quad tracker can acquire on it; the
# flat _scene remains the no-regression guard (quad never acquires
# there, so every legacy test keeps exercising the pure ArUco path).

_PMARGIN = 80            # canvas px of dark background around the platform
_PCANVAS = 480 + 2 * _PMARGIN
_PBG = 40                # background gray value
_PFG = 128               # platform gray value
# Canvas platform corners (TL,TR,BR,BL) -> irregular oblique camera quad.
_PCANVAS_CORNERS = np.array(
    [[_PMARGIN, _PMARGIN], [_PMARGIN + 480, _PMARGIN],
     [_PMARGIN + 480, _PMARGIN + 480], [_PMARGIN, _PMARGIN + 480]],
    dtype=np.float32,
)
_PCAM_QUAD = np.array(
    [[90.0, 40.0], [560.0, 55.0], [585.0, 425.0], [60.0, 405.0]],
    dtype=np.float32,
)
_H_CANVAS_TO_CAM = cv2.getPerspectiveTransform(_PCANVAS_CORNERS, _PCAM_QUAD)


def _persp_scene(
    ball_mm: tuple[float, float] | None,
    cover_markers: tuple[int, ...] = (),
    match_bg_sides: tuple[int, ...] = (),
) -> np.ndarray:
    """Render the oblique-camera platform scene; see the block comment.

    cover_markers: marker ids left platform-gray (painted over / fully
    occluded). match_bg_sides: boundary sides (0=top TL-TR, 1=right,
    2=bottom, 3=left, in canvas orientation) whose OUTSIDE strip is
    painted platform-gray — gray-on-gray, no edge contrast."""
    canvas = np.full((_PCANVAS, _PCANVAS, 3), _PBG, dtype=np.uint8)
    canvas[_PMARGIN:_PMARGIN + 480, _PMARGIN:_PMARGIN + 480] = _PFG

    strips = {
        0: (0, 0, _PCANVAS, _PMARGIN),                  # above the top side
        1: (_PMARGIN + 480, 0, _PCANVAS, _PCANVAS),     # right of the right side
        2: (0, _PMARGIN + 480, _PCANVAS, _PCANVAS),     # below the bottom side
        3: (0, 0, _PMARGIN, _PCANVAS),                  # left of the left side
    }
    for side in match_bg_sides:
        x0, y0, x1, y1 = strips[side]
        canvas[y0:y1, x0:x1] = _PFG

    marker_centers = {0: (360, 120), 1: (120, 120), 2: (120, 360), 3: (360, 360)}
    for mid, (cx, cy) in marker_centers.items():
        if mid in cover_markers:
            continue
        bmp = _marker_bitmap(mid)
        bgr = cv2.cvtColor(bmp, cv2.COLOR_GRAY2BGR)
        half = MARKER_PX // 2
        canvas[_PMARGIN + cy - half:_PMARGIN + cy + half,
               _PMARGIN + cx - half:_PMARGIN + cx + half] = bgr

    if ball_mm is not None:
        px_per_mm = 480 / 240.0
        bx = int(_PMARGIN + 240 + ball_mm[0] * px_per_mm)
        by = int(_PMARGIN + 240 - ball_mm[1] * px_per_mm)
        cv2.circle(canvas, (bx, by), 14, ORANGE_BGR, -1)

    cam = cv2.warpPerspective(canvas, _H_CANVAS_TO_CAM, (640, 480))
    return cv2.flip(cam, 1)


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


class TestPerspectiveSceneBaseline:
    """Pins the _persp_scene builder itself: the ORIGINAL ArUco pipeline
    must track on the oblique perspective scene BEFORE the quad tracker
    builds on it (any later quad failure is then the quad's, not the
    scene's)."""

    def test_ball_center_and_offset_track(self) -> None:
        # Fresh tracker per position: this pins the SCENE GEOMETRY (a
        # 36 mm instant teleport would otherwise trip the glitch veto,
        # which is filter behavior pinned elsewhere).
        for ball_mm in ((0.0, 0.0), (30.0, -20.0)):
            t = _tracker()
            s = t.process(_persp_scene(ball_mm), frame_ts=1.0)
            assert s is not None
            assert s.x_mm == pytest.approx(ball_mm[0], abs=2.5)
            assert s.y_mm == pytest.approx(ball_mm[1], abs=2.5)

    def test_one_covered_marker_parallelogram_completes(self) -> None:
        t = _tracker()
        s = t.process(_persp_scene((0.0, 0.0), cover_markers=(2,)), frame_ts=1.0)
        assert s is not None
        assert s.x_mm == pytest.approx(0.0, abs=3.0)
        assert s.y_mm == pytest.approx(0.0, abs=3.0)

    def test_two_covered_markers_is_todays_limit(self) -> None:
        # The ArUco path needs >= 3 markers, so TWO covered = loss. The
        # quad integration lifts this limit — pinned in the quad E-tests.
        t = _tracker()
        scene = _persp_scene((0.0, 0.0), cover_markers=(1, 2))
        assert t.process(scene, frame_ts=1.0) is None


def _run_to_lock(t: BallTracker, scene: np.ndarray, extra: int = 2) -> BallState | None:
    """Process enough frames of `scene` for the quad tracker to seed,
    acquire, and lock; returns the last BallState."""
    s: BallState | None = None
    for i in range(t.quad.acq_frames + extra):
        s = t.process(scene, frame_ts=1.0 + i / 30)
    return s


class TestQuadPrimarySource:
    """The boundary-quad source ladder end-to-end (E-tests 1/2/3/5):
    quad acquisition on the perspective scene, marker-occlusion immunity,
    the beyond-ArUco 2-markers-covered case, and quad-path H stability."""

    def test_acquires_locks_and_becomes_source(self) -> None:
        for ball_mm in ((0.0, 0.0), (30.0, -20.0)):
            t = _tracker()
            s = _run_to_lock(t, _persp_scene(ball_mm))
            assert t.quad.state == STATE_LOCKED
            assert t.homography_source == "quad"
            assert s is not None
            assert s.x_mm == pytest.approx(ball_mm[0], abs=2.5)
            assert s.y_mm == pytest.approx(ball_mm[1], abs=2.5)

    def test_one_marker_covered_position_unmoved(self) -> None:
        # THE rig failure mode: the ball transits a marker. With the quad
        # as source, fully covering a marker must not move the ball.
        t = _tracker()
        s_clean = _run_to_lock(t, _persp_scene((30.0, -20.0)))
        assert s_clean is not None
        covered = _persp_scene((30.0, -20.0), cover_markers=(0,))
        s_cov = t.process(covered, frame_ts=2.0)
        assert s_cov is not None
        assert t.homography_source == "quad"
        assert s_cov.x_mm == pytest.approx(s_clean.x_mm, abs=1.0)
        assert s_cov.y_mm == pytest.approx(s_clean.y_mm, abs=1.0)

    def test_two_markers_covered_still_tracks(self) -> None:
        # Impossible on the ArUco path (3-marker parallelogram is its
        # floor — pinned in TestPerspectiveSceneBaseline): the quad needs
        # no markers at all once locked. Runs past a cross-check frame;
        # the failed ArUco solve there must not count as a strike.
        t = _tracker()
        s_clean = _run_to_lock(t, _persp_scene((30.0, -20.0)))
        assert s_clean is not None
        covered = _persp_scene((30.0, -20.0), cover_markers=(1, 2))
        s = None
        for i in range(20):
            s = t.process(covered, frame_ts=2.0 + i / 30)
            assert s is not None
            assert t.homography_source == "quad"
        assert t.quad.state == STATE_LOCKED
        assert s is not None
        assert s.x_mm == pytest.approx(s_clean.x_mm, abs=1.5)
        assert s.y_mm == pytest.approx(s_clean.y_mm, abs=1.5)

    def test_quad_path_h_bit_stable_on_identical_frames(self) -> None:
        # Mirrors TestHomographyStability for the quad source: identical
        # frames -> bit-identical H (corner deadband LP + fresh solve).
        t = _tracker()
        scene = _persp_scene((0.0, 0.0))
        _run_to_lock(t, scene)
        internal = cv2.flip(scene, 1)     # what process() sees post-flip
        h1 = t._resolve_homography(internal)
        h2 = t._resolve_homography(internal)
        assert h1 is not None and h2 is not None
        assert t.homography_source == "quad"
        assert np.array_equal(h1, h2)

    def test_flat_scene_never_acquires_pure_aruco_path(self) -> None:
        # The no-regression guard, stated explicitly: the legacy flat
        # _scene has no visible boundary (the platform fills the frame),
        # so the quad cannot acquire and every existing test transparently
        # exercises the unchanged ArUco path.
        t = _tracker()
        scene = _scene((0.0, 0.0))
        for i in range(15):
            assert t.process(scene, frame_ts=1.0 + i / 30) is not None
        assert t.quad.state != STATE_LOCKED
        assert t.homography_source == "aruco"

    def test_quad_disabled_short_circuits_to_aruco(self) -> None:
        t = _tracker()
        t.quad_enabled = False
        s = _run_to_lock(t, _persp_scene((30.0, -20.0)))
        assert s is not None
        assert t.quad.state == STATE_UNLOCKED     # never even seeded
        assert t.homography_source == "aruco"
        assert s.x_mm == pytest.approx(30.0, abs=2.5)


class TestQuadRobustness:
    """E-tests 4/6: the ball ON the boundary (exclusion + trim), and the
    gray-on-gray fail-closed fallback with recovery."""

    def test_ball_sliding_on_boundary_stays_smooth(self) -> None:
        # The ball's silhouette overlaps the top boundary while sliding
        # along it: the exclusion zone + trimmed fit must hold the
        # corners still and the position glitch-free.
        t = _tracker()
        _run_to_lock(t, _persp_scene((-30.0, 118.0)))
        assert t.quad.state == STATE_LOCKED
        assert t.quad.last_corners_cam is not None
        corners0 = t.quad.last_corners_cam.copy()
        prev_raw: tuple[float, float] | None = None
        for i in range(31):
            x = -30.0 + 2.0 * i                     # 2 mm/frame slide
            s = t.process(_persp_scene((x, 118.0)), frame_ts=2.0 + i / 30)
            assert s is not None
            assert t.homography_source == "quad"
            assert s.raw_x_mm is not None and s.raw_y_mm is not None
            if prev_raw is not None:
                jump = float(np.hypot(s.raw_x_mm - prev_raw[0],
                                      s.raw_y_mm - prev_raw[1]))
                assert jump < 4.0                   # commanded 2 mm + noise
            prev_raw = (s.raw_x_mm, s.raw_y_mm)
        assert t.quad.last_corners_cam is not None
        drift = np.hypot(*(t.quad.last_corners_cam - corners0).T)
        assert float(drift.max()) < 1.0

    def test_gray_on_gray_falls_to_aruco_and_relocks(self) -> None:
        # Two boundary sides lose contrast: the quad fails CLOSED, the
        # ladder falls to ArUco (ball still tracked, today's behavior),
        # the lock drops after the miss budget — and restoring contrast
        # relocks within the acquisition window.
        t = _tracker()
        _run_to_lock(t, _persp_scene((0.0, 0.0)))
        flat = _persp_scene((0.0, 0.0), match_bg_sides=(0, 1))
        for i in range(t.quad.max_miss_frames + 2):
            s = t.process(flat, frame_ts=2.0 + i / 30)
            assert s is not None
            assert t.homography_source == "aruco"
        assert t.quad.state != STATE_LOCKED
        s = _run_to_lock(t, _persp_scene((0.0, 0.0)))
        assert s is not None
        assert t.quad.state == STATE_LOCKED
        assert t.homography_source == "quad"


class TestQuadGlitchImmunity:
    """E-test 7: the rig failure mode reproduced and killed. A 60-frame
    sweep slides the ball straight across marker 0, progressively
    occluding it. Both halves are pinned because the synthetic is fully
    deterministic (no RNG): the ArUco-only baseline DOES glitch (this
    pin proves the scene reproduces the rig's >4 mm raw jumps), and the
    quad path shows ZERO >4 mm jumps with the source never leaving
    "quad" — including across cross-check frames, which the
    ball-near-marker guard must suppress (without it, the slow transit
    reads as persistent ArUco disagreement and forces a reacquire FROM
    the glitched H; caught in sim before it reached the rig)."""

    def _sweep_raw_jumps(self, t: BallTracker) -> list[float]:
        jumps: list[float] = []
        prev: tuple[float, float] | None = None
        for i in range(60):
            x = 40.0 + 40.0 * i / 59.0              # straight across marker 0
            s = t.process(_persp_scene((x, 60.0)), frame_ts=2.0 + i / 30)
            assert s is not None
            assert s.raw_x_mm is not None and s.raw_y_mm is not None
            if prev is not None:
                jumps.append(float(np.hypot(s.raw_x_mm - prev[0],
                                            s.raw_y_mm - prev[1])))
            prev = (s.raw_x_mm, s.raw_y_mm)
        return jumps

    def test_aruco_baseline_reproduces_the_glitch(self) -> None:
        t = _tracker()
        t.quad_enabled = False
        assert t.process(_persp_scene((40.0, 60.0)), frame_ts=1.0) is not None
        jumps = self._sweep_raw_jumps(t)
        assert max(jumps) > 4.0                     # the failure exists here

    def test_quad_kills_the_glitch(self) -> None:
        t = _tracker()
        _run_to_lock(t, _persp_scene((40.0, 60.0)))
        assert t.quad.state == STATE_LOCKED
        jumps = self._sweep_raw_jumps(t)
        assert t.homography_source == "quad"
        assert t.quad.state == STATE_LOCKED         # never reacquired
        assert max(jumps) < 2.5                     # commanded 0.68 mm + noise


class TestVelocity:
    def test_first_frame_velocity_zero(self) -> None:
        t = _tracker()
        state = t.process(_scene((0.0, 0.0)), frame_ts=1.0)
        assert state is not None
        assert state.vx_mm_s == 0.0

    def test_velocity_low_pass_legacy_mode(self) -> None:
        # Pins the LEGACY velocity semantics (regression reference); the
        # production default is now mode="alpha_beta".
        t = _tracker()
        t.measurement.mode = "legacy"
        t.process(_scene((0.0, 0.0)), frame_ts=1.0)
        dt = 1.0 / 30.0
        state = t.process(_scene((6.0, 0.0)), frame_ts=1.0 + dt)
        assert state is not None
        # raw vx ~ 6mm / dt = 180 mm/s; first filtered sample = alpha * raw
        expected = BALL_VEL_FILTER_ALPHA * (6.0 / dt)
        assert state.vx_mm_s == pytest.approx(expected, rel=0.25)

    def test_velocity_alpha_beta_second_frame(self) -> None:
        t = _tracker()   # default mode from settings
        assert t.measurement.mode == "alpha_beta"
        t.process(_scene((0.0, 0.0)), frame_ts=1.0)
        dt = 1.0 / 30.0
        state = t.process(_scene((6.0, 0.0)), frame_ts=1.0 + dt)
        assert state is not None
        # innovation ~6 mm >= INNOV_FULL -> beta = BETA_MAX; v = (beta/dt)*r
        expected = (TRACKER_AB_BETA_MAX / dt) * 6.0
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
        t.measurement.mode = "legacy"   # the pos low-pass is a legacy-path feature
        t.pos_filter_enabled = True
        t.process(_scene((0.0, 0.0)), frame_ts=1.0)
        t.process(_scene(None), frame_ts=1.0 + 1 / 30)      # loss
        state = t.process(_scene((30.0, -20.0)), frame_ts=1.0 + 2 / 30)
        assert state is not None
        # With stale prev_ball_mm the filter would drag this toward (0,0).
        assert state.x_mm == pytest.approx(30.0, abs=2.0)
        assert state.y_mm == pytest.approx(-20.0, abs=2.0)

    def test_alpha_beta_reacquire_seeds_at_new_position(self) -> None:
        t = _tracker()   # default mode alpha_beta
        t.process(_scene((0.0, 0.0)), frame_ts=1.0)
        t.process(_scene(None), frame_ts=1.0 + 1 / 30)      # loss -> AB reset
        state = t.process(_scene((30.0, -20.0)), frame_ts=1.0 + 2 / 30)
        assert state is not None
        # Re-seeded at the new position with zero velocity, no stale blend.
        assert state.x_mm == pytest.approx(30.0, abs=2.0)
        assert state.y_mm == pytest.approx(-20.0, abs=2.0)
        assert state.vx_mm_s == 0.0
        assert state.vy_mm_s == 0.0


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


class TestPreFlipped:
    def test_pre_flipped_matches_internal_flip(self) -> None:
        # Handing in an already-mirrored frame with pre_flipped=True must
        # give the same detection as the internal-flip path.
        t1 = _tracker()
        t2 = _tracker()
        scene = _scene((25.0, 15.0))
        s1 = t1.process(scene, frame_ts=1.0)
        s2 = t2.process(cv2.flip(scene, 1), frame_ts=1.0, pre_flipped=True)
        assert s1 is not None and s2 is not None
        assert s2.x_mm == pytest.approx(s1.x_mm, abs=0.5)
        assert s2.y_mm == pytest.approx(s1.y_mm, abs=0.5)

    def test_gain_buffer_reused_across_frames(self) -> None:
        # Software gain > 1.02 routes through the preallocated dst buffer;
        # two frames must reuse the same member buffer (no per-frame alloc).
        t = _tracker()
        scene = _scene((0.0, 0.0))
        t.process(scene, frame_ts=1.0, brightness_gain=1.5)
        buf1 = t._gain_buf
        t.process(scene.copy(), frame_ts=1.0 + 1 / 30, brightness_gain=1.5)
        assert buf1 is not None
        assert t._gain_buf is buf1


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


class TestMarkerCenterFilter:
    """Drives _filter_marker_center directly: deadband freeze, slow-alpha
    convergence, fast-alpha ramp tracking, and the >40 px snap reset."""

    def _seed(self, t: BallTracker) -> np.ndarray:
        return t._filter_marker_center(0, np.array([100.0, 100.0], dtype=np.float32))

    def test_deadband_step_returns_prev_unchanged(self) -> None:
        t = _tracker()
        seeded = self._seed(t)
        out = t._filter_marker_center(0, np.array([100.25, 100.0], dtype=np.float32))
        assert np.array_equal(out, seeded)   # frozen, bit-identical

    def test_small_steps_converge_with_slow_alpha(self) -> None:
        t = _tracker()
        self._seed(t)
        target = np.array([100.6, 100.0], dtype=np.float32)
        out = None
        for _ in range(10):
            out = t._filter_marker_center(0, target)
        assert out is not None
        # Converges to within the deadband of the target, then freezes.
        assert abs(float(out[0]) - 100.6) <= t.aruco_center_deadband_px + 1e-5
        assert float(out[0]) > 100.0

    def test_ramp_lag_below_1_5px_after_3_frames(self) -> None:
        t = _tracker()
        self._seed(t)
        out = None
        for k in range(1, 4):   # 3 px/frame ramp
            cur = np.array([100.0 + 3.0 * k, 100.0], dtype=np.float32)
            out = t._filter_marker_center(0, cur)
        assert out is not None
        assert abs(float(out[0]) - 109.0) < 1.5

    def test_41px_jump_snaps_to_current(self) -> None:
        t = _tracker()
        self._seed(t)
        out = t._filter_marker_center(0, np.array([141.0, 100.0], dtype=np.float32))
        assert float(out[0]) == pytest.approx(141.0)
        assert float(out[1]) == pytest.approx(100.0)


class TestHomographyStability:
    def test_same_scene_twice_yields_identical_h(self) -> None:
        # At rest the marker-center deadband freezes the filtered centers,
        # so the per-frame getPerspectiveTransform solve is deterministic.
        t = _tracker()
        scene = _scene((0.0, 0.0))
        assert t.process(scene, frame_ts=1.0) is not None
        assert t._last_H is not None
        h1 = t._last_H.copy()
        assert t.process(scene.copy(), frame_ts=1.0 + 1 / 30) is not None
        h2 = t._last_H
        assert h2 is not None
        assert np.array_equal(h1, h2)


class TestSubpixelPipeline:
    def test_detection_works_with_both_subpix_flags_on(self) -> None:
        # Accuracy claims need real imagery; this pins "no crash + the mm
        # mapping stays within the existing tolerances" with the detector
        # SUBPIX refinement AND the full-res cornerSubPix pass enabled.
        t = _tracker()
        assert t.aruco_fullres_subpix is True
        assert t.aruco_params.cornerRefinementMethod == cv2.aruco.CORNER_REFINE_SUBPIX
        state = t.process(_scene((25.0, 15.0)), frame_ts=1.0)
        assert state is not None
        assert state.x_mm == pytest.approx(25.0, abs=2.0)
        assert state.y_mm == pytest.approx(15.0, abs=2.0)

    def test_find_ball_center_returns_floats_when_subpixel(self) -> None:
        t = _tracker()
        assert t.ball_subpixel is True
        mask = np.zeros((WARP, WARP), dtype=np.uint8)
        cv2.circle(mask, (300, 200), 14, 255, -1)
        found = t.find_ball_center(mask)
        assert found is not None
        (x, y), radius = found
        assert isinstance(x, float) and isinstance(y, float)
        assert isinstance(radius, float)

    def test_find_ball_center_int_casts_when_subpixel_disabled(self) -> None:
        t = _tracker()
        t.ball_subpixel = False
        mask = np.zeros((WARP, WARP), dtype=np.uint8)
        cv2.circle(mask, (300, 200), 14, 255, -1)
        found = t.find_ball_center(mask)
        assert found is not None
        (x, y), radius = found
        assert x == int(x) and y == int(y) and radius == int(radius)
