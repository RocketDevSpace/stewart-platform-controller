"""
Unit tests for cv/quad_tracker.py: sub-pixel edge extraction, trimmed
total-least-squares line fits, corner intersection, and the
PlatformQuadTracker acquire -> lock -> track state machine on synthetic
boundary frames (a bright quad on a dark background, Gaussian-blurred
for realistic soft edges). Integration with BallTracker's source ladder
is covered separately in tests/test_ball_tracker.py.
"""

import cv2
import numpy as np

from cv.quad_tracker import (
    STATE_ACQUIRING,
    STATE_LOCKED,
    STATE_UNLOCKED,
    PlatformQuadTracker,
    edge_offsets,
    fit_line_tls,
    intersect_lines,
)

WARP = 480
WARP_CORNERS = np.array(
    [[0.0, 0.0], [WARP, 0.0], [WARP, WARP], [0.0, WARP]], dtype=np.float32
)

# An irregular convex quad, mimicking the oblique-camera perspective.
# Integer coordinates so fillConvexPoly rasterizes it exactly.
CORNERS_TRUE = np.array(
    [[110.0, 70.0], [530.0, 90.0], [550.0, 410.0], [90.0, 390.0]],
    dtype=np.float64,
)
H_TRUE = cv2.getPerspectiveTransform(CORNERS_TRUE.astype(np.float32), WARP_CORNERS)


def _quad_frame(
    corners: np.ndarray = CORNERS_TRUE,
    bg: int = 40,
    fg: int = 128,
    blur_sigma: float = 1.2,
    ball_px: tuple[float, float] | None = None,
    ball_r: int = 25,
) -> np.ndarray:
    """640x480 gray frame: filled platform quad on a flat background."""
    img = np.full((480, 640), bg, dtype=np.uint8)
    cv2.fillConvexPoly(img, np.round(corners).astype(np.int32), int(fg))
    if ball_px is not None:
        cv2.circle(img, (int(ball_px[0]), int(ball_px[1])), int(ball_r), 200, -1)
    blurred: np.ndarray = cv2.GaussianBlur(img, (0, 0), float(blur_sigma))
    return blurred


def _locked_tracker(
    gray: np.ndarray | None = None, h: np.ndarray = H_TRUE
) -> PlatformQuadTracker:
    qt = PlatformQuadTracker(WARP)
    if gray is None:
        gray = _quad_frame()
    for _ in range(qt.acq_frames):
        qt.seed_from_h(h, gray)
    assert qt.state == STATE_LOCKED
    return qt


def _soft_step(
    taps: int, pos: float, lo: float = 40.0, hi: float = 128.0, width: float = 0.9
) -> np.ndarray:
    """Logistic intensity step centered at tap position `pos`."""
    j = np.arange(taps, dtype=np.float64)
    return lo + (hi - lo) / (1.0 + np.exp(-(j - pos) / width))


class TestEdgeOffsets:
    def test_subpixel_accuracy_below_0p15_px(self) -> None:
        taps, band = 11, 5
        for frac in (-0.4, -0.2, 0.0, 0.2, 0.4):
            prof = _soft_step(taps, band + frac)[None, :]
            off, _, ok = edge_offsets(prof, +1.0, 6.0)
            assert ok[0]
            assert abs(off[0] - frac) < 0.15, f"frac={frac}: got {off[0]}"

    def test_polarity_mismatch_rejected(self) -> None:
        # Rising edge scored with falling polarity: peak score is
        # negative, far below the contrast floor.
        prof = _soft_step(11, 5.0)[None, :]
        _, _, ok = edge_offsets(prof, -1.0, 6.0)
        assert not ok[0]

    def test_falling_edge_with_negative_polarity(self) -> None:
        prof = _soft_step(11, 5.2, lo=128.0, hi=40.0)[None, :]
        off, grad, ok = edge_offsets(prof, -1.0, 6.0)
        assert ok[0]
        assert abs(off[0] - 0.2) < 0.15
        assert grad[0] < 0.0

    def test_unknown_polarity_finds_edge_and_reports_sign(self) -> None:
        # Acquisition mode (polarity 0): |gradient| scoring finds the
        # edge either way; the signed gradient is what polarity
        # learning accumulates.
        rising = _soft_step(11, 5.0)[None, :]
        falling = _soft_step(11, 5.0, lo=128.0, hi=40.0)[None, :]
        _, g_r, ok_r = edge_offsets(rising, 0.0, 6.0)
        _, g_f, ok_f = edge_offsets(falling, 0.0, 6.0)
        assert ok_r[0] and ok_f[0]
        assert g_r[0] > 0.0 > g_f[0]

    def test_low_contrast_fails_closed(self) -> None:
        # Gray-on-gray: a 2-count step is far under the 6.0/px floor.
        prof = _soft_step(11, 5.0, lo=126.0, hi=128.0)[None, :]
        _, _, ok = edge_offsets(prof, 0.0, 6.0)
        assert not ok[0]


class TestFitLineTLS:
    def test_clean_fit_recovers_line(self) -> None:
        rng = np.random.default_rng(3)
        x = np.linspace(0.0, 60.0, 30)
        y = 3.7 + rng.normal(0.0, 0.05, 30)
        fit = fit_line_tls(np.stack([x, y], axis=1), np.array([0.0, 1.0]), 0.75)
        assert fit is not None
        n_vec, c, inliers = fit
        assert inliers == 30
        assert abs(n_vec[1]) > 0.999          # normal ~ (0, 1)
        assert abs(c - 3.7) < 0.05

    def test_trim_rejects_20pct_gross_outliers(self) -> None:
        # 6 of 30 points thrown 8 px off the line (a ball silhouette
        # overlapping the band): the median-centered MAD trim drops
        # exactly the outliers and the refit recovers the true line.
        rng = np.random.default_rng(7)
        x = np.linspace(0.0, 60.0, 30)
        y = 3.7 + rng.normal(0.0, 0.05, 30)
        pts = np.stack([x, y], axis=1)
        pts[::5, 1] += 8.0
        fit = fit_line_tls(pts, np.array([0.0, 1.0]), 0.75)
        assert fit is not None
        n_vec, c, inliers = fit
        assert inliers == 24
        assert abs(n_vec[1]) > 0.999
        assert abs(c - 3.7) < 0.1

    def test_outward_hint_orients_normal(self) -> None:
        pts = np.stack([np.linspace(0, 10, 12), np.full(12, 5.0)], axis=1)
        fit_up = fit_line_tls(pts, np.array([0.0, 1.0]), 0.75)
        fit_down = fit_line_tls(pts, np.array([0.0, -1.0]), 0.75)
        assert fit_up is not None and fit_down is not None
        assert fit_up[0][1] > 0.999 and fit_up[1] > 0
        assert fit_down[0][1] < -0.999 and fit_down[1] < 0

    def test_degenerate_input_returns_none(self) -> None:
        pts = np.tile([[3.0, 4.0]], (10, 1))     # all coincident
        assert fit_line_tls(pts, np.array([0.0, 1.0]), 0.75) is None
        assert fit_line_tls(pts[:1], np.array([0.0, 1.0]), 0.75) is None


class TestIntersectLines:
    def test_perpendicular_lines_intersect_exactly(self) -> None:
        p = intersect_lines(
            np.array([1.0, 0.0]), 100.0, np.array([0.0, 1.0]), 200.0
        )
        assert p is not None
        assert np.allclose(p, [100.0, 200.0])

    def test_near_parallel_rejected(self) -> None:
        a = np.radians(10.0)                     # sin 10 deg < the 20 deg floor
        p = intersect_lines(
            np.array([1.0, 0.0]), 100.0,
            np.array([np.cos(a), np.sin(a)]), 120.0,
        )
        assert p is None

    def test_25_degrees_accepted(self) -> None:
        a = np.radians(25.0)
        p = intersect_lines(
            np.array([1.0, 0.0]), 100.0,
            np.array([np.cos(a), np.sin(a)]), 120.0,
        )
        assert p is not None


class TestAcquisition:
    def test_unlocked_update_returns_none(self) -> None:
        qt = PlatformQuadTracker(WARP)
        assert qt.state == STATE_UNLOCKED
        assert qt.update(_quad_frame()) is None

    def test_seed_acquire_lock(self) -> None:
        qt = PlatformQuadTracker(WARP)
        gray = _quad_frame()
        qt.seed_from_h(H_TRUE, gray)
        assert qt.state == STATE_ACQUIRING
        for _ in range(qt.acq_frames - 1):
            qt.seed_from_h(H_TRUE, gray)
        assert qt.state == STATE_LOCKED
        # Bright platform on dark background: intensity FALLS outward,
        # so every side's learned polarity is negative.
        assert np.all(qt._polarity == -1.0)

    def test_acquisition_failure_resets_consecutive_count(self) -> None:
        qt = PlatformQuadTracker(WARP)
        gray = _quad_frame()
        blank = np.full((480, 640), 40, dtype=np.uint8)
        for _ in range(qt.acq_frames - 1):
            qt.seed_from_h(H_TRUE, gray)
        qt.seed_from_h(H_TRUE, blank)            # no edges: streak broken
        assert qt.state == STATE_ACQUIRING
        for _ in range(qt.acq_frames - 1):
            qt.seed_from_h(H_TRUE, gray)
        assert qt.state == STATE_ACQUIRING       # 9 consecutive: not yet
        qt.seed_from_h(H_TRUE, gray)
        assert qt.state == STATE_LOCKED

    def test_gray_on_gray_never_locks(self) -> None:
        # Contrast floor fails CLOSED: the caller stays on ArUco.
        qt = PlatformQuadTracker(WARP)
        gray = _quad_frame(fg=44)                # 4-count edge << floor
        for _ in range(qt.acq_frames + 5):
            qt.seed_from_h(H_TRUE, gray)
        assert qt.state == STATE_ACQUIRING
        assert qt.update(gray) is None


class TestLockedTracking:
    def test_h_matches_seed_h_within_1px(self) -> None:
        # The offset calibration maps silhouette fits into the ArUco
        # (seed) frame, so on a static scene the quad H and the seed H
        # agree to sub-pixel for platform-interior points.
        qt = _locked_tracker()
        h = qt.update(_quad_frame())
        assert h is not None
        probes = np.array(
            [[[320.0, 240.0]], [[200.0, 150.0]], [[450.0, 350.0]]],
            dtype=np.float64,
        )
        via_quad = cv2.perspectiveTransform(probes, h)
        via_seed = cv2.perspectiveTransform(probes, H_TRUE.astype(np.float64))
        err = np.hypot(*(via_quad - via_seed).reshape(-1, 2).T)
        assert float(err.max()) < 1.0

    def test_h_bit_stable_on_identical_frames(self) -> None:
        # Same contract the ArUco path pins in TestHomographyStability:
        # identical frames -> bit-identical H (corner deadband LP).
        qt = _locked_tracker()
        gray = _quad_frame()
        h1 = qt.update(gray)
        h2 = qt.update(gray)
        assert h1 is not None and h2 is not None
        assert np.array_equal(h1, h2)

    def test_tracks_integer_shift(self) -> None:
        qt = _locked_tracker()
        assert qt.last_corners_cam is not None
        before = qt.last_corners_cam.copy()
        shifted = _quad_frame(CORNERS_TRUE + np.array([3.0, 2.0]))
        for _ in range(5):
            assert qt.update(shifted) is not None
        assert qt.last_corners_cam is not None
        moved = qt.last_corners_cam - before
        assert np.allclose(moved, [3.0, 2.0], atol=1.0)

    def test_miss_streak_drops_to_unlocked(self) -> None:
        qt = _locked_tracker()
        blank = np.full((480, 640), 40, dtype=np.uint8)
        for _ in range(qt.max_miss_frames):
            assert qt.update(blank) is None
        assert qt.state == STATE_LOCKED          # within the hold budget
        assert qt.update(blank) is None
        assert qt.state == STATE_UNLOCKED        # budget exceeded

    def test_ball_on_boundary_with_exclusion_stays_put(self) -> None:
        # A ball silhouette overlapping side 0's midpoint: with the
        # exclusion zone the fit ignores the contaminated samples and
        # the corners stay put.
        qt = _locked_tracker()
        assert qt.last_corners_cam is not None
        clean_corners = qt.last_corners_cam.copy()
        ball_at = (320.0, 80.0)                  # on side 0 (TL -> TR)
        dirty = _quad_frame(ball_px=ball_at)
        h = qt.update(dirty, prev_ball_px=ball_at)
        assert h is not None
        assert qt.last_corners_cam is not None
        drift = np.hypot(*(qt.last_corners_cam - clean_corners).T)
        assert float(drift.max()) < 1.0


class TestAcquisitionDiagnostics:
    """last_diag: the rig-visibility readout (why is a side failing?)."""

    def test_low_contrast_sides_report_reason_and_measured_grad(self) -> None:
        qt = PlatformQuadTracker(WARP)
        gray = _quad_frame(fg=44)                # 4-count edge << the 6.0 floor
        qt.seed_from_h(H_TRUE, gray)
        assert qt.last_diag is not None
        sides = qt.last_diag["sides"]
        assert len(sides) == 4
        for d in sides:
            assert d["reason"] == "low-contrast"
            # The measured contrast is reported so min_grad can be tuned
            # against reality; it must be a real (nonzero) measurement
            # under the floor.
            assert 0.0 < d["grad"] < qt.min_grad
        assert qt.last_diag["fit_corners"] is None

    def test_clipped_side_reported_when_prediction_leaves_frame(self) -> None:
        # Shift the predicted quad so one side falls off the image edge:
        # that side must report "clipped", not "low-contrast".
        shifted = CORNERS_TRUE + np.array([0.0, -120.0])   # top side above frame
        h_off = cv2.getPerspectiveTransform(
            shifted.astype(np.float32), WARP_CORNERS
        )
        qt = PlatformQuadTracker(WARP)
        qt.seed_from_h(h_off, _quad_frame())
        assert qt.last_diag is not None
        assert qt.last_diag["sides"][0]["reason"] == "clipped"

    def test_successful_fit_reports_all_ok_with_corners(self) -> None:
        qt = PlatformQuadTracker(WARP)
        qt.seed_from_h(H_TRUE, _quad_frame())
        assert qt.last_diag is not None
        assert all(d["reason"] == "ok" for d in qt.last_diag["sides"])
        assert all(d["inliers"] >= qt.min_inliers for d in qt.last_diag["sides"])
        fit_corners = qt.last_diag["fit_corners"]
        assert fit_corners is not None
        assert np.allclose(fit_corners, CORNERS_TRUE, atol=2.0)

    def test_acq_progress_counts_consecutive_goods(self) -> None:
        qt = PlatformQuadTracker(WARP)
        gray = _quad_frame()
        assert qt.acq_progress == (0, qt.acq_frames)
        qt.seed_from_h(H_TRUE, gray)
        qt.seed_from_h(H_TRUE, gray)
        assert qt.acq_progress == (2, qt.acq_frames)


class TestCrossCheck:
    def _h_shifted(self, dx: float) -> np.ndarray:
        c = (CORNERS_TRUE + np.array([dx, 0.0])).astype(np.float32)
        result: np.ndarray = cv2.getPerspectiveTransform(c, WARP_CORNERS)
        return result

    def test_agreement_passes_and_clears_streak(self) -> None:
        qt = _locked_tracker()
        assert qt.notify_aruco_h(H_TRUE) is True
        assert qt.notify_aruco_h(self._h_shifted(8.0)) is False   # strike 1
        assert qt.state == STATE_LOCKED          # transient: quad preferred
        assert qt.notify_aruco_h(H_TRUE) is True                  # clears
        assert qt.notify_aruco_h(self._h_shifted(8.0)) is False
        assert qt.notify_aruco_h(self._h_shifted(8.0)) is False
        assert qt.state == STATE_LOCKED          # 2 consecutive: still held

    def test_persistent_disagreement_forces_reacquire(self) -> None:
        qt = _locked_tracker()
        for _ in range(qt.crosscheck_fails_to_reacq):
            qt.notify_aruco_h(self._h_shifted(8.0))
        assert qt.state == STATE_UNLOCKED

    def test_not_applicable_while_unlocked(self) -> None:
        qt = PlatformQuadTracker(WARP)
        assert qt.notify_aruco_h(H_TRUE) is True
