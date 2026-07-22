"""
Unit tests for control/patterns.py — path pattern generators.

Rules under test:
- Every PATTERNS entry is resampled to uniform ~PATH_POINT_SPACING_MM
  arc-length spacing (including the closed wrap gap), stays inside the
  PATH_MAX_RADIUS_MM circle, and is a well-formed (N, 2) float64 array.
- Shape invariants: circle radius exact; heart mirror-symmetric about
  x=0; star radial distance alternates between the outer and inner radii
  (10 local extrema over the loop).
- from_points: oversized points are clamped onto the r=85 circle with a
  logged warning; open paths keep their endpoints; < 2 points is rejected.
"""
import logging

import numpy as np
import pytest

from control.patterns import PATTERNS, Path, circle, from_points, heart, star
from settings import PATH_MAX_RADIUS_MM, PATH_POINT_SPACING_MM


def _gaps(path: Path) -> np.ndarray:
    """Adjacent point-to-point distances, including the wrap gap when closed."""
    pts = path.points
    if path.closed:
        pts = np.vstack([pts, pts[:1]])
    d = np.diff(pts, axis=0)
    return np.asarray(np.hypot(d[:, 0], d[:, 1]))


@pytest.mark.parametrize("label", sorted(PATTERNS))
class TestRegistryInvariants:
    def test_shape_and_dtype(self, label: str) -> None:
        path = PATTERNS[label]()
        pts = path.points
        assert pts.dtype == np.float64
        assert pts.ndim == 2
        assert pts.shape[1] == 2
        assert pts.shape[0] >= 20
        assert path.closed is True
        assert path.name != ""

    def test_all_radii_inside_clamp_circle(self, label: str) -> None:
        pts = PATTERNS[label]().points
        r = np.hypot(pts[:, 0], pts[:, 1])
        assert float(r.max()) <= PATH_MAX_RADIUS_MM + 1e-6

    def test_uniform_spacing_including_wrap(self, label: str) -> None:
        gaps = _gaps(PATTERNS[label]())
        lo = 0.9 * PATH_POINT_SPACING_MM
        hi = 1.1 * PATH_POINT_SPACING_MM
        # Arc-length spacing is exactly uniform; a straight-line gap can only
        # be SHORTER than its arc, and only meaningfully so for the few sample
        # pairs that straddle a sharp corner (square/star vertices, the heart
        # cusps) where the chord cuts the corner. No gap may exceed the
        # spacing, and at least 90% must sit within +/-10% of it.
        assert float(gaps.max()) <= hi
        in_band = (gaps >= lo) & (gaps <= hi)
        assert float(in_band.mean()) >= 0.9


class TestCircle:
    def test_radius_exact(self) -> None:
        pts = circle().points
        r = np.hypot(pts[:, 0], pts[:, 1])
        assert bool(np.all(np.abs(r - 65.0) < 0.1))


class TestHeart:
    def test_symmetric_about_x_equals_zero(self) -> None:
        pts = heart().points
        mirrored = pts * np.array([-1.0, 1.0])
        # For every mirrored point there must be an original point within 1 mm.
        d = np.linalg.norm(pts[None, :, :] - mirrored[:, None, :], axis=-1)
        assert float(d.min(axis=1).max()) < 1.0


class TestStar:
    def test_radial_distance_alternates_with_ten_extrema(self) -> None:
        pts = star().points
        r = np.hypot(pts[:, 0], pts[:, 1])
        prev = np.roll(r, 1)
        nxt = np.roll(r, -1)
        maxima = (r > prev) & (r > nxt)
        minima = (r < prev) & (r < nxt)
        assert int(maxima.sum()) == 5
        assert int(minima.sum()) == 5
        # Maxima cluster at the outer tips, minima at the inner vertices.
        assert float(r[maxima].min()) > 60.0
        assert float(r[minima].max()) < 32.0


class TestFromPoints:
    def test_oversized_point_clamped_onto_circle_with_warning(
        self, caplog: pytest.LogCaptureFixture
    ) -> None:
        with caplog.at_level(logging.WARNING, logger="control.patterns"):
            path = from_points(
                [(0.0, 0.0), (120.0, 0.0)], closed=False, name="oversize"
            )
        warnings = [
            rec for rec in caplog.records if rec.levelno == logging.WARNING
        ]
        assert len(warnings) == 1
        assert "oversize" in warnings[0].getMessage()
        assert "120" in warnings[0].getMessage()  # worst overshoot named
        r = np.hypot(path.points[:, 0], path.points[:, 1])
        assert float(r.max()) == pytest.approx(PATH_MAX_RADIUS_MM, abs=1e-9)

    def test_open_path_keeps_endpoints(self) -> None:
        path = from_points([(-40.0, 0.0), (40.0, 0.0)], closed=False, name="line")
        assert path.closed is False
        np.testing.assert_allclose(path.points[0], [-40.0, 0.0], atol=1e-9)
        np.testing.assert_allclose(path.points[-1], [40.0, 0.0], atol=1e-9)

    def test_rejects_fewer_than_two_points(self) -> None:
        with pytest.raises(ValueError):
            from_points([(1.0, 2.0)])
