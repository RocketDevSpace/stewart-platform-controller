"""
control/patterns.py

Path patterns for ball path following — pure geometry. Each generator
returns a Path of (N, 2) float64 points in platform mm coordinates,
radially clamped to PATH_MAX_RADIUS_MM and resampled to uniform
PATH_POINT_SPACING_MM arc-length spacing through one shared pipeline
(generate samples -> clamp -> resample).

No Qt, no hardware, no IK — playback is control/path_follower.py's job
(the PATTERNS registry mirrors routines/ROUTINES).
"""
import logging
import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass

import numpy as np

from settings import PATH_MAX_RADIUS_MM, PATH_POINT_SPACING_MM

_log = logging.getLogger(__name__)


@dataclass(frozen=True)
class Path:
    """A follow-able path in platform coordinates.

    points: (N, 2) float64, mm, uniform arc-length spacing
            (PATH_POINT_SPACING_MM along the polyline).
    closed: last point connects back to the first (wrap segment).
    name:   human-readable identifier (logging / telemetry).
    """

    points: np.ndarray
    closed: bool
    name: str


def _clamp_to_radius(points: np.ndarray, name: str) -> np.ndarray:
    """Scale any point outside PATH_MAX_RADIUS_MM radially onto the circle."""
    pts = np.asarray(points, dtype=np.float64)
    r = np.hypot(pts[:, 0], pts[:, 1])
    over = r > PATH_MAX_RADIUS_MM
    if not bool(np.any(over)):
        return pts
    _log.warning(
        "Path %r: %d point(s) outside r=%.1f mm (worst %.1f mm) clamped radially",
        name,
        int(np.count_nonzero(over)),
        PATH_MAX_RADIUS_MM,
        float(r[over].max()),
    )
    out = pts.copy()
    out[over] *= (PATH_MAX_RADIUS_MM / r[over])[:, None]
    return out


def _resample(points: np.ndarray, closed: bool, spacing_mm: float) -> np.ndarray:
    """Resample a polyline to uniform arc-length spacing.

    Closed paths get the first point appended virtually so the wrap
    segment counts toward total length; the n sample positions then span
    [0, L) (the wrap gap is implicit and equals the spacing too).
    """
    pts = np.asarray(points, dtype=np.float64)
    if closed:
        pts = np.vstack([pts, pts[:1]])
    seg = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(cum[-1])
    n = max(2, round(total / float(spacing_mm)))
    if closed:
        s = np.linspace(0.0, total, n, endpoint=False)
    else:
        s = np.linspace(0.0, total, n)
    x = np.interp(s, cum, pts[:, 0])
    y = np.interp(s, cum, pts[:, 1])
    return np.column_stack([x, y])


def _build(points: np.ndarray, closed: bool, name: str) -> Path:
    """Shared pipeline: clamp to the workspace circle, then resample."""
    pts = _clamp_to_radius(np.asarray(points, dtype=np.float64), name)
    pts = _resample(pts, closed, PATH_POINT_SPACING_MM)
    return Path(points=pts, closed=closed, name=name)


# ---------------------------
# Generators
# ---------------------------


def circle(radius_mm: float = 65.0) -> Path:
    theta = np.linspace(0.0, 2.0 * math.pi, 360, endpoint=False)
    pts = np.column_stack(
        [radius_mm * np.cos(theta), radius_mm * np.sin(theta)]
    )
    return _build(pts, closed=True, name=f"circle(r={radius_mm:g})")


def square(half_diag_mm: float = 70.0) -> Path:
    # DIAMOND orientation — rotated 45 deg so the edges pass midway between
    # the ArUco markers at (+/-60, +/-60); axis-aligned corners would dwell
    # 7 mm from the marker centers.
    d = float(half_diag_mm)
    pts = np.array(
        [[d, 0.0], [0.0, d], [-d, 0.0], [0.0, -d]], dtype=np.float64
    )
    return _build(pts, closed=True, name=f"square(diamond, d={2.0 * d:g})")


def heart(scale: float = 4.0) -> Path:
    t = np.linspace(0.0, 2.0 * math.pi, 720, endpoint=False)
    x = scale * 16.0 * np.sin(t) ** 3
    # +10 mm recenters the raw y range; resulting max radius ~69 mm.
    y = (
        scale
        * (
            13.0 * np.cos(t)
            - 5.0 * np.cos(2.0 * t)
            - 2.0 * np.cos(3.0 * t)
            - np.cos(4.0 * t)
        )
        + 10.0
    )
    return _build(
        np.column_stack([x, y]), closed=True, name=f"heart(scale={scale:g})"
    )


def star(outer_mm: float = 70.0, inner_mm: float = 28.0, points: int = 5) -> Path:
    # Tips deliberately off the 45-degree marker diagonals (first tip at
    # 90 deg, then every 72 deg).
    n_vert = 2 * points
    k = np.arange(n_vert)
    angles = np.deg2rad(90.0 + k * (360.0 / n_vert))
    radii = np.where(k % 2 == 0, float(outer_mm), float(inner_mm))
    pts = np.column_stack([radii * np.cos(angles), radii * np.sin(angles)])
    return _build(pts, closed=True, name=f"star({points}pt, r={outer_mm:g})")


def from_points(
    points_mm: Sequence[tuple[float, float]],
    closed: bool = True,
    name: str = "custom",
) -> Path:
    """Build a Path from caller-supplied (x, y) mm points (clamp + resample)."""
    pts = np.asarray(points_mm, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[0] < 2 or pts.shape[1] != 2:
        raise ValueError(
            f"from_points needs at least 2 (x, y) points, got shape {pts.shape}"
        )
    return _build(pts, closed=closed, name=name)


# Labels state the actual motion envelope (they populate the GUI dropdown,
# mirroring routines/ROUTINES).
PATTERNS: dict[str, Callable[[], Path]] = {
    # r=50 keeps the BALL'S EDGE (~+15-20 mm past its center) clear of
    # the ArUco inner corners at ~84.85 mm. Rig data 2026-07-23: on the
    # r=65 circle the ball grazes a marker 4x per lap — tracking-glitch
    # rate (>4 mm single-frame jumps) tripled near the diagonals and
    # fed a ~1 Hz path oscillation.
    "Circle (r=50, marker-safe)": lambda: circle(50.0),
    "Circle (r=65)": circle,
    "Square (diamond, d=140)": square,
    "Heart (~65mm)": heart,
    "Star (5pt, r=70)": star,
}
