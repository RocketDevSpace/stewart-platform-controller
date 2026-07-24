"""
cv/quad_tracker.py

PlatformQuadTracker — primary homography source from the platform's four
gray BOUNDARY edges (2026-07-24 boundary-quad rework).

Why: the ball occludes the ArUco markers (at ±60 mm) during path
following — rig-measured >4 mm single-frame homography glitches on 19%
of frames near the marker diagonals (3x baseline), which the PID chases
into a ~1 Hz path oscillation. The platform boundary at ±120 mm is
unreachable by the ball (max center excursion ~85 mm), and each side
offers dozens of edge samples versus 16 marker corners. So: sample the
four boundary lines, fit each with a trimmed total-least-squares line,
intersect adjacent lines for the corners, and build the same
camera->warp H the ArUco path builds. ArUco is demoted to acquisition
seed, identity/scale reference, periodic cross-check, and fallback.

Pure numpy + cv2 — no Qt, no camera, no threads (the measurement_filter
extraction precedent). The integration ladder (quad -> aruco -> stale ->
miss) lives in cv/ball_tracker.py.

State machine:
- UNLOCKED:  nothing to track. seed_from_h() (an ArUco solve) starts
  acquisition.
- ACQUIRING: each seed_from_h() re-projects the predicted quad from the
  FRESH ArUco H and attempts a wide-band (BAND_ACQ) boundary fit with
  UNKNOWN polarity. acq_frames consecutive good fits lock, freezing:
  (a) per-side edge polarity (mean signed outward gradient — the
      background contrast direction is unknown and may differ per side);
  (b) per-side offset between the fitted silhouette line and the
      ArUco-predicted line (the platform has thickness — from the
      oblique camera the silhouette edge is NOT the top-surface line;
      the frozen offset maps silhouette fits back into ArUco's frame,
      leaving only a static sub-mm residual).
- LOCKED:    update() tracks each side in a narrow band (BAND_TRACK)
  around the previous frame's corrected lines, applies the offset
  calibration, gates the result (corner step / side ratio / angles /
  convexity), low-passes the corners (PointDeadbandLP — identical
  frames give a bit-stable H, same contract as the ArUco path), and
  returns a fresh getPerspectiveTransform each frame. Identity persists
  by construction: side k is only ever searched in side k's previous
  band, and a 90-degree slip would move corners ~250 px — far above the
  step gate. max_miss_frames consecutive failures drop to UNLOCKED.

Cross-check (LOCKED): notify_aruco_h() compares a fresh ArUco solve's
boundary corners against the quad's. A transient disagreement keeps the
quad (transient ArUco error IS the failure mode this module exists
for); crosscheck_fails_to_reacq consecutive strikes force UNLOCKED —
only ArUco carries absolute identity and scale, so a persistently
divergent quad has locked onto a false structure and must reacquire.

Gray-on-gray fails CLOSED: below the min_grad contrast floor no side
fits, acquisition never completes, and the caller stays on ArUco —
exactly today's behavior.
"""

from __future__ import annotations

import time

import cv2
import numpy as np

from cv.measurement_filter import PointDeadbandLP
from settings import (
    TRACKER_QUAD_ACQ_FRAMES,
    TRACKER_QUAD_BALL_EXCLUDE_PX,
    TRACKER_QUAD_BAND_ACQ_PX,
    TRACKER_QUAD_BAND_TRACK_PX,
    TRACKER_QUAD_CORNER_ALPHA_FAST,
    TRACKER_QUAD_CORNER_ALPHA_SLOW,
    TRACKER_QUAD_CORNER_DEADBAND_PX,
    TRACKER_QUAD_CORNER_FAST_PX,
    TRACKER_QUAD_CORNER_SNAP_PX,
    TRACKER_QUAD_CROSSCHECK_FAILS_TO_REACQ,
    TRACKER_QUAD_CROSSCHECK_TOL_PX,
    TRACKER_QUAD_MAX_ANGLE_DELTA_DEG,
    TRACKER_QUAD_MAX_CORNER_STEP_PX,
    TRACKER_QUAD_MAX_MISS_FRAMES,
    TRACKER_QUAD_MIN_GRAD,
    TRACKER_QUAD_MIN_INLIERS,
    TRACKER_QUAD_SAMPLES_PER_SIDE,
    TRACKER_QUAD_SIDE_RATIO_TOL,
    TRACKER_QUAD_TRIM_RESID_PX,
)

STATE_UNLOCKED = "unlocked"
STATE_ACQUIRING = "acquiring"
STATE_LOCKED = "locked"

# Sample placement along each side: end margins keep the bands clear of
# the corners (where two edges meet and the 1-D profile is ambiguous).
_SAMPLE_T_MIN = 0.07
_SAMPLE_T_MAX = 0.93
# sin(20 deg): reject corner intersections of near-parallel lines.
_MIN_CORNER_SIN = 0.342
# A predicted side shorter than this is geometric garbage, not a fit.
_MIN_SIDE_PX = 20.0

# One side's fit: (unit outward normal, c, inlier count) with the line
# in normal form  n_out . x = c.
LineFit = tuple[np.ndarray, float, int]


def edge_offsets(
    profiles: np.ndarray, polarity: float, min_grad: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Sub-pixel edge location in each 1-D intensity profile.

    profiles: (N, 2B+1) taps along each sample's outward normal — tap
    index j is offset (j - B) px from the band center, increasing
    OUTWARD. polarity: +1 (intensity rises outward at the edge), -1
    (falls), or 0 = unknown (acquisition: score by |gradient|).

    Returns (offsets, signed_grads, ok): offsets (N,) px relative to the
    band center; signed_grads (N,) the raw outward gradient at each
    chosen peak (polarity learning); ok (N,) bool — peak at/above
    min_grad AND interior enough for the parabolic refinement.

    Edge = extremum of the polarity-signed central-difference gradient,
    refined by a parabolic fit over the 3 gradient samples around the
    peak (delta clamped to ±0.5 px — beyond that the parabola is
    extrapolating, not interpolating).
    """
    prof = np.asarray(profiles, dtype=np.float32)
    n, taps = prof.shape
    if taps < 5:
        raise ValueError(f"profiles too narrow for gradient+parabola: {taps} taps")
    band = (taps - 1) // 2
    # Central difference: g[:, j] is the gradient at tap j+1.
    g = 0.5 * (prof[:, 2:] - prof[:, :-2])
    score = np.abs(g) if polarity == 0.0 else float(polarity) * g
    idx = np.argmax(score, axis=1)
    rows = np.arange(n)
    peak = score[rows, idx]
    # Parabola needs both gradient neighbors: idx in [1, cols-2].
    interior = (idx >= 1) & (idx <= g.shape[1] - 2)
    ok = (peak >= float(min_grad)) & interior
    idx_c = np.clip(idx, 1, g.shape[1] - 2)
    sm1 = score[rows, idx_c - 1]
    sp1 = score[rows, idx_c + 1]
    denom = sm1 - 2.0 * score[rows, idx_c] + sp1
    with np.errstate(divide="ignore", invalid="ignore"):
        delta = np.where(np.abs(denom) > 1e-9, 0.5 * (sm1 - sp1) / denom, 0.0)
    delta = np.clip(delta, -0.5, 0.5)
    offsets = (idx.astype(np.float64) + 1.0 + delta) - float(band)
    signed_grads = g[rows, idx].astype(np.float64)
    return offsets, signed_grads, ok


def _tls(pts: np.ndarray) -> tuple[np.ndarray, float] | None:
    """Total-least-squares line through pts: eigenvector of the smallest
    eigenvalue of the 2x2 scatter is the normal. None if degenerate."""
    m = pts.mean(axis=0)
    q = pts - m
    s = q.T @ q
    w, v = np.linalg.eigh(s)   # ascending eigenvalues
    if not np.all(np.isfinite(v)) or w[1] <= 1e-12:
        return None            # points (near-)coincident
    n_vec = v[:, 0]
    return n_vec, float(n_vec @ m)


def fit_line_tls(
    points: np.ndarray,
    outward: np.ndarray,
    trim_floor_px: float,
    mad_k: float = 2.5,
) -> LineFit | None:
    """2-pass trimmed total-least-squares line fit. Deterministic (no
    RANSAC): pass 1 fits all points, residuals beyond
    max(mad_k * 1.4826 * MAD, trim_floor_px) are trimmed, pass 2 refits
    the survivors. Returns (n_out, c, inliers) with the unit normal
    oriented along the `outward` hint, or None if degenerate."""
    pts = np.asarray(points, dtype=np.float64)
    if pts.shape[0] < 2:
        return None
    fit = _tls(pts)
    if fit is None:
        return None
    n_vec, c = fit
    r = pts @ n_vec - c
    # Center residuals on their MEDIAN before thresholding: clustered
    # gross outliers (a ball silhouette overlapping the band) drag the
    # pass-1 line off the inlier bulk, so raw |r| would trim the bulk
    # itself; deviation from the median residual isolates the outliers.
    dev = np.abs(r - np.median(r))
    mad = float(np.median(dev))
    thresh = max(mad_k * 1.4826 * mad, float(trim_floor_px))
    keep = dev <= thresh
    n_inliers = int(keep.sum())
    if 2 <= n_inliers < pts.shape[0]:
        refit = _tls(pts[keep])
        if refit is not None:
            n_vec, c = refit
    if float(n_vec @ np.asarray(outward, dtype=np.float64)) < 0.0:
        n_vec = -n_vec
        c = -c
    return n_vec, c, n_inliers


def intersect_lines(
    n1: np.ndarray,
    c1: float,
    n2: np.ndarray,
    c2: float,
    min_sin: float = _MIN_CORNER_SIN,
) -> np.ndarray | None:
    """Intersection of two normal-form lines; None when the lines meet
    at less than asin(min_sin) (unit normals: |det| = sin(angle))."""
    det = float(n1[0] * n2[1] - n1[1] * n2[0])
    if abs(det) < float(min_sin):
        return None
    x = (c1 * n2[1] - c2 * n1[1]) / det
    y = (n1[0] * c2 - n2[0] * c1) / det
    return np.array([x, y], dtype=np.float64)


def _corner_angles_deg(corners: np.ndarray) -> np.ndarray:
    """Interior angle at each corner (degrees), from the two adjacent
    side vectors."""
    sides = np.roll(corners, -1, axis=0) - corners     # side j: corner j -> j+1
    incoming = -np.roll(sides, 1, axis=0)              # at corner j: -(side j-1)
    dots = np.sum(sides * incoming, axis=1)
    crosses = sides[:, 0] * incoming[:, 1] - sides[:, 1] * incoming[:, 0]
    angles: np.ndarray = np.degrees(np.abs(np.arctan2(crosses, dots)))
    return angles


class PlatformQuadTracker:
    """Track the platform boundary quad; see the module docstring."""

    def __init__(self, warp_size_px: int) -> None:
        w = float(int(warp_size_px))
        # Warp-image corners in the SAME order the camera corners are
        # seeded in (from H_aruco^-1 of exactly these points), so the
        # corner correspondence — and with it the platform identity/
        # orientation — is fixed by construction at acquisition.
        self.warp_corners = np.array(
            [[0.0, 0.0], [w, 0.0], [w, w], [0.0, w]], dtype=np.float32
        )

        self.samples_per_side = int(TRACKER_QUAD_SAMPLES_PER_SIDE)
        self.band_track_px = int(TRACKER_QUAD_BAND_TRACK_PX)
        self.band_acq_px = int(TRACKER_QUAD_BAND_ACQ_PX)
        self.min_grad = float(TRACKER_QUAD_MIN_GRAD)
        self.min_inliers = int(TRACKER_QUAD_MIN_INLIERS)
        self.trim_resid_px = float(TRACKER_QUAD_TRIM_RESID_PX)
        self.ball_exclude_px = float(TRACKER_QUAD_BALL_EXCLUDE_PX)
        self.acq_frames = int(TRACKER_QUAD_ACQ_FRAMES)
        self.max_miss_frames = int(TRACKER_QUAD_MAX_MISS_FRAMES)
        self.max_corner_step_px = float(TRACKER_QUAD_MAX_CORNER_STEP_PX)
        self.side_ratio_tol = float(TRACKER_QUAD_SIDE_RATIO_TOL)
        self.max_angle_delta_deg = float(TRACKER_QUAD_MAX_ANGLE_DELTA_DEG)
        self.crosscheck_tol_px = float(TRACKER_QUAD_CROSSCHECK_TOL_PX)
        self.crosscheck_fails_to_reacq = int(TRACKER_QUAD_CROSSCHECK_FAILS_TO_REACQ)

        self._corner_lp = [
            PointDeadbandLP(
                TRACKER_QUAD_CORNER_DEADBAND_PX,
                TRACKER_QUAD_CORNER_FAST_PX,
                TRACKER_QUAD_CORNER_ALPHA_SLOW,
                TRACKER_QUAD_CORNER_ALPHA_FAST,
                TRACKER_QUAD_CORNER_SNAP_PX,
            )
            for _ in range(4)
        ]

        # --- Mutable tracking state (cleared by reset()) ---
        self.state: str = STATE_UNLOCKED
        self.last_corners_cam: np.ndarray | None = None
        self.last_inliers: np.ndarray = np.zeros(4, dtype=int)
        self.last_update_ms: float = 0.0
        # Diagnostics from the LAST fit attempt (rig visibility: why is
        # a side failing?): {"pred_corners": (4,2), "fit_corners":
        # (4,2)|None, "gate": str|None, "sides": [4 x {"reason",
        # "usable", "edges", "inliers", "grad"}]}. reason: "ok" /
        # "clipped" (too few samples in frame after ball exclusion) /
        # "low-contrast" (edge gradient under min_grad — grad carries
        # the measured mean peak |gradient| to tune against) /
        # "few-inliers" / "degenerate".
        self.last_diag: dict | None = None
        self._pred_corners: np.ndarray | None = None
        self._polarity: np.ndarray = np.zeros(4)
        self._side_offset: np.ndarray = np.zeros(4)
        self._acq_good: int = 0
        self._acq_grad_sum: np.ndarray = np.zeros(4)
        self._acq_offset_sum: np.ndarray = np.zeros(4)
        self._miss_count: int = 0
        self._crosscheck_fails: int = 0
        self.reset()

    def reset(self) -> None:
        """Drop to UNLOCKED; a fresh ArUco seed is required to reacquire."""
        self.state = STATE_UNLOCKED
        self.last_corners_cam = None
        self.last_inliers = np.zeros(4, dtype=int)
        self._pred_corners = None
        self._polarity = np.zeros(4)
        self._side_offset = np.zeros(4)
        self._acq_good = 0
        self._acq_grad_sum = np.zeros(4)
        self._acq_offset_sum = np.zeros(4)
        self._miss_count = 0
        self._crosscheck_fails = 0
        self.last_diag = None
        for lp in self._corner_lp:
            lp.reset()

    @property
    def acq_progress(self) -> tuple[int, int]:
        """(consecutive good fits so far, fits required to lock)."""
        return self._acq_good, self.acq_frames

    # =========================
    # Public API
    # =========================

    def seed_from_h(self, h_cam_to_warp: np.ndarray, gray: np.ndarray) -> None:
        """Feed one ArUco solve while NOT locked: start or continue
        acquisition (see the module docstring). The predicted quad is
        re-projected from the fresh H every call, so acquisition tracks
        a moving platform. No-op while LOCKED — locked-state ArUco
        solves go to notify_aruco_h() for cross-checking."""
        if self.state == STATE_LOCKED:
            return
        corners = self._project_h_corners(h_cam_to_warp)
        if corners is None:
            return
        if self.state == STATE_UNLOCKED:
            self._begin_acquisition()
        self._pred_corners = corners
        self._acquire_step(gray)

    def update(
        self,
        gray: np.ndarray,
        prev_ball_px: tuple[float, float] | None = None,
    ) -> np.ndarray | None:
        """One LOCKED tracking step: fit the four boundary lines in
        narrow bands around the previous corrected lines, gate, filter,
        and return a fresh camera->warp H — or None when this frame's
        fit failed (the caller falls down the source ladder).
        prev_ball_px (flipped-camera coords) enables the ball-exclusion
        zone. Returns None without touching gray when not LOCKED."""
        if self.state != STATE_LOCKED or self._pred_corners is None:
            return None
        t0 = time.perf_counter()
        h = self._track_step(gray, prev_ball_px)
        self.last_update_ms = (time.perf_counter() - t0) * 1e3
        if h is None:
            self._miss_count += 1
            if self._miss_count > self.max_miss_frames:
                self.reset()
        else:
            self._miss_count = 0
        return h

    def notify_aruco_h(self, h_cam_to_warp: np.ndarray) -> bool:
        """Cross-check a fresh ArUco solve against the locked quad.

        Returns True when the check passed (or wasn't applicable). A
        mean corner disagreement above crosscheck_tol_px is a strike but
        the quad stays the source (transient ArUco error is the rig
        failure mode this module exists for); crosscheck_fails_to_reacq
        CONSECUTIVE strikes force a reset to UNLOCKED — only ArUco
        carries absolute identity/scale, so a persistently divergent
        quad has locked onto a false structure and must reacquire."""
        if self.state != STATE_LOCKED or self.last_corners_cam is None:
            return True
        corners = self._project_h_corners(h_cam_to_warp)
        if corners is None:
            return True
        diff = corners - self.last_corners_cam
        d = float(np.mean(np.hypot(diff[:, 0], diff[:, 1])))
        if d <= self.crosscheck_tol_px:
            self._crosscheck_fails = 0
            return True
        self._crosscheck_fails += 1
        if self._crosscheck_fails >= self.crosscheck_fails_to_reacq:
            self.reset()
        return False

    # =========================
    # Internal: acquisition
    # =========================

    def _begin_acquisition(self) -> None:
        self.state = STATE_ACQUIRING
        self._acq_good = 0
        self._acq_grad_sum = np.zeros(4)
        self._acq_offset_sum = np.zeros(4)
        self._polarity = np.zeros(4)
        self._side_offset = np.zeros(4)

    def _acquire_step(self, gray: np.ndarray) -> None:
        t0 = time.perf_counter()
        assert self._pred_corners is not None
        fits = self._fit_sides(
            gray, self._pred_corners, self.band_acq_px, learn=True
        )
        self.last_update_ms = (time.perf_counter() - t0) * 1e3
        if fits is None:
            # Consecutive-good requirement: any failure restarts the count.
            self._acq_good = 0
            self._acq_grad_sum = np.zeros(4)
            self._acq_offset_sum = np.zeros(4)
            return
        lines, grads, offsets = fits
        self._acq_good += 1
        self._acq_grad_sum += grads
        self._acq_offset_sum += offsets
        if self._acq_good >= self.acq_frames:
            self._lock(lines)

    def _lock(self, lines: list[LineFit]) -> None:
        """Freeze polarity + offset calibration and go LOCKED."""
        self._polarity = np.sign(self._acq_grad_sum)
        self._polarity[self._polarity == 0.0] = 1.0
        self._side_offset = self._acq_offset_sum / float(self._acq_good)
        corners = self._corners_from_lines(lines, apply_offset=True)
        if corners is None:
            self._acq_good = 0          # degenerate at the finish: keep acquiring
            return
        for lp, c in zip(self._corner_lp, corners):
            lp.reset()
            lp.filter(c)
        self._pred_corners = corners
        self.last_corners_cam = corners.copy()
        self._miss_count = 0
        self._crosscheck_fails = 0
        self.state = STATE_LOCKED

    # =========================
    # Internal: locked tracking
    # =========================

    def _track_step(
        self,
        gray: np.ndarray,
        prev_ball_px: tuple[float, float] | None,
    ) -> np.ndarray | None:
        assert self._pred_corners is not None
        fits = self._fit_sides(
            gray, self._pred_corners, self.band_track_px,
            learn=False, ball_px=prev_ball_px,
        )
        if fits is None:
            return None
        lines, _, _ = fits
        corners = self._corners_from_lines(lines, apply_offset=True)
        if corners is None:
            return None
        if not self._corners_pass_gates(corners):
            if self.last_diag is not None:
                self.last_diag["gate"] = "gate-failed"
            return None
        filtered = np.stack(
            [lp.filter(c) for lp, c in zip(self._corner_lp, corners)]
        ).astype(np.float64)
        self._pred_corners = filtered
        self.last_corners_cam = filtered.copy()
        h: np.ndarray = cv2.getPerspectiveTransform(
            filtered.astype(np.float32), self.warp_corners
        )
        return h

    def _corners_pass_gates(self, corners: np.ndarray) -> bool:
        """Validation gates vs the previous frame's accepted quad: per-
        corner step, convexity, side-length drift, corner-angle drift.
        Any failure = quad failure this frame (fall to ArUco)."""
        prev = self._pred_corners
        assert prev is not None
        step = corners - prev
        if float(np.max(np.hypot(step[:, 0], step[:, 1]))) > self.max_corner_step_px:
            return False
        sides = np.roll(corners, -1, axis=0) - corners
        crosses = (
            sides[:, 0] * np.roll(sides, -1, axis=0)[:, 1]
            - sides[:, 1] * np.roll(sides, -1, axis=0)[:, 0]
        )
        if not (np.all(crosses > 0.0) or np.all(crosses < 0.0)):
            return False                # non-convex
        lens = np.hypot(sides[:, 0], sides[:, 1])
        prev_sides = np.roll(prev, -1, axis=0) - prev
        prev_lens = np.hypot(prev_sides[:, 0], prev_sides[:, 1])
        if np.any(prev_lens < _MIN_SIDE_PX) or np.any(lens < _MIN_SIDE_PX):
            return False
        if float(np.max(np.abs(lens / prev_lens - 1.0))) > self.side_ratio_tol:
            return False
        d_ang = np.abs(_corner_angles_deg(corners) - _corner_angles_deg(prev))
        if float(np.max(d_ang)) > self.max_angle_delta_deg:
            return False
        return True

    # =========================
    # Internal: geometry
    # =========================

    def _project_h_corners(self, h: np.ndarray) -> np.ndarray | None:
        """warp corners through H^-1 -> camera-space quad (4,2)."""
        try:
            h_inv = np.linalg.inv(np.asarray(h, dtype=np.float64))
        except np.linalg.LinAlgError:
            return None
        pts = cv2.perspectiveTransform(
            self.warp_corners.reshape(1, 4, 2).astype(np.float64), h_inv
        )
        if pts is None:
            return None
        corners = np.asarray(pts, dtype=np.float64).reshape(4, 2)
        if not np.all(np.isfinite(corners)):
            return None
        return corners

    def _fit_sides(
        self,
        gray: np.ndarray,
        pred_corners: np.ndarray,
        band_px: int,
        learn: bool,
        ball_px: tuple[float, float] | None = None,
    ) -> tuple[list[LineFit], np.ndarray, np.ndarray] | None:
        """Sample all four sides' edge bands (ONE cv2.remap for every
        tap), extract sub-pixel edges, and fit each side's line.

        Returns (lines, mean signed outward gradient per side, fitted-
        line offset vs the predicted line per side) — the latter two are
        only meaningful when learn=True (acquisition). None when any
        side fails (too few inliers / weak contrast / degenerate)."""
        s = self.samples_per_side
        b = int(band_px)
        c1 = np.asarray(pred_corners, dtype=np.float64)
        c2 = np.roll(c1, -1, axis=0)
        d = c2 - c1
        side_lens = np.hypot(d[:, 0], d[:, 1])
        if np.any(~np.isfinite(side_lens)) or np.any(side_lens < _MIN_SIDE_PX):
            return None
        u = d / side_lens[:, None]
        normal = np.stack([u[:, 1], -u[:, 0]], axis=1)
        centroid = c1.mean(axis=0)
        mid = 0.5 * (c1 + c2)
        flip = np.sum(normal * (mid - centroid), axis=1) < 0.0
        normal[flip] = -normal[flip]

        t = np.linspace(_SAMPLE_T_MIN, _SAMPLE_T_MAX, s)
        base = c1[:, None, :] + t[None, :, None] * d[:, None, :]     # (4,S,2)
        if not learn:
            # Tracked corners live in the offset-corrected (top-surface)
            # frame; the physical silhouette edge sits side_offset px
            # outward — center the search bands on it.
            base = base + self._side_offset[:, None, None] * normal[:, None, :]
        offs = np.arange(-b, b + 1, dtype=np.float64)
        taps = (
            base[:, :, None, :]
            + offs[None, None, :, None] * normal[:, None, None, :]
        )                                                            # (4,S,T,2)

        h_img, w_img = gray.shape[:2]
        map_x = taps[..., 0].reshape(4 * s, -1).astype(np.float32)
        map_y = taps[..., 1].reshape(4 * s, -1).astype(np.float32)
        prof = cv2.remap(
            gray, map_x, map_y, cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_REPLICATE,
        )
        prof4 = np.asarray(prof, dtype=np.float32).reshape(4, s, -1)

        usable = (
            (taps[..., 0] >= 0.0) & (taps[..., 0] <= w_img - 1.0)
            & (taps[..., 1] >= 0.0) & (taps[..., 1] <= h_img - 1.0)
        ).all(axis=2)                                                # (4,S)
        if ball_px is not None and self.ball_exclude_px > 0.0:
            bp = np.asarray(ball_px, dtype=np.float64)
            dist = np.hypot(base[..., 0] - bp[0], base[..., 1] - bp[1])
            usable &= dist > self.ball_exclude_px

        # Every side is evaluated even after one fails, so last_diag is
        # COMPLETE on failure — the rig overlay needs to show all four
        # verdicts, not just the first bad one.
        lines: list[LineFit] = []
        grads = np.zeros(4)
        offsets_vs_pred = np.zeros(4)
        sides_diag: list[dict] = []
        all_ok = True
        for k in range(4):
            n_usable = int(usable[k].sum())
            diag: dict = {
                "reason": "ok", "usable": n_usable,
                "edges": 0, "inliers": 0, "grad": 0.0,
            }
            # Measured contrast for tuning: mean per-sample peak
            # |gradient| over the in-frame samples — the number
            # min_grad is compared against.
            g_abs = np.abs(0.5 * (prof4[k][:, 2:] - prof4[k][:, :-2]))
            if n_usable > 0:
                diag["grad"] = float(np.mean(g_abs.max(axis=1)[usable[k]]))
            pol = 0.0 if learn else float(self._polarity[k])
            e_off, e_grad, ok = edge_offsets(prof4[k], pol, self.min_grad)
            ok = ok & usable[k]
            diag["edges"] = int(ok.sum())
            if n_usable < self.min_inliers:
                diag["reason"] = "clipped"
            elif int(ok.sum()) < self.min_inliers:
                diag["reason"] = "low-contrast"
            else:
                pts = base[k][ok] + e_off[ok, None] * normal[k][None, :]
                fit = fit_line_tls(pts, normal[k], self.trim_resid_px)
                if fit is None:
                    diag["reason"] = "degenerate"
                elif fit[2] < self.min_inliers:
                    diag["reason"] = "few-inliers"
                    diag["inliers"] = int(fit[2])
                else:
                    diag["inliers"] = int(fit[2])
                    lines.append(fit)
                    if learn:
                        grads[k] = float(np.mean(e_grad[ok]))
                        n_vec, c, _ = fit
                        c_pred = 0.5 * float(n_vec @ c1[k] + n_vec @ c2[k])
                        offsets_vs_pred[k] = c - c_pred
            if diag["reason"] != "ok":
                all_ok = False
            sides_diag.append(diag)

        self.last_diag = {
            "pred_corners": np.asarray(pred_corners, dtype=np.float64).copy(),
            "fit_corners": None,
            "gate": None,
            "sides": sides_diag,
        }
        self.last_inliers = np.array([d["inliers"] for d in sides_diag])
        if not all_ok:
            return None
        # Raw silhouette corners (no offset correction) for the overlay:
        # "the fit found the boundary HERE".
        self.last_diag["fit_corners"] = self._corners_from_lines(
            lines, apply_offset=False
        )
        return lines, grads, offsets_vs_pred

    def _corners_from_lines(
        self, lines: list[LineFit], apply_offset: bool
    ) -> np.ndarray | None:
        """Adjacent-line intersections -> corners (4,2); corner j joins
        side j-1 and side j. apply_offset shifts each fitted silhouette
        line inward by its calibrated offset (top-surface frame)."""
        corners = np.zeros((4, 2))
        for j in range(4):
            k_prev = (j - 1) % 4
            n1, cc1, _ = lines[k_prev]
            n2, cc2, _ = lines[j]
            if apply_offset:
                cc1 = cc1 - self._side_offset[k_prev]
                cc2 = cc2 - self._side_offset[j]
            p = intersect_lines(n1, cc1, n2, cc2)
            if p is None:
                return None
            corners[j] = p
        return corners
