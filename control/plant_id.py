"""
control/plant_id.py

System identification for the ball-on-plate plant: the scripted probe
sequence, the per-frame recording, and the open-loop replay fit that
recovers the plant parameters (g_eff, full-loop latency, stiction cone,
warp, biases) from a ~78 s recording.

Replaces the old step-test estimator's approach (2 scalar features
inverted through an ideal 2nd-order model) with trajectory matching:
replay the RECORDED commands through control/plant_model.py over many
short windows and pick the parameters that minimize the Huber position
residual. Short (1.5 s) windows keep open-loop drift in the regime
where MODEL error dominates; ≥ one self-rock period per window plus the
Huber loss keeps the ball's unmodeled 0.6-0.9 Hz rock from biasing the
fit; the relay toggles' command edges decorrelate latency (translates
the response in time) from g_eff (scales its curvature).

Pure numpy + plant_model. No Qt, no hardware, no scipy.
"""
from __future__ import annotations

import bisect
import math
import threading
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

from control.plant_model import PlantParams, delayed_commands, replay_batch

HZ = 30.0
DT = 1.0 / HZ

# Fit grid bounds (module constants — not user tunables)
_G_GRID = np.arange(100.0, 251.0, 15.0)          # mm/s^2 per deg
_L_GRID = np.arange(0.0, 0.201, DT)              # s (refined to ~5 ms later)
_STICTION_GRID = np.arange(0.0, 0.61, 0.1)       # deg
_HUBER_DELTA_MM = 3.0
_WINDOW_FRAMES = 45                              # 1.5 s @ 30 Hz
_WINDOW_STRIDE = 30                              # 1.0 s
_MIN_COMPLETION = 0.60                           # refuse fits below this
_RESIDUAL_CONFIDENCE_RATIO = 3.0                 # residual > 3x noise -> low


# ---------------------------------------------------------------------------
# Probe script
# ---------------------------------------------------------------------------


def _build_segments() -> list[tuple[float, float, float, str]]:
    """(start_time_s, tx, ty, label) piecewise-constant target timeline."""
    seg: list[tuple[float, float, float, str]] = []
    t = 0.0

    def add(dur: float, tx: float, ty: float, label: str) -> None:
        nonlocal t
        seg.append((t, tx, ty, label))
        t += dur

    add(3.0, 0.0, 0.0, "S0_quiet")
    add(2.0, 10.0, 0.0, "S1_precheck")
    add(1.5, 0.0, 0.0, "S1_precheck")
    for i in range(6):                                    # S2: relay X
        add(2.2, 25.0 if i % 2 == 0 else -25.0, 0.0, "S2_relay_x")
    for i in range(6):                                    # S3: relay Y
        add(2.2, 0.0, 25.0 if i % 2 == 0 else -25.0, "S3_relay_y")
    for i in range(4):                                    # S4: diagonals
        s = 18.0 if i % 2 == 0 else -18.0
        add(2.2, s, s if i < 2 else -s, "S4_diag")
    for tx in (8.0, 0.0, -8.0, 0.0):                      # S5: stiction X
        add(2.5, tx, 0.0, "S5_stiction")
    for ty in (8.0, 0.0, -8.0, 0.0):                      # S5: stiction Y
        add(2.5, 0.0, ty, "S5_stiction")
    add(4.0, 30.0, 0.0, "S6_baseline")                    # S6: steps
    add(4.0, 0.0, 0.0, "S6_baseline")
    add(4.0, 0.0, 30.0, "S6_baseline")
    add(4.0, 0.0, 0.0, "S6_baseline")
    seg.append((t, 0.0, 0.0, "end"))
    return seg


class ProbeScript:
    """The scripted probe: piecewise-constant targets on a frame clock.

    Targets are RELATIVE to the session center; segments advance on
    elapsed time — no settle gates anywhere (the plant's failure to
    settle is data, not a blocker).
    """

    def __init__(self) -> None:
        self._segments = _build_segments()
        self._starts = [s[0] for s in self._segments]
        self.total_s = self._segments[-1][0]

    def target_at(self, elapsed_s: float) -> tuple[float, float]:
        i = bisect.bisect_right(self._starts, max(0.0, elapsed_s)) - 1
        i = min(i, len(self._segments) - 2)
        _, tx, ty, _ = self._segments[i]
        return tx, ty

    def label_at(self, elapsed_s: float) -> str:
        i = bisect.bisect_right(self._starts, max(0.0, elapsed_s)) - 1
        i = min(i, len(self._segments) - 2)
        return self._segments[i][3]

    def segment_index(self, elapsed_s: float) -> int:
        i = bisect.bisect_right(self._starts, max(0.0, elapsed_s)) - 1
        return min(i, len(self._segments) - 2)

    @property
    def segment_count(self) -> int:
        return len(self._segments) - 1

    def progress(self, elapsed_s: float) -> float:
        return max(0.0, min(1.0, elapsed_s / self.total_s))

    def window_of(self, label_prefix: str) -> tuple[float, float]:
        """(start_s, end_s) covering all segments whose label starts
        with the prefix."""
        start = None
        end = None
        for i, (t0, _, _, label) in enumerate(self._segments[:-1]):
            if label.startswith(label_prefix):
                if start is None:
                    start = t0
                end = self._segments[i + 1][0]
        if start is None or end is None:
            return 0.0, 0.0
        return start, float(end)


class ProbeRecording:
    """Preallocated per-frame recording filled during the probe.

    Columns: t, raw_x, raw_y, filt_x, filt_y, vx, vy, roll_cmd,
    pitch_cmd, target_x, target_y.

    RAW detections (pre-filter) are what the fit replays against — the
    fitted latency is then the true camera→servo→plant pipeline delay
    with clean identifiability. The FILTERED trace rides along so the
    alpha-beta filter's group delay can be MEASURED per session
    (cross-correlation raw↔filtered); prediction must compensate
    pipeline + filter lag, and fitting against the filtered trace would
    double-count the filter once the design sims (which run the real
    filter in the loop) use the fitted plant.

    Rows are begun by the autotuner's update() and completed by
    record_command() after the PID compute — exact frame pairing.
    """

    COLS = 11

    def __init__(self, capacity: int = 3000) -> None:
        self._data = np.zeros((capacity, self.COLS), dtype=np.float64)
        self._n = 0
        self._row_open = False

    def begin_row(
        self,
        t: float,
        raw_x: float,
        raw_y: float,
        filt_x: float,
        filt_y: float,
        vx: float,
        vy: float,
        tx: float,
        ty: float,
    ) -> None:
        if self._n >= len(self._data):
            return
        self._data[self._n, 0:7] = (t, raw_x, raw_y, filt_x, filt_y, vx, vy)
        self._data[self._n, 9:11] = (tx, ty)
        self._row_open = True

    def complete_row(self, roll_cmd: float, pitch_cmd: float) -> None:
        if not self._row_open or self._n >= len(self._data):
            return
        self._data[self._n, 7:9] = (roll_cmd, pitch_cmd)
        self._n += 1
        self._row_open = False

    def __len__(self) -> int:
        return self._n

    def arrays(self) -> np.ndarray:
        """Snapshot copy of the completed rows (thread handoff safe)."""
        return self._data[: self._n].copy()


# ---------------------------------------------------------------------------
# Fit
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class PlantFit:
    """The identified plant + fit quality + S0 disturbance stats."""

    params: PlantParams
    residual_rms_mm: float
    noise_rms_mm: float
    low_confidence: bool
    rock_amp_mm: float
    rock_freq_hz: float
    windows_used: int
    filter_lag_s: float = 0.0     # measured alpha-beta group delay
    notes: str = ""

    @property
    def predict_s(self) -> float:
        """The prediction horizon this plant wants: pipeline latency +
        measured filter lag (what CONTROL_PREDICT_S compensates)."""
        return self.params.latency_s + self.filter_lag_s


@dataclass
class _Resampled:
    t: np.ndarray
    x: np.ndarray            # raw positions (replay residual space)
    y: np.ndarray
    vx: np.ndarray           # smoothed-derivative velocities (ICs)
    vy: np.ndarray
    roll: np.ndarray
    pitch: np.ndarray
    xs: np.ndarray           # box-smoothed positions (regression space)
    ys: np.ndarray
    window_starts: np.ndarray = field(default_factory=lambda: np.zeros(0, int))


def _resample(rows: np.ndarray) -> _Resampled:
    """Uniform-grid resample of the (possibly jittered) recording.

    Fit space: RAW positions (cols 1,2) for residuals + position ICs,
    sent commands (cols 7,8). Velocity ICs come from CENTRAL
    DIFFERENCES of the raw positions — NOT the alpha-beta velocity: its
    1-3 frame group delay makes replay under-predict early window
    motion, which the fit would misread as extra latency (measured:
    a ~3-frame L bias before this).
    """
    t_raw = rows[:, 0] - rows[0, 0]
    n = int(t_raw[-1] / DT) + 1
    t = np.arange(n) * DT

    def col(c: int) -> np.ndarray:
        return np.asarray(np.interp(t, t_raw, rows[:, c]))

    x = col(1)
    y = col(2)
    # 5-frame smoothing before differencing (noise sigma / ~2), then a
    # stick deadzone: sub-noise IC velocities are zeroed so a PARKED
    # window replays parked — noise-broken stiction makes the model
    # ball wander with amplitude proportional to g, which the fit would
    # otherwise minimize by shrinking g (measured: -12% bias).
    kern = np.ones(5) / 5.0
    xs = np.convolve(x, kern, mode="same")
    ys = np.convolve(y, kern, mode="same")
    vx = np.asarray(np.gradient(xs, DT))
    vy = np.asarray(np.gradient(ys, DT))
    slow = np.hypot(vx, vy) < 5.0
    vx[slow] = 0.0
    vy[slow] = 0.0
    starts = np.arange(0, n - _WINDOW_FRAMES, _WINDOW_STRIDE)
    return _Resampled(
        t, x, y, vx, vy, col(7), col(8), xs, ys,
        window_starts=starts,
    )


def _measure_filter_lag(rows: np.ndarray) -> float:
    """Alpha-beta group delay: cross-correlate filtered vs raw position
    over the whole recording, integer-frame argmax + parabolic refine."""
    raw = rows[:, 1] + rows[:, 2]
    filt = rows[:, 3] + rows[:, 4]
    raw = raw - raw.mean()
    filt = filt - filt.mean()
    max_shift = 8
    scores = []
    for k in range(0, max_shift + 1):
        a = raw[: len(raw) - k]
        b = filt[k:]
        denom = math.sqrt(float(np.dot(a, a) * np.dot(b, b))) or 1.0
        scores.append(float(np.dot(a, b)) / denom)
    k = int(np.argmax(scores))
    if 0 < k < max_shift:
        s0, s1, s2 = scores[k - 1], scores[k], scores[k + 1]
        denom2 = s0 - 2 * s1 + s2
        frac = 0.5 * (s0 - s2) / denom2 if abs(denom2) > 1e-12 else 0.0
        k_ref = k + max(-0.5, min(0.5, frac))
    else:
        k_ref = float(k)
    return k_ref * DT


def _huber_loss(residual: np.ndarray) -> float:
    a = np.abs(residual)
    d = _HUBER_DELTA_MM
    quad = 0.5 * np.minimum(a, d) ** 2
    lin = d * (a - np.minimum(a, d))
    return float(np.sum(quad + lin))


def _window_loss_batch(
    rs: _Resampled,
    g: np.ndarray,
    latency: np.ndarray,
    stiction: np.ndarray,
    warp_c: np.ndarray,
    bias_r: np.ndarray,
    bias_p: np.ndarray,
) -> np.ndarray:
    """Total Huber loss per parameter combo, summed over all windows.

    All parameter arrays share shape (C,). Latency must be constant
    within the call (commands are shifted once) — callers loop distinct
    latency values.
    """
    lat = float(latency[0])
    roll_act = delayed_commands(rs.t, rs.roll, lat)
    pitch_act = delayed_commands(rs.t, rs.pitch, lat)

    w = rs.window_starts
    n_w = len(w)
    n_c = len(g)
    idx = w[:, None] + np.arange(_WINDOW_FRAMES)[None, :]     # (W, F)

    # Trajectories: combos x windows, flattened.
    ra = np.broadcast_to(roll_act[idx], (n_c, n_w, _WINDOW_FRAMES))
    pa = np.broadcast_to(pitch_act[idx], (n_c, n_w, _WINDOW_FRAMES))
    ra = ra.reshape(n_c * n_w, _WINDOW_FRAMES)
    pa = pa.reshape(n_c * n_w, _WINDOW_FRAMES)

    def tile(v: np.ndarray) -> np.ndarray:
        return np.repeat(v, n_w)

    x0 = np.tile(rs.x[w], n_c)
    y0 = np.tile(rs.y[w], n_c)
    vx0 = np.tile(rs.vx[w], n_c)
    vy0 = np.tile(rs.vy[w], n_c)

    xs, ys = replay_batch(
        DT, ra, pa, x0, y0, vx0, vy0,
        g_eff=tile(g), stiction_deg=tile(stiction), warp_c=tile(warp_c),
        bias_roll=tile(bias_r), bias_pitch=tile(bias_p),
    )
    rec_x = np.broadcast_to(rs.x[idx], (n_c, n_w, _WINDOW_FRAMES))
    rec_y = np.broadcast_to(rs.y[idx], (n_c, n_w, _WINDOW_FRAMES))
    res_x = xs.reshape(n_c, n_w, _WINDOW_FRAMES) - rec_x
    res_y = ys.reshape(n_c, n_w, _WINDOW_FRAMES) - rec_y

    a = np.abs(np.stack([res_x, res_y]))
    d = _HUBER_DELTA_MM
    quad = 0.5 * np.minimum(a, d) ** 2
    lin = d * (a - np.minimum(a, d))
    return np.asarray(np.sum(quad + lin, axis=(0, 2, 3)))


def _loss_single(rs: _Resampled, p: PlantParams) -> float:
    ones = np.ones(1)
    return float(_window_loss_batch(
        rs, ones * p.g_eff, ones * p.latency_s, ones * p.stiction_deg,
        ones * p.warp_c_deg_per_mm, ones * p.bias_roll_deg,
        ones * p.bias_pitch_deg,
    )[0])


def _s0_stats(rows: np.ndarray, script: ProbeScript) -> tuple[float, float, float]:
    """(noise_rms_mm, rock_amp_mm, rock_freq_hz) from the S0 quiet hold."""
    t0, t1 = script.window_of("S0")
    t = rows[:, 0] - rows[0, 0]
    m = (t >= t0) & (t < t1)
    if m.sum() < 30:
        return 0.3, 0.0, 0.0
    x = rows[m, 1]
    y = rows[m, 2]
    xz = x - x.mean()
    yz = y - y.mean()
    noise = float(np.sqrt(np.mean(xz ** 2 + yz ** 2)))
    # Dominant oscillation in 0.3-2 Hz
    n = len(xz)
    f = np.fft.rfftfreq(n, DT)
    px = np.abs(np.fft.rfft(xz * np.hanning(n))) ** 2
    py = np.abs(np.fft.rfft(yz * np.hanning(n))) ** 2
    p = px + py
    band = (f >= 0.3) & (f <= 2.0)
    if not band.any() or p[band].max() <= 0:
        return noise, 0.0, 0.0
    # Sub-bin frequency via parabolic interpolation on the log power
    # peak — the 3 s S0 window alone gives only 1/3 Hz bin resolution,
    # which mis-centered the rock notch by a full bin (0.66 vs 0.8).
    band_idx = np.where(band)[0]
    k = int(band_idx[np.argmax(p[band])])
    if 0 < k < len(p) - 1 and p[k] > 0:
        lp = np.log(np.maximum(p[k - 1: k + 2], 1e-30))
        denom = lp[0] - 2 * lp[1] + lp[2]
        frac_bin = 0.5 * (lp[0] - lp[2]) / denom if abs(denom) > 1e-12 else 0.0
        frac_bin = max(-0.5, min(0.5, frac_bin))
        fpk = float((k + frac_bin) * (f[1] - f[0]))
    else:
        fpk = float(f[k])
    # Amplitude of the band component (rough): std of bandpassed signal
    frac = p[band].max() / max(p[(f > 0.05)].sum(), 1e-9)
    amp = float(math.sqrt(max(frac, 0.0)) * noise * 2.0)
    return noise, amp, fpk


def _fft_notch(sig: np.ndarray, f0: float, width_hz: float) -> np.ndarray:
    """Zero-phase band-stop via FFT bin masking (offline ID use only)."""
    if f0 <= 0.0:
        return sig
    spec = np.fft.rfft(sig)
    f = np.fft.rfftfreq(len(sig), DT)
    spec[np.abs(f - f0) <= width_hz] = 0.0
    return np.asarray(np.fft.irfft(spec, n=len(sig)))


def _regress_at_latency(
    rs: _Resampled, lat: float, notch_hz: float = 0.0
) -> tuple[PlantParams, float] | None:
    """Linear least-squares fit of the plant at a fixed latency.

    The model is LINEAR in the parameters once L is fixed:
        ax = g*pitch_d - (g*c)*x - (g*bp) - (g*mu)*vhat_x
        ay = -g*roll_d - (g*c)*y + (g*br) - (g*mu)*vhat_y
    so measured acceleration regresses onto [cmd, pos, 1, vhat] and
    theta = (g, g*c, g*bp, g*br, g*mu) falls out of one lstsq — convex,
    no basins (a replay-loss grid search demonstrably trapped far from
    the true minimum). Only MOVING samples enter (friction direction
    well-defined; kinetic regime). Returns (params, sse_per_sample).
    """
    # PREFILTER BOTH SIDES identically (standard system-ID hygiene):
    # the acceleration below carries the position box-smoothing, so the
    # command and position regressors get the same box — a one-sided
    # smoothing mismatch attenuates the fitted g (measured: -12%).
    kern = np.ones(5) / 5.0
    roll_d = np.convolve(delayed_commands(rs.t, rs.roll, lat), kern, "same")
    pitch_d = np.convolve(delayed_commands(rs.t, rs.pitch, lat), kern, "same")

    ax = np.asarray(np.gradient(rs.vx, DT))
    ay = np.asarray(np.gradient(rs.vy, DT))
    xs_r = rs.xs
    ys_r = rs.ys
    if notch_hz > 0.0:
        # The ball's self-rock feeds back through the controller, so
        # its acceleration is CORRELATED with the command regressor
        # (closed-loop ID bias). S0 measured its frequency — remove
        # that band from BOTH sides of the regression (zero-phase).
        w = 0.15
        ax = _fft_notch(ax, notch_hz, w)
        ay = _fft_notch(ay, notch_hz, w)
        roll_d = _fft_notch(roll_d, notch_hz, w)
        pitch_d = _fft_notch(pitch_d, notch_hz, w)
        xs_r = _fft_notch(xs_r, notch_hz, w)
        ys_r = _fft_notch(ys_r, notch_hz, w)
    speed = np.hypot(rs.vx, rs.vy)
    interior = np.zeros(len(rs.t), dtype=bool)
    interior[5:-5] = True
    moving = (speed > 15.0) & interior
    if moving.sum() < 200:
        return None

    n = int(moving.sum())
    zeros = np.zeros(n)
    ones = np.ones(n)
    vhx = rs.vx[moving] / speed[moving]
    vhy = rs.vy[moving] / speed[moving]

    # Stacked x-then-y design WITH the friction column (dropping it
    # biases g — friction opposes command-driven motion, so the omitted
    # variable correlates with the command regressor: measured -11%).
    a_x = np.column_stack(
        [pitch_d[moving], -xs_r[moving], -ones, zeros, -vhx]
    )
    a_y = np.column_stack(
        [-roll_d[moving], -ys_r[moving], zeros, ones, -vhy]
    )
    a = np.vstack([a_x, a_y])
    b = np.concatenate([ax[moving], ay[moving]])
    theta, _, _, _ = np.linalg.lstsq(a, b, rcond=None)
    g = float(theta[0])
    if g < 20.0:
        return None
    gc = float(theta[1])
    gbp = float(theta[2])
    gbr = float(theta[3])
    # DITHER LINEARIZATION: under a strong self-rock the ball never
    # truly sticks — the oscillation dithers away static friction, so
    # the regression's small coefficient is the plant's real EFFECTIVE
    # friction in that condition (the right value for design sims run
    # with the rock injected). No attempt is made to recover the
    # would-be no-dither cone; it is not the operative physics.
    mu = max(0.0, float(theta[4]) / g)

    params = PlantParams(
        g_eff=g,
        latency_s=float(lat),
        stiction_deg=mu,
        warp_c_deg_per_mm=max(0.0, gc / g),
        bias_pitch_deg=gbp / g,
        bias_roll_deg=gbr / g,
    )
    resid = b - a @ theta
    return params, float(np.mean(resid ** 2))


def fit_plant(
    rows: np.ndarray,
    script: ProbeScript | None = None,
    progress_cb: Callable[[float], None] | None = None,
    cancel: threading.Event | None = None,
) -> PlantFit | None:
    """Fit the plant to a probe recording. Returns None if cancelled or
    the recording is too short (< _MIN_COMPLETION of the script).

    Two stages: (1) latency scan — at each candidate L the remaining
    parameters are a CONVEX linear regression on the acceleration
    equation (see _regress_at_latency); parabolic refine on the L
    score. (2) the window-replay Huber residual at the fitted params
    (nonlinear, stiction-aware) is computed for confidence/reporting.
    """
    script = script or ProbeScript()
    if len(rows) < 60:
        return None
    duration = rows[-1, 0] - rows[0, 0]
    if duration < _MIN_COMPLETION * script.total_s:
        return None

    rs = _resample(rows)
    if len(rs.window_starts) < 8:
        return None
    noise_rms, rock_amp, rock_freq = _s0_stats(rows, script)

    # --- Stage 1: latency scan over a fine grid; convex sub-fit each.
    # The measured self-rock band is notched out of the regression when
    # prominent (>= 1 mm amplitude).
    notch_hz = rock_freq if rock_amp > 1.0 else 0.0
    lat_grid = np.arange(0.0, 0.201, 0.005)
    results: list[tuple[float, PlantParams] | None] = []
    scores: list[float] = []
    for i, lat in enumerate(lat_grid):
        if cancel is not None and cancel.is_set():
            return None
        r = _regress_at_latency(rs, float(lat), notch_hz=notch_hz)
        results.append(None if r is None else (r[1], r[0]))
        scores.append(math.inf if r is None else r[1])
        if progress_cb:
            progress_cb(0.8 * (i + 1) / len(lat_grid))
    if all(math.isinf(s) for s in scores):
        return None
    k = int(np.argmin(scores))
    best = results[k]
    assert best is not None
    p = best[1]

    # --- Stage 2: replay residual at the fit (confidence + report).
    best_loss = _loss_single(rs, p)
    if progress_cb:
        progress_cb(1.0)

    # --- Confidence
    n_res = len(rs.window_starts) * _WINDOW_FRAMES * 2
    residual_rms = math.sqrt(2.0 * best_loss / max(n_res, 1))
    nominal_loss = _loss_single(rs, PlantParams())
    improved = best_loss < 0.9 * nominal_loss
    on_bound = (
        p.g_eff <= 60.0 or p.g_eff >= 300.0
        or p.latency_s >= 0.199 or p.stiction_deg >= 0.79
    )
    low_conf = (not improved) or on_bound or (
        residual_rms > _RESIDUAL_CONFIDENCE_RATIO * max(noise_rms, 0.1)
    )
    notes = []
    if not improved:
        notes.append("no improvement over nominal")
    if on_bound:
        notes.append("parameter on bound")
    return PlantFit(
        params=p,
        residual_rms_mm=residual_rms,
        noise_rms_mm=noise_rms,
        low_confidence=low_conf,
        rock_amp_mm=rock_amp,
        rock_freq_hz=rock_freq,
        windows_used=len(rs.window_starts),
        filter_lag_s=_measure_filter_lag(rows),
        notes="; ".join(notes),
    )
