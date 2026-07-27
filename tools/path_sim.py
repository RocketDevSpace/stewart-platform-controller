"""
tools/path_sim.py

Closed-loop path-following feasibility simulation: a point-mass
ball-on-plate plant driven by the REAL control chain — AlphaBetaFilter2D
(the production tracker filter) -> BallController.compute_with_terms
(PathFollower + SetpointArbiter + PIDCore, fake clock) — with the tilt
command mapped back to ball acceleration through the same effective
gravity constant the autotuner uses (PD_AUTOTUNE_G_EFF).

CAVEAT: the plant model has no servo dynamics, no serial latency, and no
friction — every result here is an OPTIMISTIC bound on the real rig. Its
job is proving the adaptive pacing law stable and the PATH_SPEED_*
defaults sane, not predicting real tracking error to the millimetre.

No hardware, no camera, no Qt. Run from the repo root:

    python tools/path_sim.py
    python tools/path_sim.py --pattern "Star (5pt, r=70)" --duration 40
    python tools/path_sim.py --speed 10,20,30,40,50,60 --duration 30
"""

from __future__ import annotations

import argparse
import math
import random
import sys
from dataclasses import dataclass
from pathlib import Path as _FsPath
from typing import Callable

_REPO_ROOT = str(_FsPath(__file__).resolve().parents[1])
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import numpy as np  # noqa: E402

from control.ball_controller import BallController  # noqa: E402
from control.patterns import PATTERNS, Path, circle  # noqa: E402
from control.plant_model import (  # noqa: E402
    PlantParams,
    ServoLag,
    apply_rolling_resistance,
    plant_step,
)
from core.platform_state import BallState  # noqa: E402
from cv.measurement_filter import AlphaBetaFilter2D  # noqa: E402
from settings import (  # noqa: E402
    PD_AUTOTUNE_G_EFF,
    PD_DEFAULT_KD,
    PD_DEFAULT_KI,
    PD_DEFAULT_KP,
)
from tools.jitter_bench import FakeClock  # noqa: E402

# Effective plant gain: mm/s^2 of ball acceleration per degree of tilt.
G_EFF = float(PD_AUTOTUNE_G_EFF)


@dataclass(frozen=True)
class SimResult:
    """Feasibility metrics for one closed-loop path-following run."""

    max_err_mm: float
    mean_err_mm: float
    laps: int
    final_progress: float
    mean_advance_mm_s: float
    err_trace: list[float]


def _apply_rolling_resistance(
    ax: float,
    ay: float,
    vx: float,
    vy: float,
    roll_resist_deg: float,
) -> tuple[float, float, float, float]:
    """Delegates to control/plant_model.py (the ONE plant model — the
    system-ID fitter and gain designer replay through the same physics).
    Kept under the old name for existing importers."""
    return apply_rolling_resistance(ax, ay, vx, vy, roll_resist_deg, G_EFF)


def _path_total_len(path: Path) -> float:
    """Total polyline arc length (wrap segment included when closed) —
    the same total PathFollower normalizes path_s_mm against."""
    pts = np.asarray(path.points, dtype=np.float64)
    seg = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
    total = float(np.sum(seg))
    if path.closed:
        total += float(np.hypot(pts[0, 0] - pts[-1, 0], pts[0, 1] - pts[-1, 1]))
    return total


def simulate_path_following(
    path: Path,
    speed_mm_s: float,
    duration_s: float,
    hz: int = 30,
    kp: float = PD_DEFAULT_KP,
    kd: float = PD_DEFAULT_KD,
    noise_mm: float = 0.15,
    seed: int = 0,
    start_pos: tuple[float, float] | None = None,
    warp_c_deg_per_mm: float = 0.0,
    warp_bias_roll_deg: float = 0.0,
    warp_bias_pitch_deg: float = 0.0,
    integral_enabled: bool = True,
    roll_resist_deg: float = 0.06,
    latency_frames: int = 2,
) -> SimResult:
    """Run the closed loop for duration_s at hz steps/s; return metrics.

    Plant: point mass, semi-implicit Euler, acceleration
    ax = +G_EFF * pitch_cmd, ay = -G_EFF * roll_cmd — the inverse of
    pd_core's axis mapping (pitch_raw = +pd_x, roll_raw = -pd_y), so a
    positive pd_x accelerates the ball toward +x, closing the loop with
    the correct sign. Measurement noise is Gaussian(0, noise_mm) per
    axis, seeded for determinism; the production AlphaBetaFilter2D sits
    between the "camera" and the controller exactly as on the rig.

    Warp field (rig-measured 2026-07-23): the plate's required level
    compensation is position-dependent. warp_c_deg_per_mm models a
    center-attracting bowl — the disturbance tilt the controller must
    cancel at (x, y) is d_pitch = c*x, d_roll = -c*y (the rig measured
    c ≈ 0.0055: 0.36 deg at r=65, ball equilibrium 8 mm inside the
    circle). warp_bias_* adds a constant tilt error (stale saved trim).
    The plant feels commanded-minus-disturbance:
        ax = +G_EFF * (pitch_cmd - d_pitch)
        ay = -G_EFF * (roll_cmd  - d_roll)
    so with zero command the ball is pulled toward center + bias — the
    field that deadlocked path following on the rig.
    """
    if duration_s <= 0.0 or hz <= 0:
        raise ValueError("duration_s and hz must be positive")

    dt = 1.0 / float(hz)
    steps = int(round(duration_s * hz))
    rng = random.Random(seed)

    clock = FakeClock()
    ctrl = BallController(
        kp=kp, kd=kd, clock=clock, auto_trim_enabled=integral_enabled
    )
    ctrl.set_path(path)
    ctrl.set_path_speed(speed_mm_s)
    ctrl.start_path()

    filt = AlphaBetaFilter2D()  # one persistent instance, as on the rig

    if start_pos is None:
        x = float(path.points[0, 0])
        y = float(path.points[0, 1])
    else:
        x, y = float(start_pos[0]), float(start_pos[1])
    vx = 0.0
    vy = 0.0

    total_len = _path_total_len(path)
    initial_s: float | None = None
    final_s = 0.0
    laps = 0
    final_progress = 0.0
    err_trace: list[float] = []

    # Pipeline latency model (camera exposure + processing + serial +
    # servo motion ≈ 2-3 frames on the rig): the plant acts on the
    # command from latency_frames ago. This is what CONTROL_PREDICT_S /
    # PATH_FF_LOOKAHEAD_S exist to compensate.
    from collections import deque as _deque
    cmd_queue: _deque[tuple[float, float]] = _deque(
        [(0.0, 0.0)] * max(0, int(latency_frames))
    )

    plant = PlantParams(
        g_eff=G_EFF,
        latency_s=latency_frames * dt,   # informational; queue models it
        stiction_deg=roll_resist_deg,
        warp_c_deg_per_mm=warp_c_deg_per_mm,
        bias_roll_deg=warp_bias_roll_deg,
        bias_pitch_deg=warp_bias_pitch_deg,
    )

    for _ in range(steps):
        clock.t += dt

        measured_x = x + rng.gauss(0.0, noise_mm)
        measured_y = y + rng.gauss(0.0, noise_mm)
        fx, fy, fvx, fvy = filt.update(measured_x, measured_y, clock.t)

        ball_state = BallState(x_mm=fx, y_mm=fy, vx_mm_s=fvx, vy_mm_s=fvy)
        roll_cmd, pitch_cmd, terms = ctrl.compute_with_terms(ball_state)

        s_now = float(terms["path_s_mm"])
        if initial_s is None:
            initial_s = s_now
        final_s = s_now
        laps = int(terms["path_lap"])
        final_progress = float(terms["path_progress"])

        err = math.hypot(
            x - float(terms["target_x_mm"]), y - float(terms["target_y_mm"])
        )
        err_trace.append(err)

        # Delayed command reaches the plant this frame; the shared
        # plant model (warp + stiction + Euler) advances the ball.
        cmd_queue.append((roll_cmd, pitch_cmd))
        roll_act, pitch_act = cmd_queue.popleft()
        x, y, vx, vy = plant_step(
            x, y, vx, vy, roll_act, pitch_act, dt, plant
        )

    # Wrap-aware total arc advanced: path_s_mm wraps every lap on closed
    # paths, so unwrap via the lap count.
    total_advance = final_s + laps * total_len - (initial_s or 0.0)
    return SimResult(
        max_err_mm=max(err_trace),
        mean_err_mm=sum(err_trace) / len(err_trace),
        laps=laps,
        final_progress=final_progress,
        mean_advance_mm_s=total_advance / duration_s,
        err_trace=err_trace,
    )


# =====================================================================
# Harmonic orbit sim (2026-07-27)
# =====================================================================
# A richer plant than simulate_path_following's (which stays byte-
# identical — its pins are frozen): fractional-latency command replay +
# first-order servo lag (the gain_design._run_closed_loop pattern), on
# the rig-warp field. Both simulate_orbit and simulate_carrot_circle
# run through the SAME driver and compute the SAME metrics, so the
# smoothness A/B is honest.

# servo_tau_s here is a CHOSEN constant (the fit lives in autotune
# Apply, not persisted); every orbit bound is calibrated against it.
_ORBIT_PLANT = PlantParams(
    g_eff=G_EFF,
    latency_s=2.0 / 30.0,
    stiction_deg=0.06,
    warp_c_deg_per_mm=0.0055,
    bias_roll_deg=0.6,
    bias_pitch_deg=0.6,
    servo_tau_s=0.06,
)


@dataclass(frozen=True)
class OrbitSimResult:
    """Circle-quality metrics for one closed-loop run (orbit OR carrot
    follower — same fields, same computation)."""
    laps: int
    mean_radius_err_mm: float      # mean |r_ball - R|, last 2 laps
    radial_ripple_mm: float        # std(r_ball - R), last 2 laps
    lap_ripple_mm: list[float]     # per-lap std(r_ball - R) (learning curve)
    tangential_speed_std: float    # std(v . e_t), last 2 laps — smoothness
    max_err_mm: float              # max tracking error while settled
    err_trace: list[float]


class _CircleMetrics:
    """Per-lap radius/speed statistics from true ball state."""

    def __init__(self, radius_mm: float) -> None:
        self._r = float(radius_mm)
        self._by_lap: dict[int, list[tuple[float, float]]] = {}
        self.err_trace: list[float] = []
        self._max_err = 0.0

    def add(
        self, lap: int, x: float, y: float, vx: float, vy: float,
        track_err_mm: float, settled: bool,
    ) -> None:
        self.err_trace.append(track_err_mm)
        if not settled:
            return
        self._max_err = max(self._max_err, track_err_mm)
        r = math.hypot(x, y)
        v_tan = 0.0
        if r > 1e-6:
            v_tan = (-y * vx + x * vy) / r
        self._by_lap.setdefault(int(lap), []).append((r - self._r, v_tan))

    def result(self, min_samples: int = 60) -> OrbitSimResult:
        laps = sorted(
            k for k, v in self._by_lap.items() if len(v) >= min_samples
        )
        lap_ripple = [
            float(np.std([s[0] for s in self._by_lap[k]])) for k in laps
        ]
        last2 = [s for k in laps[-2:] for s in self._by_lap[k]]
        if last2:
            r_errs = np.array([s[0] for s in last2])
            v_tans = np.array([s[1] for s in last2])
            mean_r = float(np.mean(np.abs(r_errs)))
            ripple = float(np.std(r_errs))
            v_std = float(np.std(v_tans))
        else:
            mean_r = ripple = v_std = float("inf")
        return OrbitSimResult(
            laps=len(laps),
            mean_radius_err_mm=mean_r,
            radial_ripple_mm=ripple,
            lap_ripple_mm=lap_ripple,
            tangential_speed_std=v_std,
            max_err_mm=self._max_err,
            err_trace=self.err_trace,
        )


def _drive_circle_sim(
    ctrl: BallController,
    radius_mm: float,
    duration_s: float,
    hz: int,
    noise_mm: float,
    seed: int,
    plant: PlantParams,
    clock: FakeClock,
    settled_fn: Callable[[dict, int], bool],
    lap_fn: Callable[[dict], int],
    kick_at_s: float | None = None,
    kick_mm: float = 0.0,
) -> OrbitSimResult:
    """Shared closed-loop driver: real filter -> controller chain,
    fractional-latency command replay + ServoLag, rig-warp plant."""
    dt = 1.0 / float(hz)
    steps = int(round(duration_s * hz))
    rng = random.Random(seed)
    filt = AlphaBetaFilter2D()
    metrics = _CircleMetrics(radius_mm)

    x, y, vx, vy = float(radius_mm), 0.0, 0.0, 0.0
    roll_hist: list[float] = []
    pitch_hist: list[float] = []
    lag_roll = ServoLag(plant.servo_tau_s)
    lag_pitch = ServoLag(plant.servo_tau_s)
    lat_frames = plant.latency_s / dt   # fractional

    kick_step = (
        int(round(kick_at_s * hz)) if kick_at_s is not None else None
    )

    for i in range(steps):
        clock.t += dt
        if kick_step is not None and i == kick_step:
            x += kick_mm

        fx, fy, fvx, fvy = filt.update(
            x + rng.gauss(0.0, noise_mm),
            y + rng.gauss(0.0, noise_mm),
            clock.t,
        )
        roll_cmd, pitch_cmd, terms = ctrl.compute_with_terms(
            BallState(x_mm=fx, y_mm=fy, vx_mm_s=fvx, vy_mm_s=fvy)
        )
        roll_hist.append(roll_cmd)
        pitch_hist.append(pitch_cmd)

        # Fractional-latency lookup on the fixed frame grid, then the
        # first-order servo lag.
        k = i - lat_frames
        if k <= 0:
            roll_del = pitch_del = 0.0
        else:
            k0 = int(k)
            frac = k - k0
            k1 = min(k0 + 1, i)
            roll_del = (1 - frac) * roll_hist[k0] + frac * roll_hist[k1]
            pitch_del = (1 - frac) * pitch_hist[k0] + frac * pitch_hist[k1]
        roll_act = lag_roll.step(roll_del, dt)
        pitch_act = lag_pitch.step(pitch_del, dt)

        err = math.hypot(
            x - float(terms["target_x_mm"]), y - float(terms["target_y_mm"])
        )
        metrics.add(
            int(lap_fn(terms)), x, y, vx, vy, err, bool(settled_fn(terms, i)),
        )
        x, y, vx, vy = plant_step(x, y, vx, vy, roll_act, pitch_act, dt, plant)

    return metrics.result()


def simulate_orbit(
    radius_mm: float = 50.0,
    speed_mm_s: float = 40.0,
    duration_s: float = 90.0,
    hz: int = 30,
    kp: float = PD_DEFAULT_KP,
    kd: float = PD_DEFAULT_KD,
    ki: float = PD_DEFAULT_KI,
    noise_mm: float = 0.15,
    seed: int = 0,
    plant: PlantParams | None = None,
    kick_at_s: float | None = None,
    kick_mm: float = 0.0,
    cone: bool = False,
) -> OrbitSimResult:
    """Closed-loop harmonic-orbit run on the rig-warp + servo-lag plant.

    cone=False pins the CLOSED-LOOP machinery (reference + ILC), which
    stays available behind ORBIT_CONE_ONLY; cone=True runs the shipping
    open-loop cone mode."""
    clock = FakeClock()
    ctrl = BallController(kp=kp, kd=kd, ki=ki, clock=clock)
    ctrl._orbit.cone_only = bool(cone)
    ctrl.set_orbit_radius(radius_mm)
    ctrl.set_orbit_speed(speed_mm_s)
    ctrl.start_orbit()
    settled_states = ("cone",) if cone else ("track",)
    return _drive_circle_sim(
        ctrl, radius_mm, duration_s, hz, noise_mm, seed,
        plant if plant is not None else _ORBIT_PLANT, clock,
        settled_fn=lambda terms, i: terms["orbit_state"] in settled_states,
        lap_fn=lambda terms: terms["orbit_lap"],
        kick_at_s=kick_at_s, kick_mm=kick_mm,
    )


def simulate_carrot_circle(
    radius_mm: float = 50.0,
    speed_mm_s: float = 40.0,
    duration_s: float = 90.0,
    hz: int = 30,
    kp: float = PD_DEFAULT_KP,
    kd: float = PD_DEFAULT_KD,
    ki: float = PD_DEFAULT_KI,
    noise_mm: float = 0.15,
    seed: int = 0,
    plant: PlantParams | None = None,
) -> OrbitSimResult:
    """The carrot path follower on circle(radius_mm), through the SAME
    driver/plant/metrics as simulate_orbit — the honest smoothness A/B.
    "Settled" = past a 5 s pursuit spin-up (the follower has no state
    machine to gate on)."""
    clock = FakeClock()
    ctrl = BallController(kp=kp, kd=kd, ki=ki, clock=clock)
    ctrl.set_path(circle(radius_mm=radius_mm))
    ctrl.set_path_speed(speed_mm_s)
    ctrl.start_path()
    settle_steps = 5 * hz
    return _drive_circle_sim(
        ctrl, radius_mm, duration_s, hz, noise_mm, seed,
        plant if plant is not None else _ORBIT_PLANT, clock,
        settled_fn=lambda terms, i: i >= settle_steps,
        lap_fn=lambda terms: terms["path_lap"],
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Path-following feasibility sim (no servo dynamics, latency, "
            "or friction — an optimistic bound)."
        )
    )
    parser.add_argument(
        "--pattern",
        choices=sorted(PATTERNS),
        default="Circle (r=65)",
        help="pattern label from control/patterns.PATTERNS",
    )
    parser.add_argument(
        "--speed",
        default="10,20,30,40,50",
        help="comma-separated speed sweep in mm/s",
    )
    parser.add_argument(
        "--duration", type=float, default=30.0, help="sim duration per speed, s"
    )
    parser.add_argument(
        "--warp-c", type=float, default=0.0,
        help="bowl warp coefficient, deg of tilt error per mm from center "
             "(rig-measured ~0.0055)",
    )
    parser.add_argument(
        "--warp-bias-roll", type=float, default=0.0,
        help="constant roll tilt error, deg (stale trim)",
    )
    parser.add_argument(
        "--warp-bias-pitch", type=float, default=0.0,
        help="constant pitch tilt error, deg (stale trim)",
    )
    parser.add_argument(
        "--orbit", action="store_true",
        help="run the harmonic-orbit A/B (orbit vs carrot follower on "
             "the rig-warp + servo-lag plant) instead of the path sweep",
    )
    parser.add_argument(
        "--radius", type=float, default=50.0, help="orbit radius, mm"
    )
    args = parser.parse_args(argv)

    if args.orbit:
        speeds = [float(s) for s in args.speed.split(",")]
        print(f"=== orbit A/B: r={args.radius:g} mm, {args.duration:g} s ===")
        for v in speeds:
            o = simulate_orbit(args.radius, v, args.duration)
            c = simulate_carrot_circle(args.radius, v, args.duration)
            print(
                f"v={v:5.1f}  orbit: laps {o.laps} ripple "
                f"{o.radial_ripple_mm:5.2f} mm vstd {o.tangential_speed_std:6.2f}"
                f"   carrot: laps {c.laps} ripple {c.radial_ripple_mm:5.2f} "
                f"mm vstd {c.tangential_speed_std:6.2f}"
            )
        return 0

    path = PATTERNS[args.pattern]()
    speeds = [float(s) for s in args.speed.split(",")]

    warp_note = (
        f" warp c={args.warp_c:g} bias=({args.warp_bias_roll:g},"
        f"{args.warp_bias_pitch:g})"
        if (args.warp_c or args.warp_bias_roll or args.warp_bias_pitch)
        else ""
    )
    print(
        f"=== path_sim: {path.name} ({args.duration:g} s per run)"
        f"{warp_note} ==="
    )
    header = (
        f"{'speed':>8} {'max_err':>9} {'mean_err':>9} {'laps':>5} "
        f"{'mean_advance':>13}"
    )
    print(header)
    print(f"{'mm/s':>8} {'mm':>9} {'mm':>9} {'':>5} {'mm/s':>13}")
    ok = True
    for v in speeds:
        res = simulate_path_following(
            path, v, args.duration,
            warp_c_deg_per_mm=args.warp_c,
            warp_bias_roll_deg=args.warp_bias_roll,
            warp_bias_pitch_deg=args.warp_bias_pitch,
        )
        ok = ok and all(
            math.isfinite(m)
            for m in (res.max_err_mm, res.mean_err_mm, res.mean_advance_mm_s)
        )
        print(
            f"{v:>8.1f} {res.max_err_mm:>9.2f} {res.mean_err_mm:>9.2f} "
            f"{res.laps:>5d} {res.mean_advance_mm_s:>13.2f}"
        )
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
