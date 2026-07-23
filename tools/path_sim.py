"""
tools/path_sim.py

Closed-loop path-following feasibility simulation: a point-mass
ball-on-plate plant driven by the REAL control chain — AlphaBetaFilter2D
(the production tracker filter) -> BallController.compute_with_terms
(PathFollower + SetpointArbiter + PDCore, fake clock) — with the tilt
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

_REPO_ROOT = str(_FsPath(__file__).resolve().parents[1])
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import numpy as np  # noqa: E402

from control.ball_controller import BallController  # noqa: E402
from control.patterns import PATTERNS, Path  # noqa: E402
from core.platform_state import BallState  # noqa: E402
from cv.measurement_filter import AlphaBetaFilter2D  # noqa: E402
from settings import (  # noqa: E402
    PD_AUTOTUNE_G_EFF,
    PD_DEFAULT_KD,
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
    ctrl = BallController(kp=kp, kd=kd, clock=clock)
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

        # Plant step (semi-implicit Euler) against the warp field.
        d_pitch = warp_c_deg_per_mm * x + warp_bias_pitch_deg
        d_roll = -warp_c_deg_per_mm * y + warp_bias_roll_deg
        ax = G_EFF * (pitch_cmd - d_pitch)
        ay = -G_EFF * (roll_cmd - d_roll)
        vx += ax * dt
        vy += ay * dt
        x += vx * dt
        y += vy * dt

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
    args = parser.parse_args(argv)

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
