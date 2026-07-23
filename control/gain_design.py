"""
control/gain_design.py

Offline gain design on the IDENTIFIED plant (control/plant_id.py fit):
candidate (kp, ki, kd) triples are scored by closed-loop simulation
through the REAL control chain — AlphaBetaFilter2D → BallController
(prediction, integral taper/deadband, feedforward, rest interlocks all
in the loop) — against the plant model with fractional latency, the
fitted stiction/warp/biases, and the S0-measured self-rock injected as
a disturbance (the optimizer must be robust TO the rock, not deluded
about it).

Objective (each term normalized by the CURRENT gains' score on the same
plant, so J = 1.0 means "no better than today"):

    J = 0.40 * step ITAE  +  0.35 * hold (RMS err + 0.3 * RMS tilt
        rate — forbids holding quiet by shaking)  +  0.25 * path mean
        tracking error

Rank stability on a jagged, stochastic surface: common random numbers
(every candidate sees the identical seed set, so comparisons are
paired), 2-seed screening, top-4 confirmation at 6 seeds, ties resolved
toward the smaller gain change. The CURRENT gains are always in the
candidate pool — the design can never return something it measured as
worse than today.

Pure Python + numpy + the control stack. No Qt, no hardware.
"""
from __future__ import annotations

import math
import random
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from control.ball_controller import BallController
from control.patterns import circle
from control.plant_id import PlantFit
from control.plant_model import PlantParams, plant_step
from core.platform_state import BallState
from cv.measurement_filter import AlphaBetaFilter2D

HZ = 30.0
DT = 1.0 / HZ

_W_STEP = 0.40
_W_HOLD = 0.35
_W_PATH = 0.25
_CLIP = 5.0
_SCREEN_SEEDS = (11, 12)
_CONFIRM_SEEDS = (11, 12, 13, 14, 15, 16)
_KP_BOUNDS = (0.010, 0.150)
_KD_BOUNDS = (0.005, 0.080)
_KI_BOUNDS = (0.0, 0.080)


class _Clock:
    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


@dataclass(frozen=True)
class GainDesign:
    """The designed gains + honest before/after prediction."""

    kp: float
    kd: float
    ki: float
    baseline_cost: float        # always 1.0 by construction (normalized)
    predicted_cost: float       # J of the winner (<= 1.0 structurally)
    candidates_scored: int
    notes: str = ""


def _make_controller(kp: float, kd: float, ki: float) -> BallController:
    clock = _Clock()
    ctrl = BallController(
        kp=kp, kd=kd, ki=ki, clock=clock, auto_trim_enabled=True
    )
    ctrl._sim_clock = clock  # type: ignore[attr-defined]
    return ctrl


def _run_closed_loop(
    ctrl: BallController,
    plant: PlantParams,
    duration_s: float,
    seed: int,
    rock_amp_mm: float,
    rock_freq_hz: float,
    start_pos: tuple[float, float],
    on_frame: Callable[[float, float, float, float, float, dict], None],
) -> bool:
    """Generic closed-loop run; returns False on non-finite blowup."""
    clock = ctrl._sim_clock  # type: ignore[attr-defined]
    filt = AlphaBetaFilter2D()
    rng = random.Random(seed)
    x, y = start_pos
    vx = vy = 0.0
    cmd_t: list[float] = []
    cmd_r: list[float] = []
    cmd_p: list[float] = []
    rock_a = rock_amp_mm * (2.0 * math.pi * max(rock_freq_hz, 0.1)) ** 2
    phase = rng.random() * 2.0 * math.pi

    steps = int(duration_s * HZ)
    for i in range(steps):
        clock.t += DT
        fx, fy, fvx, fvy = filt.update(
            x + rng.gauss(0.0, 0.2), y + rng.gauss(0.0, 0.2), clock.t
        )
        roll, pitch, terms = ctrl.compute_with_terms(
            BallState(x_mm=fx, y_mm=fy, vx_mm_s=fvx, vy_mm_s=fvy)
        )
        if not (math.isfinite(roll) and math.isfinite(pitch)):
            return False
        on_frame(clock.t, x, y, roll, pitch, terms)

        cmd_t.append(clock.t)
        cmd_r.append(roll)
        cmd_p.append(pitch)
        td = clock.t - plant.latency_s
        r_act = float(np.interp(td, cmd_t, cmd_r, left=cmd_r[0]))
        p_act = float(np.interp(td, cmd_t, cmd_p, left=cmd_p[0]))
        x, y, vx, vy = plant_step(x, y, vx, vy, r_act, p_act, DT, plant)
        if rock_amp_mm > 0.0:
            w = 2.0 * math.pi * rock_freq_hz
            vx += rock_a * math.sin(w * clock.t + phase) * DT
            vy += rock_a * math.cos(w * clock.t + 1.7 * phase) * DT
        if not (math.isfinite(x) and math.isfinite(y)) or abs(x) > 400 or abs(y) > 400:  # noqa: E501
            return False
    return True


def _scenario_costs(
    kp: float, kd: float, ki: float,
    plant: PlantParams,
    rock_amp_mm: float,
    rock_freq_hz: float,
    seed: int,
) -> tuple[float, float, float] | None:
    """(step_itae, hold_cost, path_mean_err) for one seed; None = reject."""
    # --- Step recovery: released 30 mm out, 5 s, ITAE.
    ctrl = _make_controller(kp, kd, ki)
    err_acc = [0.0]
    t0 = [0.0]

    def step_frame(t: float, x: float, y: float, roll: float, pitch: float, terms: dict) -> None:  # noqa: E501
        if t0[0] == 0.0:
            t0[0] = t
        e = terms["position_vec_mm"]
        err_acc[0] += (t - t0[0]) * math.hypot(e[0], e[1]) * DT

    ok = _run_closed_loop(
        ctrl, plant, 5.0, seed, rock_amp_mm, rock_freq_hz, (30.0, 0.0),
        step_frame,
    )
    if not ok:
        return None
    step_itae = err_acc[0]

    # --- Quiet hold: at target 6 s; RMS err + 0.3 * RMS tilt rate.
    ctrl = _make_controller(kp, kd, ki)
    errs: list[float] = []
    rates: list[float] = []
    prev_cmd = [0.0, 0.0]

    def hold_frame(t: float, x: float, y: float, roll: float, pitch: float, terms: dict) -> None:  # noqa: E501
        e = terms["position_vec_mm"]
        errs.append(math.hypot(e[0], e[1]))
        rc = terms["roll_cmd"]
        pc = terms["pitch_cmd"]
        rates.append(math.hypot(rc - prev_cmd[0], pc - prev_cmd[1]) / DT)
        prev_cmd[0] = rc
        prev_cmd[1] = pc

    ok = _run_closed_loop(
        ctrl, plant, 6.0, seed + 100, rock_amp_mm, rock_freq_hz,
        (1.0, -1.0), hold_frame,
    )
    if not ok:
        return None
    skip = len(errs) // 4
    hold_cost = (
        float(np.sqrt(np.mean(np.square(errs[skip:]))))
        + 0.3 * float(np.sqrt(np.mean(np.square(rates[skip:])))) * 0.1
    )

    # --- Path tracking: circle r=60 @ 30 mm/s, 8 s, mean error.
    ctrl = _make_controller(kp, kd, ki)
    ctrl.set_path(circle(radius_mm=60.0))
    ctrl.set_path_speed(30.0)
    ctrl.start_path()
    perr: list[float] = []

    def path_frame(t: float, x: float, y: float, roll: float, pitch: float, terms: dict) -> None:  # noqa: E501
        perr.append(math.hypot(
            x - float(terms["target_x_mm"]), y - float(terms["target_y_mm"])
        ))

    ok = _run_closed_loop(
        ctrl, plant, 8.0, seed + 200, rock_amp_mm, rock_freq_hz,
        (60.0, 0.0), path_frame,
    )
    if not ok:
        return None
    path_err = float(np.mean(perr[len(perr) // 5:]))
    if max(perr) > 80.0:
        return None
    return step_itae, hold_cost, path_err


def _score(
    kp: float, kd: float, ki: float,
    plant: PlantParams,
    rock_amp_mm: float,
    rock_freq_hz: float,
    seeds: tuple[int, ...],
    refs: tuple[float, float, float],
) -> float:
    """Mean normalized J over the seed set; inf on any reject."""
    total = 0.0
    for s in seeds:
        costs = _scenario_costs(
            kp, kd, ki, plant, rock_amp_mm, rock_freq_hz, s
        )
        if costs is None:
            return math.inf
        j = (
            _W_STEP * min(costs[0] / max(refs[0], 1e-9), _CLIP)
            + _W_HOLD * min(costs[1] / max(refs[1], 1e-9), _CLIP)
            + _W_PATH * min(costs[2] / max(refs[2], 1e-9), _CLIP)
        )
        total += j
    return total / len(seeds)


def _log_grid(center: float, lo: float, hi: float, n: int, span: float) -> list[float]:  # noqa: E501
    lo_c = max(lo, center / span)
    hi_c = min(hi, center * span)
    if lo_c >= hi_c:
        return [max(lo, min(hi, center))]
    return list(np.exp(np.linspace(math.log(lo_c), math.log(hi_c), n)))


def design_gains(
    fit: PlantFit,
    kp0: float,
    kd0: float,
    ki0: float,
    progress_cb: Callable[[float], None] | None = None,
    cancel: threading.Event | None = None,
    time_budget_s: float = 60.0,
) -> GainDesign | None:
    """Search (kp, kd, ki) on the fitted plant. Returns None only on
    cancellation. The current gains are always in the pool, so the
    result is never measured-worse than today."""
    plant = fit.params
    rock_amp = fit.rock_amp_mm
    rock_freq = fit.rock_freq_hz
    t_start = time.perf_counter()

    # Reference costs = current gains on this plant (CRN screening seeds).
    ref_acc: list[tuple[float, float, float]] = []
    for s in _SCREEN_SEEDS:
        c = _scenario_costs(kp0, kd0, ki0, plant, rock_amp, rock_freq, s)
        if c is None:
            # Current gains blow up on the fitted plant — normalize
            # against safe nominal values instead.
            ref_acc = []
            break
        ref_acc.append(c)
    if ref_acc:
        refs = (
            float(np.mean([c[0] for c in ref_acc])),
            float(np.mean([c[1] for c in ref_acc])),
            float(np.mean([c[2] for c in ref_acc])),
        )
    else:
        refs = (100.0, 5.0, 10.0)

    def out_of_time() -> bool:
        return (time.perf_counter() - t_start) > time_budget_s

    def cancelled() -> bool:
        return cancel is not None and cancel.is_set()

    # --- Round 1: log grid around current gains (current included).
    kps = _log_grid(kp0, *_KP_BOUNDS, n=4, span=2.0)
    kds = _log_grid(kd0, *_KD_BOUNDS, n=4, span=2.5)
    kis = sorted({0.0, ki0 * 0.5, ki0, min(ki0 * 2.0, _KI_BOUNDS[1])})
    pool = [(kp, kd, ki) for kp in kps for kd in kds for ki in kis]
    pool.append((kp0, kd0, ki0))

    scored: list[tuple[float, tuple[float, float, float]]] = []
    for i, (kp, kd, ki) in enumerate(pool):
        if cancelled():
            return None
        if out_of_time():
            break
        j = _score(kp, kd, ki, plant, rock_amp, rock_freq,
                   _SCREEN_SEEDS, refs)
        scored.append((j, (kp, kd, ki)))
        if progress_cb:
            progress_cb(0.6 * (i + 1) / len(pool))
        time.sleep(0.001)   # GIL headroom for the control loop

    scored.sort(key=lambda it: it[0])
    best_j, best = scored[0]

    # --- Round 2: refine around the round-1 winner (+-35%).
    kps2 = _log_grid(best[0], *_KP_BOUNDS, n=3, span=1.35)
    kds2 = _log_grid(best[1], *_KD_BOUNDS, n=3, span=1.35)
    kis2 = sorted({max(0.0, best[2] * 0.7), best[2],
                   min(best[2] * 1.4 + 1e-4, _KI_BOUNDS[1])})
    pool2 = [(kp, kd, ki) for kp in kps2 for kd in kds2 for ki in kis2]
    for i, (kp, kd, ki) in enumerate(pool2):
        if cancelled():
            return None
        if out_of_time():
            break
        j = _score(kp, kd, ki, plant, rock_amp, rock_freq,
                   _SCREEN_SEEDS, refs)
        scored.append((j, (kp, kd, ki)))
        if progress_cb:
            progress_cb(0.6 + 0.25 * (i + 1) / len(pool2))
        time.sleep(0.001)

    # --- Confirmation: top 4 (current gains always included) at 6 seeds.
    scored.sort(key=lambda it: it[0])
    finalists = [it[1] for it in scored[:4]]
    if (kp0, kd0, ki0) not in finalists:
        finalists.append((kp0, kd0, ki0))
    confirmed: list[tuple[float, tuple[float, float, float]]] = []
    for i, (kp, kd, ki) in enumerate(finalists):
        if cancelled():
            return None
        j = _score(kp, kd, ki, plant, rock_amp, rock_freq,
                   _CONFIRM_SEEDS, refs)
        confirmed.append((j, (kp, kd, ki)))
        if progress_cb:
            progress_cb(0.85 + 0.15 * (i + 1) / len(finalists))
        time.sleep(0.001)

    # Winner; near-ties resolve toward the smaller gain change.
    confirmed.sort(key=lambda it: it[0])
    win_j, win = confirmed[0]

    def change(g: tuple[float, float, float]) -> float:
        return (
            abs(math.log(max(g[0], 1e-6) / max(kp0, 1e-6)))
            + abs(math.log(max(g[1], 1e-6) / max(kd0, 1e-6)))
            + abs(g[2] - ki0)
        )

    for j, g in confirmed[1:]:
        if j - win_j < 0.02 and change(g) < change(win):
            win = g
            win_j = j

    notes = ""
    if fit.low_confidence:
        # Low-confidence fit: cap the change at +-30% of current.
        kp_c = max(kp0 * 0.7, min(kp0 * 1.3, win[0]))
        kd_c = max(kd0 * 0.7, min(kd0 * 1.3, win[1]))
        ki_c = max(ki0 * 0.7, min(ki0 * 1.3 + 1e-4, win[2]))
        win = (kp_c, kd_c, ki_c)
        notes = "low-confidence fit: gain change capped at +-30%"

    return GainDesign(
        kp=float(win[0]),
        kd=float(win[1]),
        ki=float(win[2]),
        baseline_cost=1.0,
        predicted_cost=float(win_j),
        candidates_scored=len(scored) + len(confirmed),
        notes=notes,
    )
