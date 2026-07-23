"""
control/plant_model.py

Ball-on-plate plant physics — the ONE model shared by the feasibility
sim (tools/path_sim.py), the system-ID fitter (control/plant_id.py),
and the gain-design search (control/gain_design.py).

Model (rig-derived, 2026-07-23):
    d_pitch = warp_c * x + bias_pitch          # plate warp + stale trim
    d_roll  = -warp_c * y + bias_roll
    ax = g_eff * (pitch_act - d_pitch)         # mm/s^2 per deg
    ay = -g_eff * (roll_act - d_roll)
plus Coulomb-style rolling resistance (a slow ball under net tilt inside
the resistance cone STICKS), semi-implicit Euler at the loop rate, and
command latency. The closed-loop sim models latency as an integer frame
queue; the ID fitter uses CONTINUOUS latency via linear interpolation of
the recorded command series at t - L (the true value sits between
frames; quantizing would alias into g_eff).

Pure numpy, no Qt, no hardware.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PlantParams:
    """The identified (or assumed) plant."""

    g_eff: float = 171.0            # mm/s^2 of ball accel per deg of tilt
    latency_s: float = 2.0 / 30.0   # full-loop command latency (pure delay)
    stiction_deg: float = 0.06      # rolling-resistance cone, deg equiv
    warp_c_deg_per_mm: float = 0.0  # center-attracting bowl coefficient
    bias_roll_deg: float = 0.0      # constant tilt error (stale trim)
    bias_pitch_deg: float = 0.0
    # First-order ACTUATOR lag (servo response), separate from the pure
    # delay: the servos take tens of ms to reach a commanded angle,
    # which rounds command edges. A fit that lacks this soaks the
    # attenuation into a falsely LOW g_eff — the 2026-07-23 rig session
    # fitted g=104 vs ~171 real and the gain design "compensated" into
    # violent oscillation. tau=0 = ideal actuator (legacy behavior).
    servo_tau_s: float = 0.0


def lag_filter(series: np.ndarray, dt: float, tau_s: float) -> np.ndarray:
    """First-order lag applied to a command series (offline/fit use)."""
    if tau_s <= 0.0:
        return series
    alpha = 1.0 - math.exp(-dt / tau_s)
    out = np.empty_like(series)
    acc = float(series[0])
    for i in range(len(series)):
        acc += alpha * (float(series[i]) - acc)
        out[i] = acc
    return out


class ServoLag:
    """First-order actuator state for closed-loop sims."""

    def __init__(self, tau_s: float, initial: float = 0.0) -> None:
        self._tau = float(tau_s)
        self.value = float(initial)

    def step(self, cmd: float, dt: float) -> float:
        if self._tau <= 0.0:
            self.value = float(cmd)
        else:
            alpha = 1.0 - math.exp(-dt / self._tau)
            self.value += alpha * (float(cmd) - self.value)
        return self.value


def apply_rolling_resistance(
    ax: float,
    ay: float,
    vx: float,
    vy: float,
    roll_resist_deg: float,
    g_eff: float,
) -> tuple[float, float, float, float]:
    """Coulomb-style rolling resistance, roll_resist_deg equivalent tilt.

    Slow ball + net tilt inside the resistance cone -> sticks (velocity
    zeroed, no acceleration). Moving ball -> constant deceleration
    g_eff * roll_resist_deg opposing the velocity. Returns possibly
    modified (ax, ay, vx, vy).
    """
    if roll_resist_deg <= 0.0:
        return ax, ay, vx, vy
    a_res = g_eff * roll_resist_deg
    speed = math.hypot(vx, vy)
    if speed < 0.5 and math.hypot(ax, ay) <= a_res:
        return 0.0, 0.0, 0.0, 0.0
    if speed > 1e-9:
        ax -= a_res * vx / speed
        ay -= a_res * vy / speed
    return ax, ay, vx, vy


def plant_step(
    x: float,
    y: float,
    vx: float,
    vy: float,
    roll_act: float,
    pitch_act: float,
    dt: float,
    params: PlantParams,
) -> tuple[float, float, float, float]:
    """One semi-implicit Euler step; returns the new (x, y, vx, vy).

    The arithmetic mirrors the original tools/path_sim.py inline plant
    exactly (same expression order) so the feasibility pins stay
    bit-identical through the extraction.
    """
    d_pitch = params.warp_c_deg_per_mm * x + params.bias_pitch_deg
    d_roll = -params.warp_c_deg_per_mm * y + params.bias_roll_deg
    ax = params.g_eff * (pitch_act - d_pitch)
    ay = -params.g_eff * (roll_act - d_roll)
    ax, ay, vx, vy = apply_rolling_resistance(
        ax, ay, vx, vy, params.stiction_deg, params.g_eff
    )
    vx += ax * dt
    vy += ay * dt
    x += vx * dt
    y += vy * dt
    return x, y, vx, vy


def delayed_commands(
    t: np.ndarray, cmds: np.ndarray, latency_s: float
) -> np.ndarray:
    """The command series as seen by the plant: cmds(t - latency_s) by
    linear interpolation, clamped to the first sample before the start
    (the plant held the pre-recording command)."""
    return np.asarray(
        np.interp(t - latency_s, t, cmds, left=float(cmds[0]))
    )


def replay(
    t: np.ndarray,
    roll_cmd: np.ndarray,
    pitch_cmd: np.ndarray,
    x0: float,
    y0: float,
    vx0: float,
    vy0: float,
    params: PlantParams,
) -> tuple[np.ndarray, np.ndarray]:
    """Open-loop replay of a recorded command series from an initial
    state; returns simulated positions (xs, ys) aligned with t.

    Latency is CONTINUOUS (delayed_commands). Scalar reference
    implementation — replay_batch is the vectorized equivalent the fit
    grid uses.
    """
    r_act = delayed_commands(t, roll_cmd, params.latency_s)
    p_act = delayed_commands(t, pitch_cmd, params.latency_s)
    n = len(t)
    xs = np.empty(n)
    ys = np.empty(n)
    x, y, vx, vy = float(x0), float(y0), float(vx0), float(vy0)
    xs[0] = x
    ys[0] = y
    for i in range(1, n):
        dt = float(t[i] - t[i - 1])
        x, y, vx, vy = plant_step(
            x, y, vx, vy, float(r_act[i - 1]), float(p_act[i - 1]),
            dt, params,
        )
        xs[i] = x
        ys[i] = y
    return xs, ys


def replay_batch(
    dt: float,
    roll_act: np.ndarray,
    pitch_act: np.ndarray,
    x0: np.ndarray,
    y0: np.ndarray,
    vx0: np.ndarray,
    vy0: np.ndarray,
    g_eff: np.ndarray,
    stiction_deg: np.ndarray,
    warp_c: np.ndarray,
    bias_roll: np.ndarray,
    bias_pitch: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Vectorized open-loop replay of K trajectories in lockstep.

    Shapes: roll_act/pitch_act (K, T) — already latency-shifted per
    trajectory; every other input (K,). Returns (xs, ys) of shape
    (K, T). Fixed dt (the fitter resamples windows onto the frame
    grid). Numerically equivalent to looping `replay` per trajectory.
    """
    k, n = roll_act.shape
    xs = np.empty((k, n))
    ys = np.empty((k, n))
    x = x0.astype(np.float64).copy()
    y = y0.astype(np.float64).copy()
    vx = vx0.astype(np.float64).copy()
    vy = vy0.astype(np.float64).copy()
    xs[:, 0] = x
    ys[:, 0] = y
    a_res = g_eff * stiction_deg
    for i in range(1, n):
        d_pitch = warp_c * x + bias_pitch
        d_roll = -warp_c * y + bias_roll
        ax = g_eff * (pitch_act[:, i - 1] - d_pitch)
        ay = -g_eff * (roll_act[:, i - 1] - d_roll)
        speed = np.hypot(vx, vy)
        acc = np.hypot(ax, ay)
        stick = (speed < 0.5) & (acc <= a_res)
        moving = speed > 1e-9
        inv = np.where(moving, 1.0 / np.maximum(speed, 1e-9), 0.0)
        ax = np.where(stick, 0.0, ax - np.where(moving, a_res * vx * inv, 0.0))
        ay = np.where(stick, 0.0, ay - np.where(moving, a_res * vy * inv, 0.0))
        vx = np.where(stick, 0.0, vx) + ax * dt
        vy = np.where(stick, 0.0, vy) + ay * dt
        x = x + vx * dt
        y = y + vy * dt
        xs[:, i] = x
        ys[:, i] = y
    return xs, ys
