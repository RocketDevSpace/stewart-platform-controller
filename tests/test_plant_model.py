"""
Unit tests for control/plant_model.py — the shared ball-on-plate plant.

Pins: the frame-queue equivalence of fractional latency at exact frame
multiples, scalar/vectorized replay equivalence across parameter
variety, stiction stick/slip behavior, and delayed-command
interpolation. The extraction itself is pinned by the feasibility suite
(tools/path_sim.py delegates here and its recorded numbers must not
move).
"""
import numpy as np
import pytest

from control.plant_model import (
    PlantParams,
    delayed_commands,
    plant_step,
    replay,
    replay_batch,
)

HZ = 30.0
DT = 1.0 / HZ


def _cmd_series(n: int = 120, seed: int = 3) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    rng = np.random.default_rng(seed)
    t = np.arange(n) * DT
    # Square-ish toggles + noise — transient-rich like a probe.
    roll = 0.4 * np.sign(np.sin(2 * np.pi * 0.4 * t)) + 0.05 * rng.standard_normal(n)
    pitch = 0.3 * np.sign(np.cos(2 * np.pi * 0.3 * t)) + 0.05 * rng.standard_normal(n)
    return t, roll, pitch


class TestDelayedCommands:
    def test_zero_latency_is_identity(self) -> None:
        t, roll, _ = _cmd_series()
        assert np.allclose(delayed_commands(t, roll, 0.0), roll)

    def test_one_frame_latency_shifts_by_one(self) -> None:
        t, roll, _ = _cmd_series()
        d = delayed_commands(t, roll, DT)
        assert np.allclose(d[1:], roll[:-1])
        assert d[0] == roll[0]          # pre-start clamp

    def test_half_frame_latency_interpolates(self) -> None:
        t = np.arange(4) * DT
        c = np.array([0.0, 1.0, 2.0, 3.0])
        d = delayed_commands(t, c, DT / 2)
        assert d[2] == pytest.approx(1.5)


class TestFrameQueueEquivalence:
    def test_fractional_latency_matches_integer_queue(self) -> None:
        """L = 2 frames via interpolation == the sim's 2-frame queue."""
        t, roll, pitch = _cmd_series()
        params = PlantParams(
            g_eff=171.0, latency_s=2 * DT, stiction_deg=0.06,
            warp_c_deg_per_mm=0.0055, bias_roll_deg=0.2, bias_pitch_deg=-0.1,
        )
        xs, ys = replay(t, roll, pitch, 5.0, -3.0, 0.0, 0.0, params)

        # Reference: manual integration with an integer command queue.
        from collections import deque
        q = deque([(roll[0], pitch[0])] * 2)
        x, y, vx, vy = 5.0, -3.0, 0.0, 0.0
        ref_x = [x]
        ref_y = [y]
        for i in range(1, len(t)):
            q.append((roll[i - 1], pitch[i - 1]))
            r_act, p_act = q.popleft()
            x, y, vx, vy = plant_step(x, y, vx, vy, r_act, p_act, DT, params)
            ref_x.append(x)
            ref_y.append(y)
        assert np.allclose(xs, ref_x, atol=1e-9)
        assert np.allclose(ys, ref_y, atol=1e-9)


class TestBatchEquivalence:
    def test_batch_matches_scalar_across_param_variety(self) -> None:
        t, roll, pitch = _cmd_series()
        combos = [
            PlantParams(120.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            PlantParams(171.0, 0.05, 0.06, 0.0055, 0.2, -0.1),
            PlantParams(220.0, 0.12, 0.3, 0.002, -0.15, 0.25),
            PlantParams(171.0, 0.0333, 0.5, 0.0, 0.0, 0.4),
        ]
        k = len(combos)
        roll_act = np.stack(
            [delayed_commands(t, roll, p.latency_s) for p in combos]
        )
        pitch_act = np.stack(
            [delayed_commands(t, pitch, p.latency_s) for p in combos]
        )
        xs_b, ys_b = replay_batch(
            DT, roll_act, pitch_act,
            x0=np.full(k, 5.0), y0=np.full(k, -3.0),
            vx0=np.full(k, 10.0), vy0=np.full(k, -4.0),
            g_eff=np.array([p.g_eff for p in combos]),
            stiction_deg=np.array([p.stiction_deg for p in combos]),
            warp_c=np.array([p.warp_c_deg_per_mm for p in combos]),
            bias_roll=np.array([p.bias_roll_deg for p in combos]),
            bias_pitch=np.array([p.bias_pitch_deg for p in combos]),
        )
        for i, p in enumerate(combos):
            xs_s, ys_s = replay(t, roll, pitch, 5.0, -3.0, 10.0, -4.0, p)
            assert np.allclose(xs_b[i], xs_s, atol=1e-9), f"combo {i} x"
            assert np.allclose(ys_b[i], ys_s, atol=1e-9), f"combo {i} y"


class TestStiction:
    def test_slow_ball_inside_cone_sticks(self) -> None:
        params = PlantParams(g_eff=171.0, stiction_deg=0.3)
        x, y, vx, vy = plant_step(1.0, 0.0, 0.1, 0.0, 0.0, 0.2, DT, params)
        # 0.2 deg commanded < 0.3 cone (warp 0): stuck, no motion.
        assert (vx, vy) == (0.0, 0.0)
        assert x == pytest.approx(1.0)

    def test_strong_tilt_breaks_loose(self) -> None:
        params = PlantParams(g_eff=171.0, stiction_deg=0.3)
        x, y, vx, vy = plant_step(1.0, 0.0, 0.0, 0.0, 0.0, 0.5, DT, params)
        assert vx > 0.0                  # net accel escapes the cone
