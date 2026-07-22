"""
tools/jitter_bench.py

Headless A/B jitter bench: feeds deterministic synthetic position streams
through the REAL production chain — MeasurementFilter (the exact filtering
BallTracker.process() runs) -> BallController.compute_with_terms (fake
clock, dt = 1/fps) -> IK via core.ik_engine -> ServoDriver.send_angles
(real safety clip, slew policy, and int(round()) quantization) with a
capturing SerialManager stub instead of a port.

No hardware, no camera, no Qt. Run from the repo root:

    python tools/jitter_bench.py --profile quiescent
    python tools/jitter_bench.py --profile impulse --json
    python tools/jitter_bench.py --csv session.csv     # "t,x,y" rows
                                                       # (VISION_POSITION_LOG_PATH output)

The point is a BASELINE: run it before and after each perf change and
compare command flip rates / send counts / d-term saturation.
"""

from __future__ import annotations

import argparse
import json
import math
import random
import statistics
import sys
from pathlib import Path

_REPO_ROOT = str(Path(__file__).resolve().parents[1])
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from control.ball_controller import BallController  # noqa: E402
from core.ik_engine import IKEngine  # noqa: E402
from core.platform_state import BallState, Pose  # noqa: E402
from cv.measurement_filter import FILTER_MODES, MeasurementFilter  # noqa: E402
from hardware.serial_manager import SerialManager  # noqa: E402
from hardware.servo_driver import ServoDriver  # noqa: E402
from settings import MAX_TILT_DEG, PD_D_TERM_LIMIT_DEG  # noqa: E402

PROFILES = ("quiescent", "step3hz", "ramp", "impulse")
_DEFAULT_FRAMES = 900
_DEFAULT_FPS = 30.0
_DEFAULT_SEED = 42


class FakeClock:
    """Deterministic clock injected into BallController (slew/trim dt)."""

    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


class CapturingSerial(SerialManager):
    """SerialManager stub: records payloads, never touches a port.

    Subclassing (instead of a MagicMock) keeps ServoDriver's constructor
    contract type-checked while the REAL quantization/slew code runs."""

    def __init__(self) -> None:
        super().__init__(port="BENCH", baud=0)
        self.payloads: list[bytes] = []

    def is_connected(self) -> bool:
        return True

    def send(self, data: bytes) -> bool:
        self.payloads.append(data)
        return True

    def send_latest(self, data: bytes) -> bool:
        # The bench is synchronous: every enqueued command "reaches the
        # firmware" (latest-wins coalescing never drops here).
        self.payloads.append(data)
        return True


# ---------------------------------------------------------------------------
# Input profiles — lists of (t, x_mm, y_mm) raw measurements
# ---------------------------------------------------------------------------


def profile_quiescent(
    frames: int, dt: float, rng: random.Random
) -> list[tuple[float, float, float]]:
    """Static point + uniform +/-0.5 mm noise + a 0.75 mm step every 10
    frames (the camera-noise regime that causes servo chatter at rest)."""
    samples = []
    base_x = 0.0
    for i in range(frames):
        if i > 0 and i % 10 == 0:
            base_x = 0.75 - base_x  # 0 <-> 0.75 toggle
        x = base_x + rng.uniform(-0.5, 0.5)
        y = rng.uniform(-0.5, 0.5)
        samples.append((i * dt, x, y))
    return samples


def profile_step3hz(
    frames: int, dt: float, rng: random.Random
) -> list[tuple[float, float, float]]:
    """Pure 3 Hz 0.75 mm square wave on x (no noise)."""
    half_period_s = 1.0 / (2.0 * 3.0)
    samples = []
    for i in range(frames):
        t = i * dt
        x = 0.75 if (int(t / half_period_s) % 2) else 0.0
        samples.append((t, x, 0.0))
    return samples


def profile_ramp(
    frames: int, dt: float, rng: random.Random
) -> list[tuple[float, float, float]]:
    """Speed ramps 0 -> 300 mm/s over 1 s, then stays constant."""
    samples = []
    x = 0.0
    for i in range(frames):
        t = i * dt
        speed = 300.0 * min(1.0, t / 1.0)
        samples.append((t, x, 0.0))
        x += speed * dt
    return samples


def profile_impulse(
    frames: int, dt: float, rng: random.Random
) -> list[tuple[float, float, float]]:
    """Rest, then a 5 mm/frame kick for 6 frames, then exponential decay."""
    rest_frames = min(60, frames // 3)
    samples = []
    x = 0.0
    for i in range(frames):
        if rest_frames <= i < rest_frames + 6:
            x += 5.0
        elif i >= rest_frames + 6:
            x *= 0.92
        samples.append((i * dt, x, 0.0))
    return samples


def load_csv(path: str) -> list[tuple[float, float, float]]:
    """Read "t,x,y" rows (VISION_POSITION_LOG_PATH format); skips
    non-numeric lines (headers, blanks)."""
    samples: list[tuple[float, float, float]] = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            parts = line.strip().split(",")
            if len(parts) < 3:
                continue
            try:
                t, x, y = float(parts[0]), float(parts[1]), float(parts[2])
            except ValueError:
                continue
            samples.append((t, x, y))
    return samples


def build_profile(
    name: str, frames: int, fps: float, seed: int
) -> list[tuple[float, float, float]]:
    rng = random.Random(seed)
    dt = 1.0 / fps
    builders = {
        "quiescent": profile_quiescent,
        "step3hz": profile_step3hz,
        "ramp": profile_ramp,
        "impulse": profile_impulse,
    }
    if name not in builders:
        raise ValueError(f"unknown profile {name!r} (choose from {PROFILES})")
    return builders[name](frames, dt, rng)


# ---------------------------------------------------------------------------
# Bench core
# ---------------------------------------------------------------------------


def run_bench(
    samples: list[tuple[float, float, float]],
    label: str,
    filter_mode: str | None = None,
) -> dict[str, object]:
    """Push the samples through the production chain; return the metrics.

    filter_mode overrides settings.TRACKER_FILTER_MODE for A/B runs
    (None = whatever the settings default is)."""
    if not samples:
        raise ValueError("no samples to bench")

    clock = FakeClock()
    meas = MeasurementFilter() if filter_mode is None else MeasurementFilter(filter_mode)
    controller = BallController(max_tilt_deg=float(MAX_TILT_DEG), clock=clock)
    ik = IKEngine()
    serial = CapturingSerial()
    driver = ServoDriver(serial)

    prev_arm = None
    valid_frames = 0
    ik_failures = 0
    d_sat_frames = 0
    rest_frames = 0
    d_cap = None if PD_D_TERM_LIMIT_DEG is None else float(PD_D_TERM_LIMIT_DEG)

    for t, x_raw, y_raw in samples:
        clock.t = t
        filtered = meas.update(x_raw, y_raw, t)
        if filtered is None:
            meas.reset()  # outlier gate tripped -> tracker-miss semantics
            continue
        x_mm, y_mm, vx, vy = filtered
        valid_frames += 1

        state = BallState(x_mm=x_mm, y_mm=y_mm, vx_mm_s=vx, vy_mm_s=vy)
        roll_deg, pitch_deg, terms = controller.compute_with_terms(state)
        d_x, d_y = terms["d_term"]
        if d_cap is not None and d_cap > 0:
            if abs(d_x) >= d_cap - 1e-9 or abs(d_y) >= d_cap - 1e-9:
                d_sat_frames += 1
        if terms.get("rest_mode_active", False):
            rest_frames += 1

        result = ik.solve(
            Pose(x=0.0, y=0.0, z=0.0, roll=roll_deg, pitch=pitch_deg, yaw=0.0),
            prev_arm,
        )
        if not result.success:
            ik_failures += 1
            continue
        prev_arm = result.arm_points
        # Real production send path: safety clip + slew policy + int
        # quantization all happen inside ServoDriver.send_angles.
        driver.send_angles(
            [float(a) for a in result.servo_angles_deg], streaming=True
        )

    frames = len(samples)
    duration_s = max(1e-9, samples[-1][0] - samples[0][0]) if frames > 1 else 1e-9
    duration_min = duration_s / 60.0

    # Parse the captured integer commands: b"S,a0,...,a5,speedDelay\n"
    commands: list[list[int]] = []
    for payload in serial.payloads:
        fields = payload.decode("ascii").strip().split(",")
        commands.append([int(v) for v in fields[1:7]])

    flips = [0] * 6
    max_delta = 0
    for prev, cur in zip(commands, commands[1:]):
        for s in range(6):
            delta = abs(cur[s] - prev[s])
            if delta > 0:
                flips[s] += 1
            max_delta = max(max_delta, delta)

    std_per_servo = [
        (statistics.pstdev(col) if len(col) > 1 else 0.0)
        for col in (
            [cmd[s] for cmd in commands] for s in range(6)
        )
    ]

    return {
        "profile": label,
        "filter_mode": meas.mode,
        "frames": frames,
        "duration_s": round(duration_s, 3),
        "valid_frames": valid_frames,
        "ik_failures": ik_failures,
        "total_sends": len(commands),
        "sends_per_frame": round(len(commands) / frames, 4),
        "flips_per_min_per_servo": [
            round(f / duration_min, 1) for f in flips
        ],
        "flips_per_min_total": round(sum(flips) / duration_min, 1),
        "cmd_std_deg_per_servo": [round(s, 3) for s in std_per_servo],
        "max_frame_cmd_delta_deg": max_delta,
        "d_term_saturation_frac": round(
            (d_sat_frames / valid_frames) if valid_frames else 0.0, 4
        ),
        # Fraction of controller frames spent in near-target rest mode
        # (terms["rest_mode_active"]; 0.0 when the key is absent).
        "rest_duty_frac": round(
            (rest_frames / valid_frames) if valid_frames else 0.0, 4
        ),
    }


def run_profile(
    profile: str,
    frames: int = _DEFAULT_FRAMES,
    fps: float = _DEFAULT_FPS,
    seed: int = _DEFAULT_SEED,
    filter_mode: str | None = None,
) -> dict[str, object]:
    """Build a named profile and bench it (test/API entry point)."""
    return run_bench(
        build_profile(profile, frames, fps, seed), profile, filter_mode
    )


def format_report(report: dict[str, object]) -> str:
    lines = [f"=== jitter_bench: {report['profile']} ==="]
    for key, value in report.items():
        if key == "profile":
            continue
        lines.append(f"  {key:<26} {value}")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Headless jitter bench over the real filter/PD/IK/servo chain."
    )
    parser.add_argument("--profile", choices=PROFILES, default="quiescent")
    parser.add_argument("--csv", help="replay a 't,x,y' CSV instead of a profile")
    parser.add_argument("--frames", type=int, default=_DEFAULT_FRAMES)
    parser.add_argument("--fps", type=float, default=_DEFAULT_FPS)
    parser.add_argument("--seed", type=int, default=_DEFAULT_SEED)
    parser.add_argument(
        "--filter-mode",
        choices=FILTER_MODES,
        default=None,
        help="override settings.TRACKER_FILTER_MODE for A/B comparison",
    )
    parser.add_argument("--json", action="store_true", help="emit JSON")
    args = parser.parse_args(argv)

    if args.csv:
        samples = load_csv(args.csv)
        label = f"csv:{args.csv}"
    else:
        samples = build_profile(args.profile, args.frames, args.fps, args.seed)
        label = args.profile

    report = run_bench(samples, label, args.filter_mode)
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        print(format_report(report))

    numeric = [
        v for v in report.values()
        if isinstance(v, (int, float)) and not isinstance(v, bool)
    ]
    return 0 if all(math.isfinite(float(v)) for v in numeric) else 1


if __name__ == "__main__":
    raise SystemExit(main())
