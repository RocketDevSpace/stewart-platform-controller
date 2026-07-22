"""
tools/camera_probe.py

On-rig camera capability probe: for each fps target (60/50/30) and
exposure mode (auto / -5 / -6) it opens the camera with DSHOW at 640x480
MJPG, warms up, measures the real frame period and brightness, prints a
table, and recommends the best config. Run from the repo root on the rig:

    python tools/camera_probe.py --index 1

Exits gracefully (no traceback) when the camera is absent.
"""
from __future__ import annotations

import argparse
import statistics
import time
from dataclasses import dataclass

import cv2

_FPS_TARGETS = [60, 50, 30]
_EXPOSURES: list[object] = ["auto", -5, -6]
_WARMUP_FRAMES = 10
_MEASURE_FRAMES = 60
_MIN_GRAY = 25.0
_PERIOD_SLACK = 1.2


@dataclass
class ProbeResult:
    fps_target: int
    exposure: object
    ok: bool
    median_ms: float = 0.0
    p90_ms: float = 0.0
    gray_mean: float = 0.0


def _probe_one(index: int, fps: int, exposure: object) -> ProbeResult:
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    try:
        if not cap.isOpened():
            return ProbeResult(fps, exposure, ok=False)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        fourcc = cv2.VideoWriter.fourcc(*"MJPG")
        cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        cap.set(cv2.CAP_PROP_FPS, float(fps))
        if exposure == "auto":
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)  # DSHOW: 0.75 = auto
        else:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # DSHOW: 0.25 = manual
            cap.set(cv2.CAP_PROP_EXPOSURE, float(exposure))  # type: ignore[arg-type]

        for _ in range(_WARMUP_FRAMES):
            if not cap.read()[0]:
                return ProbeResult(fps, exposure, ok=False)

        periods_ms: list[float] = []
        grays: list[float] = []
        last = time.perf_counter()
        for _ in range(_MEASURE_FRAMES):
            ok, frame = cap.read()
            now = time.perf_counter()
            if not ok or frame is None:
                return ProbeResult(fps, exposure, ok=False)
            periods_ms.append((now - last) * 1000.0)
            last = now
            grays.append(
                float(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).mean())
            )

        periods_ms.sort()
        p90 = periods_ms[min(len(periods_ms) - 1, int(0.9 * len(periods_ms)))]
        return ProbeResult(
            fps, exposure, ok=True,
            median_ms=statistics.median(periods_ms),
            p90_ms=p90,
            gray_mean=statistics.fmean(grays),
        )
    finally:
        cap.release()  # release between configs, always


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Camera fps/exposure probe.")
    parser.add_argument("--index", type=int, default=1, help="camera index")
    args = parser.parse_args(argv)

    # Availability check first: one open attempt, graceful exit if absent.
    probe = cv2.VideoCapture(args.index, cv2.CAP_DSHOW)
    available = probe.isOpened()
    probe.release()
    if not available:
        print(f"[camera_probe] no camera at index {args.index} - nothing to "
              "do (run this on the rig; try --index 0).")
        return 0

    results: list[ProbeResult] = []
    print(f"[camera_probe] probing index {args.index} (640x480 MJPG, DSHOW)")
    print(f"{'fps':>4} {'exposure':>9} {'median ms':>10} {'p90 ms':>8} {'gray':>7}")
    for fps in _FPS_TARGETS:
        for exposure in _EXPOSURES:
            r = _probe_one(args.index, fps, exposure)
            results.append(r)
            if r.ok:
                print(f"{r.fps_target:>4} {str(r.exposure):>9} "
                      f"{r.median_ms:>10.2f} {r.p90_ms:>8.2f} {r.gray_mean:>7.1f}")
            else:
                print(f"{fps:>4} {str(exposure):>9} {'FAILED':>10}")

    # Recommendation: highest fps whose median period meets the target
    # within slack AND is bright enough to track.
    best: ProbeResult | None = None
    for r in results:
        if not r.ok:
            continue
        if r.median_ms <= _PERIOD_SLACK * (1000.0 / r.fps_target) and r.gray_mean >= _MIN_GRAY:
            if best is None or r.fps_target > best.fps_target or (
                r.fps_target == best.fps_target and r.gray_mean > best.gray_mean
            ):
                best = r
    if best is None:
        print("[camera_probe] RECOMMENDATION: none - no config met "
              f"period <= {_PERIOD_SLACK}x target and gray >= {_MIN_GRAY}.")
    else:
        print(f"[camera_probe] RECOMMENDATION: {best.fps_target} fps, "
              f"exposure {best.exposure} "
              f"(median {best.median_ms:.2f} ms, gray {best.gray_mean:.1f}).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
