"""
tools/latency_bench.py

On-rig serial RTT bench: sends N spaced neutral-ish commands through the
real ServoDriver path and reports command->ack round-trip percentiles
from SerialManager.rtt_stats(). Run from the repo root on the rig:

    python tools/latency_bench.py --port COM8

Exits gracefully (no traceback) when the hardware is absent.
"""
import argparse
import sys
import time
from pathlib import Path

_REPO_ROOT = str(Path(__file__).resolve().parents[1])
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from hardware.serial_manager import SerialManager  # noqa: E402
from hardware.servo_driver import ServoDriver  # noqa: E402
from settings import SERIAL_BAUD, SERIAL_PORT  # noqa: E402

_SPACING_S = 0.05  # well beyond typical RTT; keeps exactly one outstanding


def _percentile(sorted_vals: list[float], p: float) -> float:
    return sorted_vals[min(len(sorted_vals) - 1, int(round(p * (len(sorted_vals) - 1))))]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Serial command->ack RTT bench.")
    parser.add_argument("--port", default=SERIAL_PORT, help="serial port (e.g. COM8)")
    parser.add_argument("--n", type=int, default=100)
    args = parser.parse_args(argv)

    manager = SerialManager(args.port, SERIAL_BAUD)
    if not manager.connect():
        print(f"[latency_bench] no hardware on {args.port} - nothing to do "
              "(run this on the rig; check the port with Device Manager).")
        return 0

    driver = ServoDriver(manager)
    samples: list[float] = []
    try:
        for i in range(args.n):
            # Neutral-ish alternation (90/91) so every send is a real move.
            before = manager.rtt_stats()[2]
            if not driver.send_angles([90.0 if i % 2 == 0 else 91.0] * 6):
                print("[latency_bench] send failed - aborting")
                break
            deadline = time.perf_counter() + 1.0
            while time.perf_counter() < deadline:
                _ema, last_ms, count = manager.rtt_stats()
                if count > before:
                    samples.append(last_ms)
                    break
                time.sleep(0.002)
            time.sleep(_SPACING_S)
    finally:
        manager.disconnect()

    if not samples:
        print("[latency_bench] no acks matched - old firmware or link issue.")
        return 1
    samples.sort()
    print(f"[latency_bench] {len(samples)} matched RTT samples on {args.port}:")
    print(f"  p50 = {_percentile(samples, 0.50):.2f} ms")
    print(f"  p90 = {_percentile(samples, 0.90):.2f} ms")
    print(f"  max = {samples[-1]:.2f} ms   (EMA {manager.rtt_stats()[0]:.2f} ms)")
    print("NOTE: vision-stage latency (frame_to_cmd) is read from the GUI "
          "timing plot while vision mode runs - this bench covers serial only.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
