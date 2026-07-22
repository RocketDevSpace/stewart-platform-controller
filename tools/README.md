# tools/

Measurement and bench utilities from the 2026-07-22 performance pass.
None are imported by the app; run them from the repo root.

## jitter_bench.py — headless controller A/B bench (no hardware)

Feeds synthetic ball-motion profiles (or a recorded CSV) through the REAL
chain — measurement filter → BallController → IK → ServoDriver (capturing
fake serial) — and reports integer command flips/min, send counts, command
std, max per-frame delta, d-term saturation, and rest-mode duty.

```
python tools/jitter_bench.py --profile quiescent            # rest-noise case
python tools/jitter_bench.py --profile impulse              # fast-strike case
python tools/jitter_bench.py --profile quiescent --filter-mode legacy   # A/B
python tools/jitter_bench.py --csv session.csv --json       # replay real motion
```

Record real motion for replay by setting `VISION_POSITION_LOG_PATH` in
settings (writes `t,x,y` per valid frame while vision mode runs).

## latency_bench.py — on-rig serial RTT (hardware)

Sends 100 spaced commands through the real ServoDriver path and reports
command→ack round-trip percentiles: `python tools/latency_bench.py --port COM8`

## camera_probe.py — on-rig camera capability sweep (hardware)

Measures REAL frame periods and brightness across fps targets (60/50/30)
and exposure modes, and prints a recommendation:
`python tools/camera_probe.py --index 1`
(2026-07-22 result on the rig camera: hard-capped ~31 fps at 640x480 MJPG
under both DSHOW and MSMF — 30 fps retained.)
