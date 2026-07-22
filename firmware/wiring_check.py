"""Wiring/clocking check: wiggle each servo +/-2 deg in physical clockwise order.

Order (viewer facing the flat-edge pair, per firmware/README.md):
front-left -> left -> back-left -> back-right -> right -> front-right,
which is command index 5, 4, 3, 2, 1, 0  (pins D7, D6, D5, D4, D3, D2).

Safety: excursion is limited to +/-2 deg around neutral (90). The firmware
additionally clamps angles 0-180 and ramps at speedDelay ms/deg.

Standalone rig utility: deliberately imports NO app code (works from a bare
clone next to a bench Arduino), so it builds the "S,..." string inline — a
documented exemption from CLAUDE.md hard constraint 4 (see setup.cfg note).

Usage:  python firmware/wiring_check.py [PORT]
        PORT defaults to COM8 (this dev machine's Uno; check Device Manager).
"""
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM8"
BAUD = 115200
NEUTRAL = 90
DELTA = 2                # max excursion, degrees
WIGGLES = 3              # back-and-forth cycles per servo
DWELL_S = 0.25           # pause at each end of a wiggle
GAP_S = 2.5              # pause between servos
SPEED_DELAY = 10         # firmware ramp, ms per degree

# Physical clockwise order starting front-left (see module docstring).
TEST_ORDER = [
    (5, "D7", "1st - FRONT-LEFT (the pair facing you, left one)"),
    (4, "D6", "2nd - LEFT side"),
    (3, "D5", "3rd - BACK-LEFT"),
    (2, "D4", "4th - BACK-RIGHT"),
    (1, "D3", "5th - RIGHT side"),
    (0, "D2", "6th - FRONT-RIGHT (the pair facing you, right one)"),
]


def send(ser: serial.Serial, angles: list[int], speed_delay: int) -> None:
    cmd = "S," + ",".join(str(a) for a in angles) + f",{speed_delay}\n"
    ser.write(cmd.encode())
    # Wait for the completion ack so wiggle timing stays crisp.
    deadline = time.time() + 3.0
    buf = b""
    while time.time() < deadline and b"\n" not in buf:
        buf += ser.read(64)
    line = buf.decode(errors="replace").strip()
    if not line.startswith("[OK]"):
        print(f"  ! unexpected response: {line!r}")


def main() -> None:
    print(f"Opening {PORT} @ {BAUD} (board auto-resets, ~2.5s)...")
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(2.5)
    ser.reset_input_buffer()

    neutral = [NEUTRAL] * 6
    send(ser, neutral, 0)
    print("All servos at neutral 90. Starting wiggle sequence.\n")

    for idx, pin, label in TEST_ORDER:
        print(f"=== {label}  ->  command index {idx}, pin {pin} ===")
        time.sleep(GAP_S)
        for _ in range(WIGGLES):
            up = list(neutral)
            up[idx] = NEUTRAL + DELTA
            down = list(neutral)
            down[idx] = NEUTRAL - DELTA
            send(ser, up, SPEED_DELAY)
            time.sleep(DWELL_S)
            send(ser, down, SPEED_DELAY)
            time.sleep(DWELL_S)
        send(ser, neutral, SPEED_DELAY)
        print("   done, back at neutral.\n")

    ser.close()
    print("Sequence complete. Expected physical order was:")
    for _, pin, label in TEST_ORDER:
        print(f"  {label}  ({pin})")


if __name__ == "__main__":
    main()
