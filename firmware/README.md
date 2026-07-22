# Arduino Firmware

Companion firmware for the Stewart platform's Arduino Uno R3. The Python app
talks to it over USB serial at 250000 baud (firmware v2; the host auto-falls
back to 115200 for v1 boards — see `hardware/serial_manager.py` and
`hardware/servo_driver.py`).

## Firmware v2 (2026-07-22) — what runs on the board now

The performance pass replaced the reconstructed v1 sketch with
`stewart_platform_uno_v2/stewart_platform_uno_v2.ino` — real, compiled
source (not a reconstruction), flashed and validated on the rig 2026-07-22.
Changes vs v1:

- **New `T` protocol** — tenth-degree resolution: `T,d0,d1,d2,d3,d4,d5\n`
  with `d` in tenth-degrees 0..1800, written via `writeMicroseconds`
  (0.1° command grid vs v1's whole degrees). Instant write, terse `k\n`
  ack (2 bytes vs v1's ~60-byte verbose ack); malformed input acks `e\n`.
- **Legacy `S` protocol retained bit-compatible** (whole degrees,
  `speedDelay` ramp, verbose completion ack) — `wiring_check.py`, rollback,
  and host-side large-move ramping all still work. Position state is shared
  between the two protocols, so an S ramp after a T move starts from the
  true pose.
- **250000 baud** (0% UART timer error at 16 MHz, vs 3.5% at 115200).
- **No Arduino `String`, no blocking `readStringUntil`** — fixed char
  buffer with non-blocking accumulation. 4.5 KB flash (v1: 6.2 KB).
- Boot banner is now **`[READY v2]`** — the host parses the version from
  the banner to decide whether the T protocol is available.

**Ground truth is now `flash_dump_v2_2026-07-22.hex`** — byte-for-byte what
runs on the board. The v1 dump is retained as the rollback image.

Flash v2, or roll back to v1:

```
avrdude -c arduino -p atmega328p -P COM8 -b 115200 -U flash:w:flash_dump_v2_2026-07-22.hex:i   # v2 (current)
avrdude -c arduino -p atmega328p -P COM8 -b 115200 -U flash:w:flash_dump_2026-07-20.hex:i      # v1 (rollback)
```

Measured RTT on this rig (COM8):

| Firmware | Baud | Ack | RTT p50 |
|---|---|---|---|
| v1 | 115200 | verbose `[OK] ...` | 9.7 ms (after the host read-latency fix; 57 ms before it) |
| v2 | 250000 | terse `k` | **4.4 ms** (p90 5.0, max 5.7) |

`wiring_check.py` now takes an optional baud argument:
`python firmware/wiring_check.py [PORT] [BAUD]` — defaults COM8 / 250000
(v2); pass 115200 for v1. It uses the S protocol, which both firmware
versions speak.

## Provenance — v1 capture (2026-07-20)

**The original sketch source was never committed and was not found on this
machine.** On 2026-07-20 the running firmware was captured directly off the
board:

| File | What it is |
|---|---|
| `flash_dump_2026-07-20.hex` | Full 32 KB flash image read off the Uno via avrdude (Optiboot bootloader, read-only). Was ground truth until the v2 flash on 2026-07-22; now the **rollback image**. |
| `stewart_platform_uno/stewart_platform_uno.ino` | **Reconstructed** v1 source: pin array, clamps, and message strings recovered from disassembly; ramp semantics measured empirically over serial. Behavior-accurate, but not the original file. |

To restore a board to the exact v1 firmware without trusting the
reconstruction, flash the v1 dump directly (see the v2 section above for
the current-firmware command):

```
avrdude -c arduino -p atmega328p -P COM8 -b 115200 -U flash:w:flash_dump_2026-07-20.hex:i
```

(Uses the serial bootloader; will not overwrite the bootloader itself.)

## Pin map

Recovered from the flash dump (`servoPins` array at flash offset 0x1EA0,
attach loop confirmed in disassembly at 0xC84):

| Command index (`S,a0..a5`) | Arduino digital pin |
|---|---|
| servo 0 | **D2** |
| servo 1 | **D3** |
| servo 2 | **D4** |
| servo 3 | **D5** |
| servo 4 | **D6** |
| servo 5 | **D7** |

Contiguous block **D2–D7**. The Servo library generates pulses in software
(Timer1 interrupt), so these do NOT need to be hardware-PWM pins. Pins 0/1
are left free for serial, 8-13 unused.

## Servo clocking — which leg plugs into which pin

Servo indices are defined by the geometry in `config.py`
(`SERVO_SHAFTS_XY`). Viewed **from directly above**, with the +X axis of the
platform frame pointing right (+X is the face that servos 0 and 5 straddle —
the "front" pair), **indices run counterclockwise starting front-right**:

```
        top-down view, +X --> right

              servo 1 (D3)
   servo 2 (D4)    12:00ish
      10:00              servo 0 (D2)
                            2:00
           [ platform ]           --> +X (front face:
                            4:00      pair 0/5)
      8:00               servo 5 (D7)
   servo 3 (D5)     6:00ish
              servo 4 (D6)
```

| Servo | Pin | Shaft XY (mm) | Angle from +X | Clock position (top-down) |
|---|---|---|---|---|
| 0 | D2 | (93.0, 37.5) | +22° | ~2:00 (front-left of +X face) |
| 1 | D3 | (-14.0, 99.3) | +98° | ~11:50 |
| 2 | D4 | (-79.0, 61.8) | +142° | ~10:15 |
| 3 | D5 | (-79.0, -61.8) | -142° | ~7:45 |
| 4 | D6 | (-14.0, -99.3) | -98° | ~6:10 |
| 5 | D7 | (93.0, -37.5) | -22° | ~4:00 (front-right of +X face) |

Mirror pairs: **(0,5)** front face, **(1,2)** rear-left face, **(3,4)**
rear-right face. Per `SERVO_SIGN` in `config.py`, odd indices (1, 3, 5)
rotate in the opposite sense to even indices — each pair's two servos mirror
each other, so orient the horns accordingly when assembling.

**Sanity check after wiring:** with servos plugged in, send a small nudge to
servo 0 only (e.g. `S,100,90,90,90,90,90,5`) and confirm the leg at the
front-left position moves. Repeat per index before first full-platform test.

## Legacy `S` serial protocol (verified against live board 2026-07-20; retained bit-compatible in v2)

This section documents the v1 wire protocol. Firmware v2 speaks it
unchanged and adds the `T` tenth-degree protocol described in the v2
section above.

- **Port:** 115200 baud (v1) / 250000 baud (v2), 8N1. Opening the port
  auto-resets the Uno (~2 s boot, then the banner: `[READY]` on v1,
  `[READY v2]` on v2).
- **Boot:** all six servos attach and are written to 90°. Expect a jump to
  neutral at power-on if legs are elsewhere.
- **Command:** `S,a0,a1,a2,a3,a4,a5,speedDelay\n`
  - Must start with `S,` → else `[ERROR] Invalid command format. Must start with S,`
  - Exactly 7 values → else `[ERROR] Expected 6 angles + speedDelay (7 values total)`
  - Angles clamped to **0–180** on the firmware side (the Python side also
    clamps in `core/safety.py` — both rails are real).
  - `speedDelay` clamped to **0–20 ms** (sent 50, board echoed 20).
- **Motion:** all servos ramp **concurrently**, 1°/step,
  `delay(speedDelay)` per step → move duration ≈ `maxDelta × speedDelay` ms.
  `speedDelay=0` = instant write. Measured: 30° @ 10 ms → ~343 ms;
  30° @ 20 ms → ~657 ms; no-op move → ~31 ms.
- **Ack:** `[OK] Updated angles: a0, ..., a5 | speedDelay=N` prints **after
  the ramp finishes** — it is a completion ack, not a receipt ack. A
  `speedDelay>0` command therefore blocks the firmware's command loop for the
  whole ramp; streaming sends use `speedDelay=0` (built in
  `hardware/servo_driver.py`).
- **Host-side usage (2026-07 overhaul + perf pass):** the Python side uses a
  hybrid dispatch (`hardware/servo_driver.py`): on v2 firmware, small
  streaming moves go out as `T` commands on the 0.1° grid; any per-servo
  jump over `settings.SERVO_SLEW_INSTANT_MAX_DEG` — and everything on
  v1 or with `SERVO_PROTOCOL = "legacy"` — goes out as `S` with a nonzero
  `speedDelay` per the `core/safety.select_speed_delay` policy so the ramp
  happens in firmware. `connect()` waits for the boot banner (3 s cap),
  parses the firmware version from it, and auto-retries at 115200 when no
  banner appears at the configured 250000 (v1 firmware); see
  `hardware/serial_manager.py`.

## Board identification

- Arduino Uno R3 (VID 2341 / PID 0043), serial `8513030333835101B090`.
- On the dev machine (2026-07-20) it enumerated as **COM8** — note
  `settings.py` default is COM4, which on this machine is a Bluetooth port.
  Check Device Manager per README before first run.
