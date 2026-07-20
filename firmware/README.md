# Arduino Firmware

Companion firmware for the Stewart platform's Arduino Uno R3. The Python app
talks to it over USB serial at 115200 baud (see `hardware/serial_manager.py`
and `hardware/servo_driver.py`).

## Provenance — read this first

**The original sketch source was never committed and was not found on this
machine.** On 2026-07-20 the running firmware was captured directly off the
board:

| File | What it is |
|---|---|
| `flash_dump_2026-07-20.hex` | Full 32 KB flash image read off the Uno via avrdude (Optiboot bootloader, read-only). **Ground truth** — this is byte-for-byte what runs on the board. |
| `stewart_platform_uno/stewart_platform_uno.ino` | **Reconstructed** source: pin array, clamps, and message strings recovered from disassembly; ramp semantics measured empirically over serial. Behavior-accurate, but not the original file. |

To restore a board to the exact current firmware without trusting the
reconstruction, flash the dump directly:

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

## Serial protocol (verified against live board 2026-07-20)

- **Port:** 115200 baud, 8N1. Opening the port auto-resets the Uno
  (~2 s boot, then `[READY]`).
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
  whole ramp; the GUI/vision loop always sends `speedDelay=0` (built in
  `hardware/servo_driver.py`), which keeps round-trip ~20-30 ms.

## Board identification

- Arduino Uno R3 (VID 2341 / PID 0043), serial `8513030333835101B090`.
- On the dev machine (2026-07-20) it enumerated as **COM8** — note
  `settings.py` default is COM4, which on this machine is a Bluetooth port.
  Check Device Manager per README before first run.
