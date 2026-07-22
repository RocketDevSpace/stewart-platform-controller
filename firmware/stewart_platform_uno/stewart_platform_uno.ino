// ============================================================================
// Stewart Platform servo firmware — RECONSTRUCTED SOURCE
// ============================================================================
// The original .ino was never committed and could not be found on any machine.
// This file was reconstructed on 2026-07-20 from:
//   1. A full flash dump of the running Arduino Uno R3 (see
//      firmware/flash_dump_2026-07-20.hex — that file is the ground truth).
//   2. Disassembly of that dump (pin array, attach loop, 0-180 clamp, and
//      string literals were all recovered directly from the binary).
//   3. Empirical serial probing of the live board (ramp timing, speedDelay
//      clamp, ack ordering).
// Behavior below matches every observation. Exact original variable names,
// comments, and incidental code structure are NOT recoverable — do not treat
// this file as a byte-faithful copy. If it is ever re-flashed, re-verify
// against docs in firmware/README.md.
//
// Verified behavior summary:
//   - Boot: attach servos on pins 2-7, write 90 to all, print "[READY]".
//   - Command: "S,a0,a1,a2,a3,a4,a5,speedDelay\n"
//       * must start with "S," else error
//       * exactly 7 values else error
//       * angles constrained to 0-180 (clamp found in disassembly @0x118e)
//       * speedDelay constrained to 0-20 ms (empirical: sent 50, echoed 20)
//   - Motion: all servos ramp CONCURRENTLY 1 degree per step,
//     delay(speedDelay) between steps; total time = maxDelta * speedDelay.
//     speedDelay=0 is effectively an instant write.
//   - Ack "[OK] Updated angles: ..." prints AFTER the ramp completes.
// ============================================================================

#include <Servo.h>

const int NUM_SERVOS = 6;

// Recovered verbatim from the flash dump (.data image @ flash 0x1EA0,
// RAM 0x0112): six 16-bit ints {2,3,4,5,6,7}. The attach loop walks this
// array in ascending index order, so command index i drives pin 2 + i.
int servoPins[NUM_SERVOS] = {2, 3, 4, 5, 6, 7};

Servo servos[NUM_SERVOS];
int currentAngles[NUM_SERVOS] = {90, 90, 90, 90, 90, 90};

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].write(90);          // observed in disasm before the attach loop
    currentAngles[i] = 90;
  }
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoPins[i]);
  }
  Serial.println("[READY]");
}

void loop() {
  if (!Serial.available()) {
    return;
  }
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) {
    return;
  }

  if (!line.startsWith("S,")) {
    Serial.println("[ERROR] Invalid command format. Must start with S,");
    return;
  }

  // Parse the 7 comma-separated values after "S,"
  int values[7];
  int count = 0;
  int start = 2;  // skip "S,"
  while (count < 7 && start <= (int)line.length()) {
    int comma = line.indexOf(',', start);
    String token = (comma == -1) ? line.substring(start)
                                 : line.substring(start, comma);
    values[count++] = token.toInt();
    if (comma == -1) {
      start = line.length() + 1;
    } else {
      start = comma + 1;
    }
  }
  if (count != 7) {
    Serial.println("[ERROR] Expected 6 angles + speedDelay (7 values total)");
    return;
  }

  int targetAngles[NUM_SERVOS];
  for (int i = 0; i < NUM_SERVOS; i++) {
    targetAngles[i] = constrain(values[i], 0, 180);
  }
  int speedDelay = constrain(values[6], 0, 20);

  // Ramp all servos concurrently, 1 degree per step.
  bool moving = true;
  while (moving) {
    moving = false;
    for (int i = 0; i < NUM_SERVOS; i++) {
      if (currentAngles[i] != targetAngles[i]) {
        currentAngles[i] += (targetAngles[i] > currentAngles[i]) ? 1 : -1;
        servos[i].write(currentAngles[i]);
        moving = true;
      }
    }
    if (moving && speedDelay > 0) {
      delay(speedDelay);
    }
  }

  Serial.print("[OK] Updated angles: ");
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.print(currentAngles[i]);
    Serial.print(i < NUM_SERVOS - 1 ? ", " : "");
  }
  Serial.print(" | speedDelay=");
  Serial.println(speedDelay);
}
