// ============================================================================
// Stewart Platform servo firmware v2 (2026-07-22 perf pass)
// ============================================================================
// Successor to the reconstructed v1 sketch. Changes vs v1:
//   - 250000 baud (0% UART timer error at 16 MHz, vs 3.5% at 115200).
//   - New "T" protocol: tenth-degree resolution via writeMicroseconds
//     (0.1 deg command grid vs v1's whole degrees), terse "k" ack.
//   - Legacy "S" protocol retained BIT-COMPATIBLE (whole degrees, ramp via
//     speedDelay, verbose ack) for wiring_check, rollback paths, and
//     host-side large-move ramping.
//   - No Arduino String (heap fragmentation) and no blocking
//     readStringUntil: fixed char buffer, non-blocking accumulation.
// Boot behavior unchanged: attach D2-D7, all servos to neutral, banner.
// The banner is "[READY v2]" — the host parses the version to decide
// whether the T protocol is available.
//
// Protocols:
//   T,d0,d1,d2,d3,d4,d5\n     d = tenth-degrees 0..1800; instant write.
//                             Ack: "k\n". Malformed: "e\n".
//   S,a0,...,a5,speedDelay\n  v1 semantics: whole degrees 0..180, ramp all
//                             servos concurrently 1 deg per speedDelay ms
//                             (clamped 0..20), verbose ack AFTER the ramp.
// ============================================================================

#include <Servo.h>

const long BAUD = 250000;
const int NUM_SERVOS = 6;
int servoPins[NUM_SERVOS] = {2, 3, 4, 5, 6, 7};
Servo servos[NUM_SERVOS];

// Position state in TENTH-degrees (0..1800) — shared by both protocols so
// an S ramp after a T move starts from the true position.
int currentTenth[NUM_SERVOS] = {900, 900, 900, 900, 900, 900};

const int BUF_LEN = 72;
char buf[BUF_LEN];
int bufPos = 0;

// Servo-library default pulse range: 544..2400 us over 0..180 deg.
int tenthToMicros(int tenth) {
  return (int)(544L + ((long)tenth * 1856L) / 1800L);
}

void writeTenth(int i, int tenth) {
  currentTenth[i] = tenth;
  servos[i].writeMicroseconds(tenthToMicros(tenth));
}

void setup() {
  Serial.begin(BAUD);
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoPins[i]);
    writeTenth(i, 900);
  }
  Serial.println(F("[READY v2]"));
}

// Parse an int from buf starting at *pos (fields separated by ','); returns
// false on malformed input. Advances *pos past the next separator.
bool parseField(int *pos, int *out) {
  int v = 0;
  bool any = false;
  bool neg = false;
  if (buf[*pos] == '-') { neg = true; (*pos)++; }
  while (buf[*pos] >= '0' && buf[*pos] <= '9') {
    v = v * 10 + (buf[*pos] - '0');
    (*pos)++;
    any = true;
  }
  if (!any) return false;
  if (buf[*pos] == ',') (*pos)++;
  else if (buf[*pos] != '\0') return false;
  *out = neg ? -v : v;
  return true;
}

void handleT() {
  int vals[NUM_SERVOS];
  int pos = 2;  // skip "T,"
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (!parseField(&pos, &vals[i])) { Serial.println(F("e")); return; }
  }
  if (buf[pos] != '\0') { Serial.println(F("e")); return; }
  for (int i = 0; i < NUM_SERVOS; i++) {
    writeTenth(i, constrain(vals[i], 0, 1800));
  }
  Serial.println(F("k"));
}

void handleS() {
  int vals[NUM_SERVOS + 1];
  int pos = 2;  // skip "S,"
  for (int i = 0; i < NUM_SERVOS + 1; i++) {
    if (!parseField(&pos, &vals[i])) {
      Serial.println(F("[ERROR] Expected 6 angles + speedDelay (7 values total)"));
      return;
    }
  }
  if (buf[pos] != '\0') {
    Serial.println(F("[ERROR] Expected 6 angles + speedDelay (7 values total)"));
    return;
  }

  int targetTenth[NUM_SERVOS];
  for (int i = 0; i < NUM_SERVOS; i++) {
    targetTenth[i] = constrain(vals[i], 0, 180) * 10;
  }
  int speedDelay = constrain(vals[NUM_SERVOS], 0, 20);

  // v1 ramp semantics: all servos step 1 deg (10 tenths) concurrently,
  // delay(speedDelay) between steps.
  bool moving = true;
  while (moving) {
    moving = false;
    for (int i = 0; i < NUM_SERVOS; i++) {
      int diff = targetTenth[i] - currentTenth[i];
      if (diff != 0) {
        int step = constrain(diff, -10, 10);
        writeTenth(i, currentTenth[i] + step);
        moving = true;
      }
    }
    if (moving && speedDelay > 0) delay(speedDelay);
  }

  Serial.print(F("[OK] Updated angles: "));
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.print(currentTenth[i] / 10);
    if (i < NUM_SERVOS - 1) Serial.print(F(", "));
  }
  Serial.print(F(" | speedDelay="));
  Serial.println(speedDelay);
}

void handleLine() {
  if (buf[0] == 'T' && buf[1] == ',') handleT();
  else if (buf[0] == 'S' && buf[1] == ',') handleS();
  else if (buf[0] != '\0') {
    Serial.println(F("[ERROR] Invalid command format. Must start with S,"));
  }
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      buf[bufPos] = '\0';
      // Strip a trailing '\r' (host may send CRLF).
      if (bufPos > 0 && buf[bufPos - 1] == '\r') buf[bufPos - 1] = '\0';
      handleLine();
      bufPos = 0;
    } else if (bufPos < BUF_LEN - 1) {
      buf[bufPos++] = c;
    } else {
      bufPos = 0;  // overlong line: drop it
    }
  }
}
