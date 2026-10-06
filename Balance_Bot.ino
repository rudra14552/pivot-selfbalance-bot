// Self-balancing bot - Arduino Uno/Nano (ATmega328P)
// MPU6050 + 2x stepper (DRV8825, M0/M1/M2 on D2/D3/D9) + HC-05
//
// Microstepping default: 1/8
//
// STABILITY FIRST:
//   * Hard cut: |tilt| > FALL_ANGLE (15 deg) -> motors off.
//   * Soft limit: |tilt| > SOFT_TILT (12 deg) -> remote commands and position hold
//     have ZERO authority (pure balancing). Authority fades in linearly from
//     CMD_TILT_FULL (7 deg) down to 0 at SOFT_TILT. Fast gyro wobble fades it too.
//   * Commanded lean (drive + hold) is capped at TRIM_MAX (6 deg), so the commands
//     alone can never push the bot toward the limit.
//   * Remote packets are validated strictly; malformed / out-of-range ones are dropped.
//   * IMU read failures or a dead accelerometer reading are handled (motors off / gyro only).
//
// Control structure:
//   POSITION PID (gains x y z):
//   lean  = -(x*position + y*integral(position) + z*wheelSpeed)   when standing still
//   lean  = speed PI (target speed from the remote)               while driving
//   BALANCE PID (gains p i d):
//   tilt  = angle - setpoint - lean
//   accel = Kp*tilt + Ki*integral + Kd*gyroRate         (steps/s^2)
//   speed += accel*dt
//   left  wheel = speed + turn
//   right wheel = speed - turn
//
// Position hold: small deadband around home (no hunting), filtered + capped lean,
// strong wheel-speed damping -> settles quickly and steadily. Stability over precision.
//
// DRIVE COMMANDS (send as a single letter):
//   F forward   B backward   L left   R right   S stop
//   G fwd-left  I fwd-right  H back-left  J back-right
//   v<n> = top forward/back speed (steps/s)   t<n> = top rotation speed (steps/s)
//   C<fwd%>,<turn%>  proportional (-100..100 each)
//
// Starting point: p280 i0 d15 x0.0005 z0.001
// NOTE: gains saved in EEPROM override these defaults. Send  z0.001  then  save  once.

#include <Wire.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// ---------- Pins ----------
SoftwareSerial BT(10, 11);          // HC-05: RX=10, TX=11
#define LEFT_STEP_PIN   5           // PORTD5 (used directly in ISR)
#define RIGHT_STEP_PIN  6           // PORTD6 (used directly in ISR)
#define LEFT_DIR_PIN    7
#define RIGHT_DIR_PIN   8
#define EN_PIN          4           // LOW = enabled
#define MS0_PIN         2           // M0 on both DRV8825
#define MS1_PIN         3           // M1 on both DRV8825
#define MS2_PIN         9           // M2 on both DRV8825

// ---------- Tunables ----------
const unsigned long LOOP_US   = 5000;     // 200 Hz control loop
const float ALPHA             = 0.98;     // complementary filter
const float FALL_ANGLE        = 15.0;     // deg: HARD limit, cut motors
const float SOFT_TILT         = 12.0;     // deg: above this commands/hold have zero authority
const float CMD_TILT_FULL     = 7.0;      // deg: below this commands have full authority
const float RATE_FULL         = 40.0;     // deg/s: below this commands have full authority
const float RATE_ZERO         = 90.0;     // deg/s: above this commands have zero authority
const float RECOVER_ANGLE     = 5.0;      // deg: resume balancing
const float MAX_RATE          = 4000.0;   // steps/s ceiling per wheel
const float MIN_RATE          = 40.0;     // below this -> wheel stops
const float I_MAX             = 20.0;
const float TRIM_MAX          = 6.0;      // max lean (deg) from drive + hold combined
const float SETPOINT_MAX      = 5.0;      // deg: limit for the 's' command
const uint8_t IMU_FAIL_MAX    = 10;       // consecutive bad IMU reads (50 ms) -> motors off
const unsigned long CMD_TIMEOUT = 60;     // ms: flush a command with no newline
const int DEFAULT_MICROSTEP   = 8;        // 1/8

// Drive behaviour
const float TURN_SLEW         = 3000.0;   // steps/s^2 change in turn rate
const int   TURN_SIGN         = -1;       // set to 1 if L and R are swapped
const int   DRIVE_SIGN        = 1;        // set to -1 if F leans/drives backward
const unsigned long DRIVE_TIMEOUT_MS = 600; // auto-stop if the remote goes silent (0 = latch until 'S')
const bool DEBUG_REMOTE = true;           // print received drive commands to USB Serial (set false when done)

// ---------- MPU6050 ----------
const uint8_t MPU = 0x68;
float angleOffset  = 0.0;   // accel mounting offset (deg), saved in EEPROM
float gyroBias     = 0.0;   // raw counts, measured at boot / cal
float currentAngle = 0.0;

// ---------- Control gains ----------
float Kp = 280.0, Ki = 0.0, Kd = 15.0, setpoint = 0.0;
// Position-hold PID (separate from the balance PID above)
float KpPos = 0.0005;       // x: lean (deg) per step of position error
float KiPos = 0.0;          // y: lean (deg) per step*second of accumulated error
float KdPos = 0.0010;       // z: lean (deg) per step/s of wheel speed (damping -> fast, steady stop)
float posInt = 0.0;         // integral of position error (step*s)
float leanHold = 0.0;       // filtered P+I part of the position PID output (deg)
float outerAcc = 0.0;
uint8_t outerDiv = 0;
const float LEAN_I_MAX     = 3.0;      // deg: max lean from the position integral
const float HOLD_LEAN_MAX  = 3.0;      // deg: max lean from the hold (P+I+D) alone
const float POS_DEADBAND   = 60.0;     // steps: no hold force inside this band (no hunting)
const float HOLD_FILTER    = 0.3;      // 0..1 low-pass on the hold P/I output (50 Hz tick)
float integral = 0.0, speedCmd = 0.0;

// Learned balance point: the tilt (deg, relative to 'setpoint') at which the bot really
// stands without wheel acceleration (centre-of-gravity offset). Learned while standing still.
// It is added to the balance target so forward and backward both start from the true
// equilibrium instead of one direction having to first "wind up" past the offset.
float restTilt = 0.0;
const float REST_FILTER = 0.002;   // per 5 ms loop -> ~2.5 s time constant (slow and safe)
const float REST_MAX    = 4.0;     // deg: never learn more than this

// ---------- Drive state ----------
float driveSpeed  = 1200.0; // steps/s at 100% forward/back (v command)
float turnSpeed   = 800.0;  // steps/s differential at 100% rotation (t command)
float targetSpeed = 0.0;    // commanded drive speed in steps/s (+ = forward)
float leanRamped  = 0.0;    // lean command after slew limiting
float driveI      = 0.0;    // integral of the speed error while driving (deg of lean)
const float LEAN_SLEW   = 6.0;     // deg/s: how fast the lean command may change
const float DRIVE_KP    = 0.0010;  // deg of lean per step/s of speed error
const float DRIVE_KI    = 0.0008;  // deg of lean per step of accumulated speed error
const float DRIVE_I_MAX = 4.0;     // deg: max lean from the integral
float targetTurn  = 0.0;    // commanded turn rate (steps/s)
float turnRate    = 0.0;    // ramped turn rate actually applied
unsigned long lastDriveCmd = 0;

bool botActive   = true;
bool fallen      = false;
bool calibrating = false;
int  microstep   = DEFAULT_MICROSTEP;
uint8_t imuFails = 0;

// ---------- Stepper state (shared with ISRs) ----------
volatile unsigned int intervalL = 0, intervalR = 0;   // timer ticks between steps
volatile bool dirFwdL = true, dirFwdR = true;
volatile long stepCountL = 0, stepCountR = 0;         // + forward, - backward

unsigned long lastLoop = 0;

// Auto-zero of the held position
unsigned long settleUntil = 0;           // keep re-zeroing position until this time (ms)
unsigned long stillSince  = 0;           // when bot became upright + still (0 = not yet)
const unsigned long SETTLE_MS = 1000;
const unsigned long STILL_MS  = 500;
const float STILL_RATE        = 25.0;    // deg/s

// Braking -> hold: after a stop command (button released) the home position keeps
// following the bot until it has actually slowed down, then locks there and holds.
const unsigned long BRAKE_SETTLE_MS = 300;   // minimum re-home time after stop
const unsigned long BRAKE_MAX_MS    = 3000;  // give up waiting for it to slow down
const float HOLD_ENGAGE_SPEED       = 250.0; // steps/s: below this the hold locks in
bool braking = false;
unsigned long brakeStart = 0;
void startBraking() {
  braking = true;
  brakeStart = millis();
  settleUntil = millis() + BRAKE_SETTLE_MS;
}

// ---------- EEPROM ----------
#define ADDR_KP      0
#define ADDR_KI      4
#define ADDR_KD      8
#define ADDR_SP     12
#define ADDR_CHECK  16
#define ADDR_OFFSET 20
#define ADDR_KX     24
#define ADDR_KW     28
#define ADDR_CHECK2 32
#define ADDR_PXP    48
#define ADDR_PXI    52
#define ADDR_PXD    56
#define ADDR_CHECK4 60
#define ADDR_DRV    64
#define ADDR_TRN    68
#define ADDR_CHECK5 72

// ---------- Output helpers (no String) ----------
void say(const __FlashStringHelper* s) { BT.println(s); Serial.println(s); }
void sayKV(const __FlashStringHelper* k, float v) {
  BT.print(k); BT.println(v, 4);
  Serial.print(k); Serial.println(v, 4);
}
void sayKS(const __FlashStringHelper* k, const char* v) {
  BT.print(k); BT.println(v);
  Serial.print(k); Serial.println(v);
}

// ---------- EEPROM ----------
void saveToEEPROM() {
  EEPROM.put(ADDR_KP, Kp);
  EEPROM.put(ADDR_KI, Ki);
  EEPROM.put(ADDR_KD, Kd);
  EEPROM.put(ADDR_SP, setpoint);
  EEPROM.put(ADDR_OFFSET, angleOffset);
  EEPROM.update(ADDR_CHECK, 0xAB);
  EEPROM.put(ADDR_PXP, KpPos);
  EEPROM.put(ADDR_PXI, KiPos);
  EEPROM.put(ADDR_PXD, KdPos);
  EEPROM.update(ADDR_CHECK4, 0xAE);
  EEPROM.put(ADDR_DRV, driveSpeed);
  EEPROM.put(ADDR_TRN, turnSpeed);
  EEPROM.update(ADDR_CHECK5, 0xAF);
  say(F("saved to eeprom"));
}

void loadFromEEPROM() {
  if (EEPROM.read(ADDR_CHECK) == 0xAB) {
    EEPROM.get(ADDR_KP, Kp);
    EEPROM.get(ADDR_KI, Ki);
    EEPROM.get(ADDR_KD, Kd);
    EEPROM.get(ADDR_SP, setpoint);
    EEPROM.get(ADDR_OFFSET, angleOffset);
    Serial.println(F("eeprom loaded"));
  } else {
    Serial.println(F("no eeprom data - using defaults"));
  }
  if (EEPROM.read(ADDR_CHECK4) == 0xAE) {
    EEPROM.get(ADDR_PXP, KpPos);
    EEPROM.get(ADDR_PXI, KiPos);
    EEPROM.get(ADDR_PXD, KdPos);
  }
  if (EEPROM.read(ADDR_CHECK5) == 0xAF) {
    EEPROM.get(ADDR_DRV, driveSpeed);
    EEPROM.get(ADDR_TRN, turnSpeed);
  }
  setpoint = constrain(setpoint, -SETPOINT_MAX, SETPOINT_MAX);   // guard against a bad saved value
}

// ---------- Microstepping (DRV8825) ----------
//   1/1: LLL   1/2: HLL   1/4: LHL   1/8: HHL   1/16: LLH   1/32: HLH
void setMicrostep(int m) {
  bool a, b, c;
  switch (m) {
    case 1:  a = 0; b = 0; c = 0; break;
    case 2:  a = 1; b = 0; c = 0; break;
    case 4:  a = 0; b = 1; c = 0; break;
    case 8:  a = 1; b = 1; c = 0; break;
    case 16: a = 0; b = 0; c = 1; break;
    case 32: a = 1; b = 0; c = 1; break;
    default: say(F("use m1 m2 m4 m8 m16 m32")); return;
  }
  digitalWrite(MS0_PIN, a);
  digitalWrite(MS1_PIN, b);
  digitalWrite(MS2_PIN, c);
  microstep = m;
  sayKV(F("microstep = 1/"), m);
}

// ---------- Stepper timers ----------
// Timer1 free-runs at 2 MHz (prescaler 8). Each wheel has its own compare
// channel (A = left, B = right); every ISR pulses its step pin and schedules
// its own next step, so the wheels can run at different speeds (turning).
ISR(TIMER1_COMPA_vect) {
  PORTD |=  _BV(5);
  __builtin_avr_delay_cycles(32);               // ~2 us pulse
  PORTD &= ~_BV(5);
  unsigned int next = OCR1A + intervalL;
  unsigned int nowT = TCNT1;
  if ((int)(next - nowT) < 10) next = nowT + intervalL;   // we were late: don't wrap
  OCR1A = next;
  if (dirFwdL) stepCountL++; else stepCountL--;
}

ISR(TIMER1_COMPB_vect) {
  PORTD |=  _BV(6);
  __builtin_avr_delay_cycles(32);
  PORTD &= ~_BV(6);
  unsigned int next = OCR1B + intervalR;
  unsigned int nowT = TCNT1;
  if ((int)(next - nowT) < 10) next = nowT + intervalR;
  OCR1B = next;
  if (dirFwdR) stepCountR++; else stepCountR--;
}

void timerInit() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = _BV(CS11);                 // normal mode, prescaler 8
  TIMSK1 &= ~(_BV(OCIE1A) | _BV(OCIE1B));
  interrupts();
}

void setRateL(float rate) {           // rate >= 0 (steps/s)
  if (rate < MIN_RATE) { TIMSK1 &= ~_BV(OCIE1A); return; }
  if (rate > MAX_RATE) rate = MAX_RATE;
  unsigned int iv = (unsigned int)(2000000UL / (unsigned long)rate);
  noInterrupts();
  bool wasOn = TIMSK1 & _BV(OCIE1A);
  intervalL = iv;
  if (!wasOn) {
    OCR1A = TCNT1 + iv;
    TIFR1 = _BV(OCF1A);               // clear any stale flag
    TIMSK1 |= _BV(OCIE1A);
  }
  interrupts();
}

void setRateR(float rate) {
  if (rate < MIN_RATE) { TIMSK1 &= ~_BV(OCIE1B); return; }
  if (rate > MAX_RATE) rate = MAX_RATE;
  unsigned int iv = (unsigned int)(2000000UL / (unsigned long)rate);
  noInterrupts();
  bool wasOn = TIMSK1 & _BV(OCIE1B);
  intervalR = iv;
  if (!wasOn) {
    OCR1B = TCNT1 + iv;
    TIFR1 = _BV(OCF1B);
    TIMSK1 |= _BV(OCIE1B);
  }
  interrupts();
}

void setDirL(bool fwd) {
  if (fwd == dirFwdL) return;
  dirFwdL = fwd;
  digitalWrite(LEFT_DIR_PIN, fwd ? HIGH : LOW);
}

void setDirR(bool fwd) {
  if (fwd == dirFwdR) return;
  dirFwdR = fwd;
  digitalWrite(RIGHT_DIR_PIN, fwd ? LOW : HIGH);   // right motor is mirrored
}

// Signed wheel speeds in steps/s, positive = forward
void driveWheels(float l, float r) {
  setDirL(l > 0);
  setDirR(r > 0);
  setRateL(fabs(l));
  setRateR(fabs(r));
}

void zeroPosition() {
  noInterrupts();
  stepCountL = 0;
  stepCountR = 0;
  interrupts();
}

long readPosition() {                 // average of both wheels (spinning in place = 0)
  noInterrupts();
  long l = stepCountL, r = stepCountR;
  interrupts();
  return (l + r) / 2;
}

void clearDriveTargets() {
  targetSpeed = 0;
  driveI      = 0;
  targetTurn = 0;
  turnRate   = 0;
  leanRamped = 0;
  leanHold = 0;          // also reset the position PID
  posInt   = 0;
  outerAcc = 0;
  outerDiv = 0;
}

void motorsOff() {
  setRateL(0);
  setRateR(0);
  speedCmd = 0;
  clearDriveTargets();
  digitalWrite(EN_PIN, HIGH);
}

void motorsOn() {
  digitalWrite(EN_PIN, LOW);
}

// ---------- Drive commands ----------
// Returns true if c was a drive letter.
bool handleDrive(char c) {
  float f = 0, t = 0;
  switch (toupper(c)) {
    case 'F': f =  1; break;
    case 'B': f = -1; break;
    case 'L': t = -1; break;
    case 'R': t =  1; break;
    case 'G': f =  1; t = -0.6; break;
    case 'I': f =  1; t =  0.6; break;
    case 'H': f = -1; t = -0.6; break;
    case 'J': f = -1; t =  0.6; break;
    case 'Q': t = -1.5; break;          // spin in place (remote): counter-clockwise
    case 'E': t =  1.5; break;          // spin in place (remote): clockwise
    case 'S': break;
    default:  return false;
  }
  if (DEBUG_REMOTE) {
    Serial.print(F("drive ")); Serial.print(c);
    Serial.println((!botActive || fallen) ? F("  IGNORED (stopped/fallen)") : F(""));
  }
  if (!botActive || fallen) { clearDriveTargets(); return true; }

  float newSpeed = f * driveSpeed * DRIVE_SIGN;
  if (newSpeed == 0 && (targetSpeed != 0 || targetTurn != 0)) startBraking();  // brake, then hold where it stops
  targetSpeed  = newSpeed;
  targetTurn   = t * turnSpeed * TURN_SIGN;
  lastDriveCmd = millis();
  return true;
}

// Proportional drive from the remote: fp = forward/back %, tp = rotate % (both -100..100).
// 100% forward = driveSpeed steps/s (v command), 100% rotate = turnSpeed (t command).
void handleAnalog(float fp, float tp) {
  fp = constrain(fp, -100.0, 100.0);
  tp = constrain(tp, -100.0, 100.0);
  if (DEBUG_REMOTE) {
    Serial.print(F("C ")); Serial.print(fp, 0); Serial.print(' '); Serial.print(tp, 0);
    Serial.println((!botActive || fallen) ? F("  IGNORED (stopped/fallen)") : F(""));
  }
  if (!botActive || fallen) { clearDriveTargets(); return; }

  float newSpeed = fp / 100.0 * driveSpeed * DRIVE_SIGN;
  float tScale  = (fp != 0) ? 0.6 : 1.5;       // gentler turn while moving, faster spin in place
  if (newSpeed == 0 && (targetSpeed != 0 || targetTurn != 0)) startBraking();
  targetSpeed  = newSpeed;
  targetTurn   = tp / 100.0 * tScale * turnSpeed * TURN_SIGN;
  lastDriveCmd = millis();
}

// Strict parser for "C<int>,<int>" with both values in -100..100.
// Anything else (corrupted bytes, missing comma, out of range) is rejected.
bool parseAnalog(const char* s, float &fp, float &tp) {
  const char* p = s + 1;
  char* end;
  long a = strtol(p, &end, 10);
  if (end == p || *end != ',') return false;
  p = end + 1;
  long b = strtol(p, &end, 10);
  if (end == p || *end != 0) return false;
  if (a < -100 || a > 100 || b < -100 || b > 100) return false;
  fp = (float)a;
  tp = (float)b;
  return true;
}

// ---------- IMU ----------
static int16_t read16() {
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

// Reads accel Y, Z and gyro X: AX AY AZ TEMP GX = 10 bytes
bool readIMU(int16_t &ay, int16_t &az, int16_t &gx) {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU, (uint8_t)10, (uint8_t)true) < 10) return false;
  read16();                // ax (unused)
  ay = read16();
  az = read16();
  read16();                // temp (unused)
  gx = read16();
  return true;
}

float accelAngle(int16_t ay, int16_t az) {
  return atan2((float)ay, (float)az) * 180.0 / PI - angleOffset;
}

void calibrateGyroQuick() {
  long sum = 0; int n = 0;
  for (int i = 0; i < 300; i++) {
    int16_t ay, az, gx;
    if (readIMU(ay, az, gx)) { sum += gx; n++; }
    delay(3);
  }
  if (n > 0) gyroBias = (float)sum / n;
}

void calibrateIMU() {
  calibrating = true;
  motorsOff();
  say(F("calibration: keep bot upright and still, 10 s"));

  float angleSum = 0; long gyroSum = 0; int n = 0;
  unsigned long start = millis();
  int lastRemaining = 11;

  while (millis() - start < 10000) {
    int16_t ay, az, gx;
    if (readIMU(ay, az, gx)) {
      angleSum += atan2((float)ay, (float)az) * 180.0 / PI;
      gyroSum  += gx;
      n++;
    }
    int remaining = 10 - (int)((millis() - start) / 1000);
    if (remaining != lastRemaining) {
      lastRemaining = remaining;
      sayKV(F("sec remaining: "), remaining);
    }
    delay(10);
  }

  if (n > 0) {
    angleOffset = angleSum / n;
    gyroBias    = (float)gyroSum / n;
  }
  currentAngle = 0;
  integral = 0;
  zeroPosition();
  saveToEEPROM();
  sayKV(F("offset = "), angleOffset);
  sayKV(F("gyro bias = "), gyroBias);
  say(F("done - send 'start'"));

  botActive = false;       // stay stopped until 'start'
  lastLoop = micros();
  calibrating = false;
}

// ---------- Commands ----------
void handleCommand(char* s) {
  while (*s == ' ') s++;
  int len = strlen(s);
  while (len > 0 && s[len - 1] == ' ') s[--len] = 0;
  if (len == 0) return;

  // Drive letters (also "FFFF" if an app sends several with no newline).
  // No echo here: Bluetooth TX would stall the stepper timing.
  bool same = true;
  for (int i = 1; i < len; i++) if (s[i] != s[0]) same = false;
  if (same && handleDrive(s[0])) return;

  // Proportional remote command: C<fwd%>,<turn%>  e.g. C50,-30  (silent, no echo)
  // Strictly validated - a corrupted packet is dropped, never half-applied.
  if (toupper(s[0]) == 'C' && (isDigit(s[1]) || s[1] == '-' || s[1] == '+')) {
    float fp, tp;
    if (parseAnalog(s, fp, tp)) handleAnalog(fp, tp);
    else if (DEBUG_REMOTE) Serial.println(F("bad C packet - dropped"));
    return;
  }

  sayKS(F("rx: "), s);

  if (!strcasecmp(s, "show")) {
    sayKV(F("p        = "), Kp);
    sayKV(F("i        = "), Ki);
    sayKV(F("d        = "), Kd);
    sayKV(F("x (pos P)= "), KpPos);
    sayKV(F("y (pos I)= "), KiPos);
    sayKV(F("z (pos D)= "), KdPos);
    sayKV(F("v (speed) = "), driveSpeed);
    sayKV(F("t (turn) = "), turnSpeed);
    sayKV(F("setpoint = "), setpoint);
    sayKV(F("offset   = "), angleOffset);
    sayKV(F("rest tilt= "), restTilt);
    sayKV(F("microstep= 1/"), microstep);
    sayKV(F("angle    = "), currentAngle);
    sayKV(F("position = "), (float)readPosition());
    sayKS(F("bot      = "), botActive ? "running" : "stopped");
    sayKS(F("fallen   = "), fallen ? "yes" : "no");
    return;
  }
  if (!strcasecmp(s, "start")) {
    botActive = true; integral = 0; speedCmd = 0;
    clearDriveTargets();
    zeroPosition();
    settleUntil = millis() + SETTLE_MS;
    if (!fallen) motorsOn();
    say(F("bot started"));
    return;
  }
  if (!strcasecmp(s, "stop")) {
    botActive = false; integral = 0;
    motorsOff();
    say(F("bot stopped"));
    return;
  }
  if (!strcasecmp(s, "zero")) { zeroPosition(); say(F("position zeroed")); return; }
  if (!strcasecmp(s, "cal"))  { calibrateIMU(); return; }
  if (!strcasecmp(s, "save")) { saveToEEPROM(); return; }

  // microstepping: m1 m2 m4 m8 m16 m32
  if (toupper(s[0]) == 'M' && isDigit(s[1])) {
    if (botActive) { say(F("send 'stop' first")); return; }
    setMicrostep(atoi(s + 1));
    return;
  }

  // single key + number: p/i/d/s/x/y/z/v/t
  char key = toupper(s[0]);
  char* v = s + 1;
  while (*v == ' ') v++;
  if (key != 'P' && key != 'I' && key != 'D' && key != 'S' &&
      key != 'X' && key != 'Y' && key != 'Z' && key != 'V' && key != 'T') {
    Serial.println(F("ignored - bad key")); return;
  }
  if (!(isDigit(*v) || *v == '-' || *v == '.')) {
    Serial.println(F("ignored - bad value")); return;
  }
  float val = atof(v);
  switch (key) {
    case 'P': Kp = val; sayKV(F("p = "), Kp); break;
    case 'I': Ki = val; integral = 0; sayKV(F("i = "), Ki); break;
    case 'D': Kd = val; sayKV(F("d = "), Kd); break;
    case 'S': setpoint = constrain(val, -SETPOINT_MAX, SETPOINT_MAX); sayKV(F("setpoint = "), setpoint); break;
    case 'X': KpPos = val; sayKV(F("pos P (x) = "), KpPos); break;
    case 'Y': KiPos = val; posInt = 0; sayKV(F("pos I (y) = "), KiPos); break;
    case 'Z': KdPos = val; sayKV(F("pos D (z) = "), KdPos); break;
    case 'V': driveSpeed = constrain(fabs(val), 0, MAX_RATE * 0.8); sayKV(F("drive speed (steps/s) = "), driveSpeed); break;
    case 'T': turnSpeed  = constrain(fabs(val), 0, MAX_RATE * 0.6); sayKV(F("turn speed (steps/s) = "), turnSpeed); break;
  }
  say(F("(send 'save' to keep)"));
}

// ---------- Input buffering ----------
char lineBuf[24];
uint8_t lineLen = 0;
unsigned long lastByteTime = 0;

void flushLine() {
  if (lineLen == 0) return;
  lineBuf[lineLen] = 0;
  lineLen = 0;
  handleCommand(lineBuf);
}

void feed(char c) {
  lastByteTime = millis();
  if (c == '\n' || c == '\r') { flushLine(); return; }
  if (lineLen < sizeof(lineBuf) - 1) lineBuf[lineLen++] = c;
}

// ---------- Setup ----------
void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  Wire.begin();
  Wire.setClock(400000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif

  pinMode(LEFT_STEP_PIN, OUTPUT);
  pinMode(RIGHT_STEP_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(MS0_PIN, OUTPUT);
  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);

  digitalWrite(EN_PIN, HIGH);                 // off until we're ready
  digitalWrite(LEFT_DIR_PIN, HIGH);           // matches dirFwdL = true
  digitalWrite(RIGHT_DIR_PIN, LOW);           // matches dirFwdR = true
  setMicrostep(DEFAULT_MICROSTEP);

  // wake MPU6050, gyro +/-250 dps, accel +/-2g, DLPF ~44 Hz
  Wire.beginTransmission(MPU); Wire.write(0x6B); Wire.write(0);    Wire.endTransmission(true);
  Wire.beginTransmission(MPU); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(true);
  Wire.beginTransmission(MPU); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission(true);

  loadFromEEPROM();
  timerInit();

  say(F("keep bot still - measuring gyro bias..."));
  calibrateGyroQuick();

  int16_t ay, az, gx;
  if (readIMU(ay, az, gx)) currentAngle = accelAngle(ay, az);

  motorsOn();
  lastLoop = micros();
  settleUntil = millis() + SETTLE_MS;

  say(F("self-balancing bot ready"));
  say(F("drive: F B L R S (G I H J diagonals), v t = speeds"));
}

// ---------- Loop ----------
void loop() {
  // 1. Comms
  while (BT.available())     feed((char)BT.read());
  while (Serial.available()) feed((char)Serial.read());
  if (lineLen > 0 && millis() - lastByteTime > CMD_TIMEOUT) flushLine();

  if (calibrating) return;

  // 2. Fixed-rate control loop
  unsigned long now = micros();
  if (now - lastLoop < LOOP_US) return;
  float dt = (now - lastLoop) / 1000000.0;
  lastLoop = now;

  // 3. IMU
  int16_t ay, az, gx;
  if (!readIMU(ay, az, gx)) {          // bad read: fail safe
    setRateL(0);
    setRateR(0);
    if (++imuFails >= IMU_FAIL_MAX && !fallen) {
      fallen = true;
      integral = 0;
      stillSince = 0;
      motorsOff();
      say(F("imu fault - motors off"));
    }
    return;
  }
  imuFails = 0;
  float gyroRate = ((float)gx - gyroBias) / 131.0;       // deg/s
  float accAngle = accelAngle(ay, az);
  // if the accelerometer reading is near zero (glitch), trust the gyro only for this step
  bool accOK = ((long)abs(ay) + (long)abs(az)) > 6000L;
  float predAngle = currentAngle + gyroRate * dt;
  currentAngle = accOK ? (ALPHA * predAngle + (1.0 - ALPHA) * accAngle) : predAngle;

  // 4. Fall detection with hysteresis (uses true tilt, not the trim)
  float trueTilt = currentAngle - setpoint;
  if (!fallen && fabs(trueTilt) > FALL_ANGLE) {
    fallen = true;
    integral = 0;
    stillSince = 0;
    motorsOff();
    say(F("bot fallen - motors off"));
  } else if (fallen) {
    zeroPosition();
    if (fabs(trueTilt) <= RECOVER_ANGLE && fabs(gyroRate) < STILL_RATE) {
      if (stillSince == 0) stillSince = millis();
      if (millis() - stillSince >= STILL_MS) {
        fallen = false;
        integral = 0;
        speedCmd = 0;
        stillSince = 0;
        clearDriveTargets();
        zeroPosition();
        settleUntil = millis() + SETTLE_MS;
        if (botActive) motorsOn();
        say(F("bot upright - resuming, position zeroed"));
      }
    } else {
      stillSince = 0;
    }
  }
  if (fallen || !botActive) return;

  // 5. Optional drive timeout (auto-stop if the controller goes silent)
  if (DRIVE_TIMEOUT_MS > 0 && (targetSpeed != 0 || targetTurn != 0) &&
      millis() - lastDriveCmd > DRIVE_TIMEOUT_MS) {
    if (targetSpeed != 0 || targetTurn != 0) startBraking();
    targetSpeed = 0;
    targetTurn = 0;
  }

  // 5b. COMMAND AUTHORITY (stability first).
  //     1.0 = commands/hold fully allowed, 0.0 = pure balancing only.
  //     Fades with tilt (7 -> 12 deg) and with fast gyro motion (40 -> 90 deg/s).
  float tiltAuth = constrain((SOFT_TILT - fabs(trueTilt)) / (SOFT_TILT - CMD_TILT_FULL), 0.0, 1.0);
  float rateAuth = constrain((RATE_ZERO - fabs(gyroRate)) / (RATE_ZERO - RATE_FULL), 0.0, 1.0);
  float auth = (tiltAuth < rateAuth) ? tiltAuth : rateAuth;

  // 6. Position hold / velocity control via lean trim
  //    While driving (or just after stopping) the home position follows the bot.
  // DRIVE SPEED PI: speed error -> lean. The integral absorbs any balance-point
  // offset, so forward and backward reach the same speed.
  float leanDrive = 0;
  if (targetSpeed != 0) {
    float vErr = targetSpeed - speedCmd;
    if (auth > 0.99) {                       // no integral windup while commands are being suppressed
      driveI += DRIVE_KI * vErr * dt;
      driveI = constrain(driveI, -DRIVE_I_MAX, DRIVE_I_MAX);
    }
    leanDrive = DRIVE_KP * vErr + driveI;
  } else {
    driveI = 0;
  }
  leanRamped += constrain(leanDrive - leanRamped, -LEAN_SLEW * dt, LEAN_SLEW * dt);
  bool driving  = (targetSpeed != 0 || fabs(leanRamped) > 0.05);
  bool settling = (millis() < settleUntil);
  if (braking) {                             // stop requested: wait until it has really slowed down
    if ((fabs(speedCmd) > HOLD_ENGAGE_SPEED || fabs(leanRamped) > 0.3) &&
        millis() - brakeStart < BRAKE_MAX_MS) {
      settleUntil = millis() + 100;          // keep re-homing: the spot where it stops becomes home
      settling = true;
    } else if (!settling) {
      braking = false;                       // slow enough: hold is now locked at this spot
    }
  }
  if (auth < 0.5) {                          // wobbling hard: forget the home spot, just balance
    zeroPosition();
    settleUntil = millis() + 300;
    settling = true;
  }
  if (driving || settling) zeroPosition();   // home follows the bot

  // Learn the real balance point while standing still and upright
  if (!driving && !settling && fabs(gyroRate) < STILL_RATE && fabs(trueTilt) < 5.0) {
    restTilt += REST_FILTER * (constrain(trueTilt, -REST_MAX, REST_MAX) - restTilt);
  }

  // POSITION PID: P and I run at 50 Hz, D (wheel speed) runs every loop
  outerAcc += dt;
  if (++outerDiv >= 4) {
    float odt = outerAcc;
    outerAcc = 0;
    outerDiv = 0;

    // position error with a deadband: nothing happens inside +/-POS_DEADBAND steps,
    // outside it the error grows smoothly from zero (no jump, no hunting)
    float posErr = 0;
    if (!driving && !settling) {
      float raw = (float)readPosition();
      if (raw >  POS_DEADBAND) posErr = raw - POS_DEADBAND;
      else if (raw < -POS_DEADBAND) posErr = raw + POS_DEADBAND;
    }

    if (driving || settling || KiPos <= 0) {
      posInt = 0;                            // no integral while driving / settling
    } else {
      posInt += posErr * odt;
      float lim = LEAN_I_MAX / KiPos;
      posInt = constrain(posInt, -lim, lim); // anti-windup
    }
    float holdNow = -(KpPos * posErr + KiPos * posInt);
    leanHold += HOLD_FILTER * (holdNow - leanHold);   // low-pass: smooth, no sudden lean jumps
  }
  // D term on measured wheel speed (damps drift fast, and sets top speed while driving)
  // while driving the speed PI sets the lean; otherwise the position PID holds the spot
  float holdLean = constrain(leanHold - KdPos * speedCmd, -HOLD_LEAN_MAX, HOLD_LEAN_MAX);
  float lean = (targetSpeed != 0) ? leanRamped : (holdLean + leanRamped);
  lean *= auth;                              // stability first: fade commands out when unstable
  lean = constrain(lean, -TRIM_MAX, TRIM_MAX);
  float tilt = trueTilt - lean - restTilt;   // balance PID works toward (learned balance point + lean)

  // 7. PID -> acceleration command, integrated into speed
  integral += tilt * dt;
  integral = constrain(integral, -I_MAX, I_MAX);
  float accel = Kp * tilt + Ki * integral + Kd * gyroRate;
  speedCmd += accel * dt;
  speedCmd = constrain(speedCmd, -MAX_RATE, MAX_RATE);

  // 8. Ramp the turn rate (also scaled by authority), then split speed between the wheels
  float maxT = TURN_SLEW * dt;
  turnRate += constrain(targetTurn * auth - turnRate, -maxT, maxT);

  float l = constrain(speedCmd + turnRate, -MAX_RATE, MAX_RATE);
  float r = constrain(speedCmd - turnRate, -MAX_RATE, MAX_RATE);
  driveWheels(l, r);
}
