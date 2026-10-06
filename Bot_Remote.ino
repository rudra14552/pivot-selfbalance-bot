// ESP32 handheld motion remote for the self-balancing bot
// MPU6050 + push button (deadman). Talks to the bot's HC-05 over Bluetooth Classic.
//
// How it works:
//   * Button NOT pressed -> bot gets 'S' (stands still, only balances). Remote orientation is ignored.
//   * Button pressed     -> the current hand position becomes "normal" (all angles reset to 0).
//                           Rotating the remote RELATIVE to that moment drives the bot.
//   * Button released    -> 'S' is sent, motion stops.
//
// Angles come from integrating the gyro ONLY while the button is held, so there is
// nothing to drift between presses and the mounting orientation doesn't matter.
//
// DEADZONES (so forward doesn't leak into left/right):
//   * GYRO_DEADBAND: tiny rotation rates are ignored (no drift / hand tremor).
//   * AXIS_DOMINANCE: if one axis is clearly bigger than the other, the weaker axis
//     is zeroed. Forward/back only turns the bot if you really rotate on both axes.
//   * FWD_ON / SPIN_ON: angle threshold before anything is sent.
//
// Axis mapping (change the AX_/SIGN_ constants below to match how you hold it):
//   rotation about Z -> forward / backward
//   rotation about Y -> rotate the bot clockwise / counter-clockwise (spin in place)
//   (X is not used)
//
// Needs a CLASSIC ESP32 (ESP32-WROOM DevKit). ESP32-S2/S3/C3 have no Bluetooth Classic.

#include <Wire.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth Classic is not enabled - use a classic ESP32 board"
#endif

BluetoothSerial SerialBT;

// ---------- Pins ----------
#define BTN_PIN   4          // button between GPIO4 and GND (internal pull-up)
#define LED_PIN   2          // onboard LED: on while button held
#define SDA_PIN   21
#define SCL_PIN   22

// ---------- Bluetooth target ----------
const char* BOT_NAME = "Pivot";      // name of the bot's HC-05
const char* BOT_PIN  = "1234";       // HC-05 default pairing PIN (sometimes 0000)

// ---------- Axis mapping (0 = X, 1 = Y, 2 = Z) ----------
const int   AX_FWD   = 2;   const float SIGN_FWD  = +1;   // Z: + => forward
const int   AX_SPIN  = 1;   const float SIGN_SPIN = +1;   // Y: + => clockwise

// ---------- Thresholds (degrees rotated since the button was pressed) ----------
const float FWD_ON   = 15.0, FWD_OFF   = 10.0;    // on > off gives hysteresis (bigger = stricter)
const float SPIN_ON  = 15.0, SPIN_OFF  = 10.0;
const bool  COMBINE_MOVE_AND_TURN = false;  // strict mode: ONE axis at a time (the axis lock below enforces it)

// ---------- Deadzones ----------
const float GYRO_DEADBAND  = 1.2;    // dps: rates below this are ignored while integrating (no drift/tremor)
// AXIS LOCK: a gesture is accepted only when one axis crosses its threshold AND is at least
// AXIS_DOMINANCE times bigger than the other axis (both measured relative to their thresholds).
// Ambiguous gestures send nothing. Once accepted, the other axis is locked out (and its
// integrated angle forced to 0) until you return to neutral, so forward can never leak into left/right.
const float AXIS_DOMINANCE = 2.5;    // raise (e.g. 3.0) = stricter, lower = more forgiving
const float REL_CLAMP      = 60.0;   // deg: integrated angle is limited so you can come back to neutral quickly

// PROPORTIONAL = true : the further you rotate, the faster the bot moves (needs the new bot sketch)
// PROPORTIONAL = false: fixed-speed letters F B Q E ... exactly like the version that worked before
const bool PROPORTIONAL = true;

// Proportional control: 0 at rest, MIN_xxx_PCT right at the threshold angle,
// rising linearly to 100% at the FULL angle (degrees rotated since the button press).
const float FWD_FULL  = 40.0;      // deg for 100% forward/back
const float SPIN_FULL = 40.0;      // deg for 100% rotation
const int   MIN_FWD_PCT  = 60;     // % sent as soon as the threshold is crossed
const int   MIN_SPIN_PCT = 40;
const unsigned long SEND_MIN_MS = 100;   // max 10 updates/s (keeps the bot's Bluetooth load low)

const unsigned long KEEPALIVE_MS = 250;   // resend the active command at 4 Hz
const bool DEBUG = true;                  // print angles to USB Serial (115200)

// ---------- MPU6050 ----------
const uint8_t MPU = 0x68;
const float GYRO_LSB = 65.5;              // +/-500 dps range

float gyroBias[3] = {0, 0, 0};            // dps
float rel[3]      = {0, 0, 0};            // degrees rotated since button press

// ---------- State ----------
bool pressed = false;
int  fState = 0, rState = 0;              // -1 / 0 / +1 with hysteresis
int  lockAxis = 0;                        // 0 = undecided, 1 = forward/back only, 2 = rotate only
char lastCmd = 'S';
int  lastF = 0, lastT = 0;               // last forward% / rotate% sent
unsigned long lastSend = 0, lastUs = 0, lastDebug = 0, lastConnTry = 0;

// ---------- Button (debounced) ----------
bool lastRaw = false, stableBtn = false;
unsigned long lastChange = 0;

bool readButton() {
  bool raw = (digitalRead(BTN_PIN) == LOW);
  if (raw != lastRaw) { lastRaw = raw; lastChange = millis(); }
  if (millis() - lastChange > 25) stableBtn = raw;
  return stableBtn;
}

// ---------- MPU ----------
bool readGyro(float g[3]) {
  Wire.beginTransmission(MPU);
  Wire.write(0x43);                        // GYRO_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint16_t)MPU, (size_t)6, true) < 6) return false;
  for (int i = 0; i < 3; i++) {
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    g[i] = (int16_t)((hi << 8) | lo) / GYRO_LSB;   // dps
  }
  return true;
}

void calibrateBias() {
  float sum[3] = {0, 0, 0};
  int n = 0;
  for (int i = 0; i < 500; i++) {
    float g[3];
    if (readGyro(g)) { for (int k = 0; k < 3; k++) sum[k] += g[k]; n++; }
    delay(2);
  }
  if (n > 0) for (int k = 0; k < 3; k++) gyroBias[k] = sum[k] / n;
}

// ---------- Logic ----------
int applyHyst(int state, float v, float on, float off) {
  if (state == 0) {
    if (v >  on) return  1;
    if (v < -on) return -1;
    return 0;
  }
  if (state ==  1 && v <  off) return 0;
  if (state == -1 && v > -off) return 0;
  return state;
}

// f: +forward/-back, r: +clockwise/-counter-clockwise
// Rotation alone spins the bot in place (Q/E). With forward/back it curves (G I H J).
char pickCmd(int f, int r) {
  if (!COMBINE_MOVE_AND_TURN && f != 0) r = 0;
  if (f > 0) { if (r < 0) return 'G'; if (r > 0) return 'I'; return 'F'; }
  if (f < 0) { if (r < 0) return 'H'; if (r > 0) return 'J'; return 'B'; }
  if (r < 0) return 'Q';
  if (r > 0) return 'E';
  return 'S';
}

// Magnitude grows linearly from minPct (at the threshold) to 100 (at `full` degrees).
int pctOut(int state, float angle, float on, float full, int minPct) {
  if (state == 0) return 0;
  float x = (fabs(angle) - on) / (full - on);
  x = constrain(x, 0.0f, 1.0f);
  return state * (int)(minPct + (100 - minPct) * x);
}

// round to the nearest 5 % so the value only changes in steps (less Bluetooth traffic)
int quant5(int v) { return ((v >= 0 ? v + 2 : v - 2) / 5) * 5; }

void sendCmd(char c) {
  if (SerialBT.connected(1)) {
    SerialBT.print(c);
    SerialBT.print('\n');
  }
  lastCmd  = c;
  lastSend = millis();
}

void sendAnalog(int fp, int tp) {
  fp = constrain(fp, -100, 100);
  tp = constrain(tp, -100, 100);
  if (SerialBT.connected(1)) {
    SerialBT.print('C');
    SerialBT.print(fp);
    SerialBT.print(',');
    SerialBT.print(tp);
    SerialBT.print('\n');
  }
  lastF = fp;
  lastT = tp;
  lastSend = millis();
}

void sendStop() {
  sendCmd('S');
  lastF = lastT = 0;
}

void onPress() {
  for (int i = 0; i < 3; i++) rel[i] = 0;   // current hand position = "normal"
  fState = rState = 0;
  lockAxis = 0;
  lastCmd = 'S';
  lastF = lastT = 0;
  digitalWrite(LED_PIN, HIGH);
}

void onRelease() {
  digitalWrite(LED_PIN, LOW);
  for (int i = 0; i < 3; i++) {             // send stop a few times for reliability
    sendCmd('S');
    delay(20);
  }
  lastF = lastT = 0;
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Wire.beginTransmission(MPU); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(true);  // wake
  Wire.beginTransmission(MPU); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission(true);  // +/-500 dps
  Wire.beginTransmission(MPU); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission(true);  // DLPF ~44 Hz

  Serial.println(F("Keep the remote still - measuring gyro bias..."));
  calibrateBias();

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  SerialBT.setPin(BOT_PIN, 4);
#else
  SerialBT.setPin(BOT_PIN);
#endif
  SerialBT.begin("BotRemote", true);        // true = master
  Serial.println(F("Remote ready. Connecting to bot..."));
  SerialBT.connect(BOT_NAME);

  lastUs = micros();
}

// ---------- Loop ----------
void loop() {
  // 100 Hz
  unsigned long nowUs = micros();
  if (nowUs - lastUs < 10000) return;
  float dt = (nowUs - lastUs) / 1000000.0;
  lastUs = nowUs;

  // discard anything the bot sends back (its "rx:" echoes) so the buffer never fills
  while (SerialBT.available()) SerialBT.read();

  float g[3];
  if (!readGyro(g)) return;

  bool btn = readButton();
  if (btn && !pressed)  onPress();
  if (!btn && pressed)  onRelease();
  pressed = btn;

  if (pressed) {
    // integrate rotation since the press, with a per-axis rate deadband
    for (int i = 0; i < 3; i++) {
      float rate = g[i] - gyroBias[i];
      if (fabs(rate) < GYRO_DEADBAND) rate = 0;
      rel[i] += rate * dt;
      rel[i] = constrain(rel[i], -REL_CLAMP, REL_CLAMP);
    }

    float f = SIGN_FWD  * rel[AX_FWD];
    float r = SIGN_SPIN * rel[AX_SPIN];

    // strict axis lock (see AXIS_DOMINANCE): decide ONE axis, lock the other out
    if (lockAxis == 0) {
      float nf = fabs(f) / FWD_ON, nr = fabs(r) / SPIN_ON;   // 1.0 = at threshold
      if (nf >= 1.0 && nf > nr * AXIS_DOMINANCE)      lockAxis = 1;
      else if (nr >= 1.0 && nr > nf * AXIS_DOMINANCE) lockAxis = 2;
    }
    if (lockAxis == 1)      { r = 0; rel[AX_SPIN] = 0; }
    else if (lockAxis == 2) { f = 0; rel[AX_FWD]  = 0; }
    else                    { f = 0; r = 0; }                // undecided / ambiguous: send nothing

    fState = applyHyst(fState, f, FWD_ON,  FWD_OFF);
    rState = applyHyst(rState, r, SPIN_ON, SPIN_OFF);

    // back to neutral on the locked axis -> unlock, ready for the next gesture
    if (lockAxis == 1 && fState == 0) lockAxis = 0;
    if (lockAxis == 2 && rState == 0) lockAxis = 0;

    if (PROPORTIONAL) {
      // proportional output: -100..100 % for forward/back and rotation
      int fp = quant5(pctOut(fState, f, FWD_ON,  FWD_FULL,  MIN_FWD_PCT));
      int tp = quant5(pctOut(rState, r, SPIN_ON, SPIN_FULL, MIN_SPIN_PCT));
      if (!COMBINE_MOVE_AND_TURN && fp != 0) tp = 0;

      unsigned long nowMs = millis();
      if (fp == 0 && tp == 0) {
        if (lastF != 0 || lastT != 0) sendStop();                    // stop immediately
      } else if (fp != lastF || tp != lastT) {
        if (nowMs - lastSend >= SEND_MIN_MS) sendAnalog(fp, tp);     // value changed
      } else if (nowMs - lastSend > KEEPALIVE_MS) {
        sendAnalog(fp, tp);                                          // keep-alive
      }
    } else {
      // fixed-speed letters (the original, known-working behaviour)
      char cmd = pickCmd(fState, rState);
      if (cmd != lastCmd || (cmd != 'S' && millis() - lastSend > KEEPALIVE_MS)) {
        sendCmd(cmd);
      }
    }
  } else {
    // idle: slowly refine the gyro bias while the remote is still
    bool still = true;
    for (int i = 0; i < 3; i++) if (fabs(g[i] - gyroBias[i]) > 1.5) still = false;
    if (still) for (int i = 0; i < 3; i++) gyroBias[i] += 0.002 * (g[i] - gyroBias[i]);

    // reconnect if the link dropped (blocking, so only while the button is up)
    if (millis() - lastConnTry > 4000) {
      lastConnTry = millis();
      if (!SerialBT.connected(1)) {
        Serial.println(F("Not connected - trying..."));
        SerialBT.connect(BOT_NAME);
      }
    }
    // blink the LED when not connected
    if (!SerialBT.connected(0)) digitalWrite(LED_PIN, (millis() / 300) % 2);
    else                        digitalWrite(LED_PIN, LOW);
  }

  if (DEBUG && millis() - lastDebug > 200) {
    lastDebug = millis();
    Serial.print(pressed ? "HELD  " : "idle  ");
    Serial.print("X:"); Serial.print(rel[0], 1);
    Serial.print("  Y:"); Serial.print(rel[1], 1);
    Serial.print("  Z:"); Serial.print(rel[2], 1);
    Serial.print("  F%:"); Serial.print(lastF);
    Serial.print("  T%:"); Serial.print(lastT);
    Serial.print("  cmd:"); Serial.println(lastCmd);
  }
}
