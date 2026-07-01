#include <Wire.h>
#include <math.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

// ──────────────────────────────────────────
//  BLUETOOTH  (HC-05 on pins 2 & 3)
// ──────────────────────────────────────────
SoftwareSerial BT(10, 11);

// ──────────────────────────────────────────
//  PIN DEFINITIONS
// ──────────────────────────────────────────
#define LEFT_STEP_PIN   5
#define LEFT_DIR_PIN    7
#define RIGHT_STEP_PIN  6
#define RIGHT_DIR_PIN   8
#define EN_PIN 4

// ──────────────────────────────────────────
//  MPU6050
// ──────────────────────────────────────────
const int MPU = 0x68;
int16_t AcX, AcY, AcZ, GyX;
float accAngle, gyroRate, currentAngle;
unsigned long prevTime;

// ──────────────────────────────────────────
//  PID VARIABLES
// ──────────────────────────────────────────
float Kp       = 18.0;
float Ki       = 0.0;
float Kd       = 0.0;
float setpoint = 0.0;
float deadBand = 0.5;

float error      = 0;
float prevError  = 0;
float integral   = 0;
float derivative = 0;
float pidOutput  = 0;

const float alpha = 0.98;

bool botActive   = true;
bool calibrating = false;

// ──────────────────────────────────────────
//  FALL DETECTION
//  NEW: if the bot tips past FALL_ANGLE on either side, treat it as
//  "fallen" - stop the motors and don't run PID at all, instead of
//  fighting to recover from a hopeless angle. Once it's picked backs
//  up past FALL_ANGLE, it resumes balancing automatically.
// ──────────────────────────────────────────
const float FALL_ANGLE = 30.0;   // degrees, either side of vertical
bool fallen = false;

// ──────────────────────────────────────────
//  CALIBRATION
// ──────────────────────────────────────────
float angleOffset = 0.0;

// ──────────────────────────────────────────
//  EEPROM ADDRESSES
// ──────────────────────────────────────────
#define ADDR_KP     0
#define ADDR_KI     4
#define ADDR_KD     8
#define ADDR_SP    12
#define ADDR_CHECK 16
#define ADDR_OFFSET 20

// ──────────────────────────────────────────
//  BLUETOOTH BUFFER with timeout
// ──────────────────────────────────────────
String btBuffer            = "";
unsigned long lastByteTime = 0;
const unsigned long CMD_TIMEOUT = 200;   // ms

// ──────────────────────────────────────────
//  SEND TO BOTH
// ──────────────────────────────────────────
void btPrint(String msg) {
  BT.println(msg);
  Serial.println(msg);
}

// ──────────────────────────────────────────
//  EEPROM FUNCTIONS
// ──────────────────────────────────────────
void saveToEEPROM() {
  EEPROM.put(ADDR_KP,     Kp);
  EEPROM.put(ADDR_KI,     Ki);
  EEPROM.put(ADDR_KD,     Kd);
  EEPROM.put(ADDR_SP,     setpoint);
  EEPROM.put(ADDR_OFFSET, angleOffset);
  EEPROM.write(ADDR_CHECK, 0xAB);
  btPrint("saved to eeprom");
}

void loadFromEEPROM() {
  if (EEPROM.read(ADDR_CHECK) == 0xAB) {
    EEPROM.get(ADDR_KP,     Kp);
    EEPROM.get(ADDR_KI,     Ki);
    EEPROM.get(ADDR_KD,     Kd);
    EEPROM.get(ADDR_SP,     setpoint);
    EEPROM.get(ADDR_OFFSET, angleOffset);
    Serial.println(F("eeprom loaded"));
  } else {
    Serial.println(F("no eeprom data - using defaults"));
  }
}

// ──────────────────────────────────────────
//  CALIBRATION
// ──────────────────────────────────────────
void calibrateIMU() {
  calibrating = true;
  botActive   = false;

  btPrint("── calibration started ──");
  btPrint("keep bot upright and still");
  btPrint("sampling for 10 seconds...");

  float angleSum          = 0.0;
  int   samples           = 0;
  unsigned long startTime = millis();
  int lastCountdown       = 11;

  while (millis() - startTime < 10000) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();

    float a = atan2((float)ay, (float)az) * 180.0 / PI;
    angleSum += a;
    samples++;

    int remaining = 10 - (int)((millis() - startTime) / 1000);
    if (remaining != lastCountdown) {
      lastCountdown = remaining;
      btPrint(String(remaining) + " sec remaining...");
    }

    delay(10);
  }

  angleOffset  = angleSum / samples;
  currentAngle = 0.0;

  saveToEEPROM();

  btPrint("── calibration done! ────");
  btPrint("offset   = " + String(angleOffset) + " deg");
  btPrint("samples  = " + String(samples));
  btPrint("send 'start' to run bot");

  calibrating = false;
}

// ──────────────────────────────────────────
//  COMMAND HANDLER
// ──────────────────────────────────────────
void handleBluetooth(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  // Echo back exactly what was received — helps debug corruption
  btPrint("rx: " + cmd);

  String cmdUpper = cmd;
  cmdUpper.toUpperCase();
  cmdUpper.trim();

  // ── Full-word commands ─────────────────
  if (cmdUpper == "SHOW") {
    btPrint("──── current pid ────");
    btPrint("p        = " + String(Kp));
    btPrint("i        = " + String(Ki));
    btPrint("d        = " + String(Kd));
    btPrint("setpoint = " + String(setpoint));
    btPrint("offset   = " + String(angleOffset));
    btPrint("bot      = " + String(botActive ? "running" : "stopped"));
    btPrint("fallen   = " + String(fallen ? "yes" : "no"));
    btPrint("─────────────────────");
    return;
  }

  if (cmdUpper == "START") {
    botActive = true;
    integral  = 0;
    prevError = 0;
    btPrint("bot started");
    return;
  }

  if (cmdUpper == "STOP") {
    botActive = false;
    integral  = 0;
    prevError = 0;
    btPrint("bot stopped");
    return;
  }

  if (cmdUpper == "CAL") {
    calibrateIMU();
    return;
  }

  // ── Single-char + value commands ───────
  // Strictly validate: first char must be p/i/d/s
  // second char must be digit or minus sign
  char key   = toupper(cmd.charAt(0));
  String val = cmd.substring(1);
  val.trim();

  // Reject if value is empty or doesn't look like a number
  if (val.length() == 0) {
    btPrint("ignored - no value");
    return;
  }

  // Validate first char of value must be digit, minus, or dot
  char firstValChar = val.charAt(0);
  if (!isDigit(firstValChar) && firstValChar != '-' && firstValChar != '.') {
    btPrint("ignored - bad value: " + val);
    return;
  }

  // Validate key must be one of p i d s
  if (key != 'P' && key != 'I' && key != 'D' && key != 'S') {
    btPrint("ignored - bad key: " + String(key));
    return;
  }

  float parsedVal = val.toFloat();

  if (key == 'P') {
    Kp = parsedVal;
    saveToEEPROM();
    btPrint("p is set to " + String(Kp));

  } else if (key == 'I') {
    Ki = parsedVal;
    integral = 0;
    saveToEEPROM();
    btPrint("i is set to " + String(Ki));

  } else if (key == 'D') {
    Kd = parsedVal;
    saveToEEPROM();
    btPrint("d is set to " + String(Kd));

  } else if (key == 'S') {
    setpoint = parsedVal;
    saveToEEPROM();
    btPrint("setpoint is set to " + String(setpoint));
  }
}

// ──────────────────────────────────────────
//  STEPPER PULSE
// ──────────────────────────────────────────
void stepBoth(int pulseDelay) {
  digitalWrite(LEFT_STEP_PIN,  HIGH);
  digitalWrite(RIGHT_STEP_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(LEFT_STEP_PIN,  LOW);
  digitalWrite(RIGHT_STEP_PIN, LOW);
  delayMicroseconds(pulseDelay);
}

// ──────────────────────────────────────────
//  SETUP
// ──────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  Wire.begin();

  pinMode(LEFT_STEP_PIN,  OUTPUT);
  pinMode(LEFT_DIR_PIN,   OUTPUT);
  pinMode(RIGHT_STEP_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN,  OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);  // LOW = enabled on A4988/DRV8825

  // Wake up MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Gyro range ±250 °/s
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  loadFromEEPROM();
  prevTime = micros();

  btPrint("self-balancing bot ready");
  btPrint("cmds: p18 i0.5 d1.2 s0 start stop show cal");
  btPrint("fall cutoff at +/- " + String(FALL_ANGLE) + " deg");
}

// ──────────────────────────────────────────
//  LOOP
// ──────────────────────────────────────────
void loop() {

  // ── 1. Read Bluetooth with timeout flush ──
  while (BT.available()) {
    char c = (char)BT.read();
    lastByteTime = millis();

    if (c == '\n' || c == '\r') {
      btBuffer.trim();
      if (btBuffer.length() > 0) {
        handleBluetooth(btBuffer);
        btBuffer = "";
      }
    } else {
      if (btBuffer.length() < 32) {
        btBuffer += c;
      }
    }
  }

  // Flush if \n was dropped — fire after 200ms silence
  if (btBuffer.length() > 0 && (millis() - lastByteTime > CMD_TIMEOUT)) {
    btBuffer.trim();
    if (btBuffer.length() > 0) {
      handleBluetooth(btBuffer);
      btBuffer = "";
    }
  }

  // ── 2. Read USB Serial Monitor ─────────
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      btBuffer.trim();
      if (btBuffer.length() > 0) {
        handleBluetooth(btBuffer);
        btBuffer = "";
      }
    } else {
      btBuffer += c;
    }
  }

  // ── 3. Read MPU6050 ────────────────────
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 8, true);

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();
  GyX = Wire.read() << 8 | Wire.read();

  // ── 4. Complementary filter ────────────
  unsigned long now = micros();
  float dt = (now - prevTime) / 1000000.0;
  prevTime = now;

  accAngle     = atan2((float)AcY, (float)AcZ) * 180.0 / PI - angleOffset;
  gyroRate     = (float)GyX / 131.0;
  currentAngle = alpha * (currentAngle + gyroRate * dt) + (1.0 - alpha) * accAngle;

  // ── 4b. Fall detection ─────────────────
  // NEW: check current angle against the fall threshold every loop,
  // independent 
  of botActive/calibrating. This is a hard safety gate -
  // it sits above the normal PID logic so a fallen bot's motors stay
  // off even if "start" was sent, and it auto-clears once picked back
  // up past the threshold (no command needed to resume).
  if (!fallen && abs(currentAngle - setpoint) > FALL_ANGLE) {
    fallen    = true;
    integral  = 0;
    prevError = 0;
    btPrint("bot fallen - motors stopped (angle past " + String(FALL_ANGLE) + " deg)");
  } else if (fallen && abs(currentAngle - setpoint) <= FALL_ANGLE) {
    fallen    = false;
    integral  = 0;
    prevError = 0;
    btPrint("bot upright again - resuming balance");
  }

  if (fallen) {
    return; // motors stay off, no PID runs, until lifted back within range
  }

  // ── 5. Skip if stopped or calibrating ──
  if (!botActive || calibrating) return;

  error = currentAngle - setpoint;

  if (abs(error) < deadBand) {
    integral  = 0;
    prevError = 0;
    return;
  }
  if (!fallen && abs(currentAngle - setpoint) > FALL_ANGLE) {
    fallen    = true;
    integral  = 0;
    prevError = 0;
    digitalWrite(EN_PIN, HIGH);  // ← disable driver on fall
    btPrint("bot fallen - motors stopped (angle past " + String(FALL_ANGLE) + " deg)");

} else if (fallen && abs(currentAngle - setpoint) <= FALL_ANGLE) {
    fallen    = false;
    integral  = 0;
    prevError = 0;
    digitalWrite(EN_PIN, LOW);   // ← re-enable driver when upright
    btPrint("bot upright again - resuming balance");
}

  integral += error * dt;
  integral  = constrain(integral, -300.0, 300.0);

  derivative = (error - prevError) / dt;
  prevError  = error;

  pidOutput = Kp * error + Ki * integral + Kd * derivative;

  // ── 6. PID output → pulse delay ── (1/16 microstepping)
  // More responsive speed mapping
  int pulseDelay;
  float absPID = abs(pidOutput);

  if (absPID < 1.0) {
      return; // too small to bother, inside deadband effectively
  }

  pulseDelay = (int)(5000.0 / absPID);  // inverse: bigger error = faster steps
  pulseDelay = constrain(pulseDelay, 100, 600);

  // ── 7. Direction ───────────────────────
  if (error > 0) {
    digitalWrite(LEFT_DIR_PIN,  HIGH);
    digitalWrite(RIGHT_DIR_PIN, LOW);
  } else {
    digitalWrite(LEFT_DIR_PIN,  LOW);
    digitalWrite(RIGHT_DIR_PIN, HIGH);
  }

  stepBoth(pulseDelay);
}
