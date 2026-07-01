# Pivot — Self-Balancing Robot

Pivot is a two-wheeled, stepper-driven self-balancing robot built around an Arduino Nano, an MPU6050 IMU, and a complementary-filter + PID control loop. It's controlled and tuned live over Bluetooth (HC-05), with PID gains, setpoint, and IMU calibration offset persisted to EEPROM so they survive a power cycle.

> 🚧 **Status: Work in progress.** This is an ongoing build — hardware, wiring, and code are still being iterated on. Expect breaking changes, untuned PID values, and incomplete sections until a stable release is tagged. Contributions, issues, and suggestions are welcome.

---

## ✨ Features

- **Complementary filter** fusing accelerometer + gyro data for a stable tilt angle estimate
- **PID balance loop** with live-tunable `P`, `I`, `D`, and `setpoint` over Bluetooth
- **EEPROM persistence** — PID gains, setpoint, and calibration offset are saved and reloaded on boot
- **Guided IMU calibration routine** (`CAL`) with a 10-second countdown, run with the bot held upright and still
- **Fall detection safety cutoff** — if the tilt angle exceeds ±30° from the setpoint, motors are stopped immediately and the PID loop is skipped entirely. The bot automatically resumes balancing once it's picked back up within range, no command required
- **Dual command input** — accepts commands over both the HC-05 Bluetooth link and the USB serial monitor, with the same buffer/timeout logic on each
- **Robust serial command parser** with input validation (rejects malformed keys/values) and a 200 ms silence-based flush in case a terminator byte is dropped

---

## 🛠️ Bill of Materials

| Component | Qty |
|---|---|
| M4 threaded rod, 20 cm | 4 |
| M4 nuts | 24 |
| NEMA 17 stepper motor | 2 |
| NEMA 17 stepper motor L-bracket | 2 |
| Wheel, 10 cm diameter | 2 |
| Thick PCB (base + structural plates) | — |
| Arduino Nano | 1 |
| HC-05 Bluetooth module | 1 |
| DRV8825 stepper driver | 2 |
| 35 V 100 µF capacitor | 2 |
| MPU6050 (accelerometer/gyroscope) | 1 |
| Buck converter (motor/logic power) | 1 |
| XT60 female connector | 1 |
| 3s Lipo Battery | 1 |
| Male & female header pins | — |
| Stepper motor connecting wire | 2 |

The four threaded rods and M4 nuts form the vertical frame that sandwiches the PCB base, motor mounts, and top plate together — a simple, adjustable stack rather than a printed chassis.

---

## 🔌 Wiring / Pinout

| Signal | Arduino Nano Pin |
|---|---|
| Left stepper STEP | D5 |
| Left stepper DIR | D7 |
| Right stepper STEP | D6 |
| Right stepper DIR | D8 |
| DRV8825 EN (both drivers) | D4 |
| HC-05 RX (SoftwareSerial TX) | D10 |
| HC-05 TX (SoftwareSerial RX) | D11 |
| MPU6050 | I2C (A4 = SDA, A5 = SCL) |

**Power:** Battery → XT60 → buck converter → logic rail (Nano, HC-05, MPU6050) and motor rail (DRV8825 `VMOT`). The 100 µF/35 V capacitors sit across each DRV8825's motor supply input to absorb inductive spikes from the steppers. `EN_PIN` is held `LOW` at boot to keep both drivers enabled.

> ⚠️ Double-check DRV8825 current-limiting (Vref) trimpots against your NEMA 17's rated current before first power-up.

---

## Bluetooth Command Reference

Connect to the HC-05 at **9600 baud** (same commands also work over the USB serial monitor). Every command gets echoed back (`rx: ...`) for debugging.

| Command | Effect |
|---|---|
| `start` | Enable the balance loop |
| `stop` | Disable the balance loop (motors idle) |
| `cal` | Run the 10-second IMU calibration routine |
| `show` | Print current `P`, `I`, `D`, setpoint, offset, run state, and fall state |
| `p<value>` | Set proportional gain, e.g. `p18` |
| `i<value>` | Set integral gain, e.g. `i0.5` (also resets the integral accumulator) |
| `d<value>` | Set derivative gain, e.g. `d1.2` |
| `s<value>` | Set target setpoint angle, e.g. `s0` |

Setting `p`, `i`, `d`, or `s` immediately writes the new value to EEPROM. Malformed commands (bad key or non-numeric value) are rejected with an `ignored - ...` message rather than silently failing.

### First-time setup

1. Power on the bot and place it upright against a wall or in a stand.
2. Connect over Bluetooth or USB serial.
3. Send `cal` and keep the bot still for the 10-second countdown.
4. Send `start` to begin balancing.
5. Use `show` to check current gains, and `p` / `i` / `d` / `s` to tune live.

---

## 🛡️ Fall Detection

Every loop iteration, the current tilt angle is compared against the setpoint. If it drifts more than **±30°**, the bot is marked as fallen:

- Motors are stopped and the PID loop is skipped entirely (not just zeroed).
- The `integral` and `prevError` terms are reset so there's no windup or derivative kick when it recovers.
- A `bot fallen - motors stopped` message is sent over Bluetooth/serial.

Once the bot is lifted back within ±30° of the setpoint, it exits the fallen state and resumes balancing automatically — no command needed.

---

## 🧠 Control Loop Overview

1. Read accelerometer + gyro from the MPU6050 over I2C.
2. Compute `accAngle` from accelerometer (`atan2`), corrected by the stored calibration `angleOffset`.
3. Fuse `accAngle` with integrated gyro rate using a complementary filter (`alpha = 0.98`) to get `currentAngle`.
4. Check fall condition; halt if outside safe range.
5. Compute PID `error`, `integral`, and `derivative` against `setpoint`.
6. Convert PID output into a stepper pulse delay (faster steps for larger error, clamped to a safe range).
7. Set direction pins and pulse both steppers together via `stepBoth()`.

---

## 📦 Libraries Used

- `Wire.h` — I2C communication with the MPU6050
- `EEPROM.h` — persisting PID gains, setpoint, and calibration offset
- `SoftwareSerial.h` — HC-05 communication on pins 10/11
- `math.h` — `atan2` for tilt angle computation

---

## 🚧 Known Limitations / Ideas for Improvement

- Stepper driving is done via manual `delayMicroseconds()` pulsing rather than a timer/interrupt-based stepper library, so Bluetooth/serial reads share loop time with motor pulses.
- No physical kill switch — falls are handled purely in software via the tilt cutoff.
- Ki/Kd default to 0; the control loop currently runs closer to P-only until tuned per-build.

---

## License

MIT — free to use, modify, and share. Attribution appreciated.

---
