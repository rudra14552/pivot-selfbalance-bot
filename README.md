# Pivot - Self-Balancing Robot with Gesture Controller

Pivot is a two-wheeled, stepper-driven self-balancing robot built around an Arduino Nano, an MPU6050 IMU, and a complementary-filter + PID control loop. It is tuned live over Bluetooth (HC-05) and can be driven two ways: from a phone terminal, or with the included **gesture controller**, a handheld ESP32 + MPU6050 remote that you steer by rotating your hand. Hold the button, tilt your wrist forward or back to drive, and twist it to spin the robot. The further you rotate, the faster it goes.

---

## Demo

https://github.com/user-attachments/assets/ef759cfe-6b5a-48ea-b549-abe839f0f9cb

---

## Features

**Robot**

- **Timer-driven stepping.** Timer1 ISRs generate the STEP pulses for each wheel independently, so Bluetooth/serial handling never stalls the motors, and the wheels can run at different speeds for turning.
- **200 Hz control loop** with a complementary filter (accelerometer + gyro, `ALPHA = 0.98`).
- **Balance PID** that outputs wheel *acceleration* (steps/s^2), integrated into wheel speed.
- **Speed PI while driving.** The remote sets a target wheel speed; a PI loop turns the speed error into a lean angle, so forward and backward reach the same speed even if the balance point is slightly off.
- **Position hold PID** (`x`, `y`, `z` gains) so the bot settles and stays put instead of drifting.
- **Brake-then-hold.** When you release the remote, the bot first slows down, and the spot where it actually stops becomes its new home position.
- **Proportional driving.** Forward/back and turn speeds scale with how far you tilt the remote.
- **Learned balance point** (`restTilt`). While standing still the bot learns its real centre-of-gravity offset, so forward and backward behave the same.
- **Microstepping control.** 1/1 to 1/32 via the DRV8825 M0/M1/M2 pins (default 1/8).
- **EEPROM persistence.** Gains, setpoint, calibration offset, position-hold gains, and drive and turn speeds survive power cycles (send `save`).
- **Guided IMU calibration** (`cal`) with a 10 s countdown.
- **Layered safety.** See [Safety](#safety).

**Gesture controller (remote)**

- Hand-motion control: rotate the remote to drive, no joystick or buttons for direction
- Deadman button: the bot only moves while the button is held.
- Rotation-relative control (no fixed mounting orientation, no gyro drift between presses).
- Strict one-axis-at-a-time gesture lock so forward never leaks into turning.
- Auto-reconnect to the bot, with an LED showing the connection state.

---

## Bill of Materials

### Robot

| Component                            | Qty |
| ------------------------------------ | --- |
| M4 threaded rod, 20 cm               | 4   |
| M4 nuts                              | 24  |
| NEMA 17 stepper motor                | 2   |
| NEMA 17 stepper motor L-bracket      | 2   |
| Wheel, 10 cm diameter                | 2   |
| Thick PCB (base + structural plates) | -   |
| Arduino Nano                         | 1   |
| HC-05 Bluetooth module               | 1   |
| DRV8825 stepper driver               | 2   |
| 35 V 100 µF capacitor                | 2   |
| MPU6050 (accelerometer/gyroscope)    | 1   |
| Buck converter (motor/logic power)   | 1   |
| XT60 female connector                | 1   |
| 3S LiPo battery                      | 1   |
| Male & female header pins            | -   |
| Stepper motor connecting wire        | 2   |

The four threaded rods and M4 nuts form the vertical frame that sandwiches the PCB base, motor mounts, and top plate together: a simple, adjustable stack rather than a printed chassis.

### Gesture controller (remote)

| Component                                     | Qty |
| --------------------------------------------- | --- |
| ESP32 DevKit (**classic ESP32-WROOM**)        | 1   |
| MPU6050 (accelerometer/gyroscope)             | 1   |
| Push button (deadman)                         | 1   |
| Battery / power source for the ESP32          | 1   |

> **Warning:** The remote needs **Bluetooth Classic**. ESP32-S2, S3 and C3 boards do not have it and will not work.

---

## Wiring / Pinout

### Robot (Arduino Nano)

| Signal                               | Nano pin                          |
| ------------------------------------ | --------------------------------- |
| Left stepper STEP                    | D5                                |
| Right stepper STEP                   | D6                                |
| Left stepper DIR                     | D7                                |
| Right stepper DIR                    | D8                                |
| DRV8825 EN (both drivers)            | D4 (LOW = enabled)                |
| DRV8825 M0 (both drivers)            | D2                                |
| DRV8825 M1 (both drivers)            | D3                                |
| DRV8825 M2 (both drivers)            | D9                                |
| HC-05 TXD -> Nano RX (SoftwareSerial)| D10                               |
| Nano TX (SoftwareSerial) -> HC-05 RXD| D11 (use a 5 V to 3.3 V divider)  |
| MPU6050                              | I2C (A4 = SDA, A5 = SCL)          |

The STEP pins D5 and D6 are written directly through `PORTD` inside the ISRs, so they must stay on those pins unless you also change the port code.

The right motor is mounted mirrored, so the firmware inverts its DIR signal. If a wheel spins the wrong way, swap that motor's coil wires or flip the logic in `setDirL()` / `setDirR()`.

**Power:** Battery -> XT60 -> buck converter -> logic rail (Nano, HC-05, MPU6050) and motor rail (DRV8825 `VMOT`). A 100 µF / 35 V capacitor sits across each DRV8825's motor supply to absorb inductive spikes. The drivers are held disabled (`EN` HIGH) during start-up and are enabled at the end of `setup()`.

> **Warning:** Set the DRV8825 current limit (Vref trimpot) to match your NEMA 17's rated current before first power-up.

### Gesture controller (ESP32)

| Signal                       | ESP32 pin                    |
| ---------------------------- | ---------------------------- |
| Button (between pin and GND) | GPIO4 (internal pull-up)     |
| Status LED                   | GPIO2 (onboard LED)          |
| MPU6050 SDA                  | GPIO21                       |
| MPU6050 SCL                  | GPIO22                       |

---

## Getting started

### 1. Flash the robot

1. Open `balance_bot2v3/balance_bot2v3.ino` in the Arduino IDE.
2. Board: **Arduino Nano** (ATmega328P). All libraries used are built in (`Wire`, `EEPROM`, `SoftwareSerial`).
3. Upload.

### 2. Name the HC-05

The remote looks for a Bluetooth device called **`Pivot`** with PIN **`1234`** (some modules use `0000`).

Put the HC-05 in AT mode and set:

```
AT+NAME=Pivot
AT+ROLE=0        (slave)
AT+PSWD=1234
```

Baud rate stays at **9600**.

### 3. Calibrate and run

1. Connect a phone Bluetooth terminal to `Pivot` at 9600 baud, or use the USB serial monitor.
2. Hold the bot upright at its balance point and still. Send `cal`. Wait for the 10 s countdown to finish. Calibration saves to EEPROM and leaves the bot **stopped**.
3. Send `start`.
4. Check the gains with `show`. Tune live (see [Tuning](#tuning)), then send `save`.

> **Power-up behaviour:** After a normal power-up the bot measures the gyro bias (about 1 s, keep it still), then enables the motors and starts balancing on its own. You do not need to send `start` after a reset, only after `cal` or `stop`. Hold the bot upright when you switch it on.

> Gains saved in EEPROM override the defaults in the sketch. Changes are **not** written automatically. Send `save` once you are happy.

---

## Gesture Controller (Handheld Remote)

A small ESP32 + MPU6050 gadget that turns hand rotation into drive commands. You hold a button and tilt your hand; the bot follows.

### How it works

| State                | What the bot gets                                                            |
| -------------------- | ---------------------------------------------------------------------------- |
| Button **not** held  | `S`: the bot stands still and only balances. Remote orientation is ignored.  |
| Button **pressed**   | Your current hand position becomes "normal" (all angles reset to 0).         |
| Hand rotated         | Rotation *relative to that moment* drives the bot.                           |
| Button **released**  | `S` is sent three times for reliability. The bot brakes, then holds position. |

Angles come from integrating the gyro **only while the button is held**, so nothing drifts between presses and it does not matter how the remote is mounted. While the button is up and the remote is still, the gyro bias is slowly refined in the background.

### Gestures

| Rotation axis | Result                                                |
| ------------- | ----------------------------------------------------- |
| Z             | Forward (+) / backward (-)                            |
| Y             | Spin in place clockwise (+) / counter-clockwise (-)   |
| X             | Not used                                              |

If your remote is mounted differently, change `AX_FWD`, `AX_SPIN`, `SIGN_FWD` and `SIGN_SPIN` at the top of the sketch.

### Proportional speed

Nothing is sent until you rotate past the threshold. After that, speed rises linearly with the angle:

| Setting                | Value     | Meaning                                              |
| ---------------------- | --------- | ---------------------------------------------------- |
| `FWD_ON` / `FWD_OFF`   | 15° / 10° | Angle to start / stop forward-back (hysteresis)      |
| `FWD_FULL`             | 40°       | Angle for 100 % forward/back                         |
| `MIN_FWD_PCT`          | 60 %      | Speed sent the moment the threshold is crossed       |
| `SPIN_ON` / `SPIN_OFF` | 15° / 10° | Same, for rotation                                   |
| `SPIN_FULL`            | 40°       | Angle for 100 % rotation                             |
| `MIN_SPIN_PCT`         | 40 %      | Rotation speed at the threshold                      |

Values are rounded to the nearest 5 % to keep Bluetooth traffic low. On the bot, 100 % forward equals the `v` speed and 100 % rotation equals `t` x 1.5 when spinning in place (or `t` x 0.6 when combined with forward motion).

### Deadzones (no accidental turning)

- `GYRO_DEADBAND` (1.2 °/s): tiny rotation rates and hand tremor are ignored.
- **Axis lock:** a gesture is only accepted when one axis crosses its threshold **and** is at least `AXIS_DOMINANCE` (2.5x) larger than the other. Ambiguous gestures send nothing. Once accepted, the other axis is locked out until you return to neutral.
- `COMBINE_MOVE_AND_TURN = false`: one axis at a time (strict mode).
- `REL_CLAMP` (60°): the integrated angle is clamped so you can get back to neutral quickly.

### Link behaviour

- Sends at most 10 updates/s (`SEND_MIN_MS = 100`) and repeats the active command every 250 ms as a keep-alive. A stop is sent immediately, without waiting for the rate limit.
- If the bot stops hearing from the remote for 600 ms (`DRIVE_TIMEOUT_MS` on the bot), it brakes and stops by itself.
- The remote connects as a Bluetooth master named `BotRemote`. While the button is up, it retries the connection every 4 s. The LED **blinks** when disconnected, is **off** when connected and idle, and is **on** while the button is held.
- `PROPORTIONAL = false` switches to the older fixed-speed letter commands (`F B Q E ...`).

### Flashing the remote

1. Install the **ESP32** board package. Board: **ESP32 Dev Module** (classic ESP32).
2. Open `bot_remote2v2/bot_remote2v2.ino`. It uses `Wire` and `BluetoothSerial`, which come with the ESP32 core.
3. Upload. Keep the remote **still** at power-up while it measures the gyro bias.
4. Open the serial monitor at **115200** baud for live debug output (`X`, `Y`, `Z` angles, `F%`, `T%`, last command). Set `DEBUG = false` when done.

---

## Bluetooth Command Reference

Connect at **9600 baud** (the same commands work over USB serial). Commands are case-insensitive and are ended by a newline; if no newline arrives, the bot flushes the buffer after 60 ms. The input buffer holds 23 characters.

### Control

| Command | Effect                                                                                  |
| ------- | --------------------------------------------------------------------------------------- |
| `start` | Enable balancing and zero the held position                                             |
| `stop`  | Disable the bot (motors off)                                                            |
| `cal`   | Run the 10 s IMU calibration (saves to EEPROM, leaves the bot stopped)                  |
| `show`  | Print all gains, speeds, setpoint, offset, rest tilt, microstep, angle, position, run and fall state |
| `save`  | Write current settings to EEPROM                                                        |
| `zero`  | Zero the held position                                                                  |
| `m<n>`  | Microstepping: `m1 m2 m4 m8 m16 m32` (only while stopped)                               |

### Tuning (all take a number, e.g. `p280`)

| Key | Parameter                                                         | Default | Limit            |
| --- | ----------------------------------------------------------------- | ------- | ---------------- |
| `p` | Balance Kp                                                        | 280     |                  |
| `i` | Balance Ki                                                        | 0       |                  |
| `d` | Balance Kd                                                        | 15      |                  |
| `s` | Setpoint angle                                                    | 0       | +/-5°            |
| `x` | Position hold P (lean per step of error)                          | 0.0005  |                  |
| `y` | Position hold I                                                   | 0       |                  |
| `z` | Position hold D, damping on wheel speed (settles quickly, steadily) | 0.001 |                  |
| `v` | Top forward/back speed in steps/s                                 | 1200    | 3200 (0.8 x max) |
| `t` | Top rotation speed in steps/s                                     | 800     | 2400 (0.6 x max) |

Tuning commands take effect immediately but are only kept across power cycles after `save`.

### Drive commands

| Command            | Effect                                                         |
| ------------------ | -------------------------------------------------------------- |
| `F` / `B`          | Forward / backward                                             |
| `L` / `R`          | Turn left / right                                              |
| `G` `I` `H` `J`    | Forward-left, forward-right, back-left, back-right             |
| `Q` / `E`          | Spin in place counter-clockwise / clockwise (1.5x turn speed)  |
| `S`                | Stop (brake, then hold)                                        |
| `C<fwd>,<turn>`    | Proportional drive, both -100...100, e.g. `C50,-30`            |

The remote uses `C<fwd>,<turn>`. Malformed or out-of-range packets are dropped, never half-applied. Drive commands are not echoed back, so Bluetooth traffic does not disturb the stepper timing.

---

## Safety

| Layer                | Behaviour                                                                                          |
| -------------------- | -------------------------------------------------------------------------------------------------- |
| **Hard fall cutoff** | Tilt beyond +/-15° from the setpoint: motors off. Balancing resumes automatically once the bot is within 5° and still (under 25 °/s) for 0.5 s. Position is re-zeroed on recovery. |
| **Soft limit**       | Beyond 12° the remote and position hold have **zero** authority; the bot only balances. Authority fades in linearly from 7° down to 0 at 12°, and also fades with fast gyro wobble (40 to 90 °/s). |
| **Lean cap**         | Commanded lean (drive + hold) is capped at 6°, so commands alone can never push the bot toward the limit. |
| **Remote timeout**   | No remote packet for 600 ms: brake and stop.                                                       |
| **Packet validation**| Strict parser; corrupted packets are ignored.                                                      |
| **IMU faults**       | A failed read stops the wheels; 10 bad reads in a row (50 ms) puts the bot in the fallen state with motors off. A dead accelerometer reading falls back to gyro-only for that step. |

There is still no physical kill switch. Keep the battery connector accessible.

---

## Control Loop Overview

1. Read accelerometer Y/Z and gyro X from the MPU6050 (200 Hz).
2. Compute the tilt with a complementary filter (`ALPHA = 0.98`).
3. Check the fall limit, then compute **command authority** (fades with tilt and gyro rate).
4. **Speed PI** (while driving): wheel-speed error -> lean command (slew-limited to 6°/s). The integral term absorbs any balance-point offset.
5. **Brake then hold** (after a stop): the home position keeps following the bot until it slows below 250 steps/s (or 3 s pass), then it locks at that spot.
6. **Position hold** (when still): small deadband (60 steps) around home, filtered lean capped at 3°, plus wheel-speed damping. If authority drops below 0.5 (hard wobbling), the home spot is forgotten and the bot just balances.
7. **Learned balance point:** while standing still and upright, `restTilt` slowly tracks the tilt at which the bot actually stands without accelerating (about 2.5 s time constant, limited to +/-4°).
8. **Balance PID:** `tilt = angle - setpoint - lean - restTilt`, then `accel = Kp*tilt + Ki*integral + Kd*gyroRate`.
9. Integrate `accel` into wheel speed. The turn rate is ramped (3000 steps/s^2), scaled by authority, then added to the left wheel and subtracted from the right.
10. Timer1 ISRs turn those wheel speeds into STEP pulses.

---

## Tuning

Starting point: `p280 i0 d15 x0.0005 z0.001`.

1. Calibrate with `cal` while the bot is held at its true balance point.
2. Raise `p` until the bot reacts quickly, then add `d` to calm the oscillation.
3. If it creeps one way when standing still, adjust `s` in small steps (+/-0.2°).
4. Raise `z` for a faster, steadier stop; raise `x` for a firmer hold.
5. Set `v` and `t` to taste. Send `save`.

Check `show` after a few seconds of standing still. If `rest tilt` is more than about 2° either way, the centre of gravity is off the axle. Move the battery or weight, or re-run `cal`.

---

## Troubleshooting

| Symptom                                   | What to check                                                                 |
| ----------------------------------------- | ----------------------------------------------------------------------------- |
| Remote LED blinks forever                 | HC-05 name must be `Pivot`, PIN `1234`/`0000`, and the HC-05 must be in slave mode and powered. |
| Forward and backward feel different       | `rest tilt` in `show`; recalibrate or rebalance the weight.                   |
| Bot drives backward when told forward     | Set `DRIVE_SIGN` to `-1` in the bot sketch.                                   |
| Left and right swapped                    | Set `TURN_SIGN` to `1`.                                                       |
| Remote does nothing                       | Hold the button; rotate past 15°; watch the remote's serial monitor and the bot's USB serial (`DEBUG_REMOTE`). |
| Bot jitters or hesitates while driving    | `DEBUG_REMOTE` is `true` by default and prints every packet over USB serial. Set it to `false` once the remote works. |
| Forward leaks into turning                | Raise `AXIS_DOMINANCE` on the remote (e.g. 3.0).                              |
| Settings change after a reboot            | You forgot to send `save`; EEPROM values override the sketch defaults.        |
| Bot does not move after `cal`             | Calibration leaves the bot stopped. Send `start`.                             |

---

## Libraries Used

**Robot:** `Wire`, `EEPROM`, `SoftwareSerial`, `avr/io.h`, `avr/interrupt.h`

**Gesture controller:** `Wire`, `BluetoothSerial` (ESP32 Arduino core)

---

## Known Limitations / Ideas

- Remote uses a single deadman button; a physical kill switch on the bot would be safer.
- Position hold is P + D by default (`y = 0`, no integral). Wheel slip is not detected.
- The learned balance point (`restTilt`) is not saved to EEPROM and is relearned each power-up.
- The bot starts balancing automatically at power-up; there is no "armed" state.
- Bluetooth Classic (HC-05) range and latency are limited; a BLE or ESP-NOW link could be faster.

---

## License

MIT. Free to use, modify, and share. Attribution appreciated.
