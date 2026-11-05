# GSFTI — GodSpeed Flight Telecommunication Interpreter

Custom flight computer for **GodSpeed FR-1**, a model rocket with active stabilisation. GSFTI senses the rocket's attitude and altitude, drives a two-axis gimbal for thrust vector control (TVC), logs flight data to an SD card, and triggers parachute recovery.

This repository covers the flight computer only — PCB, firmware, sensors, and control algorithm. Airframe, motor, gimbal mechanics, and recovery hardware are documented in the full project report.

## What it does

- Detects launch from accelerometer data and starts logging and stabilisation
- Estimates pitch and roll with a Kalman filter (with a complementary filter alongside it)
- Runs a PID controller and commands two TVC servos to correct the rocket's attitude
- Detects motor burnout and disables the stabilisation controller
- Detects apogee from barometric altitude and fires the recovery pyro channel
- Aborts and deploys the parachute if the attitude exceeds 45° after launch
- Logs the full flight to `data.csv` and marks events such as apogee, deployment, and landing

## Hardware

| Component | Part | Role |
| --- | --- | --- |
| Processor | Arduino Nano Rev3 (ATmega328P) | Main controller, 5 V, 32 KB flash |
| IMU | MPU6050 | 3-axis gyroscope + 3-axis accelerometer |
| Barometric sensor | BMP180 | Pressure, temperature, and altitude |
| Storage | SD card module | Flight data logging to `data.csv` |
| Voltage regulator | LM7805 | Board power regulation |
| Pyro driver | N-channel MOSFET | Fires the parachute ejection charge |
| Indicator | 5 V buzzer | Power-on and event tones |
| Indicator | Common-cathode RGB LED | Flight-state indication |
| Connectors | Terminal blocks | Sensor, servo, and pyro connections |
| TVC actuators | 2× SG90 servos | Gimbal pitch and yaw, approx. 1.5–2.5 kg·cm torque |

The MPU6050 was chosen for combined gyro and accelerometer sensing in a single chip. The BMP180 provides the pressure and temperature readings used for altitude and apogee detection. The Arduino Nano was chosen for its compact size and moderate processing power in a space-limited avionics bay.

## PCB

Designed in Autodesk Eagle as a custom two-layer board for GSFTI.

| Property | Value |
| --- | --- |
| Layers | 2 |
| Base material | FR-1 |
| Board size | 100 × 60 |
| Thickness | 1.6 |
| Outer copper weight | 1 oz |

Layer 1 is connected to ground and layer 2 is connected to the 5 V signal. The board material is also listed in the report as FR4, standard Tg 130–140 °C — confirm the as-fabricated material before reordering.

## Pin map

Taken from the flight firmware in the project appendix.

| Signal | Pin | Notes |
| --- | ---: | --- |
| Servo X (pitch) | D5 | TVC gimbal |
| Servo Y (yaw) | D6 | TVC gimbal |
| Buzzer | D3 | Also constructed on pin 13 in firmware |
| SD card chip select | D4 | SPI SD module |
| Red LED | A0 | RGB LED, common cathode |
| Green LED | A1 | RGB LED, common cathode |
| Blue LED | A2 | RGB LED, common cathode |
| Pyro MOSFET | A3 | Parachute ejection output |
| I2C bus | A4 / A5 | MPU6050 and barometric sensor, 400 kHz |

Servo centre is 90° on both axes, with a servo gear ratio of 0.8 applied in the PID output calculation.

## Control system

Attitude is read from the MPU6050, filtered, and fed to a PID controller that moves the gimbal servos. The gimbal vectors the motor thrust in pitch and yaw, so a correction in thrust direction produces the torque that brings the rocket back toward vertical.

The controller was tuned before flight in a MATLAB Simulink model:

- A 3 degree-of-freedom (3DOF) block receives the motor thrust curve and returns the rocket's orientation.
- Inputs were rocket mass, centre of gravity, moment arm, and mass moment of inertia, measured with a pre-flight tuning setup.
- The tuned gains were then carried into the C++ flight firmware.

| Simulink input | Value |
| --- | ---: |
| Rocket mass | 1.235 kg |
| Cycle time | 1.4 s |
| Centre of gravity | 45.5 cm |
| Length from CG to rope COM | 35.5 cm |
| Rope length, table to vehicle | 63 cm |
| Mass moment of inertia | 0.12032 kg·m² |

### PID gains — two sets in the source report

The report gives two different gain sets. Both are listed here so the as-flown set can be confirmed before this README is treated as final.

| Source | P / Kp | I / Ki | D / Kd |
| --- | ---: | ---: | ---: |
| Simulink tuning table (Table 4) | 0.65 | 0.7 | 0.2 |
| Flight firmware in appendix | 0.9 | 0.3 | 0.09 |

Sensor configuration in firmware: I2C at 400 kHz, sample rate 1000 Hz, gyroscope full-scale range ±500°/s, accelerometer full-scale range ±2 g. Attitude is calculated with a Kalman filter; a complementary filter (0.93 gyro / 0.07 accelerometer) is calculated in parallel.

## Flight algorithm

The firmware runs as a gated sequence, not as free-running control from power-on:

1. **Idle** — Green LED pulses about once per second with a buzzer tone. No stabilisation or deployment.
2. **Launch detection** — If acceleration on the Y axis exceeds 2, the state changes to launched: logging and the stabilisation system turn on and the red/green LEDs indicate the change.
3. **Stabilisation** — While the motor is burning, the PID loop runs continuously: read IMU, update Kalman angles, compute PID on the error from the desired angle, and write the corrected positions to both servos.
4. **Motor burnout** — If Y acceleration falls below 0, the servos and stability controller are switched off, the buzzer sounds, and apogee detection is enabled.
5. **Apogee detection** — Altitude from the barometric sensor is compared about once per second. If altitude is falling across successive checks (firmware tests drops beyond −1, −2, and −3), the board logs apogee temperature and altitude, writes `Deploy Parachutes`, and fires the pyro channel.
6. **Launch abort** — At any point in the stabilisation phase, if either filtered angle exceeds the 45° limit, the board logs `Launch abort`, fires the pyro channel, and exits.
7. **Landing detection** — After apogee, if gyro and accelerometer readings stay near zero and unchanged for about 5 seconds, the board logs `Rocket Landed` and exits.

The pyro output only fires in the firmware when current altitude is above 2 (metres above ground altitude), as a guard against firing on the pad.

## Data logging

Flight data is written to `data.csv` on the SD card. Ground altitude is written first at startup, then a header row, then one row per loop iteration:

```text
Timer : AX : AY : AZ : GX : GY : GZ : kalAngleX : kalAngleY : Temperature : Pressure : Altitude
```

Event lines written into the same file include `Deploy Parachutes` with apogee temperature and altitude, `Launch abort`, and `Rocket Landed`.

## Firmware

- **Language:** C++ (Arduino)
- **Board:** Arduino Nano / ATmega328P
- **Serial:** 9600 baud for debug output

### Libraries

```cpp
#include <I2Cdev.h>
#include <Servo.h>
#include <Buzzer.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <BMP085.h>
#include <Kalman.h>
```

Note the naming difference in the source: the hardware list and sensor section name the **BMP180**, while the appendix firmware includes **BMP085.h** and uses a `BMP085` barometer object. Confirm which sensor and driver the flown board used and make the code and this README match.

### Power-on sequence

On reset, the firmware in the appendix does the following, in order:

1. Attaches both servos and centres them at 90°.
2. Sounds the startup buzzer tones and initialises the SD card and `data.csv`.
3. Initialises the MPU6050 over I2C, checks its `WHO_AM_I` response (0x68), and sets the starting Kalman and gyro angles from the first accelerometer reading.
4. Initialises the barometric sensor, reads temperature and pressure, and records ground altitude to the SD card.
5. Runs the LED sequence (red, then green) and a servo check: 90° → 45° / 135° → 90°.
6. Writes the `data.csv` header and enters the idle / launch-wait state.

If the SD card fails to initialise, the firmware prints `initialization failed!` over serial.

## Getting started

1. Assemble or fabricate the PCB to the specs above and fit the components in the hardware table.
2. Install the libraries listed above in your Arduino environment.
3. Flash the flight firmware to the Arduino Nano.
4. Insert a formatted SD card — the firmware expects to create and append to `data.csv`.
5. Power the board on a level surface and let it complete the power-on sequence above, including ground-altitude capture, before moving it to the rocket.
6. Check the servo sweep and LED / buzzer sequence. Do not connect a live ejection charge during bench checks.

## Project context

GSFTI was developed for GodSpeed FR-1 as part of *Design and Development of a Model Rocket with Active Stabilization*, B.Tech. in Aerospace Engineering, M. S. Ramaiah University of Applied Sciences, Bengaluru, May 2023.

- **Team:** Harsha Vardhana S, Mohammed Zaidur Rahman, Praneeth B V
- **Supervisor:** Dr. Jayahar Sivasubramanian

## Before publishing this repo

- Confirm the barometric sensor actually flown — BMP180 (hardware list) or BMP085 (firmware) — and align code, BOM, and README.
- Confirm the as-flown PID gains — Simulink table or appendix firmware values.
- Confirm the PCB base material — FR-1 (PCB table) or FR4 (note below the same table).
- Add the actual firmware source, Eagle / PCB files, and a licence. None of those files were included with the source report used for this README.
