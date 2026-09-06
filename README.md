# ESP32 PID Wall-Follower Robot

A small differential-drive robot built on an ESP32, a TB6612FNG dual motor driver, and
three HC-SR04 ultrasonic sensors (front / left / right). The robot hugs the left wall
at a fixed setpoint and automatically switches to a "centering" mode (balances left vs.
right distance) once a right-side wall is also detected — stopping and pivoting away
from dead ends. This repo holds four variants of that same PID wall-follower: a plain
standalone version, and three that add dual AS5600 wheel encoders plus a self-hosted
WiFi web dashboard.

There is currently no turn-decision maze-solving state machine (no left-hand-rule
logic) in any of these sketches — they are pure corridor/centering followers, not full
maze solvers.

## Hardware

- ESP32 DevKit (WROOM)
- TB6612FNG dual motor driver
- 2× DC gearmotors + wheels
- 3× HC-SR04 ultrasonic sensors (front, left, right)
- 2S Li-ion battery pack (or equivalent supply for motors + ESP32)
- **AS5600/WebDashboard sketches only:** 2× AS5600 magnetic rotary encoders (one per
  wheel/motor shaft, each with its own diametrically-magnetized magnet) for wheel
  odometry

## Sketches

| File | Purpose |
|---|---|
| `wall_follower_robot_esp32.ino` | Standalone PID wall follower — no WiFi, no encoders. Debug output goes to Serial only (115200 baud). Good starting point since it needs no extra wiring beyond the motors and three ultrasonic sensors. |
| `wall_follower_robot_AS5600_with_WebDashboard.ino` | Same PID wall-follower logic, plus dual AS5600 wheel encoders (angle / revolutions / RPM per wheel) and a self-hosted WiFi access point serving a live web dashboard. This is the earlier/simpler dashboard build: a plain dark UI, and turn/reverse maneuvers block the dashboard briefly (uses `delay()`). |
| `wall_follower_robot_AS5600__1_.ino` | Same encoder + WiFi dashboard combo, reworked with a more polished glass-panel teal-themed UI, an added "distance traveled" / "isMoving" readout derived from wheel odometry, and a non-blocking `serverDelay()` during turns so the dashboard keeps responding while the robot pivots. |
| `wall_follower_robot_WebDashBoard.ino` | Functionally identical to `wall_follower_robot_AS5600__1_.ino` (same odometry telemetry, same non-blocking turn handling) — just re-skinned with a neutral gray/dark color theme instead of teal, and `BASE_SPEED` reverted from 150 back to 200. |

> **Note:** the last three files are near-duplicates of each other (same encoder +
> dashboard logic, differing only in `BASE_SPEED`, dashboard color theme, and whether
> turns block the web server). Consider picking one as your canonical version and
> deleting the others once you've settled on a look/tuning you like.

## Which sketch to flash

1. **`wall_follower_robot_esp32.ino`** — if you just want PID wall-following/centering
   behavior with no encoders and no WiFi.
2. **`wall_follower_robot_AS5600__1_.ino`** or **`wall_follower_robot_WebDashBoard.ino`**
   — if you've wired up the two AS5600 encoders and want live web telemetry with
   non-blocking turns and odometry-based distance/moving stats (pick whichever color
   theme / `BASE_SPEED` you prefer — they're otherwise the same).
3. **`wall_follower_robot_AS5600_with_WebDashboard.ino`** — same encoder + dashboard
   feature set, but the earlier/simpler version, if you don't need the extra telemetry.

## Required Arduino libraries

- **ESP32 board support** ("esp32" by Espressif Systems) — Arduino-ESP32 core 3.x
- `Wire.h`, `WiFi.h`, `WebServer.h` — all ship with the ESP32 core, no separate
  Library Manager install needed. `Wire.h` is used for both AS5600 encoders (the two
  chips share the fixed I2C address `0x36`, so each gets its own hardware I2C
  peripheral: `Wire` and `Wire1`). `WiFi.h`/`WebServer.h` are only used by the three
  dashboard sketches (SoftAP + HTTP server).

## Wiring

Pin assignments are identical across all four sketches:

| Signal | Pin |
|---|---|
| Motor STBY | 13 |
| Motor AIN1 / AIN2 / PWMA (left motor) | 26 / 25 / 18 |
| Motor BIN1 / BIN2 / PWMB (right motor) | 27 / 14 / 19 |
| TRIG/ECHO Front | 5 / 21 |
| TRIG/ECHO Left | 4 / 22 |
| TRIG/ECHO Right | 16 / 17 |

**AS5600/WebDashboard sketches only** — encoder wiring:

| Signal | Pin |
|---|---|
| Left encoder SDA / SCL (`Wire`, I2C0) | 32 / 33 |
| Right encoder SDA / SCL (`Wire1`, I2C1) | 15 / 0 |

> ⚠️ GPIO0 is an ESP32 boot-strapping pin. It's used here as the right encoder's SCL
> line; a normal external pull-up holds it HIGH at boot so this works in practice, but
> keep it in mind if you see erratic boot behavior — remap it to a spare pin if needed.

## WiFi dashboard (AS5600/WebDashboard sketches only)

- On boot the ESP32 hosts its own access point — SSID `WallFollowerRobot`, password
  `robot1234` (change both `AP_SSID`/`AP_PASS` near the top of the sketch before any
  real deployment; the defaults are open to whoever knows them).
- Connect a phone or laptop to that network, then browse to the IP address printed to
  Serial at 115200 baud (the banner re-prints every 10 seconds so it's easy to catch
  even if you open the Serial Monitor late).
- The page at `/` is served once; a background poll to the `/data` JSON endpoint
  refreshes the dashboard every 400 ms with live sensor distances, mode, PID error,
  motor speeds, and (where present) per-wheel angle/revolutions/RPM and distance
  traveled.

## Tuning notes

All sketches expose their tunable constants near the top of the file:

- `MAX_DISTANCE_CM` (30) — anything beyond this is treated as "no wall."
- `WALL_PRESENT_CM` (10) — right-side distance threshold that flips the robot from
  single-wall (left) following into centering mode.
- `DESIRED_LEFT_CM` (6) — setpoint used only in single-wall (left) mode.
- `FRONT_STOP_CM` (4) — front distance that triggers a stop-and-pivot.
- `BASE_SPEED` / `MAX_SPEED` / `MIN_SPEED` — PWM bounds (0–255). `BASE_SPEED` varies
  between sketches (150–219) — tune it for your own motors/battery voltage/gearing.
- `Kp` / `Ki` / `Kd` (8 / 0 / 1 by default) — PID gains; integral is disabled out of
  the box.
- `WHEEL_DIAMETER_CM` (6.5, AS5600 sketches only) — feeds the odometry-based "distance
  traveled" stat; measure your actual wheel and set this accurately.

One inconsistency worth knowing about if you copy tuning between sketches: the
`pulseIn()` echo timeout in `wall_follower_robot_esp32.ino` is 5880 µs (~1 m range),
while all three AS5600/dashboard sketches use 25000 µs (~4 m range).

Turn/reverse maneuvers are timed open-loop, not closed-loop off the HC-SR04s (they
can't measure rotation angle), so pivot durations should be recalibrated by hand for
your motors, wheel diameter, and floor surface.

