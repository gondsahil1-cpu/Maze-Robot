# ESP32 Maze-Solving / Wall-Following Robot

A small differential-drive robot built on an ESP32, a TB6612FNG dual motor driver, and
three HC-SR04 ultrasonic sensors (front / left / right). This repo holds a progression
of sketches — from a bare sensor test up to a full interrupt-driven, OLED-equipped
maze solver — plus a standalone PID wall-follower.

## Hardware

- ESP32 DevKit (WROOM, **no PSRAM** — see note below)
- TB6612FNG dual motor driver
- 2× DC gearmotors (N20 12V 300RPM used in `maze_robot_v8_.ino`) + wheels
- 3× HC-SR04 ultrasonic sensors (front, left, right)
- SSD1306 0.96" I2C OLED — used only by `maze_robot_v8_.ino`
- Passive/active buzzer
- 2S Li-ion battery pack (or equivalent supply for motors + ESP32)

> ⚠️ **PSRAM warning:** `maze_robot_v8_.ino` uses GPIO 16/17 as plain digital I/O for
> the right sensor in some configurations. On ESP32-**WROVER** boards those pins are
> wired to the PSRAM chip and cannot be used as GPIO. Only run that pin mapping on a
> PSRAM-less WROOM DevKit, or remap the right sensor to spare pins (e.g. 5/15) as noted
> in the sketch header.

## Sketches

| File | Purpose |
|---|---|
| `sensor_test.ino` | Minimal diagnostic sketch. Reads all three HC-SR04 sensors (median-of-3 filtered) and prints distances to Serial at 115200 baud. No motor code — run this first to confirm wiring before flashing anything that drives the motors. |
| `wall_follower_robot_esp32.ino` | Standalone PID wall follower. Hugs the left wall at a fixed setpoint, and automatically switches to "centering" mode (balances left vs. right distance) when a right-side wall is also detected. Stops/pivots at dead ends. Good for open corridors, not a full maze solver (no turn-decision state machine). |
| `obstacle_robot_maze.ino` | Left-hand-rule maze solver (`LEFT > FORWARD > RIGHT > U-TURN`). Structured as an explicit pipeline: `readSensors → filterReadings → detectWalls → decideDirection → executeMovement`, with debounced "opening" detection and a small proportional wall-correction trim while driving straight. Includes a buzzer pin. Debug logging is compiled out by default (`DEBUG 0`). |
| `obstacle_robot_maze_without_Display.ino` | Same left-hand-rule pipeline/logic as `obstacle_robot_maze.ino`, with a different pin mapping and retuned constants (target wall distance, base speed, turn timing). Despite the filename, **neither maze sketch drives a display** — this is the variant meant for a build without a buzzer / with the alternate wiring. |
| `maze_robot_v8_.ino` | The full-featured build. Interrupt-driven ultrasonic subsystem (ISRs + a non-blocking sensor scheduler, no `pulseIn()` blocking), a proper `RobotState` finite-state machine (`FORWARD / TURN_LEFT / TURN_RIGHT / SEARCHING / STUCK / REVERSING / TRANSITION`), optional PID **centre-of-corridor** following (`CENTER_MODE`) with PD left-wall fallback, a stuck-robot watchdog, buzzer feedback, and live SSD1306 OLED status output. This is the most complete and actively developed version. |

## Which sketch to flash

1. **`sensor_test.ino`** — verify all three sensors read sane distances.
2. **`wall_follower_robot_esp32.ino`** — if you just want smooth corridor-centering behavior without full maze logic.
3. **`obstacle_robot_maze.ino`** / **`obstacle_robot_maze_without_Display.ino`** — for left-hand-rule maze solving on simpler hardware (no OLED).
4. **`maze_robot_v8_.ino`** — for the full build with OLED status, non-blocking sensors, and a proper state machine. Start here if your hardware matches the parts list above.

## Required Arduino libraries

Install via Library Manager (Arduino IDE) or PlatformIO before building:

- **ESP32 board support** ("esp32" by Espressif Systems) — Arduino-ESP32 core 3.x
- `Adafruit GFX Library` — only needed for `maze_robot_v8_.ino`
- `Adafruit SSD1306` — only needed for `maze_robot_v8_.ino`

(`Wire.h`, `esp_timer.h`, `soc/gpio_reg.h` ship with the ESP32 core.)

## Wiring

Pin assignments **differ between sketches** — always check the `#define` block at the
top of the file you're flashing before wiring up. Rough summary:

| Signal | wall_follower / sensor_test | obstacle_robot_maze | obstacle_robot_maze_without_Display | maze_robot_v8 |
|---|---|---|---|---|
| Motor STBY | 13 | 13 | 13 | 13 |
| Motor AIN1/AIN2/PWMA | 26/25/18 | 25/26/18 | 25/26/18 | 25/26/18 |
| Motor BIN1/BIN2/PWMB | 27/14/19 | 27/14/19 | 27/14/19 | 27/14/19 |
| TRIG/ECHO Front | 5 / 21 | 21 / 23 | 5 / 21 | 21 / 23 |
| TRIG/ECHO Left | 4 / 22 | 4 / 22 | 4 / 22 | 4 / 22 |
| TRIG/ECHO Right | 16 / 17 | 15 / 5 | 16 / 17 | 15 / 5 |
| Buzzer | — | 2 | — | 2 |
| OLED SDA/SCL (I2C) | — | — | — | 32 / 33 |

## Tuning notes

All sketches expose their tunable constants near the top of the file (wall/stop
distance thresholds, base/turn PWM speeds, PID or PD gains, and — critically — the
open-loop turn/reverse timing in milliseconds). Turn and U-turn durations are **not**
closed-loop (the HC-SR04s can't measure rotation angle), so they must be recalibrated
by hand for your specific motors, wheel diameter, and floor surface. If a motor spins
the wrong way on a positive command, flip the corresponding `MOTOR_x_INVERT` flag
(where present) rather than re-wiring the driver.

## License

Add a license of your choice (e.g. MIT) here.
