// =============================================================================
//  Left-Hand-Rule Maze Solver — ESP32 + TB6612FNG + 3x HC-SR04
//
//  Priority logic (left-hand rule): LEFT > FORWARD > RIGHT > U-TURN
//    - Median-filtered + debounced sonar readings (avoid drift-vs-opening confusion)
//    - Wall-following correction while driving forward (keeps a steady left offset)
//    - "Nudge forward" before turning at an opening, so the axle clears the corner
//
//  Pipeline (per your spec, section 8):
//    readSensors() -> filterReadings() -> detectWalls() -> decideDirection()
//      -> executeMovement() -> wallCorrection() -> readSensors() -> ...
// =============================================================================

#include <Arduino.h>

// ── Debug ─────────────────────────────────────────────────────────────────
#define DEBUG 0   // set to 0 to silence all debug output (e.g. for production)

#if DEBUG
  #define DBG_PRINT(...)    Serial.print(__VA_ARGS__)
  #define DBG_PRINTLN(...)  Serial.println(__VA_ARGS__)
  #define DBG_PRINTF(...)   Serial.printf(__VA_ARGS__)
#else
  #define DBG_PRINT(...)
  #define DBG_PRINTLN(...)
  #define DBG_PRINTF(...)
#endif

// ── Motor Driver (TB6612FNG) ─────────────────────────────────────────────
#define PWMA 18
#define AIN1 25
#define AIN2 26
#define PWMB 19
#define BIN1 27
#define BIN2 14
#define STBY 13

// ── Ultrasonic Sensors (HC-SR04) ─────────────────────────────────────────
#define TRIG_FRONT 21
#define ECHO_FRONT 23
#define TRIG_LEFT  4
#define ECHO_LEFT  22
#define TRIG_RIGHT 15
#define ECHO_RIGHT 5

// ── Motor direction inversion ────────────────────────────────────────────
// If a motor spins backward when the code commands forward (positive speed),
// set its invert flag to 1 instead of re-wiring the motor leads.
#define MOTOR_A_INVERT 0   // set to 1 if left/A wheel runs backward on "FORWARD"
#define MOTOR_B_INVERT 1   // set to 1 if right/B wheel runs backward on "FORWARD"

// =============================================================================
//  TUNING — measure your actual maze/robot before trusting these numbers
// =============================================================================
// WALL_CM: distance below which a side/front reading counts as "wall present".
//   Set this a bit larger than (corridor_width/2 - robot_half_width), so the
//   robot reliably sees the wall before it gets dangerously close, but small
//   enough that a real side opening still reads clearly above it.
#define WALL_CM        18.0f

// STOP_CM: minimum safe front distance before the robot must halt and re-decide.
#define STOP_CM        10.0f

// TARGET_LEFT_CM: desired steady-state distance to a left wall while cruising
// down a corridor. Tune to roughly the middle of your corridor width.
#define TARGET_LEFT_CM 9.0f

// OPEN_JUMP_CM / OPEN_CONFIRM_COUNT: an opening is only accepted once a side
// reading exceeds WALL_CM for this many consecutive filtered samples in a row.
// This is what separates "sonar saw a real junction" from "robot is just
// drifting away from the wall" (spec section 4).
#define OPEN_CONFIRM_COUNT 2

// Speeds
#define BASE_SPEED     190
#define TURN_SPEED     150
#define NUDGE_SPEED    150
#define REVERSE_SPEED  150

// Wall-following correction gain/limits (small trims only, never full turns)
#define CORR_KP        6.0f
#define CORR_MAX       50
#define CORR_DEADBAND_CM 1.0f

// Timing — ALL OF THESE MUST BE CALIBRATED EXPERIMENTALLY ON YOUR ROBOT.
// Sonar alone can't measure turn angle; these are open-loop time pulses.
#define NUDGE_MS       220   // creep forward so the wheel axle clears the corner
#define TURN_MS        430   // time to pivot ~90 degrees at TURN_SPEED
#define UTURN_MS        860   // time to pivot ~180 degrees (recalibrate, not just 2x TURN_MS)
#define REVERSE_MS     300
#define SETTLE_MS       40   // brief pause after stopping before the next pivot

#define ECHO_TIMEOUT_US 25000UL

// ── Debug counters/state ────────────────────────────────────────────────
static uint32_t g_loopCount = 0;

// =============================================================================
//  SENSOR READ — single raw ping
// =============================================================================
float readDistanceCmRaw(int trigPin, int echoPin, const char* label) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);

  if (duration == 0) {
    DBG_PRINTF("[SENSOR][%s] no echo within %luus (timeout) -> treating as 400cm (clear)\n",
               label, ECHO_TIMEOUT_US);
    return 400.0f;          // no echo = treat as far/clear
  }

  float cm = duration * 0.01715f;                // convert us -> cm
  DBG_PRINTF("[SENSOR][%s] echo=%luus -> %.1fcm\n", label, duration, cm);
  return cm;
}

float median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

// Three quick pings per sensor, median-filtered, to reject single-ping noise
// (spikes/dropouts) before any decision logic ever sees the number.
float readDistanceCm(int trigPin, int echoPin, const char* label) {
  float s1 = readDistanceCmRaw(trigPin, echoPin, label); delayMicroseconds(300);
  float s2 = readDistanceCmRaw(trigPin, echoPin, label); delayMicroseconds(300);
  float s3 = readDistanceCmRaw(trigPin, echoPin, label);
  float m = median3(s1, s2, s3);
  DBG_PRINTF("[FILTER][%s] samples=[%.1f, %.1f, %.1f] -> median=%.1f\n", label, s1, s2, s3, m);
  return m;
}

// =============================================================================
//  MOTOR CONTROL
// =============================================================================
void motorInit() {
  DBG_PRINTLN("[MOTOR] init: STBY, AIN1/2, BIN1/2 as OUTPUT; PWM channels attached");
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  ledcAttach(PWMA, 22000, 8);
  ledcAttach(PWMB, 22000, 8);
}

void setMotorA(int speed) {
  int constrained = constrain(speed, -255, 255);
  int applied = MOTOR_A_INVERT ? -constrained : constrained;
  digitalWrite(AIN1, applied >= 0 ? LOW : HIGH);
  digitalWrite(AIN2, applied >= 0 ? HIGH : LOW);
  ledcWrite(PWMA, abs(applied));
}

void setMotorB(int speed) {
  int constrained = constrain(speed, -255, 255);
  int applied = MOTOR_B_INVERT ? -constrained : constrained;
  digitalWrite(BIN1, applied >= 0 ? LOW : HIGH);
  digitalWrite(BIN2, applied >= 0 ? HIGH : LOW);
  ledcWrite(PWMB, abs(applied));
}

void motorStop()                     { DBG_PRINTLN("[ACTION] STOP");   setMotorA(0); setMotorB(0); }
void driveForward(int s)             { DBG_PRINTF("[ACTION] FORWARD speed=%d\n", s); setMotorA(s); setMotorB(s); }
void driveReverse(int s)             { DBG_PRINTF("[ACTION] REVERSE speed=%d\n", s); setMotorA(-s); setMotorB(-s); }
void driveDifferential(int l, int r) { DBG_PRINTF("[ACTION] DIFF L=%d R=%d\n", l, r); setMotorA(l); setMotorB(r); }
// Pivot left: left wheel backward, right wheel forward
void pivotLeft(int s)  { DBG_PRINTF("[ACTION] PIVOT LEFT speed=%d\n", s);  setMotorA(-s); setMotorB(s); }
// Pivot right: left wheel forward, right wheel backward
void pivotRight(int s) { DBG_PRINTF("[ACTION] PIVOT RIGHT speed=%d\n", s); setMotorA(s);  setMotorB(-s); }

// =============================================================================
//  SENSOR STATE (readSensors -> filterReadings -> detectWalls)
// =============================================================================
struct SensorState {
  float L, F, R;          // this loop's median-filtered readings
  bool wallL, wallF, wallR; // WALL_CM classification (true = wall present)
};

// Debounce counters: how many consecutive loops each side has read "open"
static int openStreakL = 0;
static int openStreakR = 0;

void readSensors(SensorState &s) {
  s.F = readDistanceCm(TRIG_FRONT, ECHO_FRONT, "FRONT");
  delay(10);
  s.L = readDistanceCm(TRIG_LEFT,  ECHO_LEFT,  "LEFT");
  delay(10);
  s.R = readDistanceCm(TRIG_RIGHT, ECHO_RIGHT, "RIGHT");
  delay(10);
  Serial.printf("F:%.1f L:%.1f R:%.1f\n", s.F, s.L, s.R);
}

// filterReadings(): update the open/closed debounce streaks from the latest
// median-filtered samples. detectWalls() below uses these streaks (not the
// raw sample) to decide OPEN vs WALL, so a single noisy reading can never
// trigger a turn.
void filterReadings(SensorState &s) {
  if (s.L > WALL_CM) openStreakL++; else openStreakL = 0;
  if (s.R > WALL_CM) openStreakR++; else openStreakR = 0;
  DBG_PRINTF("[FILTER] openStreakL=%d openStreakR=%d\n", openStreakL, openStreakR);
}

void detectWalls(SensorState &s) {
  s.wallF = s.F < STOP_CM;                      // front uses the stricter STOP threshold
  s.wallL = !(openStreakL >= OPEN_CONFIRM_COUNT); // confirmed open only after N consecutive loops
  s.wallR = !(openStreakR >= OPEN_CONFIRM_COUNT);
  DBG_PRINTF("[STATE] wallL=%d wallF=%d wallR=%d\n", s.wallL, s.wallF, s.wallR);
}

// =============================================================================
//  DECISION — left-hand rule junction table (spec section 7)
// =============================================================================
enum Action { ACT_FORWARD, ACT_LEFT, ACT_RIGHT, ACT_UTURN };

Action decideDirection(const SensorState &s) {
  // LEFT > FORWARD > RIGHT > BACK
  if (!s.wallL) { DBG_PRINTLN("[DECISION] left open -> LEFT");    return ACT_LEFT; }
  if (!s.wallF) { DBG_PRINTLN("[DECISION] front open -> FORWARD"); return ACT_FORWARD; }
  if (!s.wallR) { DBG_PRINTLN("[DECISION] right open -> RIGHT");  return ACT_RIGHT; }
  DBG_PRINTLN("[DECISION] all walled -> U-TURN");
  return ACT_UTURN;
}

// =============================================================================
//  WALL CORRECTION — small differential trims while driving forward
//  (spec section 3). Only engages when a left wall is actually present;
//  with no left wall to reference, just drive straight at BASE_SPEED.
// =============================================================================
void wallCorrection(float filteredL, bool wallPresent) {
  if (!wallPresent) {
    DBG_PRINTLN("[CORR] no left wall reference -> straight");
    driveForward(BASE_SPEED);
    return;
  }

  float error = TARGET_LEFT_CM - filteredL;  // >0 = too close to wall, <0 = too far
  if (fabs(error) <= CORR_DEADBAND_CM) {
    DBG_PRINTF("[CORR] L=%.1f ~= target %.1f -> straight\n", filteredL, TARGET_LEFT_CM);
    driveForward(BASE_SPEED);
    return;
  }

  int corr = (int)constrain(fabs(error) * CORR_KP, 0, CORR_MAX);
  if (error > 0) {
    // too close to left wall -> steer RIGHT: left faster, right slower
    DBG_PRINTF("[CORR] L=%.1f too close (target %.1f) -> steer RIGHT corr=%d\n", filteredL, TARGET_LEFT_CM, corr);
    driveDifferential(BASE_SPEED + corr, BASE_SPEED - corr);
  } else {
    // too far from left wall -> steer LEFT: left slower, right faster
    DBG_PRINTF("[CORR] L=%.1f too far (target %.1f) -> steer LEFT corr=%d\n", filteredL, TARGET_LEFT_CM, corr);
    driveDifferential(BASE_SPEED - corr, BASE_SPEED + corr);
  }
}

// =============================================================================
//  EXECUTE MOVEMENT
// =============================================================================
void resetSideStreaks() {
  // Once we act on an opening (or leave a junction), clear the debounce
  // counters so stale streaks don't bleed into the next corridor segment.
  openStreakL = 0;
  openStreakR = 0;
}

void executeMovement(Action a, const SensorState &s) {
  switch (a) {
    case ACT_FORWARD:
      wallCorrection(s.L, s.wallL);
      break;

    case ACT_LEFT:
      motorStop();
      delay(SETTLE_MS);
      // Nudge forward so the axle clears the corner before pivoting
      // (sonar can see the opening before the wheels are actually past it).
      driveForward(NUDGE_SPEED);
      delay(NUDGE_MS);
      motorStop();
      delay(SETTLE_MS);
      pivotLeft(TURN_SPEED);
      delay(TURN_MS);
      motorStop();
      delay(SETTLE_MS);
      driveForward(BASE_SPEED);
      resetSideStreaks();
      break;

    case ACT_RIGHT:
      motorStop();
      delay(SETTLE_MS);
      driveForward(NUDGE_SPEED);
      delay(NUDGE_MS);
      motorStop();
      delay(SETTLE_MS);
      pivotRight(TURN_SPEED);
      delay(TURN_MS);
      motorStop();
      delay(SETTLE_MS);
      driveForward(BASE_SPEED);
      resetSideStreaks();
      break;

    case ACT_UTURN:
      motorStop();
      delay(SETTLE_MS);
      pivotRight(TURN_SPEED); // pick one consistent pivot direction for 180s
      delay(UTURN_MS);
      motorStop();
      delay(SETTLE_MS);
      driveForward(BASE_SPEED);
      resetSideStreaks();
      break;
  }
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  DBG_PRINTLN("=== Left-hand-rule maze solver booting ===");
  DBG_PRINTF("[CONFIG] WALL_CM=%.1f STOP_CM=%.1f TARGET_LEFT_CM=%.1f BASE_SPEED=%d TURN_MS=%d NUDGE_MS=%d\n",
             WALL_CM, STOP_CM, TARGET_LEFT_CM, BASE_SPEED, TURN_MS, NUDGE_MS);

  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT,  INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  motorInit();
  motorStop();

  DBG_PRINTLN("[SETUP] Complete. Entering main loop.");
  Serial.println("Maze solver ready.");
}

// =============================================================================
//  MAIN LOOP:  readSensors -> filterReadings -> detectWalls -> decideDirection
//              -> executeMovement (-> wallCorrection inside FORWARD)
// =============================================================================
void loop() {
  uint32_t loopStart = millis();
  g_loopCount++;
  DBG_PRINTF("\n--- loop #%lu @ t=%lums ---\n", g_loopCount, loopStart);

  SensorState s;
  readSensors(s);
  filterReadings(s);
  detectWalls(s);

  Action a = decideDirection(s);
  executeMovement(a, s);

  DBG_PRINTF("[TIMING] loop #%lu took %lums\n", g_loopCount, millis() - loopStart);
}
