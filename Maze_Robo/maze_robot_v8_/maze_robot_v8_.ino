// =============================================================================
//  ESP32 Maze-Solving Robot — Left-Wall Following + PID Centre  [v8 — PID CENTER]
//  Hardware : ESP32 DevKit V1 (WROOM — no PSRAM), TB6612FNG, 2×N20 12 V 300 RPM,
//             3×HC-SR04 (FRONT · LEFT · RIGHT),
//             SSD1306 0.96" OLED (I2C), Active Buzzer, 2S Li-ion
//  Core     : ESP32 Arduino Core 3.x
//
//  ⚠  GPIO 16/17 WARNING — PSRAM boards
//  ──────────────────────────────────────
//  TRIG_RIGHT (16) and ECHO_RIGHT (17) are used as normal GPIO here.
//  On ESP32-WROVER and other PSRAM-equipped modules these pins are wired
//  to the PSRAM CS/clock signals and CANNOT be used as user GPIO — the
//  sensor will silently malfunction.  Only use this code on an ESP32-WROOM
//  (no-PSRAM) DevKit.  If you must use a WROVER, move the right sensor
//  trigger and echo to unused pins (e.g. 5 and 15) and update the defines.
//
//  ⚠  ISR / CRITICAL-SECTION NOTE — single-core assumption
//  ─────────────────────────────────────────────────────────
//  The ISRs write EchoData fields directly (no echoMux taken), while
//  updateSensorScheduler() reads those fields inside portENTER_CRITICAL /
//  portEXIT_CRITICAL.  On the default single-core Arduino setup (ISR and
//  loop() on the same core) portENTER_CRITICAL disables local interrupts,
//  which prevents torn reads.  This is safe in practice here, but if the
//  code is ever ported to a dual-core FreeRTOS configuration the ISRs must
//  also acquire the mux with portENTER_CRITICAL_ISR(&echoMux).
//
//  CHANGES FROM v7  (PID centre-following release)
//  ─────────────────────────────────────────────────
//  NEW — PID centre mode (compile-time selectable via CENTER_MODE flag)
//  ───────────────────────────────────────────────────────────────────
//  When CENTER_MODE = 1 the robot steers to stay equidistant between the
//  left and right walls rather than hugging the left wall at a fixed offset.
//
//  Error definition:
//    centerError = distLeft - distRight
//    Positive error → robot is closer to the right wall → steer left
//    Negative error → robot is closer to the left wall  → steer right
//    Zero           → perfectly centred
//
//  Full PID terms:
//    P  KP_CENTER * centerError              — immediate proportional steer
//    I  KI_CENTER * pidIntegral              — removes steady-state drift
//    D  KD_CENTER * (centerError - lastErr)  — damps oscillation
//
//  Integral anti-windup:
//    pidIntegral is clamped to ±PID_INTEGRAL_LIMIT after every accumulation.
//    This prevents the I term from growing so large during an open corridor
//    (where one sensor reads 100 cm) that it causes overshoot when walls
//    reappear.
//
//  Corridor validity guard:
//    Centre correction is only applied when BOTH walls are visible
//    (distLeft < CORRIDOR_VALID_CM && distRight < CORRIDOR_VALID_CM).
//    In a one-sided corridor (e.g. open right) the robot falls back to
//    left-wall PD following (CENTER_MODE = 1) or straight drive.
//    The integral accumulator is also frozen (not reset) during fallback so
//    it retains context for the next valid bilateral corridor.
//
//  Integral reset policy:
//    pidIntegral is reset to 0 on every state transition (enterState calls
//    resetPID()).  This prevents stale integral wind-up from a previous
//    straight run from biasing the first correction in a new corridor.
//
//  When CENTER_MODE = 0 (default):
//    Behaviour is identical to v7 — left-wall PD follow, right sensor used
//    for turn decisions only.
//
//  All prior fixes (v1 FIX 1 – v7 FIX A–H) retained unchanged.
//


// ─── COMPILE-TIME FLAGS ───────────────────────────────────────────────────────
#define CENTER_MODE 1   // 1 = PID centre-follow, 0 = left-wall PD (v7 behaviour)
#define DEBUG_OLED 0    // 1 = shows F:/L:/R: distances + state
#define DEBUG_SERIAL 0  // 1 = Serial sensor + state printout

// ─── LIBRARIES ────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_timer.h>     // FIX 9: ISR-safe hardware timer
#include <soc/gpio_reg.h>  // FIX 10: direct GPIO register reads in ISRs

// =============================================================================
//  FORWARD TYPES
// =============================================================================
struct BuzzNote {
  uint16_t onMs;
  uint16_t offMs;
};

enum RobotState {
  STATE_FORWARD,
  STATE_TURN_RIGHT,
  STATE_TURN_LEFT,
  STATE_SEARCHING,
  STATE_STUCK,
  STATE_REVERSING,
  STATE_TRANSITION  // FIX 7: non-blocking motor-stop pause
};

// =============================================================================
//  PIN DEFINITIONS
// =============================================================================

// ── Motor Driver (TB6612FNG) ──────────────────────────────────────────────────
#define PWMA 18
#define AIN1 25
#define AIN2 26
#define PWMB 19
#define BIN1 27
#define BIN2 14
#define STBY 13

// ── Front Ultrasonic (HC-SR04) ────────────────────────────────────────────────
#define TRIG_FRONT 21
#define ECHO_FRONT 23  // must be < 32 for GPIO_IN_REG (FIX 10)

// ── Left Ultrasonic (HC-SR04) ─────────────────────────────────────────────────
#define TRIG_LEFT 4
#define ECHO_LEFT 22  // must be < 32 for GPIO_IN_REG (FIX 10)

// ── Right Ultrasonic (HC-SR04) ────────────────────────────────────────────────
// ⚠ GPIO 16/17 are PSRAM pins on WROVER boards — see header warning above.
#define TRIG_RIGHT 15
#define ECHO_RIGHT 5  // must be < 32 for GPIO_IN_REG (FIX 10)

// ── OLED I2C ──────────────────────────────────────────────────────────────────
#define OLED_SDA 32
#define OLED_SCL 33

// ── Buzzer ────────────────────────────────────────────────────────────────────
#define BUZZER_PIN 2

// Static asserts: all ECHO pins must live in GPIO 0–31 for GPIO_IN_REG
static_assert(ECHO_FRONT < 32, "ECHO_FRONT must be < 32 for GPIO_IN_REG");
static_assert(ECHO_LEFT < 32, "ECHO_LEFT  must be < 32 for GPIO_IN_REG");
static_assert(ECHO_RIGHT < 32, "ECHO_RIGHT must be < 32 for GPIO_IN_REG");

// Compile-time bitmasks — precomputed for ISRs (FIX 10)
#define ECHO_FRONT_BIT (1UL << ECHO_FRONT)
#define ECHO_LEFT_BIT (1UL << ECHO_LEFT)
#define ECHO_RIGHT_BIT (1UL << ECHO_RIGHT)

// =============================================================================
//  TUNING CONSTANTS
// =============================================================================

// ── Sensor thresholds ────────────────────────────────────────────────────────
#define FRONT_WALL_THRESHOLD 14.0f  // cm — wall entry threshold (FIX E)
#define TURN_FRONT_CLEAR 18.0f      // cm — turn exit threshold  (FIX E)
#define RIGHT_WALL_OPEN 18.0f       // cm — right corridor considered open
#define LEFT_WALL_TARGET 10.0f      // cm — desired left-wall distance (PD mode)
#define LEFT_WALL_NEAR 7.0f         // cm — emergency too-close (FIX F)
#define LEFT_WALL_FAR 20.0f         // cm — left wall lost / open corridor

// ── PID centre-follow: corridor validity ─────────────────────────────────────
// Both sensors must read closer than this for centre PID to engage.
// Beyond this the robot is in a one-sided corridor and falls back to
// left-wall PD (or straight drive if left wall is also absent).
#define CORRIDOR_VALID_CM 60.0f  // cm — max range for a "visible" wall

// ── Drive speeds ─────────────────────────────────────────────────────────────
#define BASE_SPEED 180
#define TURN_SPEED 130
#define REVERSE_SPEED 130  // FIX A
#define CORRECTION_SCALE 0.8f

// ── Left-wall PD gains (CENTER_MODE = 0, or fallback in CENTER_MODE = 1) ─────
#define KP_WALL 4.0f
#define KD_WALL 1.5f

// ── PID centre gains (CENTER_MODE = 1) ────────────────────────────────────────
//
//  Start conservatively and tune on the bench:
//    KP_CENTER: primary steering authority — raise until it centres reliably
//    KI_CENTER: slow drift removal — keep small; raise only if offset persists
//    KD_CENTER: damping — raise if the robot oscillates side-to-side
//
//  Suggested starting point for a 180 PWM base speed, ~10 cm corridor width:
//    KP=3.0  KI=0.05  KD=1.2
//
#define KP_CENTER 3.0f
#define KI_CENTER 0.05f
#define KD_CENTER 1.2f

// Integral anti-windup clamp (in cm·iterations — not physical cm/s).
// Limits max I contribution to ±(PID_INTEGRAL_LIMIT × KI_CENTER) counts.
// At KI=0.05 and LIMIT=80: max I contribution = ±4 PWM counts (conservative).
#define PID_INTEGRAL_LIMIT 80.0f

// Max total PID correction applied to each motor (PWM counts).
// Keeps the robot from full-locking a wheel on a sharp asymmetric corridor.
#define PID_CORRECTION_MAX 70.0f

// ── Turn limits ───────────────────────────────────────────────────────────────
#define TURN_MIN_MS 300
#define TURN_MAX_MS 2500  // FIX D

// ── Timing ───────────────────────────────────────────────────────────────────
#define REVERSE_DURATION_MS 350
#define TRANSITION_PAUSE_MS 80UL  // FIX 7

// ── Stuck watchdog ────────────────────────────────────────────────────────────
#define STUCK_TIMEOUT_MS 3000
#define STUCK_FRONT_CM 10.0f

// ── Buzzer ────────────────────────────────────────────────────────────────────
#define BEEP_DURATION_MS 80

// =============================================================================
//  ULTRASONIC — INTERRUPT-DRIVEN SUBSYSTEM (three sensors)
// =============================================================================

#define TRIG_PULSE_US 12UL
#define ECHO_TIMEOUT_MS 25UL
#define SENSOR_SETTLE_MS 5UL
#define FILTER_SAMPLES 5

// ISR-shared echo measurement structs
struct EchoData {
  volatile uint32_t riseUs;
  volatile uint32_t fallUs;
  volatile bool ready;
  volatile bool active;
};

static EchoData echoFront = { 0, 0, false, false };
static EchoData echoLeft = { 0, 0, false, false };
static EchoData echoRight = { 0, 0, false, false };

static portMUX_TYPE echoMux = portMUX_INITIALIZER_UNLOCKED;

// ── Rolling median filter ─────────────────────────────────────────────────────
struct SonarFilter {
  float buf[FILTER_SAMPLES];
  uint8_t idx;
  bool filled;
  float lastValid;
};

static SonarFilter filtFront = { {}, 0, false, 100.0f };
static SonarFilter filtLeft = { {}, 0, false, 100.0f };
static SonarFilter filtRight = { {}, 0, false, 100.0f };

// ── Sensor scheduler state ────────────────────────────────────────────────────
enum SensorPhase {
  SPHASE_IDLE,
  SPHASE_TRIG_ACTIVE,
  SPHASE_WAITING_ECHO,
  SPHASE_SETTLING
};
enum SensorID { SENSOR_FRONT = 0,
                SENSOR_LEFT = 1,
                SENSOR_RIGHT = 2 };

static SensorPhase sPhase = SPHASE_IDLE;
static SensorID sActive = SENSOR_FRONT;
static uint32_t sPhaseStart = 0;
static uint32_t sWaitStart = 0;

// Published distances (read by FSM)
float distFront = 100.0f;
float distLeft = 100.0f;
float distRight = 100.0f;

// =============================================================================
//  ISR HANDLERS  (IRAM_ATTR; FIX 9; FIX 10)
// =============================================================================

void IRAM_ATTR isrEchoFront() {
  if (!echoFront.active) return;
  uint32_t t = (uint32_t)esp_timer_get_time();
  if (REG_READ(GPIO_IN_REG) & ECHO_FRONT_BIT) {
    echoFront.riseUs = t;
  } else {
    echoFront.fallUs = t;
    echoFront.ready = true;
    echoFront.active = false;
  }
}

void IRAM_ATTR isrEchoLeft() {
  if (!echoLeft.active) return;
  uint32_t t = (uint32_t)esp_timer_get_time();
  if (REG_READ(GPIO_IN_REG) & ECHO_LEFT_BIT) {
    echoLeft.riseUs = t;
  } else {
    echoLeft.fallUs = t;
    echoLeft.ready = true;
    echoLeft.active = false;
  }
}

void IRAM_ATTR isrEchoRight() {
  if (!echoRight.active) return;
  uint32_t t = (uint32_t)esp_timer_get_time();
  if (REG_READ(GPIO_IN_REG) & ECHO_RIGHT_BIT) {
    echoRight.riseUs = t;
  } else {
    echoRight.fallUs = t;
    echoRight.ready = true;
    echoRight.active = false;
  }
}

// =============================================================================
//  MEDIAN FILTER
// =============================================================================

static float computeMedian(const SonarFilter &f) {
  float tmp[FILTER_SAMPLES];
  uint8_t count = f.filled ? FILTER_SAMPLES : f.idx;
  if (count == 0) return f.lastValid;
  memcpy(tmp, f.buf, count * sizeof(float));
  for (uint8_t i = 1; i < count; i++) {
    float key = tmp[i];
    int8_t j = (int8_t)(i - 1);
    while (j >= 0 && tmp[j] > key) {
      tmp[j + 1] = tmp[j];
      j--;
    }
    tmp[j + 1] = key;
  }
  return tmp[count / 2];
}

static void pushSample(SonarFilter &f, float cm) {
  f.buf[f.idx] = cm;
  f.idx = (f.idx + 1) % FILTER_SAMPLES;
  if (f.idx == 0) f.filled = true;
  f.lastValid = computeMedian(f);
}

// =============================================================================
//  SENSOR SCHEDULER — non-blocking round-robin FRONT → LEFT → RIGHT → FRONT
// =============================================================================

static inline float pulseWidthToCm(uint32_t us) {
  return (float)us * 0.01715f;
}

// FIX H: default clause — halts visibly on bad SensorID
static void sensorResources(SensorID id, uint8_t &tpin, EchoData *&ed, SonarFilter *&sf) {
  switch (id) {
    case SENSOR_FRONT:
      tpin = TRIG_FRONT;
      ed = &echoFront;
      sf = &filtFront;
      return;
    case SENSOR_LEFT:
      tpin = TRIG_LEFT;
      ed = &echoLeft;
      sf = &filtLeft;
      return;
    case SENSOR_RIGHT:
      tpin = TRIG_RIGHT;
      ed = &echoRight;
      sf = &filtRight;
      return;
    default:
      Serial.println(F("[FATAL] sensorResources(): unknown SensorID"));
      while (true) { yield(); }
  }
}

static void triggerSensor(SensorID id) {
  uint8_t tpin;
  EchoData *ed;
  SonarFilter *sf;
  sensorResources(id, tpin, ed, sf);
  ed->ready = false;
  ed->active = true;
  digitalWrite(tpin, HIGH);
  sPhaseStart = (uint32_t)esp_timer_get_time();
  sPhase = SPHASE_TRIG_ACTIVE;
  sActive = id;
}

static void updateSensorScheduler() {
  uint32_t nowUs = (uint32_t)esp_timer_get_time();
  uint32_t nowMs = millis();
  uint8_t tpin;
  EchoData *ed;
  SonarFilter *sf;
  sensorResources(sActive, tpin, ed, sf);

  switch (sPhase) {

    case SPHASE_IDLE:
      {
        SensorID next = (SensorID)((sActive + 1) % 3);
        triggerSensor(next);
        break;
      }

    case SPHASE_TRIG_ACTIVE:
      {
        if ((nowUs - sPhaseStart) >= TRIG_PULSE_US) {
          digitalWrite(tpin, LOW);
          sWaitStart = nowMs;
          sPhase = SPHASE_WAITING_ECHO;
        }
        break;
      }

    case SPHASE_WAITING_ECHO:
      {
        bool gotReady = false;
        uint32_t snapRise = 0, snapFall = 0;
        portENTER_CRITICAL(&echoMux);
        if (ed->ready) {
          gotReady = true;
          snapRise = ed->riseUs;
          snapFall = ed->fallUs;
          ed->ready = false;
          ed->active = false;
        }
        portEXIT_CRITICAL(&echoMux);

        if (gotReady) {
          uint32_t pulseUs = snapFall - snapRise;
          float cm = pulseWidthToCm(pulseUs);
          if (cm >= 2.0f && cm <= 400.0f) pushSample(*sf, cm);
          switch (sActive) {
            case SENSOR_FRONT: distFront = filtFront.lastValid; break;
            case SENSOR_LEFT: distLeft = filtLeft.lastValid; break;
            case SENSOR_RIGHT: distRight = filtRight.lastValid; break;
            default: break;
          }
          sWaitStart = nowMs;
          sPhase = SPHASE_SETTLING;
        } else if ((nowMs - sWaitStart) >= ECHO_TIMEOUT_MS) {
          portENTER_CRITICAL(&echoMux);
          ed->active = false;
          portEXIT_CRITICAL(&echoMux);
          sPhase = SPHASE_SETTLING;
          sWaitStart = nowMs;
        }
        break;
      }

    case SPHASE_SETTLING:
      {
        if ((nowMs - sWaitStart) >= SENSOR_SETTLE_MS) sPhase = SPHASE_IDLE;
        break;
      }
  }
}

static void sonarInit() {
  pinMode(TRIG_FRONT, OUTPUT);
  digitalWrite(TRIG_FRONT, LOW);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  digitalWrite(TRIG_LEFT, LOW);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  digitalWrite(TRIG_RIGHT, LOW);
  pinMode(ECHO_RIGHT, INPUT);

  for (uint8_t i = 0; i < FILTER_SAMPLES; i++) {
    filtFront.buf[i] = 100.0f;
    filtLeft.buf[i] = 100.0f;
    filtRight.buf[i] = 100.0f;
  }
  filtFront.filled = filtLeft.filled = filtRight.filled = true;
  filtFront.lastValid = filtLeft.lastValid = filtRight.lastValid = 100.0f;

  attachInterrupt(digitalPinToInterrupt(ECHO_FRONT), isrEchoFront, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_LEFT), isrEchoLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_RIGHT), isrEchoRight, CHANGE);

  sPhase = SPHASE_IDLE;
  sActive = SENSOR_RIGHT;
}

// =============================================================================
//  BUZZER — NON-BLOCKING SEQUENCE PLAYER
// =============================================================================

static const BuzzNote SEQ_RIGHT[2] = { { 60, 60 }, { 130, 0 } };
static const BuzzNote SEQ_LEFT[2] = { { 130, 60 }, { 60, 0 } };
static const BuzzNote SEQ_STUCK[3] = { { 60, 55 }, { 60, 55 }, { 60, 0 } };

static const BuzzNote *buzzSeq = nullptr;
static uint8_t buzzSeqLen = 0;
static uint8_t buzzSeqIdx = 0;
static bool buzzInOn = false;
static unsigned long buzzNextMs = 0;

static void playSequence(const BuzzNote *seq, uint8_t len) {
  digitalWrite(BUZZER_PIN, LOW);
  buzzSeq = seq;
  buzzSeqLen = len;
  buzzSeqIdx = 0;
  buzzInOn = true;
  digitalWrite(BUZZER_PIN, HIGH);
  buzzNextMs = millis() + seq[0].onMs;
}

void checkBuzzer() {
  if (buzzSeq == nullptr || millis() < buzzNextMs) return;
  if (buzzInOn) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzInOn = false;
    uint16_t gap = buzzSeq[buzzSeqIdx].offMs;
    if (gap == 0) {
      if (++buzzSeqIdx >= buzzSeqLen) {
        buzzSeq = nullptr;
        return;
      }
      digitalWrite(BUZZER_PIN, HIGH);
      buzzInOn = true;
      buzzNextMs = millis() + buzzSeq[buzzSeqIdx].onMs;
    } else {
      buzzNextMs = millis() + gap;
    }
  } else {
    if (++buzzSeqIdx >= buzzSeqLen) {
      buzzSeq = nullptr;
      return;
    }
    digitalWrite(BUZZER_PIN, HIGH);
    buzzInOn = true;
    buzzNextMs = millis() + buzzSeq[buzzSeqIdx].onMs;
  }
}

void beepTurnRight() {
  playSequence(SEQ_RIGHT, 2);
}
void beepTurnLeft() {
  playSequence(SEQ_LEFT, 2);
}
void beepStuck() {
  playSequence(SEQ_STUCK, 3);
}

void beep(uint16_t durationMs) {
  static BuzzNote single = { BEEP_DURATION_MS, 0 };
  single.onMs = durationMs;
  playSequence(&single, 1);
}
void beep() {
  beep(BEEP_DURATION_MS);
}

// =============================================================================
//  FSM — global state variables
// =============================================================================

RobotState currentState = STATE_SEARCHING;
RobotState prevState = STATE_STUCK;

// ── Left-wall PD state (used in CENTER_MODE=0 and as fallback) ────────────────
float lastLeftError = 0.0f;

// ── PID centre state (CENTER_MODE=1) ─────────────────────────────────────────
//
//  pidIntegral  — accumulated error over time; clamped by PID_INTEGRAL_LIMIT
//  pidLastError — previous centerError for the D term
//
//  Both are reset via resetPID() on every enterState() call so stale
//  wind-up from a previous corridor never biases the first correction
//  in a new one.
//
float pidIntegral = 0.0f;
float pidLastError = 0.0f;

unsigned long turnStartMs = 0;
unsigned long stuckCheckMs = 0;
unsigned long lastOledMs = 0;

float stuckFrontSnapshot = 999.0f;

// FIX 7: non-blocking transition state
static RobotState transitionNextState = STATE_FORWARD;
static unsigned long transitionDeadline = 0;

// =============================================================================
//  PWM CHANNEL CONFIG  (ESP32 LEDC — Arduino Core 3.x)
// =============================================================================

#define PWM_FREQ 22000
#define PWM_RESOLUTION 8

// =============================================================================
//  MOTOR DRIVER
// =============================================================================

void motorStop();

void motorInit() {
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  ledcAttach(PWMA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PWMB, PWM_FREQ, PWM_RESOLUTION);
  motorStop();
}

void setMotorA(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    ledcWrite(PWMA, speed);
  } else {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    ledcWrite(PWMA, -speed);
  }
}

void setMotorB(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    ledcWrite(PWMB, speed);
  } else {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    ledcWrite(PWMB, -speed);
  }
}

void motorStop() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWMA, 0);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  ledcWrite(PWMB, 0);
}

void driveForward(int l, int r) {
  setMotorA(l);
  setMotorB(r);
}
void driveRight(int s) {
  setMotorA(s);
  setMotorB(-s);
}
void driveLeft(int s) {
  setMotorA(-s);
  setMotorB(s);
}

// =============================================================================
//  OLED — ANIMATED ROBOT FACE
// =============================================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_I2C_ADDR 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define EL_CX 38
#define ER_CX 90
#define EYE_CY 18
#define EYE_W 36
#define EYE_H 22
#define EYE_R 6
#define PUPIL_R 5
#define PUPIL_MAX_X 9
#define PUPIL_MAX_Y 5
#define EYE_H_HALF (EYE_H / 3)

#define BROW_W 28
#define BROW_Y_NORM (EYE_CY - EYE_H / 2 - 4)
#define BROW_TILT_ANGRY 4

// ── Mouth geometry ─────────────────────────────────────────────────────────
// Sits well below the eyes, with its own clear gap above the status-label
// divider line — see FACE_DIVIDER_Y / STATUS_TEXT_Y below.
#define MOUTH_CX 64
#define MOUTH_Y 41
#define MOUTH_HALF_W 16
#define MOUTH_AMP 4         // max curve depth (smile/frown), in px
#define MOUTH_TILT_AMP 4    // max linear tilt (smirk), in px
#define MOUTH_OPEN_MIN_R 2  // min radius of the open/"O" mouth
#define MOUTH_OPEN_MAX_R 6  // max radius of the open/"O" mouth

// ── Divider + status text rows ────────────────────────────────────────────
// Placed below the mouth's maximum extent (open "O" mouth reaches ~49px)
// so the divider line and state label never overlap the mouth animation.
#define FACE_DIVIDER_Y 53
#define STATUS_TEXT_Y 55

#define FWD_LOOK_FRONT_MS 2500
#define FWD_LOOK_LEFT_MS 600

#define BLINK_HALF_DOWN_MS 55
#define BLINK_CLOSED_MS 70
#define BLINK_HALF_UP_MS 55

#define SCAN_HOLD_MS 380

static float pupilX = 0.0f;
static float pupilY = 0.0f;
static float pupilTargX = 0.0f;
static float pupilTargY = 0.0f;

static bool fwdGlancing = false;
static unsigned long fwdGlanceMs = 0;

static uint8_t scanPhase = 0;
static unsigned long scanNextMs = 0;

static uint8_t eyeOpenH = EYE_H;
static uint8_t blinkStage = 0;
static unsigned long blinkMs = 0;
static unsigned long blinkNextMs = 0;

static float browTilt = 0.0f;
static float browTiltTarg = 0.0f;

// ── Mouth animation state ─────────────────────────────────────────────────
// mouthCurve : -1..+1  (frown .. smile), 0 = flat/neutral line
// mouthTilt  : -1..+1  (asymmetric smirk, used during turns)
// mouthOpen  :  0..1   (0 = closed curve, >~0.15 = draws an open "O" mouth)
// All three are smoothed toward their per-state target with lerpF(), the
// same easing helper the eyes/eyebrows already use, so the mouth reacts on
// the same cadence as the rest of the face.
static float mouthCurve = 0.0f;
static float mouthCurveTarg = 0.0f;
static float mouthTilt = 0.0f;
static float mouthTiltTarg = 0.0f;
static float mouthOpen = 0.0f;
static float mouthOpenTarg = 0.0f;

static inline float lerpF(float cur, float tgt, float rate) {
  float d = tgt - cur;
  if (fabsf(d) < 0.5f) return tgt;
  return cur + d * rate;
}

static const char *stateToString(RobotState s) {
  switch (s) {
    case STATE_FORWARD: return "FORWARD";
    case STATE_TURN_RIGHT: return "TURN RIGHT";
    case STATE_TURN_LEFT: return "TURN LEFT";
    case STATE_SEARCHING: return "SEARCHING";
    case STATE_STUCK: return "STUCK!";
    case STATE_REVERSING: return "REVERSING";
    case STATE_TRANSITION: return "...";
    default: return "???";
  }
}

static void updateEyeAnimation() {
  unsigned long now = millis();
  float targX = 0.0f, targY = 0.0f;

  switch (currentState) {
    case STATE_FORWARD:
    case STATE_TRANSITION:
      if (now >= fwdGlanceMs) {
        fwdGlancing = !fwdGlancing;
        fwdGlanceMs = now + (fwdGlancing ? FWD_LOOK_LEFT_MS : FWD_LOOK_FRONT_MS);
      }
      targX = fwdGlancing ? (float)PUPIL_MAX_X : 0.0f;
      targY = 0.0f;
      browTiltTarg = 0.0f;
      break;

    case STATE_TURN_RIGHT:
      targX = -(float)PUPIL_MAX_X;
      targY = 1.0f;
      browTiltTarg = 0.0f;
      break;

    case STATE_TURN_LEFT:
      targX = (float)PUPIL_MAX_X;
      targY = 1.0f;
      browTiltTarg = 0.0f;
      break;

    case STATE_SEARCHING:
      if (now >= scanNextMs) {
        scanPhase = (scanPhase + 1) % 5;
        scanNextMs = now + SCAN_HOLD_MS;
      }
      switch (scanPhase) {
        case 0:
        case 4: targX = 0.0f; break;
        case 1: targX = +(float)PUPIL_MAX_X; break;
        case 2: targX = 0.0f; break;
        case 3: targX = -(float)PUPIL_MAX_X; break;
        default: break;
      }
      targY = -2.0f;
      browTiltTarg = 0.0f;
      break;

    case STATE_STUCK:
    case STATE_REVERSING:
      targX = 0.0f;
      targY = 3.0f;
      browTiltTarg = (float)BROW_TILT_ANGRY;
      break;
  }

  pupilTargX = targX;
  pupilTargY = targY;
  pupilX = lerpF(pupilX, pupilTargX, 0.25f);
  pupilY = lerpF(pupilY, pupilTargY, 0.25f);
  browTilt = lerpF(browTilt, browTiltTarg, 0.20f);
}

// =============================================================================
//  OLED — MOUTH ANIMATION
//
//  The mouth reacts to the same `currentState` (and, for SEARCHING, the same
//  scanPhase) that already drives the eyes and eyebrows, so the whole face
//  reads as one coherent expression rather than eyes and mouth telling
//  different stories.
//
//  Shape model:
//    A closed mouth is a short parabola across MOUTH_HALF_W*2 px, with an
//    added linear term for asymmetric "smirk" tilt:
//
//      y(dx) = MOUTH_Y + mouthCurve * MOUTH_AMP * (1 - dx²) + mouthTilt * MOUTH_TILT_AMP * dx
//
//    where dx runs -1..+1 across the mouth width.
//      mouthCurve > 0 → corners rise above centre → smile ("U" shape)
//      mouthCurve < 0 → corners dip below centre  → frown ("∩" shape)
//      mouthTilt   swaps in a linear ramp so one corner sits higher than
//      the other — used for the TURN_LEFT / TURN_RIGHT smirk.
//
//    An "open" mouth (mouthOpen > ~0.15) instead draws a worried/talking
//    "O" — used when STUCK/REVERSING (open in alarm) and lightly during
//    SEARCHING (pursed "hmm" — small open amount).
//
//  Per-state targets (mirrors the eye/eyebrow state machine above):
//    FORWARD / TRANSITION → gentle closed smile, no tilt, closed
//    TURN_RIGHT           → light smile + smirk tilted toward the turn
//    TURN_LEFT            → light smile + smirk tilted toward the turn
//    SEARCHING            → flat/neutral curve, slightly open ("hmm")
//    STUCK / REVERSING    → frown, wide open ("uh oh")
//
//  All three parameters are eased with lerpF() at the same rate the eyes
//  use, so the mouth transitions land in sync with the pupils/eyebrows.
// =============================================================================

static void updateMouthAnimation() {
  switch (currentState) {
    case STATE_FORWARD:
    case STATE_TRANSITION:
      mouthCurveTarg = 0.7f;
      mouthTiltTarg = 0.0f;
      mouthOpenTarg = 0.0f;
      break;

    case STATE_TURN_RIGHT:
      mouthCurveTarg = 0.35f;
      mouthTiltTarg = 1.0f;
      mouthOpenTarg = 0.0f;
      break;

    case STATE_TURN_LEFT:
      mouthCurveTarg = 0.35f;
      mouthTiltTarg = -1.0f;
      mouthOpenTarg = 0.0f;
      break;

    case STATE_SEARCHING:
      // Small "hmm" pucker — flat curve, a touch open, no tilt.
      mouthCurveTarg = -0.05f;
      mouthTiltTarg = 0.0f;
      mouthOpenTarg = 0.30f;
      break;

    case STATE_STUCK:
    case STATE_REVERSING:
      mouthCurveTarg = -0.8f;
      mouthTiltTarg = 0.0f;
      mouthOpenTarg = 0.65f;
      break;
  }

  mouthCurve = lerpF(mouthCurve, mouthCurveTarg, 0.18f);
  mouthTilt = lerpF(mouthTilt, mouthTiltTarg, 0.18f);
  mouthOpen = lerpF(mouthOpen, mouthOpenTarg, 0.18f);
}

// Draws the mouth using the current mouthCurve/mouthTilt/mouthOpen state.
// Safe to call with any values — all shape parameters are derived, never
// read directly from currentState, so it also works during startupAnimation
// where the caller sets the three statics explicitly.
static void drawMouth() {
  if (mouthOpen > 0.15f) {
    // Open "O" mouth — radius grows with how "open" the expression is.
    int8_t r = (int8_t)(MOUTH_OPEN_MIN_R + mouthOpen * (float)(MOUTH_OPEN_MAX_R - MOUTH_OPEN_MIN_R));
    display.fillCircle(MOUTH_CX, MOUTH_Y + 2, r, SSD1306_WHITE);
    int8_t innerR = (int8_t)(r - 2);
    if (innerR < 1) innerR = 1;
    display.fillCircle(MOUTH_CX, MOUTH_Y + 2, innerR, SSD1306_BLACK);
    return;
  }

  // Closed mouth — short parabola + linear tilt, drawn as connected segments
  // with a 1px vertical offset for a slightly thicker, more visible line.
  const uint8_t STEPS = 8;
  int16_t prevX = 0, prevY = 0;
  for (uint8_t i = 0; i <= STEPS; i++) {
    float t = (float)i / (float)STEPS;  // 0..1 across mouth width
    float dx = -1.0f + 2.0f * t;        // -1..+1
    int16_t x = (int16_t)(MOUTH_CX - MOUTH_HALF_W + t * (MOUTH_HALF_W * 2));
    float y = MOUTH_Y
              + mouthCurve * (float)MOUTH_AMP * (1.0f - dx * dx)
              + mouthTilt * (float)MOUTH_TILT_AMP * dx;
    int16_t yi = (int16_t)y;

    if (i > 0) {
      display.drawLine(prevX, prevY, x, yi, SSD1306_WHITE);
      display.drawLine(prevX, prevY + 1, x, yi + 1, SSD1306_WHITE);
    }
    prevX = x;
    prevY = yi;
  }
}

static void updateBlinkAnimation() {
  unsigned long now = millis();
  if (blinkStage == 0) {
    if (now >= blinkNextMs) {
      blinkStage = 1;
      blinkMs = now + BLINK_HALF_DOWN_MS;
    }
    return;
  }
  if (now < blinkMs) return;
  switch (blinkStage) {
    case 1:
      eyeOpenH = EYE_H_HALF;
      blinkStage = 2;
      blinkMs = now + BLINK_CLOSED_MS;
      break;
    case 2:
      eyeOpenH = 0;
      blinkStage = 3;
      blinkMs = now + BLINK_HALF_UP_MS;
      break;
    case 3:
      eyeOpenH = EYE_H_HALF;
      blinkStage = 4;
      blinkMs = now + BLINK_HALF_DOWN_MS;
      break;
    case 4:
      eyeOpenH = EYE_H;
      blinkStage = 0;
      blinkNextMs = now + 3000UL + (unsigned long)random(0, 5001);
      break;
  }
}

static void drawEyebrows() {
  int tilt = (int)browTilt;
  display.drawLine(EL_CX - BROW_W / 2, BROW_Y_NORM, EL_CX + BROW_W / 2, BROW_Y_NORM + tilt, SSD1306_WHITE);
  display.drawLine(EL_CX - BROW_W / 2, BROW_Y_NORM - 1, EL_CX + BROW_W / 2, BROW_Y_NORM + tilt - 1, SSD1306_WHITE);
  display.drawLine(ER_CX + BROW_W / 2, BROW_Y_NORM, ER_CX - BROW_W / 2, BROW_Y_NORM + tilt, SSD1306_WHITE);
  display.drawLine(ER_CX + BROW_W / 2, BROW_Y_NORM - 1, ER_CX - BROW_W / 2, BROW_Y_NORM + tilt - 1, SSD1306_WHITE);
}

static void drawRobotFace(uint8_t eyeH, int8_t pxOffset, int8_t pyOffset) {
  if (eyeH == 0) {
    display.drawFastHLine(EL_CX - EYE_W / 2 + 4, EYE_CY, EYE_W - 8, SSD1306_WHITE);
    display.drawFastHLine(ER_CX - EYE_W / 2 + 4, EYE_CY, EYE_W - 8, SSD1306_WHITE);
    return;
  }
  int8_t maxPX = (int8_t)(EYE_W / 2 - PUPIL_R - 3);
  int8_t maxPY = (int8_t)(eyeH / 2 - PUPIL_R - 2);
  if (maxPY < 0) maxPY = 0;
  pxOffset = (int8_t)constrain((int)pxOffset, -maxPX, maxPX);
  pyOffset = (int8_t)constrain((int)pyOffset, -maxPY, maxPY);

  int16_t lx = EL_CX - EYE_W / 2, rx = ER_CX - EYE_W / 2, ey = EYE_CY - eyeH / 2;
  display.fillRoundRect(lx, ey, EYE_W, eyeH, EYE_R, SSD1306_WHITE);
  display.fillRoundRect(rx, ey, EYE_W, eyeH, EYE_R, SSD1306_WHITE);
  display.fillCircle(EL_CX + pxOffset, EYE_CY + pyOffset, PUPIL_R, SSD1306_BLACK);
  display.fillCircle(ER_CX + pxOffset, EYE_CY + pyOffset, PUPIL_R, SSD1306_BLACK);
  display.drawPixel(EL_CX + pxOffset - 2, EYE_CY + pyOffset - 2, SSD1306_WHITE);
  display.drawPixel(ER_CX + pxOffset - 2, EYE_CY + pyOffset - 2, SSD1306_WHITE);
}

void updateOled() {
  updateBlinkAnimation();
  updateEyeAnimation();
  updateMouthAnimation();

  int8_t px = (int8_t)constrain((int)pupilX, -PUPIL_MAX_X, PUPIL_MAX_X);
  int8_t py = (int8_t)constrain((int)pupilY, -PUPIL_MAX_Y, PUPIL_MAX_Y);

  display.clearDisplay();
  drawEyebrows();
  drawRobotFace(eyeOpenH, px, py);
  drawMouth();
  display.drawFastHLine(0, FACE_DIVIDER_Y, SCREEN_WIDTH, SSD1306_WHITE);

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  const char *label = stateToString(currentState);
  int16_t tx, ty;
  uint16_t tw, th;
  display.getTextBounds(label, 0, 0, &tx, &ty, &tw, &th);
  display.setCursor((int16_t)((SCREEN_WIDTH - tw) / 2), STATUS_TEXT_Y);
  display.print(label);

#if DEBUG_OLED
  display.setCursor(0, STATUS_TEXT_Y + 8);
  display.print(F("F:"));
  display.print((int)distFront);
  display.print(F(" L:"));
  display.print((int)distLeft);
  display.print(F(" R:"));
  display.print((int)distRight);
#endif

  display.display();
}

void startupAnimation() {
  display.clearDisplay();
  display.display();

  for (uint8_t h = 0; h <= EYE_H; h++) {
    display.clearDisplay();
    if (h > 0) {
      display.fillRoundRect(EL_CX - EYE_W / 2, EYE_CY - h / 2, EYE_W, h, EYE_R, SSD1306_WHITE);
      display.fillRoundRect(ER_CX - EYE_W / 2, EYE_CY - h / 2, EYE_W, h, EYE_R, SSD1306_WHITE);
      if (h >= EYE_H / 2) {
        display.fillCircle(EL_CX, EYE_CY, PUPIL_R, SSD1306_BLACK);
        display.fillCircle(ER_CX, EYE_CY, PUPIL_R, SSD1306_BLACK);
      }
    }
    display.drawFastHLine(0, FACE_DIVIDER_Y, SCREEN_WIDTH, SSD1306_WHITE);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(34, STATUS_TEXT_Y);
    display.print(F("MAZE BOT"));
    display.display();
    delay(22);
  }
  delay(150);

  const int8_t glanceX[] = { (int8_t)PUPIL_MAX_X, -(int8_t)PUPIL_MAX_X, 0 };
  const uint8_t glanceDly[] = { 250, 250, 200 };
  for (uint8_t g = 0; g < 3; g++) {
    display.clearDisplay();
    drawEyebrows();
    drawRobotFace(EYE_H, glanceX[g], 0);
    display.drawFastHLine(0, FACE_DIVIDER_Y, SCREEN_WIDTH, SSD1306_WHITE);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(34, STATUS_TEXT_Y);
    display.print(F("MAZE BOT"));
    display.display();
    delay(glanceDly[g]);
  }

  const uint8_t blinkH[] = { EYE_H_HALF, 0, EYE_H_HALF, EYE_H };
  const uint8_t blinkDly[] = { 55, 70, 55, 0 };
  for (uint8_t b = 0; b < 4; b++) {
    display.clearDisplay();
    drawEyebrows();
    drawRobotFace(blinkH[b], 0, 0);
    display.drawFastHLine(0, FACE_DIVIDER_Y, SCREEN_WIDTH, SSD1306_WHITE);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(34, STATUS_TEXT_Y);
    display.print(F("MAZE BOT"));
    display.display();
    if (blinkDly[b]) delay(blinkDly[b]);
  }
  delay(200);

  // Final "READY" frame — set a neutral closed smile explicitly, since
  // updateMouthAnimation() (which normally drives these) isn't called
  // during the startup sequence.
  mouthCurve = 0.7f;
  mouthTilt = 0.0f;
  mouthOpen = 0.0f;
  display.clearDisplay();
  drawEyebrows();
  drawRobotFace(EYE_H, 0, 0);
  drawMouth();
  display.drawFastHLine(0, FACE_DIVIDER_Y, SCREEN_WIDTH, SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  int16_t tx, ty;
  uint16_t tw, th;
  display.getTextBounds("READY", 0, 0, &tx, &ty, &tw, &th);
  display.setCursor((int16_t)((SCREEN_WIDTH - tw) / 2), STATUS_TEXT_Y);
  display.print(F("READY"));
  display.display();

  beep(200);
  delay(400);

  eyeOpenH = EYE_H;
  blinkStage = 0;
  blinkNextMs = millis() + 3000UL + (unsigned long)random(0, 5001);
  scanPhase = 0;
  scanNextMs = millis() + SCAN_HOLD_MS;
  fwdGlancing = false;
  fwdGlanceMs = millis() + FWD_LOOK_FRONT_MS;
}

// =============================================================================
//  PID STATE RESET
//  Called on every enterState() so stale integral from a previous corridor
//  never biases corrections in a new one.
// =============================================================================

static void resetPID() {
  pidIntegral = 0.0f;
  pidLastError = 0.0f;
  lastLeftError = 0.0f;
}

// =============================================================================
//  NAVIGATION — CENTRE PID  (CENTER_MODE = 1)
//
//  Error  : centerError = distLeft - distRight
//             > 0  → closer to right wall → steer left  (reduce rightSpeed)
//             < 0  → closer to left wall  → steer right (reduce leftSpeed)
//             = 0  → centred
//
//  Corridor validity:
//    Both walls must read < CORRIDOR_VALID_CM.
//    If only one wall is visible the robot falls back to left-wall PD
//    (handleForwardPD) to keep tracking a single reference rather than
//    chasing a 100 cm "phantom" wall and over-correcting.
//
//  Integral anti-windup:
//    pidIntegral is clamped to ±PID_INTEGRAL_LIMIT each iteration.
//    During fallback the accumulator is frozen — not zeroed — so the
//    controller retains corridor context when bilateral walls reappear.
// =============================================================================

#if CENTER_MODE

static void handleForwardCenterPID() {
  bool leftVisible = (distLeft < CORRIDOR_VALID_CM);
  bool rightVisible = (distRight < CORRIDOR_VALID_CM);

  if (leftVisible && rightVisible) {
    // ── Full bilateral PID correction ─────────────────────────────────────
    float error = distLeft - distRight;  // +ve → too close to right

    // Integral with anti-windup clamp
    pidIntegral += error;
    pidIntegral = constrain(pidIntegral, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);

    float dError = error - pidLastError;
    pidLastError = error;

    float correction = (KP_CENTER * error)
                       + (KI_CENTER * pidIntegral)
                       + (KD_CENTER * dError);

    correction = constrain(correction, -PID_CORRECTION_MAX, PID_CORRECTION_MAX);

    // Positive correction → steer left → slow right motor, speed left motor
    int leftSpeed = constrain(BASE_SPEED + (int)correction, 60, 255);
    int rightSpeed = constrain(BASE_SPEED - (int)correction, 60, 255);
    driveForward(leftSpeed, rightSpeed);

  } else if (leftVisible && !rightVisible) {
    // ── Right wall absent: fall back to left-wall PD ──────────────────────
    // FIX F guard still applies inside wallFollowCorrectionPD
    if (distLeft < LEFT_WALL_NEAR) {
      // Emergency hard steer away from the left wall
      driveForward(BASE_SPEED, BASE_SPEED - 80);
    } else {
      float error = distLeft - LEFT_WALL_TARGET;
      float dError = error - lastLeftError;
      lastLeftError = error;
      float corr = (KP_WALL * error + KD_WALL * dError) * CORRECTION_SCALE;
      corr = constrain(corr, -60.0f, 60.0f);
      int ls = constrain(BASE_SPEED - (int)corr, 60, 255);
      int rs = constrain(BASE_SPEED + (int)corr, 60, 255);
      driveForward(ls, rs);
    }
    // Do NOT touch pidIntegral here — freeze it for next bilateral corridor

  } else {
    // ── Both walls absent (open space) or only right visible ─────────────
    // Drive straight; FIX F emergency steer still guards extreme closeness.
    if (distLeft < LEFT_WALL_NEAR) {
      driveForward(BASE_SPEED, BASE_SPEED - 80);
    } else {
      driveForward(BASE_SPEED, BASE_SPEED);
    }
    // Freeze integral — do not accumulate on phantom walls
  }
}

#else  // CENTER_MODE = 0 ────────────────────────────────────────────────────────

// =============================================================================
//  NAVIGATION — LEFT-WALL FOLLOWING WITH PD CORRECTION  (v7 behaviour)
// =============================================================================

static void wallFollowCorrection(int &leftSpeed, int &rightSpeed) {
  float error = distLeft - LEFT_WALL_TARGET;
  float dError = error - lastLeftError;
  lastLeftError = error;
  float correction = (KP_WALL * error + KD_WALL * dError) * CORRECTION_SCALE;
  correction = constrain(correction, -60.0f, 60.0f);
  leftSpeed = constrain(BASE_SPEED - (int)correction, 60, 255);
  rightSpeed = constrain(BASE_SPEED + (int)correction, 60, 255);
}

#endif  // CENTER_MODE

// =============================================================================
//  handleForward() — single dispatch point called from STATE_FORWARD
//  Selects PID centre or PD left-wall at compile time.
// =============================================================================

void handleForward() {
#if CENTER_MODE
  handleForwardCenterPID();
#else
  // FIX F: emergency hard-steer if left wall is dangerously close
  if (distLeft < LEFT_WALL_NEAR) {
    driveForward(BASE_SPEED, BASE_SPEED - 80);
    return;
  }
  int ls = BASE_SPEED, rs = BASE_SPEED;
  if (distLeft < LEFT_WALL_FAR) wallFollowCorrection(ls, rs);
  driveForward(ls, rs);
#endif
}

// =============================================================================
//  NON-BLOCKING TRANSITION HELPER  (FIX 7)
// =============================================================================

static void beginTransition(RobotState next, unsigned long pauseMs = TRANSITION_PAUSE_MS) {
  motorStop();
  transitionNextState = next;
  transitionDeadline = millis() + pauseMs;
  currentState = STATE_TRANSITION;
}

// =============================================================================
//  FSM HELPERS — threshold predicates with hysteresis (FIX E)
// =============================================================================

static inline bool wallAhead() {
  return distFront < FRONT_WALL_THRESHOLD;
}
static inline bool frontClearExit() {
  return distFront > TURN_FRONT_CLEAR;
}
static inline bool rightOpen() {
  return distRight > RIGHT_WALL_OPEN;
}
static inline bool leftOpen() {
  return distLeft > LEFT_WALL_FAR;
}

// =============================================================================
//  FINITE STATE MACHINE
// =============================================================================

void enterState(RobotState next) {
  currentState = next;

  // Reset PID / PD integrators on every state change so stale
  // wind-up from a previous corridor does not carry over.
  resetPID();

  switch (next) {

    case STATE_TURN_RIGHT:
      driveRight(TURN_SPEED);
      turnStartMs = millis();
      beepTurnRight();
      break;

    case STATE_TURN_LEFT:
      driveLeft(TURN_SPEED);
      turnStartMs = millis();
      beepTurnLeft();
      break;

    case STATE_FORWARD:
      fwdGlancing = false;
      fwdGlanceMs = millis() + FWD_LOOK_FRONT_MS;
      break;

    case STATE_SEARCHING:
      driveForward(BASE_SPEED - 40, BASE_SPEED - 40);
      break;

    case STATE_STUCK:
      motorStop();
      beepStuck();
      setMotorA(-REVERSE_SPEED);
      setMotorB(-REVERSE_SPEED);
      turnStartMs = millis();
      currentState = STATE_REVERSING;
      break;

    // FIX A: STATE_REVERSING has real entry actions
    case STATE_REVERSING:
      setMotorA(-REVERSE_SPEED);
      setMotorB(-REVERSE_SPEED);
      turnStartMs = millis();
      break;

    case STATE_TRANSITION:
      break;
  }
}

void updateFSM() {
  unsigned long now = millis();

  // ── FIX 7: service the non-blocking transition state ─────────────────────
  if (currentState == STATE_TRANSITION) {
    if (now >= transitionDeadline) enterState(transitionNextState);
    return;
  }

  // ── Stuck watchdog ────────────────────────────────────────────────────────
  if (currentState == STATE_FORWARD || currentState == STATE_SEARCHING) {
    if (now - stuckCheckMs >= STUCK_TIMEOUT_MS) {
      stuckCheckMs = now;
      if (distFront < STUCK_FRONT_CM && fabsf(distFront - stuckFrontSnapshot) < 2.0f) {
        enterState(STATE_STUCK);
        return;
      }
      stuckFrontSnapshot = distFront;
    }
  }

  switch (currentState) {

    // ── FORWARD ──────────────────────────────────────────────────────────────
    case STATE_FORWARD:
      if (wallAhead()) {
        if (rightOpen()) beginTransition(STATE_TURN_RIGHT);
        else if (leftOpen()) beginTransition(STATE_TURN_LEFT);
        else beginTransition(STATE_REVERSING);
      } else if (leftOpen()) {
        beginTransition(STATE_TURN_LEFT);
      } else {
        handleForward();
      }
      break;

    // ── TURN RIGHT ────────────────────────────────────────────────────────────
    case STATE_TURN_RIGHT:
      if (now - turnStartMs >= TURN_MAX_MS) {
        enterState(STATE_STUCK);
      } else if ((now - turnStartMs >= TURN_MIN_MS) && frontClearExit()) {
        beginTransition(STATE_FORWARD);
      }
      break;

    // ── TURN LEFT ─────────────────────────────────────────────────────────────
    case STATE_TURN_LEFT:
      if (now - turnStartMs >= TURN_MAX_MS) {
        enterState(STATE_STUCK);
      } else if ((now - turnStartMs >= TURN_MIN_MS) && frontClearExit()) {
        if (rightOpen()) beginTransition(STATE_TURN_RIGHT);
        else beginTransition(STATE_FORWARD);
      }
      break;

    // ── SEARCHING ─────────────────────────────────────────────────────────────
    case STATE_SEARCHING:
      if (wallAhead()) {
        if (rightOpen()) beginTransition(STATE_TURN_RIGHT);
        else if (leftOpen()) beginTransition(STATE_TURN_LEFT);
        else beginTransition(STATE_REVERSING);
      } else if (!leftOpen()) {
        beginTransition(STATE_FORWARD);  // FIX G
      }
      break;

    // ── STUCK ─────────────────────────────────────────────────────────────────
    case STATE_STUCK:
      break;  // enterState(STATE_STUCK) immediately sets STATE_REVERSING

    // ── REVERSING ─────────────────────────────────────────────────────────────
    case STATE_REVERSING:
      if (now - turnStartMs >= REVERSE_DURATION_MS) {
        if (rightOpen()) beginTransition(STATE_TURN_RIGHT);
        else if (leftOpen()) beginTransition(STATE_TURN_LEFT);
        else beginTransition(STATE_REVERSING);  // FIX C
      }
      break;

    case STATE_TRANSITION:
      break;
  }
}

// =============================================================================
//  SETUP
// =============================================================================

void setup() {
  Serial.begin(115200);
  Serial.println(F("[MAZE ROBOT v8] Booting..."));
#if CENTER_MODE
  Serial.println(F("[NAV] PID centre-follow mode active."));
#else
  Serial.println(F("[NAV] Left-wall PD follow mode active."));
#endif

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  motorInit();
  randomSeed(analogRead(34));

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
    Serial.println(F("[ERROR] SSD1306 not found — check I2C wiring!"));
  } else {
    display.clearDisplay();
    display.display();
    startupAnimation();
    Serial.println(F("[OLED] Startup animation complete."));
  }

  sonarInit();
  Serial.println(F("[SONAR] 3-sensor interrupt-driven subsystem initialised."));

  {
    unsigned long warmupEnd = millis() + 200UL;
    while (millis() < warmupEnd) {
      updateSensorScheduler();
      yield();
    }
  }

  enterState(STATE_SEARCHING);
  stuckCheckMs = millis();
  stuckFrontSnapshot = distFront;

  Serial.println(F("[MAZE ROBOT v8] Ready."));
  beep(150);
}

// =============================================================================
//  MAIN LOOP
// =============================================================================

void loop() {
  unsigned long now = millis();

  updateSensorScheduler();
  updateFSM();
  checkBuzzer();

  if (currentState != prevState || now - lastOledMs >= 80) {
    lastOledMs = now;
    prevState = currentState;
    updateOled();
  }

#if DEBUG_SERIAL
  static unsigned long lastDbgMs = 0;
  if (now - lastDbgMs >= 100) {
    lastDbgMs = now;
    Serial.printf("[SENSOR] Front=%.1f  Left=%.1f  Right=%.1f  State=%s\n",
                  distFront, distLeft, distRight, stateToString(currentState));
#if CENTER_MODE
    float ce = distLeft - distRight;
    Serial.printf("[PID]    err=%.1f  integral=%.2f  lastErr=%.1f\n",
                  ce, pidIntegral, pidLastError);
#endif
  }
#endif

  yield();
}

// =============================================================================
//  END OF FILE
//
//  v7 → v8 CHANGE SUMMARY
//  ───────────────────────
//
//  NEW: PID centre-follow mode (CENTER_MODE = 1, compile-time flag)
//  ──────────────────────────────────────────────────────────────────
//
//  Error definition:
//    centerError = distLeft - distRight
//    Positive → robot drifted right → steer left
//    Negative → robot drifted left  → steer right
//
//  Controller:
//    correction = KP·error + KI·integral + KD·derivative
//    Left  motor speed = BASE_SPEED + correction
//    Right motor speed = BASE_SPEED − correction
//
//  Anti-windup:
//    pidIntegral is clamped to ±PID_INTEGRAL_LIMIT each loop iteration.
//    Prevents large open-space readings (100 cm) from winding up the I
//    term to the point of overshoot when walls reappear.
//
//  Corridor validity guard:
//    PID only fires when both distLeft and distRight < CORRIDOR_VALID_CM.
//    One-sided corridor → falls back to left-wall PD tracking.
//    Open space         → drives straight (with FIX F emergency steer).
//    Integral is frozen (not reset) during fallback.
//
//  Integral reset:
//    resetPID() called in every enterState() transition.
//    Prevents stale integral from biasing the first correction after a turn.
//
//  Backward compatibility:
//    CENTER_MODE = 0 → identical behaviour to v7.
//    All prior fixes (FIX A–H, FIX 1–10) are retained unchanged.
//
//  Tuning starting point:
//    KP_CENTER = 3.0   KI_CENTER = 0.05   KD_CENTER = 1.2
//    PID_INTEGRAL_LIMIT = 80   PID_CORRECTION_MAX = 70
//
//  v8 → v8.1 CHANGE SUMMARY  (mouth animation)
//  ──────────────────────────────────────────────
//
//  NEW: Mouth animation, tied to the same currentState the eyes/eyebrows use
//  ──────────────────────────────────────────────────────────────────────────
//
//  Shape:
//    Closed mouth = parabola + linear tilt across MOUTH_HALF_W*2 px:
//      y(dx) = MOUTH_Y + mouthCurve*MOUTH_AMP*(1-dx²) + mouthTilt*MOUTH_TILT_AMP*dx
//    Open mouth (mouthOpen > 0.15) = concentric-circle "O", radius scales
//    with mouthOpen between MOUTH_OPEN_MIN_R and MOUTH_OPEN_MAX_R.
//
//  Per-state expression (set in updateMouthAnimation(), eased with lerpF()):
//    FORWARD / TRANSITION → closed smile
//    TURN_RIGHT/LEFT      → closed smile with a smirk tilt toward the turn
//    SEARCHING            → flat, lightly parted ("hmm")
//    STUCK / REVERSING    → frown, wide open ("uh oh")
//
//  Integration points:
//    updateOled()      → calls updateMouthAnimation() then drawMouth()
//    startupAnimation()→ sets a neutral smile explicitly for the READY frame
//
//  No changes to navigation, sensors, FSM transitions, or motor control.
// =============================================================================
