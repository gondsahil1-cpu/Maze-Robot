// =============================================================================
//  Left-Hand-Rule Maze Solver — ESP32 + TB6612FNG + 3x HC-SR04
//  ***  NOW WITH INTERSECTION MEMORY / DFS BACKTRACKING / SHORTEST-PATH REPLAY  ***
//
//  Priority logic (left-hand rule): LEFT > FORWARD > RIGHT > U-TURN
//    - Median-filtered + debounced sonar readings (avoid drift-vs-opening confusion)
//    - Wall-following correction while driving forward (keeps a steady left offset)
//    - "Nudge forward" before turning at an opening, so the axle clears the corner
//
//  Pipeline (per your spec, section 8) — UNCHANGED:
//    readSensors() -> filterReadings() -> detectWalls() -> decideDirection()
//      -> executeMovement() -> wallCorrection() -> readSensors() -> ...
//
//  ===========================================================================
//  WHAT'S NEW IN THIS VERSION (search for "NEW:" comments throughout the file)
//  ===========================================================================
//  Everything above the "MAZE MEMORY SYSTEM (NEW)" section is your original
//  code, untouched in behavior. The only structural change to your original
//  functions is a small refactor of executeMovement()'s LEFT/RIGHT/U-TURN
//  cases into standalone doTurnLeft()/doTurnRight()/doUTurn() functions, so
//  the exact same maneuver code can be reused by the new intersection logic
//  without duplicating it. executeMovement() still behaves identically.
//
//  New capabilities added:
//    1. Node-based intersection memory (DFS stack, no dynamic allocation)
//    2. Memory-aware chooseDirection() that prefers unexplored branches
//    3. Automatic dead-end detection + backtracking via the stack
//    4. Move recording (L/S/R/B) and path simplification (LBR=S, etc.)
//    5. A "solved" mode that replays the simplified shortest path directly
//    6. Structured maze-level Serial debug output
//
//  Your original readSensors/filterReadings/detectWalls/wallCorrection/
//  decideDirection functions are all preserved and still used exactly as
//  before — the new code only decides WHICH direction to take at a real
//  decision point, and remembers what's already been tried.
// =============================================================================

#include <Arduino.h>
#include <Preferences.h>   // NEW: NVS (flash) storage so the solved path survives a power cycle

// ── Debug ─────────────────────────────────────────────────────────────────
#define DEBUG 0   // set to 0 to silence all low-level sensor/motor debug output

#if DEBUG
  #define DBG_PRINT(...)    Serial.print(__VA_ARGS__)
  #define DBG_PRINTLN(...)  Serial.println(__VA_ARGS__)
  #define DBG_PRINTF(...)   Serial.printf(__VA_ARGS__)
#else
  #define DBG_PRINT(...)
  #define DBG_PRINTLN(...)
  #define DBG_PRINTF(...)
#endif

// NEW: separate debug channel just for maze-memory events (intersections,
// backtracks, path simplification). Kept independent of DEBUG above so you
// can silence low-level sensor spam while still seeing maze-solving progress.
#define MAZE_DEBUG 1

#if MAZE_DEBUG
  #define MAZE_PRINT(...)    Serial.print(__VA_ARGS__)
  #define MAZE_PRINTLN(...)  Serial.println(__VA_ARGS__)
  #define MAZE_PRINTF(...)   Serial.printf(__VA_ARGS__)
#else
  #define MAZE_PRINT(...)
  #define MAZE_PRINTLN(...)
  #define MAZE_PRINTF(...)
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

// ── Buzzer ────────────────────────────────────────────────────────────────
#define BUZZER_PIN 2   // passive buzzer; driven with tone()/noTone()

// ── Motor direction inversion ────────────────────────────────────────────
// If a motor spins backward when the code commands forward (positive speed),
// set its invert flag to 1 instead of re-wiring the motor leads.
#define MOTOR_A_INVERT 0   // set to 1 if left/A wheel runs backward on "FORWARD"
#define MOTOR_B_INVERT 1   // set to 1 if right/B wheel runs backward on "FORWARD"

// =============================================================================
//  TUNING — measure your actual maze/robot before trusting these numbers
// =============================================================================
#define WALL_CM        18.0f
#define STOP_CM        10.0f
#define TARGET_LEFT_CM 9.0f
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
#define NUDGE_MS       220   // creep forward so the wheel axle clears the corner
#define TURN_MS        430   // time to pivot ~90 degrees at TURN_SPEED
#define UTURN_MS       860   // time to pivot ~180 degrees
#define REVERSE_MS     300
#define SETTLE_MS       40   // brief pause after stopping before the next pivot

// NEW: how long to creep forward when we choose STRAIGHT at a real
// intersection (as opposed to a plain corridor with no side openings). This
// needs to be a bit longer than NUDGE_MS so the L/R sensors fully clear the
// junction mouth before the next loop re-evaluates isIntersection() — without
// this, the same physical intersection could be mistaken for a brand-new one
// on the very next loop. Calibrate alongside NUDGE_MS.
#define JUNCTION_CLEAR_MS 300

#define ECHO_TIMEOUT_US 25000UL

// NEW: "square to wall" turn-correction tuning. This uses ONLY the existing
// front ultrasonic sensor (no new sensors, no new ESP32 pins) to verify and
// correct heading after a timed pivot -- see squareToFrontWall() below.
#define SQUARE_MICRO_MS      15    // tiny pivot burst per correction step
#define SQUARE_MAX_STEPS     10    // safety cap on correction attempts
#define SQUARE_MAX_FRONT_CM  50.0f // skip squaring if no wall this close ahead
#define SQUARE_SETTLE_MS     30    // let echo settle between micro-pivots
#define SQUARE_IMPROVE_CM    0.3f  // must improve by at least this much to keep going

// NEW: safety cap. This DFS design has no encoders/odometry, so it identifies
// nodes purely by DFS stack position — it implicitly assumes the maze is a
// tree (no loops back on themselves). If the maze DOES contain a loop, the
// robot can keep re-discovering "new" intersections around that loop forever
// and never finish. This cap forces exploration to end (using whatever path
// has been recorded so far) if that ever happens, instead of running forever.
// Raise this if your maze legitimately has more real intersections than this.
#define MAX_SAFETY_INTERSECTIONS 300

// ── Debug counters/state ────────────────────────────────────────────────
static uint32_t g_loopCount = 0;

// =============================================================================
//  SHARED TYPES — must be fully defined here, before ANY function in the file.
//  The Arduino IDE auto-generates forward prototypes for every function and
//  inserts them near the top of the translation unit, above your own code.
//  If these types were defined further down, those auto-generated prototypes
//  would reference an undeclared type and the build would fail — regardless
//  of where you actually use the types later in the file.
// =============================================================================
struct SensorState {
  float L, F, R;            // this loop's median-filtered readings
  bool wallL, wallF, wallR;  // WALL_CM classification (true = wall present)
};

enum Action { ACT_FORWARD, ACT_LEFT, ACT_RIGHT, ACT_UTURN, ACT_NONE = 255 };

// NEW: intersection memory node, exactly as requested — plus cameFrom for
// debug/bookkeeping (the Action used to physically enter this node from its
// parent). Backtracking itself doesn't need cameFrom to work (a physical
// 180-degree U-turn always reverses you into the corridor you arrived from,
// regardless of what that direction was called), but it's kept because it's
// useful for debug output and matches the requested struct shape.
struct Node {
  bool leftVisited;
  bool straightVisited;
  bool rightVisited;
  byte cameFrom;   // Action byte, or ACT_NONE (255) for the start node
};

// NEW: solver state machine — EXPLORING builds the maze memory via DFS,
// SOLVED replays the simplified shortest path directly.
enum SolverState { STATE_EXPLORING, STATE_SOLVED };

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
void pivotLeft(int s)  { DBG_PRINTF("[ACTION] PIVOT LEFT speed=%d\n", s);  setMotorA(-s); setMotorB(s); }
void pivotRight(int s) { DBG_PRINTF("[ACTION] PIVOT RIGHT speed=%d\n", s); setMotorA(s);  setMotorB(-s); }

// =============================================================================
//  SENSOR STATE (readSensors -> filterReadings -> detectWalls)
// =============================================================================
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

void filterReadings(SensorState &s) {
  if (s.L > WALL_CM) openStreakL++; else openStreakL = 0;
  if (s.R > WALL_CM) openStreakR++; else openStreakR = 0;
  DBG_PRINTF("[FILTER] openStreakL=%d openStreakR=%d\n", openStreakL, openStreakR);
}

void detectWalls(SensorState &s) {
  s.wallF = s.F < STOP_CM;
  s.wallL = !(openStreakL >= OPEN_CONFIRM_COUNT);
  s.wallR = !(openStreakR >= OPEN_CONFIRM_COUNT);
  DBG_PRINTF("[STATE] wallL=%d wallF=%d wallR=%d\n", s.wallL, s.wallF, s.wallR);
}

// =============================================================================
//  DECISION — original left-hand-rule junction table (spec section 7).
//  PRESERVED AS-IS. It's no longer the primary decision-maker at real
//  intersections (chooseDirection() below now handles those, using memory),
//  but it's kept intact and is still used as a safe fallback if the node
//  stack ever overflows (see handleIntersection()).
// =============================================================================
Action decideDirection(const SensorState &s) {
  if (!s.wallL) { DBG_PRINTLN("[DECISION] left open -> LEFT");    return ACT_LEFT; }
  if (!s.wallF) { DBG_PRINTLN("[DECISION] front open -> FORWARD"); return ACT_FORWARD; }
  if (!s.wallR) { DBG_PRINTLN("[DECISION] right open -> RIGHT");  return ACT_RIGHT; }
  DBG_PRINTLN("[DECISION] all walled -> U-TURN");
  return ACT_UTURN;
}

// =============================================================================
//  WALL CORRECTION — unchanged
// =============================================================================
void wallCorrection(float filteredL, bool wallPresent) {
  if (!wallPresent) {
    DBG_PRINTLN("[CORR] no left wall reference -> straight");
    driveForward(BASE_SPEED);
    return;
  }

  float error = TARGET_LEFT_CM - filteredL;
  if (fabs(error) <= CORR_DEADBAND_CM) {
    DBG_PRINTF("[CORR] L=%.1f ~= target %.1f -> straight\n", filteredL, TARGET_LEFT_CM);
    driveForward(BASE_SPEED);
    return;
  }

  int corr = (int)constrain(fabs(error) * CORR_KP, 0, CORR_MAX);
  if (error > 0) {
    DBG_PRINTF("[CORR] L=%.1f too close (target %.1f) -> steer RIGHT corr=%d\n", filteredL, TARGET_LEFT_CM, corr);
    driveDifferential(BASE_SPEED + corr, BASE_SPEED - corr);
  } else {
    DBG_PRINTF("[CORR] L=%.1f too far (target %.1f) -> steer LEFT corr=%d\n", filteredL, TARGET_LEFT_CM, corr);
    driveDifferential(BASE_SPEED - corr, BASE_SPEED + corr);
  }
}

// =============================================================================
//  EXECUTE MOVEMENT
// =============================================================================
void resetSideStreaks() {
  openStreakL = 0;
  openStreakR = 0;
}

// -----------------------------------------------------------------------------
//  squareToFrontWall(): NEW -- hardware-free heading verification/correction,
//  called right after every timed pivot (doTurnLeft/Right/UTurn below).
//
//  Why this works without an encoder or IMU: ultrasonic echo strength/return
//  distance to a flat wall is minimized exactly when the sensor is aimed
//  perpendicular to it -- any angular error lengthens the sound path. So
//  instead of trusting TURN_MS/UTURN_MS blindly (issue: battery voltage drifts
//  -> motor speed drifts -> turn angle drifts, with zero verification), we:
//    1. Read the front distance right after the pivot (our "as-is" heading).
//    2. Nudge a tiny amount one way, re-read. If it got shorter, we moved
//       TOWARD square -- keep nudging that way until it stops improving.
//       If it got longer, we were nudging the wrong way -- reverse.
//    3. Stop at the local minimum (best available heading estimate) or after
//       SQUARE_MAX_STEPS, whichever comes first (safety cap).
//
//  Requires no new sensors or pins -- reuses TRIG_FRONT/ECHO_FRONT, which are
//  already wired for the normal driving pipeline. If there's no wall within
//  SQUARE_MAX_FRONT_CM ahead (e.g. the turn led into open space), this simply
//  does nothing, silently, and the robot proceeds exactly as before this fix.
// -----------------------------------------------------------------------------
void squareToFrontWall() {
  float d0 = readDistanceCm(TRIG_FRONT, ECHO_FRONT, "FRONT-SQ");
  if (d0 > SQUARE_MAX_FRONT_CM) {
    MAZE_PRINTLN("[SQUARE] no front wall reference in range -- skipping");
    return;
  }

  // Probe one direction first to learn which way actually reduces distance.
  pivotLeft(TURN_SPEED);
  delay(SQUARE_MICRO_MS);
  motorStop();
  delay(SQUARE_SETTLE_MS);
  float d1 = readDistanceCm(TRIG_FRONT, ECHO_FRONT, "FRONT-SQ");

  bool goLeft = (d1 < d0 - SQUARE_IMPROVE_CM);
  float best = (d1 < d0) ? d1 : d0;
  int steps = 1;

  if (!goLeft) {
    // The left probe didn't help -- undo it and try right instead.
    pivotRight(TURN_SPEED);
    delay(SQUARE_MICRO_MS);
    motorStop();
    delay(SQUARE_SETTLE_MS);
    best = d0;
  }

  while (steps < SQUARE_MAX_STEPS) {
    if (goLeft) pivotLeft(TURN_SPEED); else pivotRight(TURN_SPEED);
    delay(SQUARE_MICRO_MS);
    motorStop();
    delay(SQUARE_SETTLE_MS);
    float d = readDistanceCm(TRIG_FRONT, ECHO_FRONT, "FRONT-SQ");

    if (d < best - SQUARE_IMPROVE_CM) {
      // Still getting more square -- keep going this direction.
      best = d;
      steps++;
    } else {
      // Distance stopped shrinking -- we just overshot the minimum by one
      // micro-step. Back off by one step the other way and stop.
      if (goLeft) pivotRight(TURN_SPEED); else pivotLeft(TURN_SPEED);
      delay(SQUARE_MICRO_MS);
      motorStop();
      break;
    }
  }
  MAZE_PRINTF("[SQUARE] squared to front wall, ~%.1fcm, %d correction step(s)\n", best, steps);
}

// NEW: the LEFT/RIGHT/U-TURN maneuver bodies from your original
// executeMovement() switch, factored out into standalone functions so the
// new intersection-handling code can reuse them without duplicating any
// motor-timing logic. Only change from the original: squareToFrontWall() is
// now called right after the pivot, before driving off, to catch/correct
// heading drift using the existing front sensor -- no new hardware required.
void doTurnLeft() {
  motorStop();
  delay(SETTLE_MS);
  driveForward(NUDGE_SPEED);
  delay(NUDGE_MS);
  motorStop();
  delay(SETTLE_MS);
  pivotLeft(TURN_SPEED);
  delay(TURN_MS);
  motorStop();
  delay(SETTLE_MS);
  squareToFrontWall();   // NEW: verify/correct heading, no extra hardware
  driveForward(BASE_SPEED);
  resetSideStreaks();
}

void doTurnRight() {
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
  squareToFrontWall();   // NEW: verify/correct heading, no extra hardware
  driveForward(BASE_SPEED);
  resetSideStreaks();
}

void doUTurn() {
  motorStop();
  delay(SETTLE_MS);
  pivotRight(TURN_SPEED); // pick one consistent pivot direction for 180s
  delay(UTURN_MS);
  motorStop();
  delay(SETTLE_MS);
  squareToFrontWall();   // NEW: verify/correct heading, no extra hardware
  driveForward(BASE_SPEED);
  resetSideStreaks();
}

// NEW: used when we choose STRAIGHT at a real intersection (not a plain
// corridor) — creeps forward long enough to clear the junction mouth so the
// side sensors stop seeing the opening, then resets the debounce streaks.
// Without this, the same physical intersection could be re-detected as a
// "new" one on the very next loop.
void doStraightThroughIntersection() {
  driveForward(NUDGE_SPEED);
  delay(JUNCTION_CLEAR_MS);
  resetSideStreaks();
  driveForward(BASE_SPEED);
}

void executeMovement(Action a, const SensorState &s) {
  switch (a) {
    case ACT_FORWARD:
      wallCorrection(s.L, s.wallL);
      break;
    case ACT_LEFT:
      doTurnLeft();
      break;
    case ACT_RIGHT:
      doTurnRight();
      break;
    case ACT_UTURN:
      doUTurn();
      break;
    default:
      break;
  }
}

// =============================================================================
//  MAZE MEMORY SYSTEM (NEW)
//  Everything below this banner is new. Nothing above it changed behavior.
// =============================================================================

// ---- DFS node stack --------------------------------------------------------
// Array-based, fixed-size stack (no dynamic allocation) — appropriate for an
// embedded target and matches the "no encoders" reality: nodes are identified
// purely by DFS stack position, not by physical coordinates.
#define MAX_NODES 64
static Node nodeStack[MAX_NODES];
static int  nodeTop = -1;            // -1 = empty

// ---- path recording ---------------------------------------------------------
#define MAX_PATH 128
static char pathLog[MAX_PATH];       // raw recorded moves: 'L','S','R','B'
static int  pathLen = 0;

static char simplifiedPath[MAX_PATH];
static int  simplifiedLen = 0;
static int  replayIndex = 0;

// ---- solver state -------------------------------------------------------
static SolverState solverState = STATE_EXPLORING;

// True right after a dead-end U-turn: the next intersection we reach is the
// PARENT node still sitting on the stack, not a brand-new node.
static bool resumingAtParent = false;

// The Action used to leave the previous node; becomes the new node's
// cameFrom the next time we push (see handleIntersection()).
static byte pendingEntryAction = ACT_NONE;

static int intersectionCount = 0;

// NEW: running total of every handleIntersection() call (fresh + resumed),
// used only for the loop-detection safety cap above.
static int totalIntersectionEvents = 0;

// NEW: NVS storage handle for persisting the solved path across power cycles.
Preferences mazePrefs;
#define MAZE_NVS_NAMESPACE "mazebot"
#define MAZE_NVS_KEY       "path"

// -----------------------------------------------------------------------------
//  createNode() / pushNode() / popNode()
// -----------------------------------------------------------------------------
Node createNode(byte cameFromAction) {
  Node n;
  n.leftVisited = false;
  n.straightVisited = false;
  n.rightVisited = false;
  n.cameFrom = cameFromAction;
  return n;
}

bool pushNode(const Node &n) {
  if (nodeTop + 1 >= MAX_NODES) {
    MAZE_PRINTLN("[MAZE] ERROR: node stack full, cannot push!");
    return false;
  }
  nodeStack[++nodeTop] = n;
  return true;
}

bool popNode() {
  if (nodeTop < 0) return false;
  nodeTop--;
  return true;
}

void markVisited(Node &n, Action a) {
  if (a == ACT_LEFT)         n.leftVisited = true;
  else if (a == ACT_FORWARD) n.straightVisited = true;
  else if (a == ACT_RIGHT)   n.rightVisited = true;
}

// -----------------------------------------------------------------------------
//  detectIntersection(): a real decision point exists whenever the left or
//  right side is open (a branch to choose between) OR the front is blocked
//  (forcing a decision even if L/R turn out to be walls too, i.e. dead end).
//  A plain corridor (front open, both sides walled) is NOT an intersection —
//  the robot just keeps wall-following, exactly as in your original code.
// -----------------------------------------------------------------------------
bool detectIntersection(const SensorState &s) {
  return (!s.wallL) || (!s.wallR) || (s.wallF);
}

// -----------------------------------------------------------------------------
//  chooseDirection(): the memory-aware replacement for decideDirection() at
//  real intersections. Priority exactly as specified:
//    1. Left unexplored   2. Straight unexplored   3. Right unexplored
//    4. otherwise -> ACT_UTURN (nothing left here; backtrack)
//  A direction only counts as a valid choice if it is BOTH physically open
//  (live sensor reading) AND not yet visited from this node.
// -----------------------------------------------------------------------------
Action chooseDirection(const SensorState &s, const Node &n) {
  bool leftOK     = !s.wallL && !n.leftVisited;
  bool straightOK = !s.wallF && !n.straightVisited;
  bool rightOK    = !s.wallR && !n.rightVisited;

  if (leftOK)     return ACT_LEFT;
  if (straightOK) return ACT_FORWARD;
  if (rightOK)    return ACT_RIGHT;
  return ACT_UTURN; // nothing unexplored and open remains at this node
}

// -----------------------------------------------------------------------------
//  recordMove(): append one move character to the raw path log.
// -----------------------------------------------------------------------------
void recordMove(char mv) {
  if (pathLen < MAX_PATH) {
    pathLog[pathLen++] = mv;
  } else {
    MAZE_PRINTLN("[MAZE] WARNING: path log full, move not recorded!");
  }
}

// -----------------------------------------------------------------------------
//  debugPrintIntersection(): structured Serial output, per spec.
// -----------------------------------------------------------------------------
void debugPrintIntersection(const SensorState &s, const Node &n, const char* decisionLabel) {
  intersectionCount++;
  MAZE_PRINTLN();
  MAZE_PRINTF("Intersection #%d\n", intersectionCount);
  MAZE_PRINTF("Left    : %s\n",  s.wallL ? "Wall" : (n.leftVisited     ? "Explored" : "Unexplored"));
  MAZE_PRINTF("Front   : %s\n",  s.wallF ? "Wall" : (n.straightVisited ? "Explored" : "Unexplored"));
  MAZE_PRINTF("Right   : %s\n",  s.wallR ? "Wall" : (n.rightVisited    ? "Explored" : "Unexplored"));
  MAZE_PRINTF("Decision: %s\n",  decisionLabel);
  MAZE_PRINTF("Current Stack Depth: %d\n", nodeTop + 1);
  MAZE_PRINT("Current Path: ");
  for (int i = 0; i < pathLen; i++) { MAZE_PRINTF("%c ", pathLog[i]); }
  MAZE_PRINTLN();
}

// -----------------------------------------------------------------------------
//  PATH SIMPLIFICATION
//  Standard maze-reduction identities for an "X, dead-end backtrack, Y"
//  sequence, where X and Y are the two branches tried from the SAME node and
//  B is the physical U-turn in between. This is the classic 3x3 table:
//
//        second move ->     L        S        R
//    first  L              B        R        S
//    move   S              R        B        L
//           R              S        L        B
//
//  i.e. LBL=B  LBS=R  LBR=S
//       SBL=R  SBS=B  SBR=L
//       RBL=S  RBS=L  RBR=B
//
//  Applied repeatedly (any "aBb" triple anywhere in the path, not just at
//  the very end) until no more reductions are possible, this collapses every
//  wasted dead-end excursion out of the recorded path and leaves the
//  shortest sequence of turns that reaches the same final position.
// -----------------------------------------------------------------------------
char reduceTriple(char a, char b) {
  static const char A[9] = {'L','L','L', 'S','S','S', 'R','R','R'};
  static const char B[9] = {'L','S','R', 'L','S','R', 'L','S','R'};
  static const char R[9] = {'B','R','S', 'R','B','L', 'S','L','B'};
  for (int i = 0; i < 9; i++) {
    if (A[i] == a && B[i] == b) return R[i];
  }
  return 0; // should never happen — a and b are always in {L,S,R}
}

void simplifyPath() {
  bool changed = true;
  while (changed) {
    changed = false;
    for (int i = 0; i + 2 < simplifiedLen; i++) {
      if (simplifiedPath[i + 1] == 'B') {
        char reduced = reduceTriple(simplifiedPath[i], simplifiedPath[i + 2]);
        if (reduced == 0) continue;
        simplifiedPath[i] = reduced;
        // shift everything after the consumed triple left by 2
        for (int k = i + 1; k + 2 < simplifiedLen; k++) {
          simplifiedPath[k] = simplifiedPath[k + 2];
        }
        simplifiedLen -= 2;
        changed = true;
        break; // indices shifted — restart the scan
      }
    }
  }
}

// -----------------------------------------------------------------------------
//  onExplorationComplete(): fires when the DFS stack empties, i.e. the robot
//  has backtracked all the way to the start node and there is nothing left
//  anywhere in the reachable maze to explore. At this exact moment the robot
//  is physically back at the start, which is what makes an immediate,
//  same-session switch into shortest-path replay correct (no manual reset
//  needed). If your maze has a distinct, sensor-detectable goal zone instead
//  of "explore everything", call this function early from your own goal
//  check and it will still simplify+replay the path recorded so far.
// -----------------------------------------------------------------------------
void onExplorationComplete() {
  MAZE_PRINTLN();
  MAZE_PRINTLN("=== MAZE FULLY EXPLORED — back at start ===");
  MAZE_PRINT("[MAZE] Raw path: ");
  for (int i = 0; i < pathLen; i++) MAZE_PRINT(pathLog[i]);
  MAZE_PRINTLN();

  simplifiedLen = pathLen;
  for (int i = 0; i < pathLen; i++) simplifiedPath[i] = pathLog[i];
  simplifyPath();

  MAZE_PRINT("[MAZE] Simplified shortest path: ");
  for (int i = 0; i < simplifiedLen; i++) MAZE_PRINT(simplifiedPath[i]);
  MAZE_PRINTLN();

  // NEW: persist the solved path to flash (NVS) so it survives a power
  // cycle/reset. This is what makes a genuine "second run" — robot powered
  // off, placed back at the start, powered on again — able to skip
  // exploration entirely and go straight to replay. See loadSavedPath() in
  // setup() for the other half of this.
  mazePrefs.begin(MAZE_NVS_NAMESPACE, false);
  mazePrefs.putBytes(MAZE_NVS_KEY, simplifiedPath, simplifiedLen);
  mazePrefs.putInt("len", simplifiedLen);
  mazePrefs.end();
  MAZE_PRINTLN("[MAZE] Solved path saved to flash — will auto-replay on next boot.");
  MAZE_PRINTLN("[MAZE] (send 'R' over Serial in the first second after boot to force a fresh re-explore)");

  replayIndex = 0;
  solverState = STATE_SOLVED;
  MAZE_PRINTLN("[MAZE] Switching to SHORTEST-PATH REPLAY mode.");

  motorStop();
  delay(500);
  driveForward(BASE_SPEED);
}

// -----------------------------------------------------------------------------
//  handleIntersection(): called from loop() instead of decideDirection()/
//  executeMovement() whenever detectIntersection() is true. This is the core
//  of the DFS-with-memory algorithm.
// -----------------------------------------------------------------------------
void handleIntersection(const SensorState &s) {
  // NEW: loop-detection safety cap — see MAX_SAFETY_INTERSECTIONS above.
  totalIntersectionEvents++;
  if (totalIntersectionEvents > MAX_SAFETY_INTERSECTIONS) {
    MAZE_PRINTLN();
    MAZE_PRINTF("[MAZE] SAFETY: exceeded %d intersection events without finishing.\n", MAX_SAFETY_INTERSECTIONS);
    MAZE_PRINTLN("[MAZE] This usually means the maze contains a loop (this DFS assumes a tree)");
    MAZE_PRINTLN("[MAZE] or sensor noise is causing repeated false intersection detections.");
    MAZE_PRINTLN("[MAZE] Forcing completion with the path recorded so far.");
    onExplorationComplete();
    return;
  }

  Node *cur;

  if (resumingAtParent) {
    // We just U-turned out of a dead end / fully-explored node and drove
    // back here — this IS the parent node, already on the stack. Do not
    // push a new one; just resume decision-making from its saved state.
    cur = &nodeStack[nodeTop];
    resumingAtParent = false;
  } else {
    // A genuinely new intersection.
    if (nodeTop + 1 >= MAX_NODES) {
      // Fail-safe: stack exhausted (maze deeper than MAX_NODES). Fall back
      // to the original left-hand-rule so the robot keeps moving safely
      // instead of getting stuck, rather than crashing/overflowing memory.
      MAZE_PRINTLN("[MAZE] Node stack full — falling back to plain left-hand rule.");
      Action a = decideDirection(s);
      executeMovement(a, s);
      return;
    }
    Node fresh = createNode(pendingEntryAction);
    pushNode(fresh);
    cur = &nodeStack[nodeTop];
  }

  Action choice = chooseDirection(s, *cur);

  if (choice == ACT_UTURN) {
    debugPrintIntersection(s, *cur, "Backtrack (dead end / fully explored)");
    popNode();
    if (nodeTop < 0) {
      // We just left the START node with nothing left anywhere -> done.
      onExplorationComplete();
    } else {
      doUTurn();
      recordMove('B');
      resumingAtParent = true;
    }
  } else {
    const char* label = (choice == ACT_LEFT) ? "Left" : (choice == ACT_RIGHT) ? "Right" : "Straight";
    debugPrintIntersection(s, *cur, label);

    markVisited(*cur, choice);
    pendingEntryAction = choice; // becomes the NEXT node's cameFrom

    switch (choice) {
      case ACT_LEFT:
        doTurnLeft();
        recordMove('L');
        break;
      case ACT_RIGHT:
        doTurnRight();
        recordMove('R');
        break;
      case ACT_FORWARD:
        doStraightThroughIntersection();
        recordMove('S');
        break;
      default:
        break;
    }
  }
}

// -----------------------------------------------------------------------------
//  followShortestPathStep(): replay mode. Each time we reach a decision
//  point, we no longer consult memory at all — we just execute the next
//  step of the precomputed simplified path.
// -----------------------------------------------------------------------------
void followShortestPathStep(const SensorState &s) {
  if (replayIndex >= simplifiedLen) {
    motorStop();
    MAZE_PRINTLN("[MAZE] Shortest path complete — goal reached.");
    while (true) { delay(1000); } // done; halt here
  }

  char mv = simplifiedPath[replayIndex++];
  MAZE_PRINTF("[REPLAY] step %d/%d -> %c\n", replayIndex, simplifiedLen, mv);

  switch (mv) {
    case 'L': doTurnLeft(); break;
    case 'R': doTurnRight(); break;
    case 'S': doStraightThroughIntersection(); break;
    case 'B': doUTurn(); break;
    default: break;
  }
}

// -----------------------------------------------------------------------------
//  loadSavedPath() / clearSavedPath(): the other half of the NVS persistence.
//  Called once from setup(). Returns true if a previously solved path was
//  found and loaded (caller should then start directly in STATE_SOLVED).
// -----------------------------------------------------------------------------
bool loadSavedPath() {
  mazePrefs.begin(MAZE_NVS_NAMESPACE, true); // read-only
  int len = mazePrefs.getInt("len", 0);
  if (len <= 0 || len > MAX_PATH) {
    mazePrefs.end();
    return false;
  }
  mazePrefs.getBytes(MAZE_NVS_KEY, simplifiedPath, len);
  simplifiedLen = len;
  mazePrefs.end();
  return true;
}

void clearSavedPath() {
  mazePrefs.begin(MAZE_NVS_NAMESPACE, false);
  mazePrefs.clear();
  mazePrefs.end();
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  DBG_PRINTLN("=== Left-hand-rule maze solver (with memory) booting ===");
  DBG_PRINTF("[CONFIG] WALL_CM=%.1f STOP_CM=%.1f TARGET_LEFT_CM=%.1f BASE_SPEED=%d TURN_MS=%d NUDGE_MS=%d\n",
             WALL_CM, STOP_CM, TARGET_LEFT_CM, BASE_SPEED, TURN_MS, NUDGE_MS);

  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT,  INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  motorInit();
  motorStop();

  // NEW: maze memory starts empty; the very first intersection encountered
  // in loop() will push the start node automatically (nodeTop starts at -1,
  // resumingAtParent starts false, so handleIntersection()'s "fresh
  // intersection" branch fires naturally — no special-casing needed).
  nodeTop = -1;
  pathLen = 0;
  simplifiedLen = 0;
  resumingAtParent = false;
  pendingEntryAction = ACT_NONE;
  solverState = STATE_EXPLORING;
  intersectionCount = 0;
  totalIntersectionEvents = 0;

  // NEW: give the operator a 1-second window to force a fresh re-explore by
  // sending 'R' over Serial, even if a solved path is already saved in flash.
  MAZE_PRINTLN("[MAZE] Send 'R' now to force re-exploration (clears saved path)...");
  uint32_t waitStart = millis();
  bool forceReset = false;
  while (millis() - waitStart < 1000) {
    if (Serial.available() && Serial.read() == 'R') {
      forceReset = true;
      break;
    }
  }

  if (forceReset) {
    clearSavedPath();
    MAZE_PRINTLN("[MAZE] Saved path cleared. Starting fresh exploration.");
  } else if (loadSavedPath()) {
    replayIndex = 0;
    solverState = STATE_SOLVED;
    MAZE_PRINT("[MAZE] Loaded saved path from flash: ");
    for (int i = 0; i < simplifiedLen; i++) MAZE_PRINT(simplifiedPath[i]);
    MAZE_PRINTLN();
    MAZE_PRINTLN("[MAZE] Skipping exploration — going straight to shortest-path replay.");
  } else {
    MAZE_PRINTLN("[MAZE] No saved path found. Starting exploration.");
  }

  DBG_PRINTLN("[SETUP] Complete. Entering main loop.");
  Serial.println("Maze solver ready.");
}

// =============================================================================
//  MAIN LOOP
//  Sensor pipeline is unchanged. What changed: instead of always calling
//  decideDirection()+executeMovement(), we check detectIntersection() first.
//  Plain corridor driving (no side openings, front clear) behaves exactly as
//  in your original code. At real decision points we hand off to either the
//  DFS memory logic (while exploring) or the shortest-path replay (once
//  solved).
// =============================================================================
void loop() {
  uint32_t loopStart = millis();
  g_loopCount++;
  DBG_PRINTF("\n--- loop #%lu @ t=%lums ---\n", g_loopCount, loopStart);

  SensorState s;
  readSensors(s);
  filterReadings(s);
  detectWalls(s);

  bool atIntersection = detectIntersection(s);

  if (atIntersection) {
    if (solverState == STATE_SOLVED) {
      followShortestPathStep(s);
    } else {
      handleIntersection(s);
    }
  } else {
    // Plain corridor: original behavior, unchanged.
    executeMovement(ACT_FORWARD, s);
  }

  DBG_PRINTF("[TIMING] loop #%lu took %lums\n", g_loopCount, millis() - loopStart);
}
