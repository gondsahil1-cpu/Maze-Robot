#include "Navigation.h"
#include "Motors.h"
#include "Sensors.h"
#include "WebSocketComm.h"
#include <math.h>

// ── Non-blocking motion primitive queue ──────────────────────────────────
enum Prim : uint8_t { P_NONE, P_SETTLE, P_PIVOT_L, P_PIVOT_R, P_DRIVE_CELL, P_REVERSE, P_STOP_ONLY };
struct MotionStep { Prim type; unsigned long durationMs; };

static MotionStep q[8];
static int qLen = 0, qIdx = 0;
static unsigned long stepStart = 0;
static bool motionBusy = false;
static unsigned long lastSafetyCheck = 0;

static int goalX = DEFAULT_GOAL_X, goalY = DEFAULT_GOAL_Y;
static int rx = START_X, ry = START_Y;
static Heading rHeading = NORTH;
static RobotState state = ST_IDLE;
static RobotStats stats;
static char currentAction[32] = "IDLE";
static CellCoord dfsStack[MAX_STACK];
static int dfsTop = 0;
static PathResult shortestPath;
static int pathIdx = 0;

// Celebration sub-sequence index
static int celebStep = 0;

static void enqueueClear() { qLen = 0; qIdx = 0; motionBusy = false; }
static void enqueue(Prim t, unsigned long dur) { if (qLen < 8) q[qLen++] = {t, dur}; }

static void startMotionQueue() {
  qIdx = 0;
  motionBusy = (qLen > 0);
  stepStart = millis();
  if (qLen > 0) {
    // apply first step's motor action immediately
    switch (q[0].type) {
      case P_SETTLE:     Motors::stop(); break;
      case P_PIVOT_L:    Motors::pivotLeft(TURN_SPEED); break;
      case P_PIVOT_R:    Motors::pivotRight(TURN_SPEED); break;
      case P_DRIVE_CELL: Motors::forward(BASE_SPEED); break;
      case P_REVERSE:    Motors::reverse(REVERSE_SPEED); break;
      case P_STOP_ONLY:  Motors::stop(); break;
      default: break;
    }
  }
}

static Heading turnDelta(Heading from, Heading to) {
  // returns relative turn: 0=straight, 1=90 right (CW), 2=180, 3=90 left (CCW)
  return (Heading)(((int)to - (int)from + 4) % 4);
}

// Build the queue of motion primitives to rotate from rHeading to targetHeading
// and then drive forward exactly one cell.
static void queueMoveToHeading(Heading targetHeading) {
  enqueueClear();
  int rel = turnDelta(rHeading, targetHeading);
  if (rel == 1) { // 90 right
    enqueue(P_SETTLE, SETTLE_MS);
    enqueue(P_PIVOT_R, TURN_MS);
    enqueue(P_SETTLE, SETTLE_MS);
  } else if (rel == 3) { // 90 left
    enqueue(P_SETTLE, SETTLE_MS);
    enqueue(P_PIVOT_L, TURN_MS);
    enqueue(P_SETTLE, SETTLE_MS);
  } else if (rel == 2) { // 180
    enqueue(P_SETTLE, SETTLE_MS);
    enqueue(P_PIVOT_R, UTURN_MS);
    enqueue(P_SETTLE, SETTLE_MS);
  }
  enqueue(P_DRIVE_CELL, CELL_TRAVEL_MS);
  enqueue(P_STOP_ONLY, 10);
  rHeading = targetHeading;
  startMotionQueue();
}

static Heading headingBetween(int x1, int y1, int x2, int y2) {
  if (x2 == x1 + 1) return EAST;
  if (x2 == x1 - 1) return WEST;
  if (y2 == y1 + 1) return SOUTH;
  return NORTH;
}

static void applyHeadingMove(Heading dir) {
  int dx = 0, dy = 0;
  switch (dir) {
    case NORTH: dy = -1; break;
    case SOUTH: dy = 1; break;
    case EAST:  dx = 1; break;
    case WEST:  dx = -1; break;
  }
  rx += dx; ry += dy;
  stats.cellsTraveled++;
}

// ── Setup ─────────────────────────────────────────────────────────────────
void Navigation::init() {
  maze.init();
  rx = START_X; ry = START_Y; rHeading = NORTH;
  state = ST_IDLE;
  dfsTop = 0;
  stats = RobotStats();
  strcpy(currentAction, "IDLE");
}

// ── Sensor/decision step, executed whenever the robot is idle at a cell ──
static void arriveAndDecide() {
  maze.markVisited(rx, ry);

  SensorReading sr = Sensors::readAll();
  maze.recordWallsFromSensors(rx, ry, rHeading, sr.wallFront, sr.wallLeft, sr.wallRight);

  if (rx == goalX && ry == goalY) {
    maze.at(rx, ry).isGoal = true;
    stats.goalFound = true;
  }

  WebSocketComm::sendCellUpdate(rx, ry); // stream this cell's walls to the dashboard immediately

  CellCoord unvisited[4];
  int nUnvisited = maze.countUnvisitedNeighbors(rx, ry, unvisited);

  int openCount = 0;
  const Cell &c = maze.at(rx, ry);
  if (!c.wallN) openCount++;
  if (!c.wallS) openCount++;
  if (!c.wallE) openCount++;
  if (!c.wallW) openCount++;
  if (openCount <= 1) maze.at(rx, ry).isDeadEnd = true;
  if (openCount >= 3) maze.at(rx, ry).isIntersection = true;

  stats.visitedCells = maze.visitedCount();
  stats.deadEnds = maze.deadEndCount();
  stats.intersections = maze.intersectionCount();
  stats.totalDecisions++;

  if (nUnvisited > 0) {
    // Priority: prefer the neighbor requiring the smallest turn (forward > left > right > back)
    int bestIdx = 0, bestCost = 99;
    for (int i = 0; i < nUnvisited; i++) {
      Heading h = headingBetween(rx, ry, unvisited[i].x, unvisited[i].y);
      int rel = turnDelta(rHeading, h);
      int cost = (rel == 0) ? 0 : (rel == 3 ? 1 : (rel == 1 ? 2 : 3));
      if (cost < bestCost) { bestCost = cost; bestIdx = i; }
    }
    if (dfsTop < MAX_STACK) dfsStack[dfsTop++] = {(int8_t)rx, (int8_t)ry};
    Heading target = headingBetween(rx, ry, unvisited[bestIdx].x, unvisited[bestIdx].y);
    snprintf(currentAction, sizeof(currentAction), "EXPLORE -> (%d,%d)", unvisited[bestIdx].x, unvisited[bestIdx].y);
    if (turnDelta(rHeading, target) != 0) stats.turns++;
    applyHeadingMove(target);
    queueMoveToHeading(target);
    return;
  }

  // Dead end / fully surrounded: backtrack via DFS stack
  if (dfsTop > 0) {
    CellCoord parent = dfsStack[--dfsTop];
    Heading target = headingBetween(rx, ry, parent.x, parent.y);
    strcpy(currentAction, "BACKTRACK");
    if (turnDelta(rHeading, target) != 0) stats.turns++;
    applyHeadingMove(target);
    queueMoveToHeading(target);
    return;
  }

  // Stack empty and nothing left to visit -> exploration finished
  maze.computeFloodFill(START_X, START_Y);
  state = ST_MAZE_COMPLETE;
  stats.explorationEndMs = millis();
  strcpy(currentAction, "MAZE_COMPLETE");
  WebSocketComm::sendEvent("maze_completed");
}

static void planShortestPath() {
  shortestPath = PathPlanner::solve(maze, START_X, START_Y, goalX, goalY);
  pathIdx = 0;
  strcpy(currentAction, "PATH_PLANNED");
  WebSocketComm::sendMapSnapshot(); // publish final frozen map + path
  state = ST_RUNNING_PATH;
  rx = START_X; ry = START_Y; rHeading = NORTH; // return to logical start for the run
}

static void followPathStep() {
  if (!shortestPath.found || pathIdx >= shortestPath.length - 1) {
    state = ST_CELEBRATING;
    celebStep = 0;
    strcpy(currentAction, "GOAL_REACHED");
    WebSocketComm::sendEvent("mission_complete");
    return;
  }
  CellCoord next = shortestPath.path[pathIdx + 1];
  Heading target = headingBetween(rx, ry, next.x, next.y);
  snprintf(currentAction, sizeof(currentAction), "RUN_PATH -> (%d,%d)", next.x, next.y);
  applyHeadingMove(target);
  pathIdx++;
  queueMoveToHeading(target);
}

// ── Celebration sequence (non-blocking, buzzer + spins + wiggle + bounce) ─
static void buzz(int freq, int durMs) {
#ifdef BUZZER_PIN
  tone(BUZZER_PIN, freq, durMs);
#endif
}

static void performCelebrationStep() {
  enqueueClear();
  switch (celebStep) {
    case 0: buzz(1046, 150); enqueue(P_PIVOT_R, 900); break;              // spin CW 360
    case 1: buzz(1318, 150); enqueue(P_PIVOT_L, 900); break;              // spin CCW 360
    case 2: buzz(1568, 100); enqueue(P_PIVOT_R, 200); enqueue(P_PIVOT_L, 400); enqueue(P_PIVOT_R, 200); break; // wiggle
    case 3: buzz(2093, 200); enqueue(P_DRIVE_CELL, 200); enqueue(P_REVERSE, 200); break; // happy bounce
    default:
      state = ST_IDLE;
      stats.missionComplete = true;
      strcpy(currentAction, "CELEBRATION_DONE");
      WebSocketComm::sendEvent("celebration_complete");
      return;
  }
  celebStep++;
  startMotionQueue();
}

// ── Motion queue tick (non-blocking) ──────────────────────────────────────
static void tickMotionQueue(unsigned long now) {
  if (!motionBusy) return;
  if (now - stepStart < q[qIdx].durationMs) return;

  qIdx++;
  if (qIdx >= qLen) {
    Motors::stop();
    motionBusy = false;
    return;
  }
  stepStart = now;
  switch (q[qIdx].type) {
    case P_SETTLE:     Motors::stop(); break;
    case P_PIVOT_L:    Motors::pivotLeft(TURN_SPEED); break;
    case P_PIVOT_R:    Motors::pivotRight(TURN_SPEED); break;
    case P_DRIVE_CELL: Motors::forward(BASE_SPEED); break;
    case P_REVERSE:    Motors::reverse(REVERSE_SPEED); break;
    case P_STOP_ONLY:  Motors::stop(); break;
    default: break;
  }
}

// ── Main tick ──────────────────────────────────────────────────────────────
void Navigation::tick(unsigned long now) {
  if (state == ST_EMERGENCY_STOP || state == ST_PAUSED) {
    Motors::stop();
    return;
  }

  // Safety: abort current motion if something suddenly appears in front
  // while driving forward (does not replace, just augments, cell-based nav).
  if (motionBusy && q[qIdx].type == P_DRIVE_CELL && now - lastSafetyCheck > 50) {
    lastSafetyCheck = now;
    float f = Sensors::readFrontFast();
    if (f < STOP_CM) {
      Motors::stop();
      motionBusy = false; // treat as arrived; next tick will re-sense at this cell
    }
  }

  tickMotionQueue(now);
  if (motionBusy) return; // still executing a primitive, nothing else to do this tick

  switch (state) {
    case ST_IDLE:
      Motors::stop();
      break;
    case ST_EXPLORING:
      arriveAndDecide();
      break;
    case ST_MAZE_COMPLETE:
      state = ST_PLANNING;
      strcpy(currentAction, "PLANNING_PATH");
      break;
    case ST_PLANNING:
      planShortestPath();
      break;
    case ST_RUNNING_PATH:
      followPathStep();
      break;
    case ST_CELEBRATING:
      performCelebrationStep();
      break;
    default:
      break;
  }
}

// ── Commands ───────────────────────────────────────────────────────────────
void Navigation::cmdStart() {
  if (state == ST_IDLE) {
    state = ST_EXPLORING;
    stats.explorationStartMs = millis();
    strcpy(currentAction, "EXPLORING");
  } else if (state == ST_PAUSED) {
    state = ST_EXPLORING;
  }
}
void Navigation::cmdStop() { state = ST_IDLE; Motors::stop(); enqueueClear(); }
void Navigation::cmdPause() { state = ST_PAUSED; Motors::stop(); }
void Navigation::cmdResume() { if (state == ST_PAUSED) state = ST_EXPLORING; }
void Navigation::cmdEmergencyStop() { state = ST_EMERGENCY_STOP; Motors::stop(); enqueueClear(); }
void Navigation::cmdResetMaze() { Navigation::init(); }
void Navigation::cmdClearMap() { maze.init(); dfsTop = 0; }
void Navigation::cmdCalibrate() { strcpy(currentAction, "CALIBRATING"); /* hook for future sensor calibration routine */ }
void Navigation::cmdReturnHome() {
  shortestPath = PathPlanner::solve(maze, rx, ry, START_X, START_Y);
  pathIdx = 0;
  goalX = START_X; goalY = START_Y;
  state = ST_RUNNING_PATH;
}
void Navigation::cmdSetGoal(int x, int y) { goalX = x; goalY = y; }

// ── Telemetry ────────────────────────────────────────────────────────────
RobotState Navigation::getState() { return state; }
const char* Navigation::getStateName() {
  switch (state) {
    case ST_IDLE: return "IDLE";
    case ST_EXPLORING: return "EXPLORING";
    case ST_MAZE_COMPLETE: return "MAZE_COMPLETE";
    case ST_PLANNING: return "PLANNING";
    case ST_RUNNING_PATH: return "RUNNING_PATH";
    case ST_CELEBRATING: return "CELEBRATING";
    case ST_PAUSED: return "PAUSED";
    case ST_EMERGENCY_STOP: return "EMERGENCY_STOP";
    default: return "ERROR";
  }
}
int Navigation::getX() { return rx; }
int Navigation::getY() { return ry; }
Heading Navigation::getHeading() { return rHeading; }
char Navigation::getHeadingChar() {
  switch (rHeading) { case NORTH: return 'N'; case SOUTH: return 'S'; case EAST: return 'E'; default: return 'W'; }
}
const char* Navigation::getCurrentAction() { return currentAction; }
const RobotStats& Navigation::getStats() { return stats; }
float Navigation::explorationPercent() {
  int total = 0;
  for (int y = 0; y < GRID_SIZE; y++)
    for (int x = 0; x < GRID_SIZE; x++)
      total++;
  return stats.visitedCells * 100.0f / total;
}
PathResult& Navigation::getShortestPath() { return shortestPath; }
