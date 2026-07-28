#pragma once
#include "Mapping.h"
#include "PathPlanner.h"

enum RobotState : uint8_t {
  ST_IDLE, ST_EXPLORING, ST_MAZE_COMPLETE, ST_PLANNING,
  ST_RUNNING_PATH, ST_CELEBRATING, ST_PAUSED, ST_EMERGENCY_STOP, ST_ERROR
};

struct RobotStats {
  int visitedCells = 0;
  int deadEnds = 0;
  int intersections = 0;
  int turns = 0;
  int cellsTraveled = 0;   // distance travelled, in grid cells
  int totalDecisions = 0;
  unsigned long explorationStartMs = 0;
  unsigned long explorationEndMs = 0;
  bool goalFound = false;
  bool missionComplete = false;
};

namespace Navigation {
  void init();
  void tick(unsigned long now);

  // Commands (from dashboard, via WebSocketComm)
  void cmdStart();
  void cmdStop();
  void cmdPause();
  void cmdResume();
  void cmdEmergencyStop();
  void cmdResetMaze();
  void cmdClearMap();
  void cmdCalibrate();
  void cmdReturnHome();
  void cmdSetGoal(int x, int y);

  // Telemetry accessors
  RobotState getState();
  const char* getStateName();
  int getX();
  int getY();
  Heading getHeading();
  char getHeadingChar();
  const char* getCurrentAction();
  const RobotStats& getStats();
  float explorationPercent();
  PathResult& getShortestPath();
}
