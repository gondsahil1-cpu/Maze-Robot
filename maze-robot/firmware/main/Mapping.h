#pragma once
#include "config.h"

enum Heading : uint8_t { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

struct Cell {
  bool wallN = false, wallS = false, wallE = false, wallW = false;
  bool knownN = false, knownS = false, knownE = false, knownW = false; // has this wall been observed?
  bool visited = false;
  bool isDeadEnd = false;
  bool isIntersection = false;
  bool isGoal = false;
  int16_t distance = -1;     // flood-fill distance from start (or from goal, during planning)
  int8_t parentX = -1, parentY = -1; // DFS parent, for backtracking / path reconstruction
};

// Simple (x,y) stack entry used for DFS backtracking
struct CellCoord { int8_t x, y; };

class Mapping {
public:
  void init();
  Cell& at(int x, int y);
  bool inBounds(int x, int y) const;

  // Record a wall observed from cell (x,y) facing `dir` (absolute heading)
  void setWall(int x, int y, Heading dir, bool present);

  // Translate sensor readings (front/left/right, robot-relative) taken while
  // the robot faces `heading` at (x,y) into absolute-direction walls, and
  // update both this cell and the mirrored wall on the neighbour cell.
  void recordWallsFromSensors(int x, int y, Heading heading,
                               bool wallFront, bool wallLeft, bool wallRight);

  void markVisited(int x, int y);
  bool hasUnvisitedNeighbor(int x, int y) const;
  int countUnvisitedNeighbors(int x, int y, CellCoord *out /* size 4 */) const;
  bool wallBetween(int x, int y, Heading dir) const;

  int visitedCount() const;
  int totalReachableEstimate() const; // visited + frontier-adjacent unknowns
  int deadEndCount() const;
  int intersectionCount() const;

  // Flood-fill distance computation (BFS over open passages) from a source
  // cell across all *known* open connections. Used both for "is exploration
  // complete" checks and as the basis for shortest-path planning.
  void computeFloodFill(int srcX, int srcY);

  bool isFullyExplored() const; // no reachable unknown-adjacent cells remain

  int gridW() const { return GRID_SIZE; }
  int gridH() const { return GRID_SIZE; }

  Cell grid[GRID_SIZE][GRID_SIZE];

private:
  static void deltaFor(Heading dir, int &dx, int &dy);
  static Heading opposite(Heading dir);
};

extern Mapping maze;
