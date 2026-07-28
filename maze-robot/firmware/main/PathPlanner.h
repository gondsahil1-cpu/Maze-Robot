#pragma once
#include "Mapping.h"

struct PathResult {
  CellCoord path[GRID_SIZE * GRID_SIZE];
  int length = 0;      // number of cells in path (including start & goal)
  int turns = 0;
  bool found = false;
};

namespace PathPlanner {
  // A* over the known occupancy grid, from (sx,sy) to (gx,gy).
  PathResult solve(Mapping &m, int sx, int sy, int gx, int gy);
}
