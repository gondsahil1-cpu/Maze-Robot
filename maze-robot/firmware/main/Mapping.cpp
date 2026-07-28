#include "Mapping.h"
#include <string.h>

Mapping maze;

void Mapping::init() {
  memset(grid, 0, sizeof(grid));
  for (int y = 0; y < GRID_SIZE; y++)
    for (int x = 0; x < GRID_SIZE; x++)
      grid[y][x] = Cell();
}

bool Mapping::inBounds(int x, int y) const {
  return x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE;
}

Cell& Mapping::at(int x, int y) { return grid[y][x]; }

void Mapping::deltaFor(Heading dir, int &dx, int &dy) {
  switch (dir) {
    case NORTH: dx = 0;  dy = -1; break;
    case SOUTH: dx = 0;  dy = 1;  break;
    case EAST:  dx = 1;  dy = 0;  break;
    case WEST:  dx = -1; dy = 0;  break;
  }
}

Heading Mapping::opposite(Heading dir) {
  switch (dir) {
    case NORTH: return SOUTH;
    case SOUTH: return NORTH;
    case EAST:  return WEST;
    default:    return EAST;
  }
}

void Mapping::setWall(int x, int y, Heading dir, bool present) {
  if (!inBounds(x, y)) return;
  Cell &c = at(x, y);
  switch (dir) {
    case NORTH: c.wallN = present; c.knownN = true; break;
    case SOUTH: c.wallS = present; c.knownS = true; break;
    case EAST:  c.wallE = present; c.knownE = true; break;
    case WEST:  c.wallW = present; c.knownW = true; break;
  }
  // Mirror the wall onto the neighbouring cell (a wall is shared by both sides)
  int dx, dy; deltaFor(dir, dx, dy);
  int nx = x + dx, ny = y + dy;
  if (inBounds(nx, ny)) {
    Cell &n = at(nx, ny);
    Heading back = opposite(dir);
    switch (back) {
      case NORTH: n.wallN = present; n.knownN = true; break;
      case SOUTH: n.wallS = present; n.knownS = true; break;
      case EAST:  n.wallE = present; n.knownE = true; break;
      case WEST:  n.wallW = present; n.knownW = true; break;
    }
  }
}

void Mapping::recordWallsFromSensors(int x, int y, Heading heading,
                                      bool wallFront, bool wallLeft, bool wallRight) {
  Heading front = heading;
  Heading left  = (Heading)((heading + 3) % 4); // heading - 90
  Heading right = (Heading)((heading + 1) % 4); // heading + 90
  setWall(x, y, front, wallFront);
  setWall(x, y, left,  wallLeft);
  setWall(x, y, right, wallRight);
}

bool Mapping::wallBetween(int x, int y, Heading dir) const {
  const Cell &c = grid[y][x];
  switch (dir) {
    case NORTH: return c.wallN;
    case SOUTH: return c.wallS;
    case EAST:  return c.wallE;
    default:    return c.wallW;
  }
}

void Mapping::markVisited(int x, int y) {
  if (!inBounds(x, y)) return;
  grid[y][x].visited = true;
}

int Mapping::countUnvisitedNeighbors(int x, int y, CellCoord *out) const {
  int n = 0;
  const Cell &c = grid[y][x];
  struct { Heading dir; bool wall; int dx, dy; } dirs[4] = {
    {NORTH, c.wallN, 0, -1}, {EAST, c.wallE, 1, 0}, {SOUTH, c.wallS, 0, 1}, {WEST, c.wallW, -1, 0}
  };
  for (auto &d : dirs) {
    int nx = x + d.dx, ny = y + d.dy;
    if (!inBounds(nx, ny)) continue;
    if (d.wall) continue; // wall known present -> blocked
    if (!grid[ny][nx].visited) {
      if (out) out[n] = {(int8_t)nx, (int8_t)ny};
      n++;
    }
  }
  return n;
}

bool Mapping::hasUnvisitedNeighbor(int x, int y) const {
  CellCoord tmp[4];
  return countUnvisitedNeighbors(x, y, tmp) > 0;
}

int Mapping::visitedCount() const {
  int n = 0;
  for (int y = 0; y < GRID_SIZE; y++)
    for (int x = 0; x < GRID_SIZE; x++)
      if (grid[y][x].visited) n++;
  return n;
}

int Mapping::deadEndCount() const {
  int n = 0;
  for (int y = 0; y < GRID_SIZE; y++)
    for (int x = 0; x < GRID_SIZE; x++) {
      const Cell &c = grid[y][x];
      if (c.visited && c.isDeadEnd) n++;
    }
  return n;
}

int Mapping::intersectionCount() const {
  int n = 0;
  for (int y = 0; y < GRID_SIZE; y++)
    for (int x = 0; x < GRID_SIZE; x++)
      if (grid[y][x].visited && grid[y][x].isIntersection) n++;
  return n;
}

// BFS flood fill across all connections that are *known open* (wall known
// false). Cells whose connectivity is still unknown are simply not reached,
// which is exactly what we want for both distance-labelling and the
// "fully explored" completeness check.
void Mapping::computeFloodFill(int srcX, int srcY) {
  for (int y = 0; y < GRID_SIZE; y++)
    for (int x = 0; x < GRID_SIZE; x++)
      grid[y][x].distance = -1;

  if (!inBounds(srcX, srcY)) return;

  static CellCoord queue[GRID_SIZE * GRID_SIZE];
  int head = 0, tail = 0;
  grid[srcY][srcX].distance = 0;
  queue[tail++] = {(int8_t)srcX, (int8_t)srcY};

  while (head < tail) {
    CellCoord cur = queue[head++];
    Cell &c = grid[cur.y][cur.x];
    struct { Heading dir; bool wall; int dx, dy; } dirs[4] = {
      {NORTH, c.wallN, 0, -1}, {EAST, c.wallE, 1, 0}, {SOUTH, c.wallS, 0, 1}, {WEST, c.wallW, -1, 0}
    };
    for (auto &d : dirs) {
      int nx = cur.x + d.dx, ny = cur.y + d.dy;
      if (!inBounds(nx, ny)) continue;
      if (d.wall) continue;
      if (grid[ny][nx].distance == -1) {
        grid[ny][nx].distance = c.distance + 1;
        queue[tail++] = {(int8_t)nx, (int8_t)ny};
      }
    }
  }
}

// Maze is "fully explored" once no visited cell has a reachable, unvisited
// neighbor across a known-open (or still-unknown) wall that itself sits in
// the connected component we've already mapped. Practically: every visited
// cell's neighbors are either walled-off, out of bounds, or already visited.
bool Mapping::isFullyExplored() const {
  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      if (!grid[y][x].visited) continue;
      CellCoord tmp[4];
      if (countUnvisitedNeighbors(x, y, tmp) > 0) return false;
      // Any side with an *unknown* wall state adjacent to a visited cell
      // means we haven't actually finished probing this cell either.
      const Cell &c = grid[y][x];
      if (!c.knownN || !c.knownS || !c.knownE || !c.knownW) return false;
    }
  }
  return true;
}
