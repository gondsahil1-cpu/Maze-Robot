#include "PathPlanner.h"
#include <string.h>

namespace {
  struct Node {
    int16_t g, f;
    int8_t parentX, parentY;
    bool open, closed;
  };
  Node nodes[GRID_SIZE][GRID_SIZE];

  int heuristic(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2); // Manhattan distance (grid moves are axis-aligned)
  }

  int findLowestOpenF(bool openSet[GRID_SIZE][GRID_SIZE]) {
    int bestF = 32000, bx = -1, by = -1;
    for (int y = 0; y < GRID_SIZE; y++)
      for (int x = 0; x < GRID_SIZE; x++)
        if (openSet[y][x] && nodes[y][x].f < bestF) {
          bestF = nodes[y][x].f; bx = x; by = y;
        }
    return (bx == -1) ? -1 : (by * GRID_SIZE + bx);
  }
}

PathResult PathPlanner::solve(Mapping &m, int sx, int sy, int gx, int gy) {
  PathResult result;
  if (!m.inBounds(sx, sy) || !m.inBounds(gx, gy)) return result;

  static bool openSet[GRID_SIZE][GRID_SIZE];
  memset(openSet, 0, sizeof(openSet));
  memset(nodes, 0, sizeof(nodes));

  nodes[sy][sx].g = 0;
  nodes[sy][sx].f = heuristic(sx, sy, gx, gy);
  nodes[sy][sx].parentX = -1;
  nodes[sy][sx].parentY = -1;
  openSet[sy][sx] = true;

  while (true) {
    int idx = findLowestOpenF(openSet);
    if (idx == -1) break; // no path
    int cx = idx % GRID_SIZE, cy = idx / GRID_SIZE;
    openSet[cy][cx] = false;
    nodes[cy][cx].closed = true;

    if (cx == gx && cy == gy) {
      // Reconstruct path
      CellCoord rev[GRID_SIZE * GRID_SIZE];
      int n = 0;
      int px = cx, py = cy;
      while (px != -1) {
        rev[n++] = {(int8_t)px, (int8_t)py};
        int npx = nodes[py][px].parentX, npy = nodes[py][px].parentY;
        px = npx; py = npy;
      }
      result.length = n;
      for (int i = 0; i < n; i++) result.path[i] = rev[n - 1 - i];
      // Count direction changes along the path
      result.turns = 0;
      for (int i = 2; i < n; i++) {
        int dx1 = result.path[i-1].x - result.path[i-2].x;
        int dy1 = result.path[i-1].y - result.path[i-2].y;
        int dx2 = result.path[i].x - result.path[i-1].x;
        int dy2 = result.path[i].y - result.path[i-1].y;
        if (dx1 != dx2 || dy1 != dy2) result.turns++;
      }
      result.found = true;
      return result;
    }

    const Cell &c = m.at(cx, cy);
    struct { bool wall; int dx, dy; } dirs[4] = {
      {c.wallN, 0, -1}, {c.wallE, 1, 0}, {c.wallS, 0, 1}, {c.wallW, -1, 0}
    };
    for (auto &d : dirs) {
      if (d.wall) continue;
      int nx = cx + d.dx, ny = cy + d.dy;
      if (!m.inBounds(nx, ny)) continue;
      if (nodes[ny][nx].closed) continue;
      int tentativeG = nodes[cy][cx].g + 1;
      if (!openSet[ny][nx] || tentativeG < nodes[ny][nx].g) {
        nodes[ny][nx].g = tentativeG;
        nodes[ny][nx].f = tentativeG + heuristic(nx, ny, gx, gy);
        nodes[ny][nx].parentX = cx;
        nodes[ny][nx].parentY = cy;
        openSet[ny][nx] = true;
      }
    }
  }
  return result; // found == false
}
