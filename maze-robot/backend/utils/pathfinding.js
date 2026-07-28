/**
 * A* over a sparse cell map { "x,y": {n,s,e,w,...} }.
 * Mirrors the on-robot PathPlanner so the backend can independently verify
 * or regenerate a shortest path for exports and replays.
 */
function key(x, y) { return `${x},${y}`; }

function astar(cellsByKey, start, goal) {
  const open = new Map();
  const g = new Map();
  const f = new Map();
  const cameFrom = new Map();

  const h = (x, y) => Math.abs(x - goal.x) + Math.abs(y - goal.y);

  g.set(key(start.x, start.y), 0);
  f.set(key(start.x, start.y), h(start.x, start.y));
  open.set(key(start.x, start.y), start);

  const dirs = [
    { d: 'n', dx: 0, dy: -1 },
    { d: 'e', dx: 1, dy: 0 },
    { d: 's', dx: 0, dy: 1 },
    { d: 'w', dx: -1, dy: 0 }
  ];

  while (open.size > 0) {
    let currentKey = null, currentF = Infinity, current = null;
    for (const [k, v] of open) {
      const fv = f.get(k) ?? Infinity;
      if (fv < currentF) { currentF = fv; currentKey = k; current = v; }
    }
    open.delete(currentKey);

    if (current.x === goal.x && current.y === goal.y) {
      const path = [current];
      let ck = currentKey;
      while (cameFrom.has(ck)) {
        ck = cameFrom.get(ck);
        const [x, y] = ck.split(',').map(Number);
        path.unshift({ x, y });
      }
      let turns = 0;
      for (let i = 2; i < path.length; i++) {
        const dx1 = path[i - 1].x - path[i - 2].x, dy1 = path[i - 1].y - path[i - 2].y;
        const dx2 = path[i].x - path[i - 1].x, dy2 = path[i].y - path[i - 1].y;
        if (dx1 !== dx2 || dy1 !== dy2) turns++;
      }
      return { path, length: path.length, turns, found: true };
    }

    const cell = cellsByKey[currentKey] || {};
    for (const dir of dirs) {
      if (cell[dir.d]) continue; // wall present
      const nx = current.x + dir.dx, ny = current.y + dir.dy;
      const nk = key(nx, ny);
      if (!cellsByKey[nk]) continue; // unknown/unvisited cell, not traversable
      const tentativeG = g.get(currentKey) + 1;
      if (tentativeG < (g.get(nk) ?? Infinity)) {
        cameFrom.set(nk, currentKey);
        g.set(nk, tentativeG);
        f.set(nk, tentativeG + h(nx, ny));
        open.set(nk, { x: nx, y: ny });
      }
    }
  }
  return { path: [], length: 0, turns: 0, found: false };
}

module.exports = { astar, key };
