# Architecture

## 1. Overview

```
 ┌─────────────┐   WebSocket (100ms telemetry, JSON)   ┌──────────────────┐   Socket.IO   ┌───────────────┐
 │  ESP32       │ ───────────────────────────────────▶ │  robotBridge.js  │ ────────────▶ │  Browser       │
 │  firmware    │ ◀─────────────────────────────────── │  (raw ws server) │ ◀──────────── │  Dashboard     │
 └─────────────┘        commands + acks                └──────────────────┘   commands     └───────────────┘
                                                                  │
                                                                  ▼
                                                          MongoDB (Maze, Run, User)
                                                                  │
                                                                  ▼
                                                      REST API (auth, maze, history, users)
```

The robot never talks to MongoDB or the browser directly — it only speaks a
small JSON protocol over one WebSocket connection to the backend
(`backend/websocket/robotBridge.js`), which re-broadcasts everything to
authenticated dashboard clients via Socket.IO and persists completed runs.

## 2. Firmware modules

| Module | Responsibility |
|---|---|
| `Sensors` | Median-of-3 HC-SR04 reads; fast single-ping for safety checks |
| `Mapping` | Occupancy grid: walls (N/S/E/W), visited, dead-end, intersection, BFS flood-fill, completeness check |
| `PathPlanner` | A* over the known grid, from start to goal |
| `Navigation` | The robot's finite-state machine and all motion, described below |
| `WebSocketComm` | WiFi/WS client, 100ms telemetry cadence, command intake, map streaming |
| `Motors` | Low-level TB6612FNG driving |

### Why a discrete, cell-based robot (not continuous wall-following)?

With only three ultrasonic sensors and no encoders, the only reliable way to
localize the robot is to treat the maze as a grid of fixed-size cells and
move **one cell at a time**: turn to face the intended direction, then drive
for a calibrated duration (`CELL_TRAVEL_MS`) that's tuned to cross exactly
one cell. This is the same approach used by micromouse robots. It trades
continuous-motion smoothness for *deterministic, drift-resistant*
localization — the robot's (x, y, heading) state is authoritative because
every movement is a discrete, known transition, not an estimate integrated
from noisy sensors.

### Exploration algorithm (DFS + flood fill)

At each cell, the robot:
1. Reads all three sensors and records the front/left/right walls (mirrored
   onto the neighboring cell too, since a wall is shared).
2. Streams that cell's walls to the dashboard immediately (`cell_update`).
3. Looks for unvisited, unwalled neighbors. If any exist, it pushes the
   current cell onto a DFS backtrack stack and moves to the neighbor that
   needs the smallest turn (forward > left > right > behind).
4. If there are no unvisited neighbors (dead end), it pops the stack and
   drives back to the parent cell.
5. This repeats until the stack is empty *and* the occupancy grid reports no
   reachable unknown-adjacent cells (`Mapping::isFullyExplored()`).

A BFS flood-fill (`Mapping::computeFloodFill`) is run from the start cell
once exploration finishes, both to sanity-check reachability and because the
same open-connection graph feeds the A* planner.

### Goal detection — an honest limitation

Three ultrasonic sensors cannot visually recognize a "goal marker" the way a
camera could. The goal is therefore a **configurable target coordinate**
(`DEFAULT_GOAL_X/Y`, defaulting to the maze center — the classic micromouse
convention), which can be overridden live from the dashboard's Controls page
(`SET_GOAL` command) once a human has seen the map, or hard-coded in
`config.h` if the physical maze's goal cell is known in advance. This is
called out explicitly rather than pretending ultrasonic sensors can "see" a
goal.

### Maze completion — no timers

Completion requires **all** of:
- Every reachable cell is visited (DFS stack empty).
- `Mapping::isFullyExplored()` — no visited cell has an unvisited neighbor
  across a known-open wall, and every visited cell's four walls are known.
- The goal cell has been visited (`stats.goalFound`).

Only then does the FSM transition `EXPLORING → MAZE_COMPLETE → PLANNING →
RUNNING_PATH`.

### Non-blocking motion

`loop()` never calls `delay()`. All movement (turns, cell drives, the
celebration dance) is expressed as a queue of `MotionStep { type, durationMs
}` primitives, advanced against `millis()` inside `Navigation::tick()`. A
front-sensor safety check runs every 50ms during any forward drive and can
abort the step early.

## 3. Backend

- **Express** REST API: auth (JWT, bcrypt, forgot/reset password), maze
  CRUD + file export, run history, user management.
- **Socket.IO**: authenticated browser channel (`io.use` JWT middleware),
  broadcasts `telemetry`, `cell_update`, `map_update`, `robot_event`,
  `robot_status`, `run_saved` to a shared `dashboard` room; accepts
  `robot_command` from admins only.
- **robotBridge.js**: a plain `ws` WebSocket server (path `/robot`) that the
  firmware connects to directly — kept separate from Socket.IO because the
  ESP32 Socket.IO client footprint is heavier than a raw WS client, and the
  protocol here is small and fully under our control.
- **MongoDB** via Mongoose: `User`, `Maze` (occupancy grid + shortest path +
  exported file paths), `Run` (per-exploration stats, full movement log for
  replay, logs).
- **mazeExporter.js**: on mission completion, writes `Maze_<timestamp>.json`,
  `.csv`, `.svg` and `.png` (rendered with `node-canvas`) to `backend/exports/`,
  served statically at `/exports/...` and downloadable from the dashboard.

## 4. Frontend

Static, dependency-light (Socket.IO client from CDN only) dark-glassmorphism
dashboard:

- **Login** (`login/`): sign in, forgot/reset password, JWT stored in
  `localStorage` (Remember me) or `sessionStorage`.
- **Dashboard** (`dashboard/`): live stat cards driven entirely by the
  `telemetry` Socket.IO event.
- **Live Maze** (`maze/`): canvas renderer with pan/zoom/fullscreen/reset,
  filled in cell-by-cell via `cell_update` events, with the frozen final map
  and shortest path arriving via `map_update`.
- **Controls** (`controls/`): every button maps 1:1 to a firmware command and
  waits for the robot's `ack`; disabled entirely for the `viewer` role.
- **History** (`history/`): lists saved runs from MongoDB, links to the
  exported JSON/CSV/PNG/SVG files, and replays a run's recorded path on a
  second canvas using a scrubber.
- **Settings** (`settings/`): account info + administrator user management.

## 5. Extensibility

- Adding wheel encoders: replace the calibrated `CELL_TRAVEL_MS` timing in
  `Navigation.cpp`'s motion queue with encoder tick counts — the rest of the
  FSM (cell-based decisions) is unchanged.
- Adding an IMU: use heading feedback to correct `queueMoveToHeading`'s
  pivot durations instead of fixed `TURN_MS`/`UTURN_MS`.
- Adding LiDAR/camera: `Mapping::recordWallsFromSensors` is the single choke
  point where "what did we detect" becomes "what's in the grid" — a richer
  sensor would populate the same `Cell` struct.
- Multi-robot fleets: `robotBridge.js` currently tracks one connected robot;
  extend it to a `Map<robotId, socket>` and add `robotId` to Socket.IO room
  names.
