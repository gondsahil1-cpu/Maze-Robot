# API Reference

Base URL: `http://<host>:8080`

All REST endpoints except `/api/auth/login`, `/api/auth/register`,
`/api/auth/forgot-password`, `/api/auth/reset-password` and `/api/health`
require `Authorization: Bearer <JWT>`.

## REST — Auth (`/api/auth`)

| Method | Path | Body | Notes |
|---|---|---|---|
| POST | `/register` | `{ username, email, password, role? }` | Creates a user (default role `viewer`) |
| POST | `/login` | `{ username, password, rememberMe? }` | Returns `{ token, user }` |
| POST | `/logout` | — | Stateless; client discards token |
| GET  | `/me` | — | Current user |
| POST | `/forgot-password` | `{ email }` | Sends reset email (or logs link if SMTP unset) |
| POST | `/reset-password` | `{ token, password }` | |

## REST — Maze (`/api/maze`)

| Method | Path | Role | Notes |
|---|---|---|---|
| GET | `/` | any | List saved mazes (no cell data) |
| GET | `/:id` | any | Full maze incl. occupancy grid |
| POST | `/` | administrator, robot | Save a completed maze; generates JSON/CSV/PNG/SVG |
| DELETE | `/:id` | administrator | |
| GET | `/:id/download/:format` | any | `format` = `json`\|`csv`\|`png`\|`svg` |

## REST — History (`/api/history`)

| Method | Path | Notes |
|---|---|---|
| GET | `/?page=&limit=` | Paginated run list |
| GET | `/:id` | Full run incl. `path` (for replay) and `logs` |

## REST — Users (`/api/users`) — administrator only

| Method | Path | Notes |
|---|---|---|
| GET | `/` | List users |
| PATCH | `/:id/role` | `{ role }` — `administrator`\|`viewer`\|`robot` |
| DELETE | `/:id` | |

## REST — Robot (`/api/robot`)

| Method | Path | Notes |
|---|---|---|
| GET | `/status` | Latest cached telemetry |
| POST | `/command` | `{ command, x?, y? }` — REST fallback; Socket.IO is preferred |

## Socket.IO (browser dashboard channel)

Connect with `auth: { token: <JWT> }`.

**Server → client events**
- `robot_status` — `{ connected, robotId }`
- `telemetry` — full telemetry object every ~100ms (position, heading,
  state, action, stats, wifiRSSI, latencyMs)
- `cell_update` — `{ x, y, n, s, e, w, deadEnd, intersection, goal }`, sent
  the instant a cell is newly sensed
- `map_update` — full frozen map + shortest path, sent once exploration
  completes
- `robot_event` — `{ event, timestamp }`; `event` ∈
  `maze_completed`, `mission_complete`, `celebration_complete`
- `run_saved` — `{ runId, mazeId }` after a run is persisted
- `command_ack` — robot's acknowledgement of a command

**Client → server events**
- `robot_command` — `(payload, callback)`; `payload = { command, x?, y? }`;
  `command` ∈ `START, STOP, PAUSE, RESUME, EMERGENCY_STOP, RESET_MAZE,
  CLEAR_MAP, CALIBRATE, RETURN_HOME, SET_GOAL`. Administrator role required;
  `callback({ ok, error })`.

## Raw WebSocket (robot ⇄ backend, path `/robot`)

This is the firmware's protocol, not typically used by browsers directly.

**Robot → server**
```jsonc
{ "type": "hello", "robotId": "robot-01", "token": "..." }
{ "type": "telemetry", "robotId": "...", "x": 3, "y": 5, "heading": "N",
  "state": "EXPLORING", "action": "...", "visitedCells": 12, "deadEnds": 2,
  "intersections": 3, "turns": 8, "cellsTraveled": 14, "totalDecisions": 9,
  "explorationPercent": 12.5, "goalFound": false, "missionComplete": false,
  "wifiRSSI": -54 }
{ "type": "cell_update", "x": 3, "y": 5, "n": true, "s": false, "e": false, "w": true,
  "deadEnd": false, "intersection": true, "goal": false }
{ "type": "map_snapshot", "cells": [...], "shortestPath": [...], "pathLength": 18, "pathTurns": 4 }
{ "type": "event", "event": "maze_completed" }
{ "type": "ack", "command": "START", "robotId": "robot-01" }
```

**Server → robot**
```jsonc
{ "command": "START" }
{ "command": "SET_GOAL", "x": 8, "y": 8 }
```
