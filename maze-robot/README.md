# Autonomous Maze-Solving Robot — Full Ecosystem

An ESP32 robot that explores an unknown maze using only three HC-SR04
ultrasonic sensors and wheel motion, builds its own occupancy-grid map,
computes the shortest path with A*, drives it, and celebrates — all while
streaming live telemetry to a secure, real-time web dashboard.

```
maze-robot/
├── firmware/     ESP32 Arduino sketch (FSM, mapping, path planning, WebSocket)
├── backend/      Node.js + Express + Socket.IO + MongoDB API and robot bridge
├── frontend/     Dark-glassmorphism web dashboard (static HTML/CSS/JS)
└── docs/         Architecture, setup and API reference
```

## Quick start

1. **Backend** — see `docs/SETUP.md`. In short:
   ```bash
   cd backend
   cp .env.example .env   # edit MONGO_URI, JWT_SECRET, ROBOT_AUTH_TOKEN
   npm install
   npm run start
   node scripts/createAdmin.js admin admin@example.com yourpassword
   ```
2. **Firmware** — open `firmware/main.ino` in Arduino IDE (or `arduino-cli`),
   install the `WebSockets` (Links2004) and `ArduinoJson` libraries, edit the
   WiFi/server settings in `firmware/config.h`, and flash your ESP32.
3. **Frontend** — open `frontend/login/index.html` in a browser (or serve the
   `frontend/` folder with any static file server). Set `window.MAZE_API_BASE`
   in the page, or edit `frontend/js/api.js`, to point at your backend.

## Hardware (fixed)

| Component | Notes |
|---|---|
| ESP32 | Controller |
| TB6612FNG | Dual motor driver |
| 2× N20 gear motors | Drivetrain |
| 3× HC-SR04 | Front / Left / Right ultrasonic sensors |
| Buzzer | Victory melody, feedback tones |
| WiFi | Built into ESP32 |

No LiDAR, camera, or IMU — the maze is reconstructed entirely from
ultrasonic wall detection and cell-by-cell wheel motion.

## Documentation

- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) — how exploration, mapping,
  path planning and communication fit together, and the key design decisions
  (including how "goal detection" is handled without a camera).
- [`docs/SETUP.md`](docs/SETUP.md) — full install/run instructions.
- [`docs/API.md`](docs/API.md) — REST + WebSocket/Socket.IO reference.
