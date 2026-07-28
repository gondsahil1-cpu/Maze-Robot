# Setup Guide

## 1. Backend

**Requirements:** Node.js 18+, MongoDB 6+ (local or Atlas).

```bash
cd backend
cp .env.example .env
```

Edit `.env`:
- `MONGO_URI` — your MongoDB connection string
- `JWT_SECRET` — a long random string
- `ROBOT_AUTH_TOKEN` — a shared secret; must match `ROBOT_AUTH_TOKEN` in
  `firmware/config.h`
- `CORS_ORIGIN` / `FRONTEND_URL` — where your frontend is served from
- `SMTP_*` — optional, for password-reset emails (if omitted, reset links
  are printed to the server console instead)

```bash
npm install
npm run start          # or: npm run dev  (nodemon, auto-restart)
```

The server listens on `PORT` (default 8080) for both the REST API,
Socket.IO, and the robot's raw WebSocket bridge (path `/robot`).

Create your first administrator account:
```bash
node scripts/createAdmin.js admin admin@example.com yourStrongPassword
```

`node-canvas` (used for PNG export) needs native build tools. On Debian/Ubuntu:
```bash
sudo apt-get install build-essential libcairo2-dev libpango1.0-dev libjpeg-dev libgif-dev librsvg2-dev
```

## 2. Firmware

**Requirements:** Arduino IDE (or `arduino-cli`) with ESP32 board support.

**Libraries** (Library Manager):
- `WebSockets` by Markus Sattler (Links2004/arduinoWebSockets)
- `ArduinoJson` by Benoit Blanchon (v6.x)

**Steps:**
1. Open `firmware/main.ino` (Arduino IDE will load the other `.h`/`.cpp`
   files in the same folder automatically).
2. Edit `firmware/config.h`:
   - `WIFI_SSID`, `WIFI_PASSWORD`
   - `WS_HOST` — your backend server's LAN IP
   - `WS_PORT` — must match the backend's `PORT` (default 8080)
   - `ROBOT_AUTH_TOKEN` — must match the backend's `.env` value
   - Pin assignments if your wiring differs from the defaults
3. **Calibrate the motion timing constants** for your robot on your maze's
   cell size: `CELL_TRAVEL_MS`, `TURN_MS`, `UTURN_MS`, `NUDGE_MS`. Start
   conservative (slower speeds, shorter cell size) and tune iteratively —
   these values are the single biggest factor in localization accuracy
   since the robot has no encoders.
4. Select your ESP32 board variant and COM port, then Upload.
5. Open the Serial Monitor at 115200 baud to watch `DBG_PRINTLN` output
   while tuning.

## 3. Frontend

No build step required — it's static HTML/CSS/JS.

**Quickest option:** open `frontend/login/index.html` directly in a browser,
or serve the folder:
```bash
cd frontend
npx serve .
```

Point it at your backend by setting the API base before the other scripts
load, e.g. add this in each page's `<head>` (or edit `frontend/js/api.js`
directly):
```html
<script>window.MAZE_API_BASE = 'http://192.168.1.100:8080';</script>
```

Then browse to `login/index.html`, sign in with the account created above,
and navigate to Dashboard / Live Maze / Controls / History / Settings.

## 4. First run checklist

1. Backend running, admin account created.
2. Robot powered on, connects to WiFi, then to the backend's `/robot`
   WebSocket (watch the Serial Monitor for `[WS] connected to backend
   bridge`).
3. Dashboard shows the connection pill go **Online**.
4. Controls page → **START** — the robot begins exploring; the Live Maze
   page fills in cell by cell.
5. On completion, the dashboard shows "Maze fully explored", the shortest
   path is computed and driven automatically, and the robot performs its
   celebration sequence.
6. Check the History page — the run and its exported map files
   (JSON/CSV/PNG/SVG) should appear.
