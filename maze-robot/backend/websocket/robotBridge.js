const WebSocket = require('ws');
const Maze = require('../models/Maze');
const Run = require('../models/Run');
const { exportMazeFiles } = require('../utils/mazeExporter');

/**
 * Bridges the ESP32's raw WebSocket connection (see firmware/WebSocketComm.cpp)
 * to the browser-facing Socket.IO server. One WebSocket.Server instance per
 * process; supports a single active robot connection (extendable to a map
 * of robotId -> socket for a multi-robot fleet).
 */
function createRobotBridge(io, server) {
  const wss = new WebSocket.Server({ server, path: '/robot' });

  let robotSocket = null;
  let robotId = null;
  let authenticated = false;
  let latestStatus = { connected: false };
  let currentRun = null;        // in-memory Run document being built during a live run
  let lastMapSnapshot = null;
  let lastTelemetryAt = Date.now();

  function broadcast(event, payload) {
    io.to('dashboard').emit(event, payload);
  }

  async function finalizeRun(missionComplete) {
    if (!currentRun) return;
    currentRun.completedAt = new Date();
    currentRun.explorationTimeMs = currentRun.completedAt - currentRun.startedAt;
    currentRun.missionComplete = missionComplete;

    let savedMaze = null;
    if (lastMapSnapshot) {
      const mazeDoc = await Maze.create({
        name: `Maze_${new Date().toISOString()}`,
        robotId,
        gridSize: 16,
        cells: lastMapSnapshot.cells,
        shortestPath: lastMapSnapshot.shortestPath,
        pathLength: lastMapSnapshot.pathLength,
        pathTurns: lastMapSnapshot.pathTurns,
        start: { x: 0, y: 0 },
        goal: latestStatus.goal || { x: 8, y: 8 }
      });
      const files = exportMazeFiles(mazeDoc, currentRun.stats);
      mazeDoc.files = { json: files.json, csv: files.csv, png: files.png, svg: files.svg };
      mazeDoc.name = files.baseName;
      await mazeDoc.save();
      savedMaze = mazeDoc;
      currentRun.maze = mazeDoc._id;
    }

    const runDoc = await Run.create(currentRun);
    broadcast('run_saved', { runId: runDoc._id, mazeId: savedMaze?._id });
    currentRun = null;
    lastMapSnapshot = null;
  }

  wss.on('connection', (socket) => {
    console.log('[ROBOT-BRIDGE] incoming connection, awaiting hello/auth...');

    socket.on('message', async (raw) => {
      let msg;
      try { msg = JSON.parse(raw.toString()); } catch { return; }

      if (msg.type === 'hello') {
        if (msg.token !== process.env.ROBOT_AUTH_TOKEN) {
          console.warn('[ROBOT-BRIDGE] auth failed, closing connection');
          socket.close();
          return;
        }
        authenticated = true;
        robotSocket = socket;
        robotId = msg.robotId;
        latestStatus.connected = true;
        latestStatus.robotId = robotId;
        currentRun = {
          robotId,
          startedAt: new Date(),
          path: [],
          logs: [{ t: Date.now(), level: 'info', message: 'Robot connected, run started' }],
          stats: {}
        };
        broadcast('robot_status', { connected: true, robotId });
        console.log(`[ROBOT-BRIDGE] robot '${robotId}' authenticated`);
        return;
      }

      if (!authenticated) return; // ignore anything before hello/auth

      const now = Date.now();
      const latencyMs = now - lastTelemetryAt;
      lastTelemetryAt = now;

      if (msg.type === 'telemetry') {
        latestStatus = { ...latestStatus, ...msg, connected: true, latencyMs };
        if (currentRun) {
          currentRun.path.push({ x: msg.x, y: msg.y, heading: msg.heading, action: msg.action, t: now });
          currentRun.stats = {
            visitedCells: msg.visitedCells,
            deadEnds: msg.deadEnds,
            intersections: msg.intersections,
            turns: msg.turns,
            cellsTraveled: msg.cellsTraveled
          };
        }
        broadcast('telemetry', latestStatus);
      } else if (msg.type === 'event') {
        broadcast('robot_event', { event: msg.event, timestamp: now });
        if (currentRun) {
          currentRun.logs.push({ t: now, level: 'info', message: `Event: ${msg.event}` });
        }
        if (msg.event === 'mission_complete') {
          // final map + path should already have arrived via map_snapshot
        }
        if (msg.event === 'celebration_complete') {
          await finalizeRun(true);
        }
      } else if (msg.type === 'cell_update') {
        if (!lastMapSnapshot) lastMapSnapshot = { cells: [], shortestPath: [] };
        const idx = lastMapSnapshot.cells.findIndex(c => c.x === msg.x && c.y === msg.y);
        const cellData = { x: msg.x, y: msg.y, n: msg.n, s: msg.s, e: msg.e, w: msg.w,
                            deadEnd: msg.deadEnd, intersection: msg.intersection, goal: msg.goal };
        if (idx >= 0) lastMapSnapshot.cells[idx] = cellData; else lastMapSnapshot.cells.push(cellData);
        broadcast('cell_update', cellData);
      } else if (msg.type === 'map_snapshot') {
        lastMapSnapshot = msg;
        broadcast('map_update', msg);
      } else if (msg.type === 'ack') {
        broadcast('command_ack', msg);
      }
    });

    socket.on('close', () => {
      console.log('[ROBOT-BRIDGE] robot disconnected');
      if (robotSocket === socket) {
        robotSocket = null;
        authenticated = false;
        latestStatus.connected = false;
        broadcast('robot_status', { connected: false, robotId });
      }
    });

    socket.on('error', (err) => console.error('[ROBOT-BRIDGE] socket error:', err.message));
  });

  return {
    sendCommand(cmdObj) {
      if (!robotSocket || robotSocket.readyState !== WebSocket.OPEN) return false;
      robotSocket.send(JSON.stringify(cmdObj));
      return true;
    },
    getLatestStatus() { return latestStatus; },
    getLatestMapSnapshot() { return lastMapSnapshot || { cells: [], shortestPath: [] }; },
    isRobotConnected() { return !!robotSocket && authenticated; }
  };
}

module.exports = createRobotBridge;
