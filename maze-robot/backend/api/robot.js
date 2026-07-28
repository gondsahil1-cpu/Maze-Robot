const express = require('express');
const { authenticate, requireRole } = require('../middleware/auth');

const router = express.Router();

// These endpoints let the dashboard send commands over plain REST as a
// fallback path; the primary control channel is Socket.IO (see
// websocket/socketHandler.js -> 'robot_command' event), which the frontend
// uses by default for lower latency.
module.exports = function (robotBridge) {
  // GET /api/robot/status
  router.get('/status', authenticate, (req, res) => {
    res.json(robotBridge.getLatestStatus());
  });

  // POST /api/robot/command  { command, x?, y? }
  router.post('/command', authenticate, requireRole('administrator'), (req, res) => {
    const { command, x, y } = req.body;
    const allowed = ['START', 'STOP', 'PAUSE', 'RESUME', 'EMERGENCY_STOP',
                      'RESET_MAZE', 'CLEAR_MAP', 'CALIBRATE', 'RETURN_HOME', 'SET_GOAL'];
    if (!allowed.includes(command)) return res.status(400).json({ error: 'Unknown command' });

    const sent = robotBridge.sendCommand({ command, x, y });
    if (!sent) return res.status(503).json({ error: 'Robot not connected' });
    res.json({ message: `Command ${command} sent` });
  });

  return router;
};
