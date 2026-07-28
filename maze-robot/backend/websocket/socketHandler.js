const jwt = require('jsonwebtoken');

/**
 * Wires up Socket.IO for browser dashboard clients: authenticates the
 * connection with the same JWT used for REST, joins a shared 'dashboard'
 * room (all connected viewers see the same live stream), and forwards
 * control commands to the robot via robotBridge.
 */
function attachSocketHandlers(io, robotBridge) {
  io.use((socket, next) => {
    const token = socket.handshake.auth?.token;
    if (!token) return next(new Error('Authentication required'));
    try {
      socket.user = jwt.verify(token, process.env.JWT_SECRET);
      next();
    } catch {
      next(new Error('Invalid token'));
    }
  });

  io.on('connection', (socket) => {
    socket.join('dashboard');
    socket.emit('robot_status', robotBridge.getLatestStatus());
    socket.emit('map_update', robotBridge.getLatestMapSnapshot());
    console.log(`[SOCKET.IO] client connected: ${socket.user.username} (${socket.user.role})`);

    socket.on('robot_command', (payload, cb) => {
      if (socket.user.role !== 'administrator') {
        cb?.({ ok: false, error: 'Only administrators can control the robot' });
        return;
      }
      const sent = robotBridge.sendCommand(payload);
      cb?.({ ok: sent, error: sent ? null : 'Robot not connected' });
    });

    socket.on('disconnect', () => {
      console.log(`[SOCKET.IO] client disconnected: ${socket.user.username}`);
    });
  });
}

module.exports = attachSocketHandlers;
