require('dotenv').config();
const express = require('express');
const http = require('http');
const path = require('path');
const cors = require('cors');
const helmet = require('helmet');
const rateLimit = require('express-rate-limit');
const { Server } = require('socket.io');

const connectDB = require('./config/db');
const authRoutes = require('./api/auth');
const mazeRoutes = require('./api/maze');
const historyRoutes = require('./api/history');
const usersRoutes = require('./api/users');
const robotRoutesFactory = require('./api/robot');
const createRobotBridge = require('./websocket/robotBridge');
const attachSocketHandlers = require('./websocket/socketHandler');

const app = express();
const server = http.createServer(app);
const io = new Server(server, {
  cors: { origin: process.env.CORS_ORIGIN || '*', methods: ['GET', 'POST'] }
});

// ── Middleware ─────────────────────────────────────────────────────────────
app.use(helmet({ crossOriginResourcePolicy: false }));
app.use(cors({ origin: process.env.CORS_ORIGIN || '*' }));
app.use(express.json());
app.use('/exports', express.static(path.join(__dirname, 'exports')));

const apiLimiter = rateLimit({ windowMs: 15 * 60 * 1000, max: 300 });
app.use('/api', apiLimiter);

// ── DB ───────────────────────────────────────────────────────────────────
connectDB();

// ── Robot <-> Browser bridge ───────────────────────────────────────────────
const robotBridge = createRobotBridge(io, server);
attachSocketHandlers(io, robotBridge);

// ── REST API ────────────────────────────────────────────────────────────────
app.use('/api/auth', authRoutes);
app.use('/api/maze', mazeRoutes);
app.use('/api/history', historyRoutes);
app.use('/api/users', usersRoutes);
app.use('/api/robot', robotRoutesFactory(robotBridge));

app.get('/api/health', (req, res) => {
  res.json({ status: 'ok', robotConnected: robotBridge.isRobotConnected(), time: new Date() });
});

// ── Error handling ─────────────────────────────────────────────────────────
app.use((err, req, res, next) => {
  console.error('[SERVER ERROR]', err);
  res.status(500).json({ error: 'Internal server error' });
});

const PORT = process.env.PORT || 8080;
server.listen(PORT, () => {
  console.log(`[SERVER] REST + Socket.IO listening on port ${PORT}`);
  console.log(`[SERVER] Robot WebSocket bridge listening on same port at path /robot`);
});
console.log("CORS_ORIGIN =", process.env.CORS_ORIGIN);

module.exports = { app, server, io };
