const mongoose = require('mongoose');

const runSchema = new mongoose.Schema({
  robotId:   { type: String, required: true },
  maze:      { type: mongoose.Schema.Types.ObjectId, ref: 'Maze' },
  startedAt: Date,
  completedAt: Date,
  explorationTimeMs: Number,
  stats: {
    visitedCells: Number,
    deadEnds: Number,
    intersections: Number,
    turns: Number,
    cellsTraveled: Number,
    totalDecisions: Number,
    pathLength: Number,
    pathTurns: Number
  },
  missionComplete: { type: Boolean, default: false },
  path: [{ x: Number, y: Number, heading: String, action: String, t: Number }], // full movement log for replay
  logs: [{ t: Number, level: String, message: String }],
  user: { type: mongoose.Schema.Types.ObjectId, ref: 'User' },
  createdAt: { type: Date, default: Date.now }
});

module.exports = mongoose.model('Run', runSchema);
