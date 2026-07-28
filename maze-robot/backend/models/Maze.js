const mongoose = require('mongoose');

const cellSchema = new mongoose.Schema({
  x: Number, y: Number,
  n: Boolean, s: Boolean, e: Boolean, w: Boolean,
  deadEnd: Boolean, intersection: Boolean, goal: Boolean
}, { _id: false });

const mazeSchema = new mongoose.Schema({
  name:        { type: String, required: true },
  robotId:     { type: String, required: true },
  gridSize:    { type: Number, default: 16 },
  cells:       [cellSchema],
  shortestPath: [{ x: Number, y: Number }],
  pathLength:  Number,
  pathTurns:   Number,
  start:       { x: Number, y: Number },
  goal:        { x: Number, y: Number },
  createdBy:   { type: mongoose.Schema.Types.ObjectId, ref: 'User' },
  createdAt:   { type: Date, default: Date.now },
  files: {
    json: String, csv: String, png: String, svg: String
  }
});

module.exports = mongoose.model('Maze', mazeSchema);
