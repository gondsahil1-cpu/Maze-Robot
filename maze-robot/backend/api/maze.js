const express = require('express');
const path = require('path');
const Maze = require('../models/Maze');
const { authenticate, requireRole } = require('../middleware/auth');
const { exportMazeFiles } = require('../utils/mazeExporter');

const router = express.Router();

// GET /api/maze — list saved mazes
router.get('/', authenticate, async (req, res) => {
  const mazes = await Maze.find().sort({ createdAt: -1 }).select('-cells');
  res.json({ mazes });
});

// GET /api/maze/:id — full maze detail (includes cell grid)
router.get('/:id', authenticate, async (req, res) => {
  const maze = await Maze.findById(req.params.id);
  if (!maze) return res.status(404).json({ error: 'Maze not found' });
  res.json({ maze });
});

// POST /api/maze — save a completed maze (called by the robot bridge, or admin re-save)
router.post('/', authenticate, requireRole('administrator', 'robot'), async (req, res) => {
  try {
    const { robotId, gridSize, cells, shortestPath, pathLength, pathTurns, start, goal, stats } = req.body;
    const maze = await Maze.create({
      name: `Maze_${new Date().toISOString()}`,
      robotId, gridSize, cells, shortestPath, pathLength, pathTurns, start, goal,
      createdBy: req.user.id
    });
    const files = exportMazeFiles(maze, stats);
    maze.files = { json: files.json, csv: files.csv, png: files.png, svg: files.svg };
    maze.name = files.baseName;
    await maze.save();
    res.status(201).json({ maze });
  } catch (err) {
    res.status(500).json({ error: err.message });
  }
});

// DELETE /api/maze/:id
router.delete('/:id', authenticate, requireRole('administrator'), async (req, res) => {
  await Maze.findByIdAndDelete(req.params.id);
  res.json({ message: 'Maze deleted' });
});

// GET /api/maze/:id/download/:format  (json|csv|png|svg)
router.get('/:id/download/:format', authenticate, async (req, res) => {
  const maze = await Maze.findById(req.params.id);
  if (!maze || !maze.files || !maze.files[req.params.format]) {
    return res.status(404).json({ error: 'File not found' });
  }
  const filePath = path.join(__dirname, '..', maze.files[req.params.format]);
  res.download(filePath);
});

module.exports = router;
