const express = require('express');
const Run = require('../models/Run');
const { authenticate } = require('../middleware/auth');

const router = express.Router();

// GET /api/history — list all runs (paginated)
router.get('/', authenticate, async (req, res) => {
  const page = parseInt(req.query.page) || 1;
  const limit = parseInt(req.query.limit) || 20;
  const runs = await Run.find()
    .populate('maze', 'name createdAt files')
    .populate('user', 'username')
    .sort({ createdAt: -1 })
    .skip((page - 1) * limit)
    .limit(limit)
    .select('-path -logs');
  const total = await Run.countDocuments();
  res.json({ runs, total, page, pages: Math.ceil(total / limit) });
});

// GET /api/history/:id — full run detail, including path for replay
router.get('/:id', authenticate, async (req, res) => {
  const run = await Run.findById(req.params.id).populate('maze').populate('user', 'username');
  if (!run) return res.status(404).json({ error: 'Run not found' });
  res.json({ run });
});

module.exports = router;
