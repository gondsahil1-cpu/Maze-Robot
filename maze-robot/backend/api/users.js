const express = require('express');
const User = require('../models/User');
const { authenticate, requireRole } = require('../middleware/auth');

const router = express.Router();

router.get('/', authenticate, requireRole('administrator'), async (req, res) => {
  const users = await User.find().select('-password -resetPasswordToken -resetPasswordExpires');
  res.json({ users });
});

router.patch('/:id/role', authenticate, requireRole('administrator'), async (req, res) => {
  const { role } = req.body;
  if (!['administrator', 'viewer', 'robot'].includes(role)) {
    return res.status(400).json({ error: 'Invalid role' });
  }
  const user = await User.findByIdAndUpdate(req.params.id, { role }, { new: true });
  if (!user) return res.status(404).json({ error: 'User not found' });
  res.json({ user: user.toSafeObject() });
});

router.delete('/:id', authenticate, requireRole('administrator'), async (req, res) => {
  await User.findByIdAndDelete(req.params.id);
  res.json({ message: 'User deleted' });
});

module.exports = router;
