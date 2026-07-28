// Usage: node scripts/createAdmin.js <username> <email> <password>
require('dotenv').config();
const mongoose = require('mongoose');
const User = require('../models/User');

async function main() {
  const [username, email, password] = process.argv.slice(2);
  if (!username || !email || !password) {
    console.log('Usage: node scripts/createAdmin.js <username> <email> <password>');
    process.exit(1);
  }
  await mongoose.connect(process.env.MONGO_URI || 'mongodb://localhost:27017/maze_robot');
  const existing = await User.findOne({ username });
  if (existing) {
    console.log('User already exists.');
    process.exit(0);
  }
  const user = await User.create({ username, email, password, role: 'administrator' });
  console.log(`Administrator created: ${user.username} (${user.email})`);
  process.exit(0);
}

main().catch((err) => { console.error(err); process.exit(1); });
