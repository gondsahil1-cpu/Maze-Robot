const mongoose = require('mongoose');

async function connectDB() {
  const uri = process.env.MONGO_URI || 'mongodb://localhost:27017/maze_robot';
  try {
    await mongoose.connect(uri);
    console.log(`[DB] Connected to MongoDB at ${uri}`);
  } catch (err) {
    console.error('[DB] Connection error:', err.message);
    process.exit(1);
  }

  mongoose.connection.on('disconnected', () => {
    console.warn('[DB] MongoDB disconnected, retrying in 5s...');
    setTimeout(connectDB, 5000);
  });
}

module.exports = connectDB;
