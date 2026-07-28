// Requires https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.7.5/socket.io.min.js loaded first.
function connectSocket() {
  const token = Auth.getToken();
  const socket = io(API_BASE, { auth: { token } });

  socket.on('connect_error', (err) => console.warn('[socket] connect error:', err.message));
  return socket;
}

function sendRobotCommand(socket, command, extra = {}) {
  return new Promise((resolve, reject) => {
    socket.emit('robot_command', { command, ...extra }, (ack) => {
      if (ack?.ok) resolve(ack); else reject(new Error(ack?.error || 'Command failed'));
    });
  });
}

function showToast(message, kind = 'info') {
  const el = document.createElement('div');
  el.className = 'toast glass';
  el.style.borderColor = kind === 'error' ? '#ef4444' : (kind === 'success' ? '#22c55e' : 'var(--border-glass)');
  el.textContent = message;
  document.body.appendChild(el);
  setTimeout(() => el.remove(), 3500);
}

window.connectSocket = connectSocket;
window.sendRobotCommand = sendRobotCommand;
window.showToast = showToast;
