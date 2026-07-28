// Configure this to point at your backend deployment.
const API_BASE = window.MAZE_API_BASE || 'http://localhost:8080';

const Auth = {
  getToken() { return localStorage.getItem('mr_token') || sessionStorage.getItem('mr_token'); },
  getUser() {
    try { return JSON.parse(localStorage.getItem('mr_user') || sessionStorage.getItem('mr_user') || 'null'); }
    catch { return null; }
  },
  setSession(token, user, remember) {
    const store = remember ? localStorage : sessionStorage;
    store.setItem('mr_token', token);
    store.setItem('mr_user', JSON.stringify(user));
  },
  clear() {
    localStorage.removeItem('mr_token'); localStorage.removeItem('mr_user');
    sessionStorage.removeItem('mr_token'); sessionStorage.removeItem('mr_user');
  },
  requireAuth() {
    if (!this.getToken()) window.location.href = '/login/index.html';
  },
  isAdmin() { return this.getUser()?.role === 'administrator'; }
};

async function apiFetch(path, options = {}) {
  const token = Auth.getToken();
  const headers = { 'Content-Type': 'application/json', ...(options.headers || {}) };
  if (token) headers.Authorization = `Bearer ${token}`;

  const res = await fetch(`${API_BASE}${path}`, { ...options, headers });
  if (res.status === 401) {
    Auth.clear();
    window.location.href = '/login/index.html';
    throw new Error('Unauthorized');
  }
  const data = await res.json().catch(() => ({}));
  if (!res.ok) throw new Error(data.error || `Request failed (${res.status})`);
  return data;
}

window.Auth = Auth;
window.apiFetch = apiFetch;
window.API_BASE = API_BASE;
