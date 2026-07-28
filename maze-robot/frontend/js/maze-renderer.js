class MazeRenderer {
  constructor(canvas, gridSize = 16) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.gridSize = gridSize;
    this.cellPx = 34;
    this.scale = 1;
    this.offsetX = 0;
    this.offsetY = 0;
    this.cells = new Map();     // "x,y" -> cell data
    this.shortestPath = [];
    this.robot = { x: 0, y: 0, heading: 'N' };
    this.goal = { x: 8, y: 8 };
    this.start = { x: 0, y: 0 };

    this._bindInteraction();
    this._resize();
    window.addEventListener('resize', () => this._resize());
    this._loop();
  }

  _resize() {
    const rect = this.canvas.parentElement.getBoundingClientRect();
    this.canvas.width = rect.width;
    this.canvas.height = rect.height;
  }

  _bindInteraction() {
    let dragging = false, lastX = 0, lastY = 0;
    this.canvas.addEventListener('mousedown', (e) => { dragging = true; lastX = e.clientX; lastY = e.clientY; });
    window.addEventListener('mouseup', () => dragging = false);
    window.addEventListener('mousemove', (e) => {
      if (!dragging) return;
      this.offsetX += e.clientX - lastX;
      this.offsetY += e.clientY - lastY;
      lastX = e.clientX; lastY = e.clientY;
    });
    this.canvas.addEventListener('wheel', (e) => {
      e.preventDefault();
      const delta = -e.deltaY * 0.001;
      this.zoom(1 + delta);
    }, { passive: false });
  }

  zoom(factor) { this.scale = Math.min(3, Math.max(0.4, this.scale * factor)); }
  resetView() { this.scale = 1; this.offsetX = 0; this.offsetY = 0; }
  toggleFullscreen() {
    const wrap = this.canvas.parentElement;
    if (!document.fullscreenElement) wrap.requestFullscreen?.(); else document.exitFullscreen?.();
  }

  setMapSnapshot(msg) {
    for (const c of msg.cells) this.cells.set(`${c.x},${c.y}`, c);
    this.shortestPath = msg.shortestPath || [];
  }

  updateCell(c) { this.cells.set(`${c.x},${c.y}`, c); }
  updateRobot(t) { this.robot = { x: t.x, y: t.y, heading: t.heading }; }
  setGoal(x, y) { this.goal = { x, y }; }

  _loop() {
    this._draw();
    requestAnimationFrame(() => this._loop());
  }

  _draw() {
    const { ctx, canvas, cellPx, scale } = this;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.save();
    ctx.translate(canvas.width / 2 + this.offsetX, canvas.height / 2 + this.offsetY);
    ctx.scale(scale, scale);
    const gridPx = this.gridSize * cellPx;
    ctx.translate(-gridPx / 2, -gridPx / 2);

    const pathSet = new Set(this.shortestPath.map(p => `${p.x},${p.y}`));

    for (let y = 0; y < this.gridSize; y++) {
      for (let x = 0; x < this.gridSize; x++) {
        const c = this.cells.get(`${x},${y}`);
        const px = x * cellPx, py = y * cellPx;
        let fill = getComputedStyle(document.documentElement).getPropertyValue('--cell-unknown').trim();
        if (c) fill = getComputedStyle(document.documentElement).getPropertyValue('--cell-visited').trim();
        if (pathSet.has(`${x},${y}`)) fill = getComputedStyle(document.documentElement).getPropertyValue('--cell-path').trim();
        if (c && c.goal) fill = getComputedStyle(document.documentElement).getPropertyValue('--cell-goal').trim();
        ctx.fillStyle = fill;
        ctx.fillRect(px + 1, py + 1, cellPx - 2, cellPx - 2);

        if (c && c.deadEnd) {
          ctx.fillStyle = 'rgba(0,0,0,0.25)';
          ctx.beginPath(); ctx.arc(px + cellPx / 2, py + cellPx / 2, 3, 0, 7); ctx.fill();
        }
      }
    }

    // walls
    ctx.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue('--cell-wall').trim();
    ctx.lineWidth = 3;
    for (const [key, c] of this.cells) {
      const px = c.x * cellPx, py = c.y * cellPx;
      ctx.beginPath();
      if (c.n) { ctx.moveTo(px, py); ctx.lineTo(px + cellPx, py); }
      if (c.s) { ctx.moveTo(px, py + cellPx); ctx.lineTo(px + cellPx, py + cellPx); }
      if (c.e) { ctx.moveTo(px + cellPx, py); ctx.lineTo(px + cellPx, py + cellPx); }
      if (c.w) { ctx.moveTo(px, py); ctx.lineTo(px, py + cellPx); }
      ctx.stroke();
    }

    // start marker
    ctx.fillStyle = '#94a3b8';
    ctx.font = '10px sans-serif';
    ctx.fillText('START', this.start.x * cellPx + 4, this.start.y * cellPx + 14);

    // robot icon + heading arrow
    const rx = this.robot.x * cellPx + cellPx / 2;
    const ry = this.robot.y * cellPx + cellPx / 2;
    ctx.fillStyle = getComputedStyle(document.documentElement).getPropertyValue('--cell-robot').trim();
    ctx.beginPath(); ctx.arc(rx, ry, cellPx * 0.28, 0, Math.PI * 2); ctx.fill();

    const angles = { N: -Math.PI / 2, E: 0, S: Math.PI / 2, W: Math.PI };
    const ang = angles[this.robot.heading] ?? -Math.PI / 2;
    ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(rx, ry);
    ctx.lineTo(rx + Math.cos(ang) * cellPx * 0.42, ry + Math.sin(ang) * cellPx * 0.42);
    ctx.stroke();

    ctx.restore();
  }
}
window.MazeRenderer = MazeRenderer;
