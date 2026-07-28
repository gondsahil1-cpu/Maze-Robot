const fs = require('fs');
const path = require('path');
const { createCanvas } = require('canvas');

const EXPORT_DIR = path.join(__dirname, '..', 'exports');
if (!fs.existsSync(EXPORT_DIR)) fs.mkdirSync(EXPORT_DIR, { recursive: true });

function timestampName() {
  const d = new Date();
  const pad = (n) => String(n).padStart(2, '0');
  return `Maze_${d.getFullYear()}-${pad(d.getMonth() + 1)}-${pad(d.getDate())}_${pad(d.getHours())}-${pad(d.getMinutes())}-${pad(d.getSeconds())}`;
}

const CELL_PX = 32;

function buildSvg(mazeDoc) {
  const size = mazeDoc.gridSize;
  const w = size * CELL_PX, h = size * CELL_PX;
  let svg = `<svg xmlns="http://www.w3.org/2000/svg" width="${w}" height="${h}" viewBox="0 0 ${w} ${h}">`;
  svg += `<rect width="${w}" height="${h}" fill="#111827"/>`;

  const visited = new Set(mazeDoc.cells.map(c => `${c.x},${c.y}`));
  const pathSet = new Set((mazeDoc.shortestPath || []).map(c => `${c.x},${c.y}`));

  for (const c of mazeDoc.cells) {
    const px = c.x * CELL_PX, py = c.y * CELL_PX;
    let fill = '#1d4ed8'; // visited = blue
    if (pathSet.has(`${c.x},${c.y}`)) fill = '#dc2626'; // shortest path = red
    if (c.goal) fill = '#eab308'; // goal = gold
    svg += `<rect x="${px}" y="${py}" width="${CELL_PX}" height="${CELL_PX}" fill="${fill}" stroke="#0b1220"/>`;
  }
  // unvisited/unknown cells = gray
  for (let y = 0; y < size; y++) {
    for (let x = 0; x < size; x++) {
      if (!visited.has(`${x},${y}`)) {
        svg += `<rect x="${x * CELL_PX}" y="${y * CELL_PX}" width="${CELL_PX}" height="${CELL_PX}" fill="#374151" stroke="#0b1220"/>`;
      }
    }
  }
  // walls = black lines
  for (const c of mazeDoc.cells) {
    const px = c.x * CELL_PX, py = c.y * CELL_PX;
    if (c.n) svg += `<line x1="${px}" y1="${py}" x2="${px + CELL_PX}" y2="${py}" stroke="black" stroke-width="3"/>`;
    if (c.s) svg += `<line x1="${px}" y1="${py + CELL_PX}" x2="${px + CELL_PX}" y2="${py + CELL_PX}" stroke="black" stroke-width="3"/>`;
    if (c.e) svg += `<line x1="${px + CELL_PX}" y1="${py}" x2="${px + CELL_PX}" y2="${py + CELL_PX}" stroke="black" stroke-width="3"/>`;
    if (c.w) svg += `<line x1="${px}" y1="${py}" x2="${px}" y2="${py + CELL_PX}" stroke="black" stroke-width="3"/>`;
  }
  svg += `</svg>`;
  return svg;
}

function renderPng(mazeDoc, svgFallbackText) {
  const size = mazeDoc.gridSize;
  const w = size * CELL_PX, h = size * CELL_PX;
  const canvas = createCanvas(w, h);
  const ctx = canvas.getContext('2d');
  ctx.fillStyle = '#111827';
  ctx.fillRect(0, 0, w, h);

  const visited = new Set(mazeDoc.cells.map(c => `${c.x},${c.y}`));
  const pathSet = new Set((mazeDoc.shortestPath || []).map(c => `${c.x},${c.y}`));

  for (let y = 0; y < size; y++) {
    for (let x = 0; x < size; x++) {
      if (!visited.has(`${x},${y}`)) {
        ctx.fillStyle = '#374151';
        ctx.fillRect(x * CELL_PX, y * CELL_PX, CELL_PX, CELL_PX);
      }
    }
  }
  for (const c of mazeDoc.cells) {
    let fill = '#1d4ed8';
    if (pathSet.has(`${c.x},${c.y}`)) fill = '#dc2626';
    if (c.goal) fill = '#eab308';
    ctx.fillStyle = fill;
    ctx.fillRect(c.x * CELL_PX, c.y * CELL_PX, CELL_PX, CELL_PX);
  }
  ctx.strokeStyle = 'black';
  ctx.lineWidth = 3;
  for (const c of mazeDoc.cells) {
    const px = c.x * CELL_PX, py = c.y * CELL_PX;
    ctx.beginPath();
    if (c.n) { ctx.moveTo(px, py); ctx.lineTo(px + CELL_PX, py); }
    if (c.s) { ctx.moveTo(px, py + CELL_PX); ctx.lineTo(px + CELL_PX, py + CELL_PX); }
    if (c.e) { ctx.moveTo(px + CELL_PX, py); ctx.lineTo(px + CELL_PX, py + CELL_PX); }
    if (c.w) { ctx.moveTo(px, py); ctx.lineTo(px, py + CELL_PX); }
    ctx.stroke();
  }
  return canvas.toBuffer('image/png');
}

function buildCsv(mazeDoc) {
  const rows = ['x,y,wallN,wallS,wallE,wallW,deadEnd,intersection,goal'];
  for (const c of mazeDoc.cells) {
    rows.push([c.x, c.y, c.n, c.s, c.e, c.w, c.deadEnd, c.intersection, c.goal].join(','));
  }
  return rows.join('\n');
}

/**
 * Generates JSON, CSV, SVG and PNG files for a completed maze + run and
 * writes them to backend/exports/. Returns relative file paths for storage
 * on the Maze document.
 */
function exportMazeFiles(mazeDoc, runStats) {
  const base = timestampName();
  const jsonPath = path.join(EXPORT_DIR, `${base}.json`);
  const csvPath = path.join(EXPORT_DIR, `${base}.csv`);
  const svgPath = path.join(EXPORT_DIR, `${base}.svg`);
  const pngPath = path.join(EXPORT_DIR, `${base}.png`);

  const fullJson = {
    name: base,
    robotId: mazeDoc.robotId,
    gridSize: mazeDoc.gridSize,
    map: mazeDoc.cells,
    start: mazeDoc.start,
    goal: mazeDoc.goal,
    shortestPath: mazeDoc.shortestPath,
    pathLength: mazeDoc.pathLength,
    pathTurns: mazeDoc.pathTurns,
    statistics: runStats || {},
    generatedAt: new Date().toISOString()
  };

  fs.writeFileSync(jsonPath, JSON.stringify(fullJson, null, 2));
  fs.writeFileSync(csvPath, buildCsv(mazeDoc));
  const svg = buildSvg(mazeDoc);
  fs.writeFileSync(svgPath, svg);
  fs.writeFileSync(pngPath, renderPng(mazeDoc));

  return {
    json: `/exports/${base}.json`,
    csv: `/exports/${base}.csv`,
    svg: `/exports/${base}.svg`,
    png: `/exports/${base}.png`,
    baseName: base
  };
}

module.exports = { exportMazeFiles, EXPORT_DIR };
