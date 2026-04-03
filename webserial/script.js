// VL53L5CX WebSerial Visualization
// Parses structured frames from the sensor and displays an NxN distance grid

// Global state
let port = null;
let reader = null;
let writer = null;
let keepReading = false;
let lineBuffer = '';

// Frame parsing state
let currentFrame = {
  resolution: 64,
  rate: 15,
  distances: [],
  status: []
};
let inFrame = false;

// DOM Elements
const connectBtn = document.getElementById('connect-btn');
const disconnectBtn = document.getElementById('disconnect-btn');
const rateSelect = document.getElementById('rate-select');
const resSelect = document.getElementById('res-select');
const statusDot = document.getElementById('status-dot');
const statusText = document.getElementById('status-text');
const currentResDisplay = document.getElementById('current-res');
const currentRateDisplay = document.getElementById('current-rate');
const distanceGrid = document.getElementById('distance-grid');
const serialLog = document.getElementById('serial-log');
const interpSelect = document.getElementById('interp-select');
const logToggle = document.getElementById('log-toggle');

// Initialize
document.addEventListener('DOMContentLoaded', () => {
  // Check Web Serial support
  if (!('serial' in navigator)) {
    statusText.textContent = 'Web Serial not supported (use Chrome/Edge)';
    connectBtn.disabled = true;
    return;
  }

  // Set up event listeners
  connectBtn.addEventListener('click', connect);
  disconnectBtn.addEventListener('click', disconnect);
  rateSelect.addEventListener('change', () => sendCommand(`RATE:${rateSelect.value}`));
  resSelect.addEventListener('change', () => sendCommand(`RES:${resSelect.value}`));
  logToggle.addEventListener('click', toggleLog);
  interpSelect.addEventListener('change', () => {
    if (currentFrame.distances.length > 0) renderFrame();
  });

  // Initialize grid with 8x8
  initializeGrid(64);
});

/**
 * Initialize the distance grid with the given resolution
 * @param {number} resolution - 16 (4x4) or 64 (8x8)
 */
function initializeGrid(resolution) {
  const size = resolution === 16 ? 4 : 8;
  distanceGrid.innerHTML = '';
  distanceGrid.style.gridTemplateColumns = `repeat(${size}, 1fr)`;
  
  for (let i = 0; i < resolution; i++) {
    const cell = document.createElement('div');
    cell.className = 'grid-cell';
    cell.id = `cell-${i}`;
    cell.textContent = '--';
    cell.style.backgroundColor = '#4a4a6a'; // Gray for no data
    distanceGrid.appendChild(cell);
  }
}

/**
 * Connect to the serial device
 */
async function connect() {
  try {
    port = await navigator.serial.requestPort();
    await port.open({ baudRate: 115200 });

    reader = port.readable.getReader();
    writer = port.writable.getWriter();

    keepReading = true;
    readLoop();

    // Update UI
    connectBtn.disabled = true;
    disconnectBtn.disabled = false;
    rateSelect.disabled = false;
    resSelect.disabled = false;
    statusDot.classList.add('connected');
    statusText.textContent = 'Connected';
    addLog('Connected to device', 'status');

  } catch (error) {
    console.error('Connection error:', error);
    addLog(`Connection failed: ${error.message}`, 'error');
    statusText.textContent = 'Connection failed';
  }
}

/**
 * Disconnect from the serial device
 */
async function disconnect() {
  keepReading = false;

  if (reader) {
    try {
      await reader.cancel();
      await reader.releaseLock();
    } catch (e) { /* ignore */ }
    reader = null;
  }

  if (writer) {
    try {
      await writer.close();
    } catch (e) { /* ignore */ }
    writer = null;
  }

  if (port) {
    try {
      await port.close();
    } catch (e) { /* ignore */ }
    port = null;
  }

  // Update UI
  connectBtn.disabled = false;
  disconnectBtn.disabled = true;
  rateSelect.disabled = true;
  resSelect.disabled = true;
  statusDot.classList.remove('connected');
  statusText.textContent = 'Disconnected';
  currentResDisplay.textContent = '--';
  currentRateDisplay.textContent = '--';
  addLog('Disconnected', 'status');
}

/**
 * Read loop for incoming serial data
 */
async function readLoop() {
  const decoder = new TextDecoder();
  
  while (port && keepReading) {
    try {
      const { value, done } = await reader.read();
      if (done) break;
      
      const text = decoder.decode(value);
      processData(text);
    } catch (error) {
      if (keepReading) {
        console.error('Read error:', error);
        addLog(`Read error: ${error.message}`, 'error');
      }
      break;
    }
  }

  if (keepReading) {
    disconnect();
  }
}

/**
 * Process incoming serial data
 * @param {string} data - Raw text from serial
 */
function processData(data) {
  lineBuffer += data;
  
  let newlineIdx;
  while ((newlineIdx = lineBuffer.indexOf('\n')) !== -1) {
    const line = lineBuffer.substring(0, newlineIdx).trim();
    lineBuffer = lineBuffer.substring(newlineIdx + 1);
    
    if (line) {
      parseLine(line);
    }
  }
}

/**
 * Parse a single line of serial data
 * @param {string} line - A complete line of text
 */
function parseLine(line) {
  // Add to log (but not every frame data line to avoid spam)
  if (!line.startsWith('D:') && !line.startsWith('S:')) {
    addLog(line);
  }

  if (line === 'FRAME_START') {
    inFrame = true;
    currentFrame = { resolution: 64, rate: 15, distances: [], status: [] };
    return;
  }

  if (line === 'FRAME_END') {
    inFrame = false;
    renderFrame();
    return;
  }

  if (!inFrame) return;

  if (line.startsWith('RES:')) {
    currentFrame.resolution = parseInt(line.substring(4), 10);
  } else if (line.startsWith('RATE:')) {
    currentFrame.rate = parseInt(line.substring(5), 10);
  } else if (line.startsWith('D:')) {
    currentFrame.distances = line.substring(2).split(',').map(v => parseInt(v, 10));
  } else if (line.startsWith('S:')) {
    currentFrame.status = line.substring(2).split(',').map(v => parseInt(v, 10));
  }

  // Handle OK responses from commands
  if (line.startsWith('OK RES:')) {
    const res = parseInt(line.substring(7), 10);
    resSelect.value = res;
    addLog(line, 'status');
  } else if (line.startsWith('OK RATE:')) {
    const rate = parseInt(line.substring(8), 10);
    rateSelect.value = rate;
    addLog(line, 'status');
  }
}

/**
 * Render the current frame to the grid, with optional bicubic interpolation
 */
function renderFrame() {
  const res = currentFrame.resolution;
  const distances = currentFrame.distances;

  // Update status displays
  currentResDisplay.textContent = res === 16 ? '4×4' : '8×8';
  currentRateDisplay.textContent = `${currentFrame.rate} Hz`;

  const srcSize = res === 16 ? 4 : 8;
  const interpValue = interpSelect.value;
  const targetSize = interpValue === 'none' ? srcSize : parseInt(interpValue, 10);

  // Build source 2D grid
  const srcGrid = [];
  for (let r = 0; r < srcSize; r++) {
    srcGrid.push(distances.slice(r * srcSize, (r + 1) * srcSize));
  }

  let displayGrid;
  let displaySize;
  if (targetSize === srcSize) {
    displayGrid = srcGrid;
    displaySize = srcSize;
  } else {
    displayGrid = bicubicInterpolate(srcGrid, srcSize, targetSize);
    displaySize = targetSize;
  }

  // Rebuild grid if size changed
  const totalCells = displaySize * displaySize;
  if (distanceGrid.children.length !== totalCells) {
    distanceGrid.innerHTML = '';
    distanceGrid.style.gridTemplateColumns = `repeat(${displaySize}, 1fr)`;
    for (let i = 0; i < totalCells; i++) {
      const cell = document.createElement('div');
      cell.className = 'grid-cell';
      cell.id = `cell-${i}`;
      cell.textContent = '--';
      cell.style.backgroundColor = '#4a4a6a';
      // Smaller cells for interpolated grids
      if (displaySize > 8) {
        cell.style.minWidth = `${Math.max(12, 320 / displaySize)}px`;
        cell.style.minHeight = `${Math.max(12, 320 / displaySize)}px`;
        cell.style.fontSize = displaySize > 16 ? '7px' : '9px';
      }
      distanceGrid.appendChild(cell);
    }
  }

  // Update cells
  for (let r = 0; r < displaySize; r++) {
    for (let c = 0; c < displaySize; c++) {
      const i = r * displaySize + c;
      const cell = document.getElementById(`cell-${i}`);
      if (!cell) continue;

      const dist = Math.round(displayGrid[r][c]);
      if (dist > 0 && dist < 4000) {
        cell.textContent = (interpValue === 'none') ? dist : '';
        cell.style.backgroundColor = distanceToColor(dist);
      } else {
        cell.textContent = (interpValue === 'none') ? '--' : '';
        cell.style.backgroundColor = '#4a4a6a';
      }
    }
  }
}

/**
 * Bicubic interpolation from srcSize×srcSize to targetSize×targetSize
 */
function bicubicInterpolate(srcGrid, srcSize, targetSize) {
  const result = [];
  for (let r = 0; r < targetSize; r++) {
    const row = [];
    for (let c = 0; c < targetSize; c++) {
      // Map target pixel to source coordinates
      const srcR = (r + 0.5) * srcSize / targetSize - 0.5;
      const srcC = (c + 0.5) * srcSize / targetSize - 0.5;
      row.push(bicubicSample(srcGrid, srcSize, srcR, srcC));
    }
    result.push(row);
  }
  return result;
}

/**
 * Sample a single point from the source grid using bicubic interpolation
 */
function bicubicSample(grid, size, r, c) {
  const ri = Math.floor(r);
  const ci = Math.floor(c);
  const dr = r - ri;
  const dc = c - ci;

  let val = 0;
  for (let m = -1; m <= 2; m++) {
    for (let n = -1; n <= 2; n++) {
      const sr = Math.min(Math.max(ri + m, 0), size - 1);
      const sc = Math.min(Math.max(ci + n, 0), size - 1);
      val += grid[sr][sc] * cubicWeight(m - dr) * cubicWeight(n - dc);
    }
  }
  return Math.max(1, Math.min(val, 3999));
}

/**
 * Cubic interpolation kernel (Catmull-Rom, a=-0.5)
 */
function cubicWeight(x) {
  const a = -0.5;
  const abs = Math.abs(x);
  if (abs <= 1) {
    return (a + 2) * abs * abs * abs - (a + 3) * abs * abs + 1;
  } else if (abs < 2) {
    return a * abs * abs * abs - 5 * a * abs * abs + 8 * a * abs - 4 * a;
  }
  return 0;
}

/**
 * Convert a distance value to an HSL color
 * Close (0mm) = red/warm, Far (2000mm+) = blue/purple
 * @param {number} dist - Distance in mm
 * @returns {string} CSS color string
 */
function distanceToColor(dist) {
  // Clamp distance to 0-2000mm range
  const maxDist = 2000;
  const clamped = Math.min(Math.max(dist, 0), maxDist);
  
  // Map to hue: 0 (red) -> 260 (purple)
  // Close = red (0), medium = green/cyan (120-180), far = blue/purple (220-260)
  const hue = (clamped / maxDist) * 260;
  
  // Vary saturation and lightness slightly for better visual
  const saturation = 70 + (clamped / maxDist) * 10; // 70-80%
  const lightness = 45 + (clamped / maxDist) * 10;  // 45-55%
  
  return `hsl(${hue}, ${saturation}%, ${lightness}%)`;
}

/**
 * Send a command to the device
 * @param {string} cmd - Command string
 */
async function sendCommand(cmd) {
  if (!writer) return;
  
  try {
    const encoder = new TextEncoder();
    await writer.write(encoder.encode(cmd + '\n'));
    addLog(`Sent: ${cmd}`, 'command');
  } catch (error) {
    console.error('Send error:', error);
    addLog(`Send failed: ${error.message}`, 'error');
  }
}

/**
 * Add a message to the serial log
 * @param {string} msg - Message text
 * @param {string} type - Log type: 'data', 'status', 'error', 'command'
 */
function addLog(msg, type = 'data') {
  const entry = document.createElement('div');
  entry.className = `log-entry ${type}`;
  entry.textContent = msg;
  serialLog.appendChild(entry);
  
  // Keep log from growing too large
  while (serialLog.children.length > 500) {
    serialLog.removeChild(serialLog.firstChild);
  }
  
  // Auto-scroll if expanded
  if (serialLog.classList.contains('expanded')) {
    serialLog.scrollTop = serialLog.scrollHeight;
  }
}

/**
 * Toggle the serial log visibility
 */
function toggleLog() {
  serialLog.classList.toggle('expanded');
  const toggleText = logToggle.querySelector('.log-toggle');
  if (serialLog.classList.contains('expanded')) {
    toggleText.textContent = '▲ Click to collapse';
    serialLog.scrollTop = serialLog.scrollHeight;
  } else {
    toggleText.textContent = '▼ Click to expand';
  }
}
