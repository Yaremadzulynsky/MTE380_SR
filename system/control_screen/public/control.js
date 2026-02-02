const state = {
  active: false,
  pointerId: null,
  center: { x: 0, y: 0 },
  radius: 0,
  vector: { x: 0, y: 0, speed: 0 },
  lastSentAt: 0,
  sendIntervalMs: 120,
  deadzone: 0.08
};

const elements = {
  joystick: document.getElementById('joystick'),
  handle: document.getElementById('joystickHandle'),
  vectorX: document.getElementById('vectorX'),
  vectorY: document.getElementById('vectorY'),
  speed: document.getElementById('speed'),
  feedback: document.getElementById('controlFeedback'),
  connectionStatus: document.getElementById('connectionStatus'),
  lastSync: document.getElementById('lastSync')
};

initialize();

function initialize() {
  updateGeometry();
  bindEvents();
  refreshBridgeStatus(false);
  window.addEventListener('resize', updateGeometry);
}

function bindEvents() {
  elements.joystick.addEventListener('pointerdown', handlePointerDown);
  elements.joystick.addEventListener('pointermove', handlePointerMove);
  elements.joystick.addEventListener('pointerup', handlePointerUp);
  elements.joystick.addEventListener('pointercancel', handlePointerUp);
}

function updateGeometry() {
  const rect = elements.joystick.getBoundingClientRect();
  state.center = {
    x: rect.left + rect.width / 2,
    y: rect.top + rect.height / 2
  };
  state.radius = rect.width / 2;
}

function handlePointerDown(event) {
  state.active = true;
  state.pointerId = event.pointerId;
  elements.joystick.setPointerCapture(event.pointerId);
  updateFromPointer(event);
}

function handlePointerMove(event) {
  if (!state.active || event.pointerId !== state.pointerId) {
    return;
  }

  updateFromPointer(event);
}

function handlePointerUp(event) {
  if (event.pointerId !== state.pointerId) {
    return;
  }

  state.active = false;
  elements.joystick.releasePointerCapture(event.pointerId);
  state.pointerId = null;
  updateVector(0, 0);
  sendControlCommand(true);
}

function updateFromPointer(event) {
  const dx = event.clientX - state.center.x;
  const dy = event.clientY - state.center.y;
  const distance = Math.sqrt(dx * dx + dy * dy);
  const clampedDistance = Math.min(distance, state.radius);
  const angle = Math.atan2(dy, dx);
  const clampedX = Math.cos(angle) * clampedDistance;
  const clampedY = Math.sin(angle) * clampedDistance;

  moveHandle(clampedX, clampedY);
  updateVector(clampedX / state.radius, clampedY / state.radius);
  sendControlCommand(false);
}

function moveHandle(x, y) {
  elements.handle.style.transform = `translate(${x}px, ${y}px)`;
}

function updateVector(normalizedX, normalizedY) {
  const magnitude = Math.sqrt(normalizedX ** 2 + normalizedY ** 2);
  const inDeadzone = magnitude < state.deadzone;

  const x = inDeadzone ? 0 : normalizedX;
  const y = inDeadzone ? 0 : normalizedY;
  const speed = inDeadzone ? 0 : Math.min(1, magnitude);

  state.vector = { x, y, speed };
  elements.vectorX.textContent = x.toFixed(2);
  elements.vectorY.textContent = y.toFixed(2);
  elements.speed.textContent = speed.toFixed(2);

  if (inDeadzone && !state.active) {
    moveHandle(0, 0);
  }
}

async function sendControlCommand(force) {
  const now = Date.now();
  if (!force && now - state.lastSentAt < state.sendIntervalMs) {
    return;
  }

  state.lastSentAt = now;
  const payload = {
    x: Number(state.vector.x.toFixed(3)),
    y: Number(state.vector.y.toFixed(3)),
    speed: Number(state.vector.speed.toFixed(3))
  };

  if (payload.speed === 0 && payload.x === 0 && payload.y === 0) {
    elements.feedback.textContent = 'Centered. Awaiting input.';
    if (!force) {
      return;
    }
  } else {
    elements.feedback.textContent = 'Command sent.';
  }

  const result = await requestJson('/api/control', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(payload)
  });

  if (!result.ok) {
    elements.feedback.textContent = 'Failed to send command.';
    await refreshBridgeStatus(false);
    return;
  }

  await refreshBridgeStatus(true);
}

async function refreshBridgeStatus(fallbackOnline) {
  const statusResult = await requestJson('/api/status');
  const statusPayload =
    statusResult.ok && statusResult.data ? statusResult.data.status : null;
  updateBridgeStatus(statusPayload, fallbackOnline);
}

function updateBridgeStatus(statusPayload, fallbackOnline) {
  const online = statusPayload ? statusPayload.status === 'ok' : fallbackOnline;
  const mode = statusPayload && statusPayload.mode ? statusPayload.mode : null;
  const isMock = mode === 'mock';
  const spiActive = statusPayload
    ? Boolean(statusPayload.spi_active || mode === 'spi')
    : false;
  const spiInactive = statusPayload
    ? statusPayload.spi_active === false || isMock
    : false;

  let label = 'Bridge offline';
  if (online) {
    if (isMock) {
      label = 'Bridge mock';
    } else if (spiInactive) {
      label = 'Bridge online · SPI inactive';
    } else if (spiActive) {
      label = 'Bridge online · SPI active';
    } else {
      label = 'Bridge online';
    }
  }

  elements.connectionStatus.textContent = label;
  elements.connectionStatus.classList.remove('online', 'offline', 'inactive');
  elements.connectionStatus.classList.toggle('online', online && !spiInactive && !isMock);
  elements.connectionStatus.classList.toggle('inactive', online && (spiInactive || isMock));
  elements.connectionStatus.classList.toggle('offline', !online);
  elements.lastSync.textContent = `Last sync: ${new Date().toLocaleTimeString()}`;
}

async function requestJson(url, options) {
  try {
    const response = await fetch(url, options);
    const text = await response.text();
    const data = text ? safeJsonParse(text) : null;

    return {
      ok: response.ok,
      status: response.status,
      data,
      text
    };
  } catch (err) {
    return {
      ok: false,
      status: 0,
      data: null,
      text: err.message
    };
  }
}

function safeJsonParse(text) {
  try {
    return JSON.parse(text);
  } catch (err) {
    return null;
  }
}
