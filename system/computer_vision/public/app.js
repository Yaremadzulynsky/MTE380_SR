const state = {
  safe_zone: buildVectorState(),
  danger_zone: buildVectorState(),
  target: buildVectorState(),
  line: buildVectorState(),
  retrieving: false,
  placing: false,
  pick_up_success: false,
  place_success: false,
  failed_pickup: false,
  at_home: false
};

const diagnostic = {
  enabled: false,
  pendingTraceId: null
};

const elements = {
  sendBtn: document.getElementById('sendBtn'),
  autoSend: document.getElementById('autoSend'),
  traceToggle: document.getElementById('traceToggle'),
  intervalInput: document.getElementById('intervalInput'),
  payloadPreview: document.getElementById('payloadPreview'),
  status: document.getElementById('stateMachineStatus'),
  lastSent: document.getElementById('lastSent'),
  lastTrace: document.getElementById('lastTrace')
};

let autoSendTimer = null;
const vectorControls = {};

initialize();

async function initialize() {
  bindVectorCards();
  bindFlags();
  await loadState();
  updatePreview();
  elements.sendBtn.addEventListener('click', () => sendPayload());
  elements.autoSend.addEventListener('change', handleAutoSendToggle);
  elements.traceToggle.addEventListener('change', handleTraceToggle);
  elements.intervalInput.addEventListener('change', restartAutoSend);
}

function buildVectorState() {
  return { detected: false, vector: { x: 0, y: 0 } };
}

function bindVectorCards() {
  document.querySelectorAll('.vector-card').forEach((card) => {
    const key = card.dataset.vector;
    const detectedInput = card.querySelector('input[data-field="detected"]');
    const xRange = card.querySelector('input[data-field="x"]');
    const xNumber = card.querySelector('input[data-field="xNumber"]');
    const yRange = card.querySelector('input[data-field="y"]');
    const yNumber = card.querySelector('input[data-field="yNumber"]');
    const magnitude = card.querySelector('[data-field="magnitude"]');

    const updateVector = (x, y) => {
      state[key].vector.x = clamp(parseFloat(x), -1, 1);
      state[key].vector.y = clamp(parseFloat(y), -1, 1);
      xRange.value = state[key].vector.x;
      xNumber.value = state[key].vector.x;
      yRange.value = state[key].vector.y;
      yNumber.value = state[key].vector.y;
      magnitude.textContent = computeMagnitude(state[key].vector).toFixed(2);
      updatePreview();
    };

    detectedInput.addEventListener('change', () => {
      state[key].detected = detectedInput.checked;
      updatePreview();
    });

    xRange.addEventListener('input', (event) => updateVector(event.target.value, state[key].vector.y));
    xNumber.addEventListener('input', (event) => updateVector(event.target.value, state[key].vector.y));
    yRange.addEventListener('input', (event) => updateVector(state[key].vector.x, event.target.value));
    yNumber.addEventListener('input', (event) => updateVector(state[key].vector.x, event.target.value));

    updateVector(0, 0);

    vectorControls[key] = {
      detectedInput,
      xRange,
      xNumber,
      yRange,
      yNumber,
      magnitude,
      updateVector
    };
  });
}

function bindFlags() {
  document.querySelectorAll('input[data-flag]').forEach((input) => {
    const key = input.dataset.flag;
    input.addEventListener('change', () => {
      state[key] = input.checked;
      updatePreview();
    });
  });
}

function updatePreview() {
  elements.payloadPreview.textContent = JSON.stringify(buildPreviewPayload(), null, 2);
}

async function loadState() {
  const result = await requestJson('/api/state');
  if (!result.ok || !result.data || !result.data.state) {
    return;
  }
  applyState(result.data.state);
}

function applyState(incoming) {
  ['safe_zone', 'danger_zone', 'target', 'line'].forEach((key) => {
    const controls = vectorControls[key];
    if (!controls) {
      return;
    }
    const vector = incoming[key] || buildVectorState();
    controls.detectedInput.checked = Boolean(vector.detected);
    const x = vector.vector ? vector.vector.x : 0;
    const y = vector.vector ? vector.vector.y : 0;
    controls.updateVector(x, y);
    state[key].detected = Boolean(vector.detected);
  });

  Object.keys(state).forEach((key) => {
    if (typeof state[key] === 'boolean' && key in incoming) {
      state[key] = Boolean(incoming[key]);
      const input = document.querySelector(`input[data-flag=\"${key}\"]`);
      if (input) {
        input.checked = state[key];
      }
    }
  });
}

async function sendPayload() {
  elements.status.textContent = 'Sending...';
  const payload = buildPayload();
  const result = await requestJson('/api/send', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(payload)
  });

  if (!result.ok) {
    elements.status.textContent = 'Failed';
    elements.lastSent.textContent = `Last send: error (${result.status})`;
    return;
  }

  elements.status.textContent = result.data.result && result.data.result.ok ? 'Sent' : 'Delivered';
  elements.lastSent.textContent = `Last send: ${new Date().toLocaleTimeString()}`;
}

function handleAutoSendToggle() {
  if (elements.autoSend.checked) {
    restartAutoSend();
  } else {
    clearInterval(autoSendTimer);
    autoSendTimer = null;
  }
}

function handleTraceToggle() {
  diagnostic.enabled = elements.traceToggle.checked;
  diagnostic.pendingTraceId = diagnostic.enabled ? generateTraceId() : null;
  if (!diagnostic.enabled) {
    elements.lastTrace.textContent = 'Trace: --';
  }
  updatePreview();
}

function restartAutoSend() {
  if (!elements.autoSend.checked) {
    return;
  }
  clearInterval(autoSendTimer);
  const interval = Math.max(100, parseInt(elements.intervalInput.value, 10) || 500);
  autoSendTimer = setInterval(sendPayload, interval);
}

function buildPayload() {
  const payload = { ...state };
  if (diagnostic.enabled) {
    const traceId = diagnostic.pendingTraceId || generateTraceId();
    const sentAtMs = Date.now();
    payload.trace = {
      id: traceId,
      sent_at_ms: sentAtMs
    };
    diagnostic.pendingTraceId = generateTraceId();
    elements.lastTrace.textContent = `Trace: ${traceId}`;
    updatePreview();
  }
  return payload;
}

function buildPreviewPayload() {
  if (!diagnostic.enabled) {
    return state;
  }
  return {
    ...state,
    trace: {
      id: diagnostic.pendingTraceId || 'pending',
      sent_at_ms: null
    }
  };
}

function computeMagnitude(vector) {
  return Math.sqrt(vector.x * vector.x + vector.y * vector.y);
}

function clamp(value, min, max) {
  if (Number.isNaN(value)) {
    return 0;
  }
  return Math.min(max, Math.max(min, value));
}

function generateTraceId() {
  const now = Date.now().toString(36);
  const rand = Math.random().toString(36).slice(2, 8);
  return `trace-${now}-${rand}`;
}

async function requestJson(url, options) {
  try {
    const response = await fetch(url, options);
    const text = await response.text();
    return {
      ok: response.ok,
      status: response.status,
      data: text ? JSON.parse(text) : null
    };
  } catch (err) {
    return {
      ok: false,
      status: 0,
      data: null
    };
  }
}
