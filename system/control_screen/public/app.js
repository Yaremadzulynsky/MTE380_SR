const state = {
  ranges: null
};

const elements = {
  connectionStatus: document.getElementById('connectionStatus'),
  lastSync: document.getElementById('lastSync'),
  currentP: document.getElementById('currentP'),
  currentI: document.getElementById('currentI'),
  currentD: document.getElementById('currentD'),
  inputP: document.getElementById('inputP'),
  inputI: document.getElementById('inputI'),
  inputD: document.getElementById('inputD'),
  hintP: document.getElementById('hintP'),
  hintI: document.getElementById('hintI'),
  hintD: document.getElementById('hintD'),
  feedback: document.getElementById('feedback'),
  pidForm: document.getElementById('pidForm'),
  refreshBtn: document.getElementById('refreshBtn'),
  applyBtn: document.getElementById('applyBtn')
};

initialize();

async function initialize() {
  bindEvents();
  await loadConfig();
  await refreshValues();
}

function bindEvents() {
  elements.pidForm.addEventListener('submit', handleSubmit);
  elements.refreshBtn.addEventListener('click', refreshValues);
}

async function loadConfig() {
  const result = await requestJson('/api/config');
  if (!result.ok || !result.data) {
    setFeedback('Unable to load configuration from the server.', 'error');
    return;
  }

  state.ranges = result.data.ranges || null;
  updateRangeHints();
}

function updateRangeHints() {
  const ranges = state.ranges;
  if (!ranges) {
    return;
  }

  elements.hintP.textContent = formatRangeHint(ranges.p);
  elements.hintI.textContent = formatRangeHint(ranges.i);
  elements.hintD.textContent = formatRangeHint(ranges.d);
}

function formatRangeHint(range) {
  if (!range) {
    return 'Range: --';
  }

  return `Range: ${range.min} to ${range.max}`;
}

async function refreshValues() {
  setLoading(true);
  clearInputErrors();
  setFeedback('Refreshing PID values...', 'info');

  const [statusResult, pidResult] = await Promise.all([
    requestJson('/api/status'),
    requestJson('/api/pid')
  ]);

  const statusPayload =
    statusResult.ok && statusResult.data ? statusResult.data.status : null;
  updateBridgeStatus(statusPayload, pidResult.ok);

  if (!pidResult.ok) {
    const message = pidResult.data && pidResult.data.message
      ? pidResult.data.message
      : 'Unable to reach control communication service.';
    setFeedback(message, 'error');
    setLoading(false);
    return;
  }

  const values = pidResult.data.values || {};
  updateCurrentValues(values);
  fillInputs(values);
  setFeedback('Values synced.', 'success');
  setLoading(false);
}

function updateCurrentValues(values) {
  elements.currentP.textContent = formatNumber(values.p);
  elements.currentI.textContent = formatNumber(values.i);
  elements.currentD.textContent = formatNumber(values.d);
}

function fillInputs(values) {
  if (values.p !== undefined) {
    elements.inputP.value = values.p;
  }
  if (values.i !== undefined) {
    elements.inputI.value = values.i;
  }
  if (values.d !== undefined) {
    elements.inputD.value = values.d;
  }
}

async function handleSubmit(event) {
  event.preventDefault();
  clearInputErrors();

  const payload = {
    p: elements.inputP.value,
    i: elements.inputI.value,
    d: elements.inputD.value
  };

  const validationErrors = validateInputs(payload);
  if (Object.keys(validationErrors).length > 0) {
    setFeedback('Check the highlighted fields and try again.', 'error');
    markInputErrors(validationErrors);
    return;
  }

  setLoading(true);
  setFeedback('Sending update...', 'info');

  const result = await requestJson('/api/pid', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(payload)
  });

  if (!result.ok) {
    await refreshBridgeStatus(false);
    const message = result.data && result.data.message
      ? result.data.message
      : 'Update failed on control communication service.';
    setFeedback(message, 'error');
    setLoading(false);
    return;
  }

  await refreshBridgeStatus(true);
  const updated = result.data.updated || {};
  updateCurrentValues({
    p: updated.p ?? Number(payload.p),
    i: updated.i ?? Number(payload.i),
    d: updated.d ?? Number(payload.d)
  });
  setFeedback('PID gains updated successfully.', 'success');
  setLoading(false);
}

function validateInputs(payload) {
  const errors = {};
  const ranges = state.ranges || {
    p: { min: -Infinity, max: Infinity },
    i: { min: -Infinity, max: Infinity },
    d: { min: -Infinity, max: Infinity }
  };

  Object.entries(payload).forEach(([key, value]) => {
    const numeric = Number(value);
    if (!Number.isFinite(numeric)) {
      errors[key] = 'Value must be a number.';
      return;
    }

    const range = ranges[key];
    if (numeric < range.min || numeric > range.max) {
      errors[key] = `Value must be between ${range.min} and ${range.max}.`;
    }
  });

  return errors;
}

function markInputErrors(errors) {
  if (errors.p) {
    elements.inputP.classList.add('input-error');
  }
  if (errors.i) {
    elements.inputI.classList.add('input-error');
  }
  if (errors.d) {
    elements.inputD.classList.add('input-error');
  }
}

function clearInputErrors() {
  elements.inputP.classList.remove('input-error');
  elements.inputI.classList.remove('input-error');
  elements.inputD.classList.remove('input-error');
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

function setFeedback(message, tone) {
  elements.feedback.textContent = message;
  elements.feedback.classList.remove('success', 'error');

  if (tone === 'success') {
    elements.feedback.classList.add('success');
  } else if (tone === 'error') {
    elements.feedback.classList.add('error');
  }
}

function setLoading(isLoading) {
  elements.refreshBtn.disabled = isLoading;
  elements.applyBtn.disabled = isLoading;
}

function formatNumber(value) {
  if (!Number.isFinite(value)) {
    return '--';
  }

  const rounded = Number(value).toFixed(4);
  return rounded.replace(/\.0+$/, '').replace(/(\.\d*[1-9])0+$/, '$1');
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
