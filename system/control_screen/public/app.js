const state = {
  ranges: null,
  linePidRanges: null,
  fsmStates: [],
  lineLiveTimer: null,
  suppressLineLive: false,
  settingState: false
};

const elements = {
  connectionStatus: document.getElementById('connectionStatus'),
  lastSync: document.getElementById('lastSync'),
  currentP: document.getElementById('currentP'),
  currentI: document.getElementById('currentI'),
  currentD: document.getElementById('currentD'),
  currentSpeedP: document.getElementById('currentSpeedP'),
  currentSpeedI: document.getElementById('currentSpeedI'),
  currentSpeedD: document.getElementById('currentSpeedD'),
  inputP: document.getElementById('inputP'),
  inputI: document.getElementById('inputI'),
  inputD: document.getElementById('inputD'),
  inputSpeedP: document.getElementById('inputSpeedP'),
  inputSpeedI: document.getElementById('inputSpeedI'),
  inputSpeedD: document.getElementById('inputSpeedD'),
  hintP: document.getElementById('hintP'),
  hintI: document.getElementById('hintI'),
  hintD: document.getElementById('hintD'),
  hintSpeedP: document.getElementById('hintSpeedP'),
  hintSpeedI: document.getElementById('hintSpeedI'),
  hintSpeedD: document.getElementById('hintSpeedD'),
  lineFeedback: document.getElementById('lineFeedback'),
  lfKp: document.getElementById('lfKp'),
  lfKpRange: document.getElementById('lfKpRange'),
  lfKi: document.getElementById('lfKi'),
  lfKiRange: document.getElementById('lfKiRange'),
  lfKd: document.getElementById('lfKd'),
  lfKdRange: document.getElementById('lfKdRange'),
  lfIMax: document.getElementById('lfIMax'),
  lfIMaxRange: document.getElementById('lfIMaxRange'),
  lfOutMax: document.getElementById('lfOutMax'),
  lfOutMaxRange: document.getElementById('lfOutMaxRange'),
  lfBaseSpeed: document.getElementById('lfBaseSpeed'),
  lfBaseSpeedRange: document.getElementById('lfBaseSpeedRange'),
  lfMinSpeed: document.getElementById('lfMinSpeed'),
  lfMinSpeedRange: document.getElementById('lfMinSpeedRange'),
  lfMaxSpeed: document.getElementById('lfMaxSpeed'),
  lfMaxSpeedRange: document.getElementById('lfMaxSpeedRange'),
  lfFollowMaxSpeed: document.getElementById('lfFollowMaxSpeed'),
  lfFollowMaxSpeedRange: document.getElementById('lfFollowMaxSpeedRange'),
  lfTurnSlowdown: document.getElementById('lfTurnSlowdown'),
  lfTurnSlowdownRange: document.getElementById('lfTurnSlowdownRange'),
  lfErrorSlowdown: document.getElementById('lfErrorSlowdown'),
  lfErrorSlowdownRange: document.getElementById('lfErrorSlowdownRange'),
  lfDeadband: document.getElementById('lfDeadband'),
  lfDeadbandRange: document.getElementById('lfDeadbandRange'),
  lfRotationScale: document.getElementById('lfRotationScale'),
  lfRotationScaleRange: document.getElementById('lfRotationScaleRange'),
  lfLineLagTau: document.getElementById('lfLineLagTau'),
  lfLineLagTauRange: document.getElementById('lfLineLagTauRange'),
  lfLineLagEnabled: document.getElementById('lfLineLagEnabled'),
  lfLineLagEnabledRange: document.getElementById('lfLineLagEnabledRange'),
  hintLfKp: document.getElementById('hintLfKp'),
  hintLfKi: document.getElementById('hintLfKi'),
  hintLfKd: document.getElementById('hintLfKd'),
  hintLfIMax: document.getElementById('hintLfIMax'),
  hintLfOutMax: document.getElementById('hintLfOutMax'),
  hintLfBaseSpeed: document.getElementById('hintLfBaseSpeed'),
  hintLfMinSpeed: document.getElementById('hintLfMinSpeed'),
  hintLfMaxSpeed: document.getElementById('hintLfMaxSpeed'),
  hintLfFollowMaxSpeed: document.getElementById('hintLfFollowMaxSpeed'),
  hintLfTurnSlowdown: document.getElementById('hintLfTurnSlowdown'),
  hintLfErrorSlowdown: document.getElementById('hintLfErrorSlowdown'),
  hintLfDeadband: document.getElementById('hintLfDeadband'),
  hintLfRotationScale: document.getElementById('hintLfRotationScale'),
  hintLfLineLagTau: document.getElementById('hintLfLineLagTau'),
  hintLfLineLagEnabled: document.getElementById('hintLfLineLagEnabled'),
  fsmStateSelect: document.getElementById('fsmStateSelect'),
  refreshStatesBtn: document.getElementById('refreshStatesBtn'),
  applyStateBtn: document.getElementById('applyStateBtn'),
  fsmFeedback: document.getElementById('fsmFeedback'),
  feedback: document.getElementById('feedback'),
  speedFeedback: document.getElementById('speedFeedback'),
  pidForm: document.getElementById('pidForm'),
  speedPidForm: document.getElementById('speedPidForm'),
  refreshBtn: document.getElementById('refreshBtn'),
  applyBtn: document.getElementById('applyBtn'),
  applySpeedBtn: document.getElementById('applySpeedBtn')
};

const lineFields = [
  ['kp', 'lfKp', 'lfKpRange', 'hintLfKp'],
  ['ki', 'lfKi', 'lfKiRange', 'hintLfKi'],
  ['kd', 'lfKd', 'lfKdRange', 'hintLfKd'],
  ['i_max', 'lfIMax', 'lfIMaxRange', 'hintLfIMax'],
  ['out_max', 'lfOutMax', 'lfOutMaxRange', 'hintLfOutMax'],
  ['base_speed', 'lfBaseSpeed', 'lfBaseSpeedRange', 'hintLfBaseSpeed'],
  ['min_speed', 'lfMinSpeed', 'lfMinSpeedRange', 'hintLfMinSpeed'],
  ['max_speed', 'lfMaxSpeed', 'lfMaxSpeedRange', 'hintLfMaxSpeed'],
  ['follow_max_speed', 'lfFollowMaxSpeed', 'lfFollowMaxSpeedRange', 'hintLfFollowMaxSpeed'],
  ['turn_slowdown', 'lfTurnSlowdown', 'lfTurnSlowdownRange', 'hintLfTurnSlowdown'],
  ['error_slowdown', 'lfErrorSlowdown', 'lfErrorSlowdownRange', 'hintLfErrorSlowdown'],
  ['deadband', 'lfDeadband', 'lfDeadbandRange', 'hintLfDeadband'],
  ['rotation_scale', 'lfRotationScale', 'lfRotationScaleRange', 'hintLfRotationScale'],
  ['line_lag_tau', 'lfLineLagTau', 'lfLineLagTauRange', 'hintLfLineLagTau'],
  ['line_lag_enabled', 'lfLineLagEnabled', 'lfLineLagEnabledRange', 'hintLfLineLagEnabled']
];

initialize();

async function initialize() {
  bindEvents();
  await loadConfig();
  await Promise.all([refreshValues(), refreshStateOptions()]);
}

function bindEvents() {
  elements.pidForm.addEventListener('submit', (event) => handleSubmit(event, 'heading'));
  elements.speedPidForm?.addEventListener('submit', (event) => handleSubmit(event, 'speed'));
  elements.refreshBtn.addEventListener('click', refreshValues);
  elements.refreshStatesBtn?.addEventListener('click', refreshStateOptions);
  elements.applyStateBtn?.addEventListener('click', handleSetState);
  for (const [, inputKey, rangeKey] of lineFields) {
    const input = elements[inputKey];
    const range = elements[rangeKey];
    if (!input) continue;
    input.addEventListener('input', scheduleLineLiveUpdate);
    input.addEventListener('change', scheduleLineLiveUpdate);
    if (range) {
      range.addEventListener('input', () => {
        input.value = range.value;
        scheduleLineLiveUpdate();
      });
      input.addEventListener('input', () => {
        if (!Number.isFinite(Number(input.value))) return;
        range.value = input.value;
      });
      input.addEventListener('change', () => {
        if (!Number.isFinite(Number(input.value))) return;
        range.value = input.value;
      });
    }
  }
}

async function refreshStateOptions() {
  if (!elements.fsmStateSelect) return;
  setFsmFeedback('Loading state options...', 'info');
  setStateLoading(true);
  const result = await requestJson('/api/state-machine/states');
  if (!result.ok || !result.data || !Array.isArray(result.data.states)) {
    const message = result.data?.message || 'Unable to load state options.';
    setFsmFeedback(message, 'error');
    setStateLoading(false);
    return;
  }

  const current = typeof result.data.current_state === 'string'
    ? result.data.current_state
    : '';
  state.fsmStates = result.data.states;
  elements.fsmStateSelect.innerHTML = '';
  for (const value of state.fsmStates) {
    const option = document.createElement('option');
    option.value = value;
    option.textContent = value;
    if (current && value === current) {
      option.selected = true;
    }
    elements.fsmStateSelect.appendChild(option);
  }
  setFsmFeedback(current ? `Loaded. Current state: ${current}.` : 'State options refreshed.', 'success');
  setStateLoading(false);
}

async function handleSetState() {
  const nextState = elements.fsmStateSelect?.value;
  if (!nextState) {
    setFsmFeedback('Select a target state first.', 'error');
    return;
  }

  setStateLoading(true);
  setFsmFeedback(`Setting state to ${nextState}...`, 'info');
  const result = await requestJson('/api/state-machine/set-state', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ state: nextState })
  });

  if (!result.ok) {
    const message = result.data?.message || 'Failed to set state.';
    setFsmFeedback(message, 'error');
    setStateLoading(false);
    return;
  }

  const applied = result.data?.state || nextState;
  setFsmFeedback(`State forced to ${applied}. Inputs and transient flags reset.`, 'success');
  await refreshStateOptions();
  setStateLoading(false);
}

async function loadConfig() {
  const result = await requestJson('/api/config');
  if (!result.ok || !result.data) {
    setFeedback('Unable to load configuration from the server.', 'error');
    return;
  }

  state.ranges = result.data.ranges || null;
  state.linePidRanges = result.data.linePidRanges || null;
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
  elements.hintSpeedP.textContent = formatRangeHint(ranges.speed_p || ranges.p);
  elements.hintSpeedI.textContent = formatRangeHint(ranges.speed_i || ranges.i);
  elements.hintSpeedD.textContent = formatRangeHint(ranges.speed_d || ranges.d);

  const lineRanges = state.linePidRanges || {};
  for (const [key, inputKey, rangeKey, hintKey] of lineFields) {
    const input = elements[inputKey];
    const range = elements[rangeKey];
    const bounds = lineRanges[key];
    applyLineFieldBounds(input, range, bounds);
    if (elements[hintKey]) {
      elements[hintKey].textContent = formatRangeHint(bounds);
    }
  }
}

function lineStepForRange(range) {
  if (!range) return 0.001;
  const span = Number(range.max) - Number(range.min);
  if (!Number.isFinite(span) || span <= 0) return 0.001;
  if (span <= 1) return 0.001;
  if (span <= 5) return 0.005;
  if (span <= 20) return 0.01;
  return 0.1;
}

function applyLineFieldBounds(numberInput, rangeInput, range) {
  if (!numberInput || !rangeInput || !range) return;
  const step = lineStepForRange(range);
  numberInput.min = String(range.min);
  numberInput.max = String(range.max);
  numberInput.step = String(step);
  rangeInput.min = String(range.min);
  rangeInput.max = String(range.max);
  rangeInput.step = String(step);
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

  const [statusResult, pidResult, lineResult] = await Promise.all([
    requestJson('/api/status'),
    requestJson('/api/pid'),
    requestJson('/api/line-follow-pid')
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
  if (lineResult.ok && lineResult.data && lineResult.data.values) {
    fillLineInputs(lineResult.data.values);
    setLineFeedback('Live tuning ready.', 'success');
  } else {
    setLineFeedback('Unable to load line-follow settings.', 'error');
  }
  setLoading(false);
}

function updateCurrentValues(values) {
  const heading = values.heading || values;
  const speed = values.speed || {};
  if (heading.p !== undefined) elements.currentP.textContent = formatNumber(heading.p);
  if (heading.i !== undefined) elements.currentI.textContent = formatNumber(heading.i);
  if (heading.d !== undefined) elements.currentD.textContent = formatNumber(heading.d);
  if (speed.p !== undefined) elements.currentSpeedP.textContent = formatNumber(speed.p);
  if (speed.i !== undefined) elements.currentSpeedI.textContent = formatNumber(speed.i);
  if (speed.d !== undefined) elements.currentSpeedD.textContent = formatNumber(speed.d);
}

function fillInputs(values) {
  const heading = values.heading || values;
  const speed = values.speed || {};
  if (heading.p !== undefined) {
    elements.inputP.value = heading.p;
  }
  if (heading.i !== undefined) {
    elements.inputI.value = heading.i;
  }
  if (heading.d !== undefined) {
    elements.inputD.value = heading.d;
  }
  if (speed.p !== undefined) {
    elements.inputSpeedP.value = speed.p;
  }
  if (speed.i !== undefined) {
    elements.inputSpeedI.value = speed.i;
  }
  if (speed.d !== undefined) {
    elements.inputSpeedD.value = speed.d;
  }
}

async function handleSubmit(event, section) {
  event.preventDefault();
  clearInputErrors(section);

  const isSpeed = section === 'speed';
  const payload = isSpeed
    ? {
        speed_p: elements.inputSpeedP.value,
        speed_i: elements.inputSpeedI.value,
        speed_d: elements.inputSpeedD.value
      }
    : {
        p: elements.inputP.value,
        i: elements.inputI.value,
        d: elements.inputD.value
      };

  const validationErrors = validateInputs(payload);
  if (Object.keys(validationErrors).length > 0) {
    setSectionFeedback(section, 'Check the highlighted fields and try again.', 'error');
    markInputErrors(validationErrors, section);
    return;
  }

  setLoading(true, section);
  setSectionFeedback(section, 'Sending update...', 'info');

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
    setSectionFeedback(section, message, 'error');
    setLoading(false, section);
    return;
  }

  await refreshBridgeStatus(true);
  const updated = result.data.updated || {};
  if (isSpeed) {
    updateCurrentValues({
      speed: {
        p: updated.speed_p ?? Number(payload.speed_p),
        i: updated.speed_i ?? Number(payload.speed_i),
        d: updated.speed_d ?? Number(payload.speed_d)
      }
    });
    setSectionFeedback(section, 'Speed PID gains updated successfully.', 'success');
  } else {
    updateCurrentValues({
      heading: {
        p: updated.p ?? Number(payload.p),
        i: updated.i ?? Number(payload.i),
        d: updated.d ?? Number(payload.d)
      }
    });
    setSectionFeedback(section, 'PID gains updated successfully.', 'success');
  }
  setLoading(false, section);
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
  const fieldMap = {
    p: elements.inputP,
    i: elements.inputI,
    d: elements.inputD,
    speed_p: elements.inputSpeedP,
    speed_i: elements.inputSpeedI,
    speed_d: elements.inputSpeedD
  };
  Object.entries(errors).forEach(([key]) => {
    fieldMap[key]?.classList.add('input-error');
  });
}

function clearInputErrors(section) {
  const inputs = section === 'speed'
    ? [elements.inputSpeedP, elements.inputSpeedI, elements.inputSpeedD]
    : [elements.inputP, elements.inputI, elements.inputD];
  inputs.forEach((input) => input?.classList.remove('input-error'));
}

function fillLineInputs(values) {
  state.suppressLineLive = true;
  for (const [key, inputKey, rangeKey] of lineFields) {
    const input = elements[inputKey];
    const range = elements[rangeKey];
    if (!input) continue;
    if (values[key] !== undefined && Number.isFinite(Number(values[key]))) {
      input.value = Number(values[key]);
      if (range) {
        range.value = String(Number(values[key]));
      }
    }
    input.classList.remove('input-error');
  }
  state.suppressLineLive = false;
}

function scheduleLineLiveUpdate() {
  if (state.suppressLineLive) return;
  if (state.lineLiveTimer) {
    clearTimeout(state.lineLiveTimer);
  }
  state.lineLiveTimer = setTimeout(sendLineLiveUpdate, 180);
}

async function sendLineLiveUpdate() {
  state.lineLiveTimer = null;
  const payload = {};
  const errors = {};
  const lineRanges = state.linePidRanges || {};

  for (const [key, inputKey] of lineFields) {
    const input = elements[inputKey];
    if (!input) continue;
    input.classList.remove('input-error');
    const raw = input.value;
    if (raw === '') {
      continue;
    }
    const numeric = Number(raw);
    if (!Number.isFinite(numeric)) {
      errors[key] = 'Value must be a number.';
      input.classList.add('input-error');
      continue;
    }
    const range = lineRanges[key];
    if (range && (numeric < range.min || numeric > range.max)) {
      errors[key] = `Must be between ${range.min} and ${range.max}.`;
      input.classList.add('input-error');
      continue;
    }
    payload[key] = numeric;
  }

  if (Object.keys(errors).length > 0) {
    setLineFeedback('Line-follow values out of range.', 'error');
    return;
  }

  if (payload.min_speed !== undefined && payload.max_speed !== undefined && payload.min_speed > payload.max_speed) {
    setLineFeedback('min_speed must be <= max_speed.', 'error');
    const minInput = elements.lfMinSpeed;
    const maxInput = elements.lfMaxSpeed;
    minInput?.classList.add('input-error');
    maxInput?.classList.add('input-error');
    return;
  }

  if (Object.keys(payload).length === 0) {
    setLineFeedback('Live tuning idle.', 'info');
    return;
  }

  setLineFeedback('Updating line-follow settings...', 'info');
  const result = await requestJson('/api/line-follow-pid', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(payload)
  });

  if (!result.ok) {
    const message = result.data?.message || 'Line-follow update failed.';
    setLineFeedback(message, 'error');
    return;
  }

  if (result.data?.values) {
    fillLineInputs(result.data.values);
  }
  setLineFeedback('Line-follow settings updated live.', 'success');
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
  setSectionFeedback('heading', message, tone);
}

function setSectionFeedback(section, message, tone) {
  const target = section === 'speed' ? elements.speedFeedback : elements.feedback;
  if (!target) return;
  target.textContent = message;
  target.classList.remove('success', 'error');

  if (tone === 'success') {
    target.classList.add('success');
  } else if (tone === 'error') {
    target.classList.add('error');
  }
}

function setLineFeedback(message, tone) {
  if (!elements.lineFeedback) return;
  elements.lineFeedback.textContent = message;
  elements.lineFeedback.classList.remove('success', 'error');
  if (tone === 'success') {
    elements.lineFeedback.classList.add('success');
  } else if (tone === 'error') {
    elements.lineFeedback.classList.add('error');
  }
}

function setLoading(isLoading, section) {
  if (!section || section === 'heading') {
    elements.refreshBtn.disabled = isLoading;
    elements.applyBtn.disabled = isLoading;
  }
  if (!section || section === 'speed') {
    if (elements.applySpeedBtn) {
      elements.applySpeedBtn.disabled = isLoading;
    }
  }
}

function setStateLoading(isLoading) {
  state.settingState = isLoading;
  if (elements.refreshStatesBtn) {
    elements.refreshStatesBtn.disabled = isLoading;
  }
  if (elements.applyStateBtn) {
    elements.applyStateBtn.disabled = isLoading;
  }
  if (elements.fsmStateSelect) {
    elements.fsmStateSelect.disabled = isLoading;
  }
}

function setFsmFeedback(message, tone) {
  if (!elements.fsmFeedback) return;
  elements.fsmFeedback.textContent = message;
  elements.fsmFeedback.classList.remove('success', 'error');
  if (tone === 'success') {
    elements.fsmFeedback.classList.add('success');
  } else if (tone === 'error') {
    elements.fsmFeedback.classList.add('error');
  }
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
