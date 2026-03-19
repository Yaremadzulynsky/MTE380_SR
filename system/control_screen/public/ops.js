const state = {
  enabled: false,
  groups: {},
  runnerStatusTimer: null,
  logTailTimer: null,
  logTailMs: 2500,
  defaultStreamUrl: 'http://localhost:8090/stream.mjpg'
};

const elements = {
  opsEnabled: document.getElementById('opsEnabled'),
  opsDetail: document.getElementById('opsDetail'),
  runnerMeta: document.getElementById('runnerMeta'),
  robotFeedback: document.getElementById('robotFeedback'),
  startRobotBtn: document.getElementById('startRobotBtn'),
  stopRobotBtn: document.getElementById('stopRobotBtn'),
  streamUrlInput: document.getElementById('streamUrlInput'),
  streamViewerMode: document.getElementById('streamViewerMode'),
  streamPresetMjpegBtn: document.getElementById('streamPresetMjpegBtn'),
  streamPresetWebrtcBtn: document.getElementById('streamPresetWebrtcBtn'),
  openStreamBtn: document.getElementById('openStreamBtn'),
  reloadStreamBtn: document.getElementById('reloadStreamBtn'),
  copyStreamUrlBtn: document.getElementById('copyStreamUrlBtn'),
  visionStreamImg: document.getElementById('visionStreamImg'),
  visionStreamFrame: document.getElementById('visionStreamFrame'),
  visionRtspHint: document.getElementById('visionRtspHint'),
  visionRtspUrl: document.getElementById('visionRtspUrl'),
  stackFeedback: document.getElementById('stackFeedback'),
  refreshServicesBtn: document.getElementById('refreshServicesBtn'),
  logLines: document.getElementById('logLines'),
  refreshLogsBtn: document.getElementById('refreshLogsBtn'),
  toggleTailBtn: document.getElementById('toggleTailBtn'),
  logsOutput: document.getElementById('logsOutput'),
  testFeedback: document.getElementById('testFeedback'),
  turnTestFeedback: document.getElementById('turnTestFeedback'),
  neutralControlBtn: document.getElementById('neutralControlBtn'),
  sampleInputBtn: document.getElementById('sampleInputBtn')
};

initialize().catch((err) => {
  setFeedback(elements.stackFeedback, `Initialization failed: ${err.message}`, true);
});

async function initialize() {
  bindEvents();
  await loadOpsConfig();
  await refreshRunnerStatus();
  await refreshLogs();
  startRunnerStatusRefresh();
  toggleTail();
}

function bindEvents() {
  document.querySelectorAll('[data-action]').forEach((button) => {
    button.addEventListener('click', () => runStackAction(button.dataset.action));
  });
  document.querySelectorAll('[data-turn-test]').forEach((button) => {
    button.addEventListener('click', () => runTurnTest(Number(button.dataset.turnTest)));
  });
  elements.startRobotBtn?.addEventListener('click', startRobot);
  elements.stopRobotBtn?.addEventListener('click', stopRobot);
  elements.openStreamBtn?.addEventListener('click', openVisionStream);
  elements.reloadStreamBtn?.addEventListener('click', reloadVisionStream);
  elements.copyStreamUrlBtn?.addEventListener('click', copyVisionStreamUrl);
  elements.streamPresetMjpegBtn?.addEventListener('click', applyMjpegPreset);
  elements.streamPresetWebrtcBtn?.addEventListener('click', applyWebrtcPreset);
  elements.streamViewerMode?.addEventListener('change', openVisionStream);
  elements.refreshServicesBtn?.addEventListener('click', refreshRunnerStatus);
  elements.refreshLogsBtn?.addEventListener('click', refreshLogs);
  elements.toggleTailBtn?.addEventListener('click', toggleTail);
  elements.neutralControlBtn?.addEventListener('click', sendNeutralControlTest);
  elements.sampleInputBtn?.addEventListener('click', sendSampleInputTest);
}

async function startRobot() {
  setFeedback(elements.robotFeedback, 'Sending robot start...');
  const result = await requestJson('/api/ops/robot/start', { method: 'POST' });
  if (!result.ok) {
    setFeedback(elements.robotFeedback, extractError(result, 'Robot start failed.'), true);
    return;
  }
  setFeedback(elements.robotFeedback, result.data?.message || 'Robot start command sent.');
}

async function stopRobot() {
  setFeedback(elements.robotFeedback, 'Sending robot stop...');
  const result = await requestJson('/api/ops/robot/stop', { method: 'POST' });
  if (!result.ok) {
    setFeedback(elements.robotFeedback, extractError(result, 'Robot stop failed.'), true);
    return;
  }
  setFeedback(elements.robotFeedback, result.data?.message || 'Robot stop command sent.');
}

async function loadOpsConfig() {
  const result = await requestJson('/api/ops/config');
  if (!result.ok || !result.data) {
    state.enabled = false;
    elements.opsEnabled.textContent = 'Config unavailable';
    elements.opsEnabled.classList.add('offline');
    elements.opsDetail.textContent = 'Could not load ops configuration.';
    return;
  }

  state.enabled = Boolean(result.data.enabled);
  state.groups = result.data.groups || {};
  elements.opsEnabled.textContent = state.enabled ? 'Enabled' : 'Disabled';
  elements.opsEnabled.classList.remove('online', 'offline');
  elements.opsEnabled.classList.add(state.enabled ? 'online' : 'offline');
  const scriptPath = result.data.perceptionScript || 'run_perception_rpicam.sh';
  elements.opsDetail.textContent = `Perception script: ${scriptPath}`;
  state.defaultStreamUrl = result.data.houghStreamUrl || 'http://localhost:8090/stream.mjpg';
  if (elements.streamUrlInput) {
    elements.streamUrlInput.value = state.defaultStreamUrl;
    openVisionStream();
  }
  setControlsEnabled(state.enabled);
  if (!state.enabled) {
    setFeedback(
      elements.stackFeedback,
      'Perception runner controls are disabled. Set ENABLE_PERCEPTION_RUNNER=true.',
      true
    );
  }
}

function setControlsEnabled(enabled) {
  const nodes = document.querySelectorAll('button, select, input');
  nodes.forEach((node) => {
    if (node.id === 'refreshServicesBtn') {
      node.disabled = false;
      return;
    }
    node.disabled = !enabled;
  });
}

function startRunnerStatusRefresh() {
  if (state.runnerStatusTimer) {
    clearInterval(state.runnerStatusTimer);
  }
  state.runnerStatusTimer = setInterval(() => {
    refreshRunnerStatus().catch(() => {});
  }, 2500);
}

async function refreshRunnerStatus() {
  const result = await requestJson('/api/ops/perception/status');
  if (!result.ok || !result.data) {
    const message = extractError(result, 'Failed to load runner status.');
    setFeedback(elements.stackFeedback, message, true);
    if (elements.opsEnabled) {
      elements.opsEnabled.textContent = 'Unknown';
      elements.opsEnabled.classList.remove('online');
      elements.opsEnabled.classList.add('offline');
    }
    if (elements.runnerMeta) {
      elements.runnerMeta.textContent = 'PID: --';
    }
    return;
  }
  const running = Boolean(result.data.running);
  if (elements.opsEnabled) {
    elements.opsEnabled.textContent = running ? 'Running' : 'Stopped';
    elements.opsEnabled.classList.remove('online', 'offline');
    elements.opsEnabled.classList.add(running ? 'online' : 'offline');
  }
  if (elements.runnerMeta) {
    const pid = result.data.pid ? String(result.data.pid) : '--';
    const started = result.data.startedAt
      ? new Date(result.data.startedAt).toLocaleTimeString()
      : '--';
    elements.runnerMeta.textContent = `PID: ${pid} · Started: ${started}`;
  }
}

async function runStackAction(action) {
  const endpointMap = {
    'start-perception': { url: '/api/ops/stack/up', body: { group: 'perception' } },
    'stop-perception': { url: '/api/ops/stack/stop', body: { group: 'perception' } },
    'restart-perception': { url: '/api/ops/service/restart', body: { service: 'perception-runner' } }
  };
  const target = endpointMap[action];
  if (!target) {
    setFeedback(elements.stackFeedback, `Unknown action: ${action}`, true);
    return;
  }

  setFeedback(elements.stackFeedback, 'Running command...');
  const result = await requestJson(target.url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(target.body)
  });
  if (!result.ok) {
    setFeedback(elements.stackFeedback, extractError(result, 'Command failed.'), true);
    return;
  }
  setFeedback(elements.stackFeedback, result.data?.message || 'Command complete.');
  await refreshRunnerStatus();
  await refreshLogs();
}

function setVisionWidgets(mode) {
  const img = elements.visionStreamImg;
  const frame = elements.visionStreamFrame;
  const hint = elements.visionRtspHint;
  const pre = elements.visionRtspUrl;
  if (img) {
    img.classList.toggle('hidden', mode !== 'mjpeg');
  }
  if (frame) {
    frame.classList.toggle('hidden', mode !== 'embed');
  }
  if (hint) {
    hint.classList.toggle('hidden', mode !== 'rtsp');
  }
  if (pre) {
    pre.classList.toggle('hidden', mode !== 'rtsp');
  }
}

function openVisionStream() {
  const url = (elements.streamUrlInput?.value || '').trim();
  const mode = elements.streamViewerMode?.value || 'mjpeg';
  if (!url) {
    return;
  }
  setVisionWidgets(mode);
  if (mode === 'mjpeg' && elements.visionStreamImg) {
    elements.visionStreamImg.src = url;
    return;
  }
  if (mode === 'embed' && elements.visionStreamFrame) {
    elements.visionStreamFrame.src = url;
    return;
  }
  if (mode === 'rtsp' && elements.visionRtspUrl) {
    elements.visionRtspUrl.textContent = url;
  }
}

function reloadVisionStream() {
  const url = (elements.streamUrlInput?.value || '').trim();
  const mode = elements.streamViewerMode?.value || 'mjpeg';
  if (!url) {
    return;
  }
  const separator = url.includes('?') ? '&' : '?';
  if (mode === 'mjpeg' && elements.visionStreamImg) {
    elements.visionStreamImg.src = `${url}${separator}t=${Date.now()}`;
    return;
  }
  if (mode === 'embed' && elements.visionStreamFrame) {
    elements.visionStreamFrame.src = `${url}${separator}t=${Date.now()}`;
  }
}

async function copyVisionStreamUrl() {
  const url = (elements.streamUrlInput?.value || '').trim();
  if (!url) {
    return;
  }
  try {
    await navigator.clipboard.writeText(url);
  } catch {
    // Fallback
    window.prompt('Copy stream URL', url);
  }
}

function applyMjpegPreset() {
  if (!elements.streamUrlInput) {
    return;
  }
  elements.streamViewerMode.value = 'mjpeg';
  elements.streamUrlInput.value = state.defaultStreamUrl;
  openVisionStream();
}

function applyWebrtcPreset() {
  if (!elements.streamUrlInput) {
    return;
  }
  const host = window.location.hostname || 'localhost';
  elements.streamViewerMode.value = 'embed';
  elements.streamUrlInput.value = `http://${host}:8889/hough`;
  openVisionStream();
}

async function refreshLogs() {
  const service = 'perception-runner';
  const rawLines = Number(elements.logLines?.value);
  const lines = Number.isFinite(rawLines) ? Math.max(10, Math.min(rawLines, 2000)) : 150;
  const result = await requestJson(`/api/ops/logs?service=${encodeURIComponent(service)}&lines=${lines}`);
  if (!result.ok) {
    elements.logsOutput.textContent = extractError(result, `Failed to load logs for ${service}.`);
    return;
  }
  const stdout = result.data?.stdout || '';
  const stderr = result.data?.stderr || '';
  const logs = [stdout.trim(), stderr.trim()].filter(Boolean).join('\n');
  elements.logsOutput.textContent = logs || `No logs for ${service}.`;
}

function toggleTail() {
  if (state.logTailTimer) {
    clearInterval(state.logTailTimer);
    state.logTailTimer = null;
    elements.toggleTailBtn.textContent = 'Tail: Off';
    return;
  }
  elements.toggleTailBtn.textContent = 'Tail: On';
  state.logTailTimer = setInterval(() => {
    refreshLogs().catch(() => {});
  }, state.logTailMs);
}

async function sendNeutralControlTest() {
  setFeedback(elements.testFeedback, 'Sending neutral control...');
  const result = await requestJson('/api/ops/tests/neutral-control', { method: 'POST' });
  if (!result.ok) {
    setFeedback(elements.testFeedback, extractError(result, 'Neutral control test failed.'), true);
    return;
  }
  setFeedback(elements.testFeedback, result.data?.message || 'Neutral control sent.');
}

async function sendSampleInputTest() {
  setFeedback(elements.testFeedback, 'Sending sample input...');
  const result = await requestJson('/api/ops/tests/sample-input', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ vector: { x: 0.2, y: 0.8 } })
  });
  if (!result.ok) {
    setFeedback(elements.testFeedback, extractError(result, 'Sample input test failed.'), true);
    return;
  }
  setFeedback(elements.testFeedback, result.data?.message || 'Sample input sent.');
}

async function runTurnTest(degrees) {
  if (!Number.isFinite(degrees)) {
    return;
  }
  setFeedback(elements.turnTestFeedback, `Running turn test ${degrees}°...`);
  const result = await requestJson('/api/ops/tests/turn', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ degrees })
  });
  if (!result.ok) {
    setFeedback(elements.turnTestFeedback, extractError(result, `Turn ${degrees}° test failed.`), true);
    return;
  }
  const turn = result.data?.turn || {};
  const enc = turn.encoders || {};
  const ok = Boolean(turn.success);
  const msg = [
    `Turn ${degrees}° done (${ok ? 'PASS' : 'CHECK'})`,
    `achieved ${formatSigned(turn.achieved_degrees)}°`,
    `error ${formatSigned(turn.error_degrees)}°`,
    `enc ΔL=${enc.delta_left ?? '--'} ΔR=${enc.delta_right ?? '--'}`,
    result.data?.stop?.ok ? 'auto-stop: ok' : 'auto-stop: failed'
  ].join(' · ');
  setFeedback(elements.turnTestFeedback, msg, !ok);
  await refreshRunnerStatus();
  await refreshLogs();
}

function setFeedback(element, text, isError = false) {
  if (!element) {
    return;
  }
  element.textContent = text;
  element.classList.remove('success', 'error');
  element.classList.add(isError ? 'error' : 'success');
}

function extractError(result, fallback) {
  if (result?.data?.message) {
    return result.data.message;
  }
  if (result?.text) {
    return result.text;
  }
  return fallback;
}

function escapeHtml(value) {
  return String(value)
    .replaceAll('&', '&amp;')
    .replaceAll('<', '&lt;')
    .replaceAll('>', '&gt;')
    .replaceAll('"', '&quot;')
    .replaceAll("'", '&#39;');
}

function formatSigned(value) {
  if (!Number.isFinite(Number(value))) {
    return '--';
  }
  const n = Number(value);
  return `${n >= 0 ? '+' : ''}${n.toFixed(2)}`;
}

async function requestJson(url, options = {}) {
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
