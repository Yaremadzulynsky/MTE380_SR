const state = {
  enabled: false,
  groups: {},
  services: [],
  logTailTimer: null,
  logTailMs: 2500
};

const elements = {
  opsEnabled: document.getElementById('opsEnabled'),
  opsDetail: document.getElementById('opsDetail'),
  stackFeedback: document.getElementById('stackFeedback'),
  servicesGrid: document.getElementById('servicesGrid'),
  refreshServicesBtn: document.getElementById('refreshServicesBtn'),
  logService: document.getElementById('logService'),
  logLines: document.getElementById('logLines'),
  refreshLogsBtn: document.getElementById('refreshLogsBtn'),
  toggleTailBtn: document.getElementById('toggleTailBtn'),
  logsOutput: document.getElementById('logsOutput'),
  testFeedback: document.getElementById('testFeedback'),
  neutralControlBtn: document.getElementById('neutralControlBtn'),
  sampleInputBtn: document.getElementById('sampleInputBtn')
};

initialize().catch((err) => {
  setFeedback(elements.stackFeedback, `Initialization failed: ${err.message}`, true);
});

async function initialize() {
  bindEvents();
  await loadOpsConfig();
  await refreshServices();
  await refreshLogs();
}

function bindEvents() {
  document.querySelectorAll('[data-action]').forEach((button) => {
    button.addEventListener('click', () => runStackAction(button.dataset.action));
  });
  elements.refreshServicesBtn?.addEventListener('click', refreshServices);
  elements.refreshLogsBtn?.addEventListener('click', refreshLogs);
  elements.toggleTailBtn?.addEventListener('click', toggleTail);
  elements.neutralControlBtn?.addEventListener('click', sendNeutralControlTest);
  elements.sampleInputBtn?.addEventListener('click', sendSampleInputTest);
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
  const alloySuffix = result.data.supportsAlloy ? 'alloy enabled' : 'alloy disabled';
  elements.opsDetail.textContent = `Compose: ${result.data.composeFile || 'not set'} (${alloySuffix})`;
  setControlsEnabled(state.enabled);
  if (!state.enabled) {
    setFeedback(
      elements.stackFeedback,
      'Ops dashboard is disabled. Set ENABLE_OPS_DASHBOARD=true for start/stop/log actions.',
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

async function refreshServices() {
  const result = await requestJson('/api/ops/services');
  if (!result.ok || !result.data) {
    const message = extractError(result, 'Failed to load service status.');
    setFeedback(elements.stackFeedback, message, true);
    renderServices([]);
    return;
  }
  state.services = Array.isArray(result.data.services) ? result.data.services : [];
  renderServices(state.services);
  populateServiceSelect(state.services);
}

function renderServices(services) {
  if (!elements.servicesGrid) {
    return;
  }
  if (!services.length) {
    elements.servicesGrid.innerHTML = '<p class="hint">No services available.</p>';
    return;
  }

  const html = services
    .map((serviceRow) => {
      const service = escapeHtml(serviceRow.service || 'unknown');
      const stateText = escapeHtml(serviceRow.state || 'unknown');
      const running = stateText.toLowerCase() === 'running';
      return `
        <article class="ops-service-card">
          <div>
            <div class="ops-service-name">${service}</div>
            <div class="ops-service-state ${running ? 'online' : 'offline'}">${stateText}</div>
          </div>
          <button type="button" class="ghost" data-restart-service="${service}">Restart</button>
        </article>
      `;
    })
    .join('');
  elements.servicesGrid.innerHTML = html;

  elements.servicesGrid.querySelectorAll('[data-restart-service]').forEach((button) => {
    button.addEventListener('click', async () => {
      await restartService(button.dataset.restartService);
    });
  });
}

function populateServiceSelect(services) {
  if (!elements.logService) {
    return;
  }
  const existing = elements.logService.value;
  const options = services
    .map((serviceRow) => serviceRow.service)
    .filter(Boolean)
    .map((serviceName) => `<option value="${escapeHtml(serviceName)}">${escapeHtml(serviceName)}</option>`)
    .join('');
  elements.logService.innerHTML = options;
  if (!options) {
    return;
  }
  if (existing && services.some((serviceRow) => serviceRow.service === existing)) {
    elements.logService.value = existing;
  } else {
    elements.logService.selectedIndex = 0;
  }
}

async function runStackAction(action) {
  const endpointMap = {
    'start-core': { url: '/api/ops/stack/up', body: { group: 'core' } },
    'start-full': { url: '/api/ops/stack/up', body: { group: 'full' } },
    'stop-core': { url: '/api/ops/stack/stop', body: { group: 'core' } },
    'down-full': { url: '/api/ops/stack/down', body: { group: 'full' } }
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
  await refreshServices();
}

async function restartService(service) {
  if (!service) {
    return;
  }
  setFeedback(elements.stackFeedback, `Restarting ${service}...`);
  const result = await requestJson('/api/ops/service/restart', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ service })
  });
  if (!result.ok) {
    setFeedback(elements.stackFeedback, extractError(result, 'Restart failed.'), true);
    return;
  }
  setFeedback(elements.stackFeedback, result.data?.message || `${service} restarted.`);
  await refreshServices();
}

async function refreshLogs() {
  const service = elements.logService?.value;
  if (!service) {
    return;
  }
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
    elements.toggleTailBtn.textContent = 'Start Tail';
    return;
  }
  elements.toggleTailBtn.textContent = 'Stop Tail';
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
