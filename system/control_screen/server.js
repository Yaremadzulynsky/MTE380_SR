const express = require('express');
const path = require('path');
const fs = require('fs');
const { spawn } = require('child_process');

const app = express();
const PORT = Number.parseInt(process.env.PORT, 10) || 3000;

const CONTROL_COMM_BASE_URL =
  process.env.CONTROL_COMM_BASE_URL || 'http://localhost:5000';
const CONTROL_COMM_P_PATH =
  process.env.CONTROL_COMM_P_PATH || '/pid/proportional';
const CONTROL_COMM_I_PATH =
  process.env.CONTROL_COMM_I_PATH || '/pid/integral';
const CONTROL_COMM_D_PATH =
  process.env.CONTROL_COMM_D_PATH || '/pid/derivative';
const CONTROL_COMM_HEALTH_PATH =
  process.env.CONTROL_COMM_HEALTH_PATH || '/health';
const CONTROL_COMM_CONTROL_PATH =
  process.env.CONTROL_COMM_CONTROL_PATH || '/control';
const STATE_MACHINE_BASE_URL =
  process.env.STATE_MACHINE_BASE_URL || 'http://localhost:8000';
const STATE_MACHINE_INPUT_PATH =
  process.env.STATE_MACHINE_INPUT_PATH || '/inputs';
const STATE_MACHINE_LINE_FOLLOW_PID_PATH =
  process.env.STATE_MACHINE_LINE_FOLLOW_PID_PATH || '/line-follow-pid';
const STATE_MACHINE_STATES_PATH =
  process.env.STATE_MACHINE_STATES_PATH || '/states';
const STATE_MACHINE_SET_STATE_PATH =
  process.env.STATE_MACHINE_SET_STATE_PATH || '/set-state';
const ROBOT_MOCK_URL = process.env.ROBOT_MOCK_URL || 'http://localhost:8200';
const LOG_PATH = process.env.LOG_PATH || process.env.INSIGHTS_IPC_PATH;
const SERVO_MIN_DEG = Number.parseInt(process.env.SERVO_MIN_DEG || '0', 10);
const SERVO_MAX_DEG = Number.parseInt(process.env.SERVO_MAX_DEG || '90', 10);
const ENABLE_OPS_DASHBOARD = String(process.env.ENABLE_OPS_DASHBOARD || '').toLowerCase() === 'true';
const OPS_DOCKER_BIN = process.env.OPS_DOCKER_BIN || 'docker';
const OPS_COMPOSE_FILE = process.env.OPS_COMPOSE_FILE
  || '/home/zainm/Documents/3B/380/MTE380_SR/system/docker-compose.yaml';
const OPS_COMMAND_TIMEOUT_MS = Number.parseInt(process.env.OPS_COMMAND_TIMEOUT_MS || '15000', 10);
const OPS_MAX_OUTPUT_BYTES = Number.parseInt(process.env.OPS_MAX_OUTPUT_BYTES || '262144', 10);
const OPS_LOG_TAIL_DEFAULT = Number.parseInt(process.env.OPS_LOG_TAIL_DEFAULT || '150', 10);
const OPS_LOG_TAIL_MAX = Number.parseInt(process.env.OPS_LOG_TAIL_MAX || '2000', 10);
const OPS_INCLUDE_ALLOY = String(process.env.OPS_INCLUDE_ALLOY || '').toLowerCase() === 'true';
const OPS_SUPPORTS_ALLOY = process.arch !== 'arm' || OPS_INCLUDE_ALLOY;

const DEFAULT_RANGE = { min: -1000, max: 1000 };

const pidRanges = {
  p: buildRange(process.env.PID_P_MIN, process.env.PID_P_MAX),
  i: buildRange(process.env.PID_I_MIN, process.env.PID_I_MAX),
  d: buildRange(process.env.PID_D_MIN, process.env.PID_D_MAX)
};

const linePidRanges = {
  kp: buildRange(process.env.LINE_PID_KP_MIN, process.env.LINE_PID_KP_MAX, 0, 20),
  ki: buildRange(process.env.LINE_PID_KI_MIN, process.env.LINE_PID_KI_MAX, 0, 20),
  kd: buildRange(process.env.LINE_PID_KD_MIN, process.env.LINE_PID_KD_MAX, 0, 20),
  i_max: buildRange(process.env.LINE_PID_I_MAX_MIN, process.env.LINE_PID_I_MAX_MAX, 0, 20),
  out_max: buildRange(process.env.LINE_PID_OUT_MAX_MIN, process.env.LINE_PID_OUT_MAX_MAX, 0, 20),
  base_speed: buildRange(process.env.LINE_PID_BASE_SPEED_MIN, process.env.LINE_PID_BASE_SPEED_MAX, 0, 5),
  min_speed: buildRange(process.env.LINE_PID_MIN_SPEED_MIN, process.env.LINE_PID_MIN_SPEED_MAX, 0, 5),
  max_speed: buildRange(process.env.LINE_PID_MAX_SPEED_MIN, process.env.LINE_PID_MAX_SPEED_MAX, 0, 5),
  follow_max_speed: buildRange(process.env.LINE_PID_FOLLOW_MAX_SPEED_MIN, process.env.LINE_PID_FOLLOW_MAX_SPEED_MAX, 0, 5),
  turn_slowdown: buildRange(process.env.LINE_PID_TURN_SLOWDOWN_MIN, process.env.LINE_PID_TURN_SLOWDOWN_MAX, 0, 5),
  error_slowdown: buildRange(process.env.LINE_PID_ERROR_SLOWDOWN_MIN, process.env.LINE_PID_ERROR_SLOWDOWN_MAX, 0, 5),
  deadband: buildRange(process.env.LINE_PID_DEADBAND_MIN, process.env.LINE_PID_DEADBAND_MAX, 0, 1)
};

const OPS_SERVICE_ALLOWLIST = new Set([
  'perception',
  'perception-rpicam',
  'control-communication',
  'control-screen',
  'metrics-aggregator',
  'state-machine',
  'computer-vision',
  'robot-mock',
  'prometheus',
  'loki',
  'alloy',
  'grafana',
  'node-exporter'
]);

const OPS_SERVICE_GROUPS = {
  core: ['control-communication', 'state-machine', 'control-screen', 'robot-mock'],
  observability: [
    'metrics-aggregator',
    'prometheus',
    'loki',
    ...(OPS_SUPPORTS_ALLOY ? ['alloy'] : []),
    'grafana',
    'node-exporter'
  ],
  full: [
    'control-communication',
    'state-machine',
    'control-screen',
    'robot-mock',
    'metrics-aggregator',
    'prometheus',
    'loki',
    ...(OPS_SUPPORTS_ALLOY ? ['alloy'] : []),
    'grafana',
    'node-exporter',
    'computer-vision',
    'perception'
  ]
};

const insightsLogger = createInsightsLogger(LOG_PATH);

app.use(express.json({ limit: '32kb' }));
app.use(express.static(path.join(__dirname, 'public')));

app.get(['/', '/pid'], (req, res) => {
  res.sendFile(path.join(__dirname, 'public', 'pid.html'));
});

app.get('/control', (req, res) => {
  res.sendFile(path.join(__dirname, 'public', 'control.html'));
});

app.get('/ops', (req, res) => {
  res.sendFile(path.join(__dirname, 'public', 'ops.html'));
});

app.get('/robot-mock', (req, res) => {
  res.redirect(ROBOT_MOCK_URL);
});

app.get('/api/config', (req, res) => {
  res.json({
    ranges: pidRanges,
    linePidRanges,
    controlComm: {
      baseUrl: CONTROL_COMM_BASE_URL,
      paths: {
        p: CONTROL_COMM_P_PATH,
        i: CONTROL_COMM_I_PATH,
        d: CONTROL_COMM_D_PATH,
        control: CONTROL_COMM_CONTROL_PATH
      },
      robotMockUrl: ROBOT_MOCK_URL
    },
    stateMachine: {
      baseUrl: STATE_MACHINE_BASE_URL,
      paths: {
        inputs: STATE_MACHINE_INPUT_PATH,
        lineFollowPid: STATE_MACHINE_LINE_FOLLOW_PID_PATH,
        states: STATE_MACHINE_STATES_PATH,
        setState: STATE_MACHINE_SET_STATE_PATH
      }
    }
  });
});

app.get('/api/pid', async (req, res) => {
  const [pResult, iResult, dResult] = await Promise.all([
    fetchPidValue('p'),
    fetchPidValue('i'),
    fetchPidValue('d')
  ]);

  const values = {};
  const errors = {};

  if (pResult.error) {
    errors.p = pResult.error;
  } else {
    values.p = pResult.value;
  }

  if (iResult.error) {
    errors.i = iResult.error;
  } else {
    values.i = iResult.value;
  }

  if (dResult.error) {
    errors.d = dResult.error;
  } else {
    values.d = dResult.value;
  }

  if (Object.keys(errors).length > 0) {
    logInsight('pid_fetch_error', { errors, values });
    return res.status(502).json({
      message: 'Failed to fetch PID values from control communication.',
      values,
      errors
    });
  }

  logInsight('pid_fetch_ok', { values });
  return res.json({ values });
});

app.get('/api/status', async (req, res) => {
  const health = await fetchControlHealth();
  if (health.error) {
    logInsight('control_comm_health_error', { error: health.error });
    return res.status(502).json({
      message: 'Failed to reach control communication service.',
      error: health.error
    });
  }

  logInsight('control_comm_health_ok', { status: health.status });
  return res.json({
    status: health.status
  });
});

app.post('/api/pid', async (req, res) => {
  const { p, i, d } = req.body || {};
  const updates = {};
  const validationErrors = {};

  collectUpdate('p', p, updates, validationErrors);
  collectUpdate('i', i, updates, validationErrors);
  collectUpdate('d', d, updates, validationErrors);

  if (Object.keys(updates).length === 0) {
    return res.status(400).json({
      message: 'Provide at least one PID value to update.',
      errors: validationErrors
    });
  }

  if (Object.keys(validationErrors).length > 0) {
    logInsight('pid_update_rejected', { errors: validationErrors });
    return res.status(400).json({
      message: 'One or more PID values are invalid.',
      errors: validationErrors
    });
  }

  const updateEntries = Object.entries(updates);
  const results = await Promise.all(
    updateEntries.map(([key, value]) => setPidValue(key, value))
  );

  const updated = {};
  const errors = {};

  results.forEach((result, index) => {
    const key = updateEntries[index][0];
    if (result.error) {
      errors[key] = result.error;
    } else {
      updated[key] = result.value;
    }
  });

  if (Object.keys(errors).length > 0) {
    logInsight('pid_update_error', { errors, updated });
    return res.status(502).json({
      message: 'Failed to update PID values on control communication.',
      updated,
      errors
    });
  }

  logInsight('pid_update_ok', { updated });
  return res.json({ updated });
});

app.get('/api/line-follow-pid', async (req, res) => {
  const result = await fetchLineFollowPidSettings();
  if (result.error) {
    logInsight('line_follow_pid_fetch_error', { error: result.error });
    return res.status(502).json({
      message: 'Failed to fetch line-follow PID settings.',
      error: result.error
    });
  }
  return res.json(result);
});

app.post('/api/line-follow-pid', async (req, res) => {
  const payload = req.body || {};
  if (!payload || typeof payload !== 'object' || Array.isArray(payload)) {
    return res.status(400).json({ message: 'Payload must be an object.' });
  }

  const updates = {};
  const validationErrors = {};
  for (const [key, rawValue] of Object.entries(payload)) {
    if (!linePidRanges[key]) {
      validationErrors[key] = 'Unknown setting.';
      continue;
    }
    const value = Number(rawValue);
    if (!Number.isFinite(value)) {
      validationErrors[key] = 'Value must be a finite number.';
      continue;
    }
    const range = linePidRanges[key];
    if (value < range.min || value > range.max) {
      validationErrors[key] = `Value must be between ${range.min} and ${range.max}.`;
      continue;
    }
    updates[key] = value;
  }

  if (Object.keys(validationErrors).length > 0) {
    return res.status(400).json({
      message: 'Invalid line-follow PID update.',
      errors: validationErrors
    });
  }
  if (Object.keys(updates).length === 0) {
    return res.status(400).json({ message: 'No settings to update.' });
  }

  if (
    (updates.min_speed !== undefined && updates.max_speed !== undefined && updates.min_speed > updates.max_speed)
  ) {
    return res.status(400).json({
      message: 'Invalid speed bounds.',
      errors: {
        min_speed: 'min_speed must be <= max_speed.',
        max_speed: 'max_speed must be >= min_speed.'
      }
    });
  }

  const result = await setLineFollowPidSettings(updates);
  if (result.error) {
    logInsight('line_follow_pid_update_error', { error: result.error, updates });
    return res.status(502).json({
      message: 'Failed to update line-follow PID settings.',
      error: result.error
    });
  }
  logInsight('line_follow_pid_update_ok', { updates });
  return res.json(result);
});

app.get('/api/state-machine/states', async (req, res) => {
  const result = await fetchStateMachineStates();
  if (result.error) {
    logInsight('state_machine_states_fetch_error', { error: result.error });
    return res.status(502).json({
      message: 'Failed to fetch state-machine states.',
      error: result.error
    });
  }
  return res.json(result);
});

app.post('/api/state-machine/set-state', async (req, res) => {
  const rawState = req.body?.state;
  if (typeof rawState !== 'string' || !rawState.trim()) {
    return res.status(400).json({ message: "Missing 'state' in payload." });
  }

  const result = await setStateMachineState(rawState.trim());
  if (result.error) {
    logInsight('state_machine_set_state_error', { error: result.error, state: rawState });
    return res.status(502).json({
      message: 'Failed to set state-machine state.',
      error: result.error
    });
  }
  logInsight('state_machine_set_state_ok', { state: rawState });
  return res.json(result);
});

app.get('/api/ops/config', (req, res) => {
  res.json({
    enabled: ENABLE_OPS_DASHBOARD,
    composeFile: OPS_COMPOSE_FILE,
    groups: OPS_SERVICE_GROUPS,
    supportsAlloy: OPS_SUPPORTS_ALLOY,
    architecture: process.arch
  });
});

app.get('/api/ops/services', async (req, res) => {
  if (!ENABLE_OPS_DASHBOARD) {
    return res.status(403).json({ message: 'Ops dashboard commands are disabled.' });
  }
  const result = await listComposeServices();
  if (result.error) {
    logInsight('ops_services_error', { error: result.error, detail: result.detail });
    return res.status(502).json({
      message: 'Failed to query docker compose services.',
      error: result.error,
      detail: result.detail
    });
  }
  return res.json(result);
});

app.post('/api/ops/stack/up', async (req, res) => {
  if (!ENABLE_OPS_DASHBOARD) {
    return res.status(403).json({ message: 'Ops dashboard commands are disabled.' });
  }
  const selection = resolveServiceSelection(req.body, false);
  if (selection.error) {
    return res.status(400).json({ message: selection.error });
  }

  const result = await runComposeCommand(['up', '-d', ...selection.services]);
  if (!result.ok) {
    logInsight('ops_stack_up_error', { selection, result });
    return res.status(502).json({
      message: 'Failed to start selected services.',
      ...result
    });
  }
  logInsight('ops_stack_up_ok', { selection, exitCode: result.exitCode });
  return res.json({
    message: 'Services started.',
    services: selection.services,
    ...result
  });
});

app.post('/api/ops/stack/stop', async (req, res) => {
  if (!ENABLE_OPS_DASHBOARD) {
    return res.status(403).json({ message: 'Ops dashboard commands are disabled.' });
  }
  const selection = resolveServiceSelection(req.body, false);
  if (selection.error) {
    return res.status(400).json({ message: selection.error });
  }

  const result = await runComposeCommand(['stop', ...selection.services]);
  if (!result.ok) {
    logInsight('ops_stack_stop_error', { selection, result });
    return res.status(502).json({
      message: 'Failed to stop selected services.',
      ...result
    });
  }
  logInsight('ops_stack_stop_ok', { selection, exitCode: result.exitCode });
  return res.json({
    message: 'Services stopped.',
    services: selection.services,
    ...result
  });
});

app.post('/api/ops/stack/down', async (req, res) => {
  if (!ENABLE_OPS_DASHBOARD) {
    return res.status(403).json({ message: 'Ops dashboard commands are disabled.' });
  }
  const selection = resolveServiceSelection(req.body, true);
  if (selection.error) {
    return res.status(400).json({ message: selection.error });
  }

  if (selection.serviceArgs.length === 0) {
    const result = await runComposeCommand(['down']);
    if (!result.ok) {
      logInsight('ops_stack_down_error', { selection, result });
      return res.status(502).json({
        message: 'Failed to bring down compose project.',
        ...result
      });
    }
    logInsight('ops_stack_down_ok', { selection, exitCode: result.exitCode });
    return res.json({
      message: 'Compose project brought down.',
      services: selection.services,
      ...result
    });
  }

  const stopResult = await runComposeCommand(['stop', ...selection.serviceArgs]);
  if (!stopResult.ok) {
    logInsight('ops_stack_down_error', { selection, result: stopResult });
    return res.status(502).json({
      message: 'Failed to stop selected services before removal.',
      ...stopResult
    });
  }

  const rmResult = await runComposeCommand(['rm', '-f', ...selection.serviceArgs]);
  if (!rmResult.ok) {
    logInsight('ops_stack_down_error', { selection, result: rmResult });
    return res.status(502).json({
      message: 'Failed to remove selected services.',
      ...rmResult
    });
  }

  logInsight('ops_stack_down_ok', { selection, exitCode: rmResult.exitCode });
  return res.json({
    message: 'Selected services removed.',
    services: selection.services,
    ...rmResult
  });
});

app.post('/api/ops/service/restart', async (req, res) => {
  if (!ENABLE_OPS_DASHBOARD) {
    return res.status(403).json({ message: 'Ops dashboard commands are disabled.' });
  }
  const service = sanitizeServiceName(req.body?.service);
  if (!service) {
    return res.status(400).json({ message: 'Invalid or missing service name.' });
  }

  const result = await runComposeCommand(['restart', service]);
  if (!result.ok) {
    logInsight('ops_service_restart_error', { service, result });
    return res.status(502).json({
      message: `Failed to restart ${service}.`,
      ...result
    });
  }
  logInsight('ops_service_restart_ok', { service, exitCode: result.exitCode });
  return res.json({
    message: `${service} restarted.`,
    service,
    ...result
  });
});

app.get('/api/ops/logs', async (req, res) => {
  if (!ENABLE_OPS_DASHBOARD) {
    return res.status(403).json({ message: 'Ops dashboard commands are disabled.' });
  }
  const service = sanitizeServiceName(req.query?.service);
  if (!service) {
    return res.status(400).json({ message: 'Provide a valid service query parameter.' });
  }

  const rawLines = Number.parseInt(String(req.query?.lines || OPS_LOG_TAIL_DEFAULT), 10);
  const lines = Number.isFinite(rawLines)
    ? Math.max(1, Math.min(rawLines, OPS_LOG_TAIL_MAX))
    : OPS_LOG_TAIL_DEFAULT;

  const result = await runComposeCommand(['logs', '--no-color', '--tail', String(lines), service], {
    timeoutMs: OPS_COMMAND_TIMEOUT_MS * 2
  });
  if (!result.ok) {
    logInsight('ops_logs_error', { service, lines, result });
    return res.status(502).json({
      message: `Failed to fetch logs for ${service}.`,
      service,
      lines,
      ...result
    });
  }
  return res.json({
    service,
    lines,
    ...result
  });
});

app.post('/api/ops/tests/neutral-control', async (req, res) => {
  if (!ENABLE_OPS_DASHBOARD) {
    return res.status(403).json({ message: 'Ops dashboard commands are disabled.' });
  }
  const result = await sendRedLineInput({ x: 0, y: 0 });
  if (!result || result.error) {
    logInsight('ops_neutral_test_error', { error: result?.error });
    return res.status(502).json({
      message: 'Failed to send neutral control test command.',
      error: result?.error
    });
  }
  return res.json({
    message: 'Neutral control test sent.',
    command: result.command
  });
});

app.post('/api/ops/tests/sample-input', async (req, res) => {
  if (!ENABLE_OPS_DASHBOARD) {
    return res.status(403).json({ message: 'Ops dashboard commands are disabled.' });
  }
  const vector = req.body?.vector || {};
  const parsedX = Number(vector.x);
  const parsedY = Number(vector.y);
  const command = {
    x: Number.isFinite(parsedX) ? clamp(parsedX, -1, 1) : 0.2,
    y: Number.isFinite(parsedY) ? clamp(parsedY, -1, 1) : 0.8
  };
  const result = await sendRedLineInput(command);
  if (!result || result.error) {
    logInsight('ops_sample_input_error', { error: result?.error, command });
    return res.status(502).json({
      message: 'Failed to send sample input test command.',
      error: result?.error
    });
  }
  return res.json({
    message: 'Sample input test sent.',
    command: result.command
  });
});

app.post('/api/control', async (req, res) => {
  const { x, y, speed, servo } = req.body || {};
  const errors = {};

  const parsedX = parseNumberValue(x);
  const parsedY = parseNumberValue(y);
  const parsedSpeed = parseNumberValue(speed);
  const parsedServo = parseOptionalIntegerValue(servo);

  if (!Number.isFinite(parsedX)) {
    errors.x = 'X must be a finite number.';
  }
  if (!Number.isFinite(parsedY)) {
    errors.y = 'Y must be a finite number.';
  }
  if (speed !== undefined && speed !== null && speed !== '' && !Number.isFinite(parsedSpeed)) {
    errors.speed = 'Speed must be a finite number.';
  }
  if (servo !== undefined && servo !== null && servo !== '' && !Number.isFinite(parsedServo)) {
    errors.servo = 'Servo must be an integer.';
  }

  if (Number.isFinite(parsedX) && (parsedX < -1 || parsedX > 1)) {
    errors.x = 'X must be between -1 and 1.';
  }
  if (Number.isFinite(parsedY) && (parsedY < -1 || parsedY > 1)) {
    errors.y = 'Y must be between -1 and 1.';
  }
  if (Number.isFinite(parsedSpeed) && (parsedSpeed < 0 || parsedSpeed > 1)) {
    errors.speed = 'Speed must be between 0 and 1.';
  }
  if (
    Number.isFinite(parsedServo)
    && (parsedServo < SERVO_MIN_DEG || parsedServo > SERVO_MAX_DEG)
  ) {
    errors.servo = `Servo must be between ${SERVO_MIN_DEG} and ${SERVO_MAX_DEG}.`;
  }

  if (Object.keys(errors).length > 0) {
    return res.status(400).json({
      message: 'Invalid control vector.',
      errors
    });
  }

  const command = {
    x: parsedX,
    y: parsedY
  };
  if (Number.isFinite(parsedSpeed)) {
    command.speed = parsedSpeed;
  }
  if (Number.isFinite(parsedServo)) {
    command.servo = parsedServo;
  }

  const result = await sendRedLineInput(command);

  if (!result || result.error) {
    logInsight('red_line_input_error', { error: result?.error });
    return res.status(502).json({
      message: 'Failed to send red-line input to state machine.',
      error: result?.error
    });
  }

  logInsight('red_line_input_ok', { command: result.command });
  return res.json({ command: result.command });
});

app.listen(PORT, () => {
  logInsight('control_screen_started', { port: PORT });
  logServiceLine(`Control screen running on http://localhost:${PORT}`);
});

function buildRange(minValue, maxValue, fallbackMin = DEFAULT_RANGE.min, fallbackMax = DEFAULT_RANGE.max) {
  const min = parseNumber(minValue, fallbackMin);
  const max = parseNumber(maxValue, fallbackMax);

  if (min > max) {
    return { min: max, max: min };
  }

  return { min, max };
}

function parseNumber(value, fallback) {
  if (value === undefined || value === null || value === '') {
    return fallback;
  }

  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function parseNumberValue(value) {
  if (value === undefined || value === null || value === '') {
    return Number.NaN;
  }

  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : Number.NaN;
}

function parseOptionalIntegerValue(value) {
  if (value === undefined || value === null || value === '') {
    return Number.NaN;
  }
  const parsed = Number(value);
  if (!Number.isFinite(parsed) || !Number.isInteger(parsed)) {
    return Number.NaN;
  }
  return parsed;
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function sanitizeServiceName(rawValue) {
  if (typeof rawValue !== 'string') {
    return null;
  }
  const value = rawValue.trim();
  if (!value) {
    return null;
  }
  if (!/^[a-z0-9][a-z0-9_-]*$/i.test(value)) {
    return null;
  }
  if (!OPS_SERVICE_ALLOWLIST.has(value)) {
    return null;
  }
  return value;
}

function resolveServiceSelection(payload, allowDownAll) {
  const requestedGroup = payload?.group;
  if (typeof requestedGroup === 'string' && requestedGroup.trim()) {
    const group = requestedGroup.trim();
    const services = OPS_SERVICE_GROUPS[group];
    if (!Array.isArray(services)) {
      return { error: `Unknown service group: ${group}` };
    }
    if (allowDownAll && group === 'full') {
      return { group, services, serviceArgs: [] };
    }
    return { group, services, serviceArgs: services };
  }

  const rawServices = Array.isArray(payload?.services) ? payload.services : [];
  const services = rawServices
    .map((service) => sanitizeServiceName(service))
    .filter(Boolean);
  const uniqueServices = Array.from(new Set(services));

  if (uniqueServices.length === 0) {
    if (allowDownAll) {
      return { group: 'all', services: [], serviceArgs: [] };
    }
    return { error: 'Provide a valid service group or service list.' };
  }
  return { group: 'custom', services: uniqueServices, serviceArgs: uniqueServices };
}

function collectUpdate(key, rawValue, updates, validationErrors) {
  if (rawValue === undefined || rawValue === null || rawValue === '') {
    return;
  }

  const value = Number(rawValue);
  if (!Number.isFinite(value)) {
    validationErrors[key] = 'Value must be a finite number.';
    return;
  }

  const range = pidRanges[key];
  if (value < range.min || value > range.max) {
    validationErrors[key] = `Value must be between ${range.min} and ${range.max}.`;
    return;
  }

  updates[key] = value;
}

function parsePidValue(rawText) {
  const trimmed = String(rawText || '').trim();
  if (!trimmed) {
    return Number.NaN;
  }

  try {
    const data = JSON.parse(trimmed);
    if (typeof data === 'number') {
      return data;
    }

    if (typeof data === 'string') {
      return Number(data);
    }

    if (data && typeof data === 'object') {
      const candidates = ['value', 'p', 'i', 'd', 'P', 'I', 'D'];
      for (const key of candidates) {
        if (data[key] !== undefined) {
          return Number(data[key]);
        }
      }
    }
  } catch (err) {
    // Ignore JSON parse errors and attempt numeric parsing below.
  }

  const parsed = Number(trimmed);
  return Number.isFinite(parsed) ? parsed : Number.NaN;
}

function safeJsonParse(text) {
  try {
    return JSON.parse(text);
  } catch (err) {
    return null;
  }
}

function getControlCommUrl(kind) {
  let pathPart = CONTROL_COMM_P_PATH;
  if (kind === 'i') {
    pathPart = CONTROL_COMM_I_PATH;
  } else if (kind === 'd') {
    pathPart = CONTROL_COMM_D_PATH;
  }

  return new URL(pathPart, CONTROL_COMM_BASE_URL).toString();
}

function getControlCommHealthUrl() {
  return new URL(CONTROL_COMM_HEALTH_PATH, CONTROL_COMM_BASE_URL).toString();
}

function getControlCommLineFollowPidUrl() {
  return new URL(STATE_MACHINE_LINE_FOLLOW_PID_PATH, STATE_MACHINE_BASE_URL).toString();
}

function getStateMachineStatesUrl() {
  return new URL(STATE_MACHINE_STATES_PATH, STATE_MACHINE_BASE_URL).toString();
}

function getStateMachineSetStateUrl() {
  return new URL(STATE_MACHINE_SET_STATE_PATH, STATE_MACHINE_BASE_URL).toString();
}

function getStateMachineInputsUrl() {
  return new URL(STATE_MACHINE_INPUT_PATH, STATE_MACHINE_BASE_URL).toString();
}

async function fetchControlHealth() {
  const url = getControlCommHealthUrl();

  try {
    const response = await fetch(url, {
      method: 'GET',
      headers: {
        Accept: 'application/json, text/plain'
      }
    });

    const body = await response.text();
    if (!response.ok) {
      return { error: `Upstream status ${response.status}`, detail: body };
    }

    const parsed = safeJsonParse(body);
    if (!parsed || typeof parsed !== 'object') {
      return { error: 'Upstream returned invalid status payload.', detail: body };
    }

    return { status: parsed };
  } catch (err) {
    return { error: err.message };
  }
}

async function fetchPidValue(kind) {
  const url = getControlCommUrl(kind);

  try {
    const response = await fetch(url, {
      method: 'GET',
      headers: {
        Accept: 'application/json, text/plain'
      }
    });

    const body = await response.text();
    if (!response.ok) {
      return { error: `Upstream status ${response.status}`, detail: body };
    }

    const value = parsePidValue(body);
    if (!Number.isFinite(value)) {
      return { error: 'Upstream returned invalid value.', detail: body };
    }

    return { value };
  } catch (err) {
    return { error: err.message };
  }
}

async function setPidValue(kind, value) {
  const url = getControlCommUrl(kind);

  try {
    const response = await fetch(url, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        Accept: 'application/json, text/plain'
      },
      body: JSON.stringify({ value })
    });

    const body = await response.text();
    if (!response.ok) {
      return { error: `Upstream status ${response.status}`, detail: body };
    }

    const parsed = parsePidValue(body);
    if (!Number.isFinite(parsed)) {
      return { value };
    }

    return { value: parsed };
  } catch (err) {
    return { error: err.message };
  }
}

async function sendRedLineInput(command) {
  const url = getStateMachineInputsUrl();
  const payload = {
    red_line: {
      detected: true,
      vector: {
        x: command.x,
        y: command.y
      }
    }
  };

  try {
    const response = await fetch(url, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        Accept: 'application/json, text/plain'
      },
      body: JSON.stringify(payload)
    });

    const body = await response.text();
    if (!response.ok) {
      return { error: `Upstream status ${response.status}`, detail: body };
    }

    const parsed = safeJsonParse(body);
    // Keep response shape expected by control UI.
    return { command };
  } catch (err) {
    return { error: err.message };
  }
}

async function fetchLineFollowPidSettings() {
  const url = getControlCommLineFollowPidUrl();
  try {
    const response = await fetch(url, {
      method: 'GET',
      headers: { Accept: 'application/json, text/plain' }
    });
    const body = await response.text();
    if (!response.ok) {
      return { error: `Upstream status ${response.status}`, detail: body };
    }
    const parsed = safeJsonParse(body);
    if (!parsed || typeof parsed !== 'object' || !parsed.values) {
      return { error: 'Upstream returned invalid line-follow payload.', detail: body };
    }
    return {
      values: parsed.values,
      bounds: parsed.bounds || linePidRanges
    };
  } catch (err) {
    return { error: err.message };
  }
}

async function setLineFollowPidSettings(updates) {
  const url = getControlCommLineFollowPidUrl();
  try {
    const response = await fetch(url, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        Accept: 'application/json, text/plain'
      },
      body: JSON.stringify(updates)
    });
    const body = await response.text();
    if (!response.ok) {
      return { error: `Upstream status ${response.status}`, detail: body };
    }
    const parsed = safeJsonParse(body);
    if (!parsed || typeof parsed !== 'object') {
      return { values: updates };
    }
    return {
      values: parsed.values || updates,
      updated: parsed.updated || updates
    };
  } catch (err) {
    return { error: err.message };
  }
}

async function fetchStateMachineStates() {
  const url = getStateMachineStatesUrl();
  try {
    const response = await fetch(url, {
      method: 'GET',
      headers: { Accept: 'application/json, text/plain' }
    });
    const body = await response.text();
    if (!response.ok) {
      return { error: `Upstream status ${response.status}`, detail: body };
    }
    const parsed = safeJsonParse(body);
    if (!parsed || typeof parsed !== 'object' || !Array.isArray(parsed.states)) {
      return { error: 'Upstream returned invalid state list payload.', detail: body };
    }
    return {
      states: parsed.states,
      current_state: parsed.current_state
    };
  } catch (err) {
    return { error: err.message };
  }
}

async function setStateMachineState(nextState) {
  const setStateUrl = getStateMachineSetStateUrl();
  const statesUrl = getStateMachineStatesUrl();
  try {
    const response = await fetch(setStateUrl, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        Accept: 'application/json, text/plain'
      },
      body: JSON.stringify({ state: nextState })
    });
    const body = await response.text();
    if (!response.ok) {
      return { error: `Upstream status ${response.status}`, detail: body };
    }

    const parsed = safeJsonParse(body);
    const requestedState = parsed?.state || nextState;

    const statesResponse = await fetch(statesUrl, {
      method: 'GET',
      headers: { Accept: 'application/json, text/plain' }
    });
    const statesBody = await statesResponse.text();
    if (!statesResponse.ok) {
      return { error: `Upstream status ${statesResponse.status}`, detail: statesBody };
    }
    const statesParsed = safeJsonParse(statesBody);
    const currentState = statesParsed?.current_state || requestedState;

    return {
      state: currentState
    };
  } catch (err) {
    return { error: err.message };
  }
}

async function listComposeServices() {
  const result = await runComposeCommand(['ps', '--all', '--format', 'json']);
  if (!result.ok) {
    return {
      error: result.stderr || 'docker compose ps failed.',
      detail: result.stdout
    };
  }

  const rows = parseComposePsRows(result.stdout);
  const rowMap = new Map();
  rows.forEach((row) => {
    const service = typeof row.Service === 'string' ? row.Service : null;
    if (service && OPS_SERVICE_ALLOWLIST.has(service)) {
      rowMap.set(service, row);
    }
  });

  const services = Array.from(OPS_SERVICE_ALLOWLIST).map((service) => {
    const row = rowMap.get(service);
    const state = row?.State || 'stopped';
    return {
      service,
      state,
      status: row?.Status || (state === 'running' ? 'Up' : 'Stopped'),
      containerName: row?.Name || service
    };
  });

  return {
    services,
    raw: rows
  };
}

function parseComposePsRows(stdout) {
  const text = String(stdout || '').trim();
  if (!text) {
    return [];
  }

  if (text.startsWith('[')) {
    const parsed = safeJsonParse(text);
    if (Array.isArray(parsed)) {
      return parsed;
    }
  }

  const rows = [];
  for (const line of text.split('\n')) {
    const trimmed = line.trim();
    if (!trimmed) {
      continue;
    }
    const parsed = safeJsonParse(trimmed);
    if (parsed && typeof parsed === 'object') {
      rows.push(parsed);
    }
  }
  return rows;
}

async function runComposeCommand(args, options = {}) {
  const finalArgs = ['compose', '-f', OPS_COMPOSE_FILE, ...args];
  return runCommand(OPS_DOCKER_BIN, finalArgs, options);
}

function runCommand(command, args, options = {}) {
  const timeoutMs = Number.isFinite(options.timeoutMs) ? options.timeoutMs : OPS_COMMAND_TIMEOUT_MS;
  const maxBytes = Number.isFinite(options.maxBytes) ? options.maxBytes : OPS_MAX_OUTPUT_BYTES;

  return new Promise((resolve) => {
    const child = spawn(command, args, {
      stdio: ['ignore', 'pipe', 'pipe']
    });

    let stdout = '';
    let stderr = '';
    let stdoutTruncated = false;
    let stderrTruncated = false;
    let didTimeout = false;
    let resolved = false;

    const maybeTrim = (text, isStdout) => {
      if (text.length <= maxBytes) {
        return text;
      }
      if (isStdout) {
        stdoutTruncated = true;
      } else {
        stderrTruncated = true;
      }
      return text.slice(text.length - maxBytes);
    };

    const timer = setTimeout(() => {
      didTimeout = true;
      child.kill('SIGTERM');
    }, timeoutMs);

    child.stdout.on('data', (chunk) => {
      stdout = maybeTrim(stdout + chunk.toString(), true);
    });
    child.stderr.on('data', (chunk) => {
      stderr = maybeTrim(stderr + chunk.toString(), false);
    });

    child.on('error', (err) => {
      if (resolved) {
        return;
      }
      resolved = true;
      clearTimeout(timer);
      resolve({
        ok: false,
        exitCode: -1,
        stdout,
        stderr: `${stderr}\n${err.message}`.trim(),
        timedOut: false,
        stdoutTruncated,
        stderrTruncated
      });
    });

    child.on('close', (code) => {
      if (resolved) {
        return;
      }
      resolved = true;
      clearTimeout(timer);
      resolve({
        ok: !didTimeout && code === 0,
        exitCode: typeof code === 'number' ? code : -1,
        stdout,
        stderr: didTimeout ? `${stderr}\nCommand timed out.`.trim() : stderr,
        timedOut: didTimeout,
        stdoutTruncated,
        stderrTruncated
      });
    });
  });
}

function createInsightsLogger(logPath) {
  if (!logPath) {
    return null;
  }

  try {
    fs.mkdirSync(path.dirname(logPath), { recursive: true });
    const stream = fs.createWriteStream(logPath, { flags: 'a' });
    stream.on('error', (err) => {
      console.warn(`Insights IPC pipe error: ${err.message}`);
    });
    return stream;
  } catch (err) {
    console.warn(`Unable to open log file: ${err.message}`);
    return null;
  }
}

function logServiceLine(message) {
  const entry = {
    ts: new Date().toISOString(),
    event: 'service_log',
    message
  };
  const line = JSON.stringify(entry);
  console.log(line);
  if (insightsLogger) {
    insightsLogger.write(`${line}\n`);
  }
}

function logInsight(event, payload) {
  const entry = {
    ts: new Date().toISOString(),
    event,
    payload
  };

  const line = JSON.stringify(entry);
  console.log(line);

  if (insightsLogger) {
    insightsLogger.write(`${line}\n`);
  }
}
