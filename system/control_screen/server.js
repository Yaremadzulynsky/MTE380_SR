const express = require('express');
const path = require('path');
const fs = require('fs');
const http = require('http');
const https = require('https');
const { URL } = require('url');
const { spawn } = require('child_process');

const app = express();
const PORT = Number.parseInt(process.env.PORT, 10) || 3000;

const CONTROL_COMM_BASE_URL =
  process.env.CONTROL_COMM_BASE_URL || 'http://localhost:5001';
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
const CONTROL_COMM_TURN_TEST_PATH =
  process.env.CONTROL_COMM_TURN_TEST_PATH || '/turn-test';
const CONTROL_COMM_LINE_FOLLOW_PID_PATH =
  process.env.CONTROL_COMM_LINE_FOLLOW_PID_PATH || '/line-follow-pid';
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
const SERVO_MIN_DEG = Number.parseInt(process.env.SERVO_MIN_DEG || '0', 10);
const SERVO_MAX_DEG = Number.parseInt(process.env.SERVO_MAX_DEG || '90', 10);
const OPS_ROBOT_START_STATE = (process.env.OPS_ROBOT_START_STATE || 'searching').trim();
const OPS_HOUGH_STREAM_URL = process.env.OPS_HOUGH_STREAM_URL || 'http://localhost:8090/stream.mjpg';
const PERCEPTION_RUNNER_ENABLED = String(process.env.ENABLE_PERCEPTION_RUNNER || 'true').toLowerCase() !== 'false';
const PERCEPTION_RUN_SCRIPT = process.env.PERCEPTION_RUN_SCRIPT
  || path.resolve(__dirname, '..', 'run_perception_rpicam.sh');
const PERCEPTION_RUN_CWD = process.env.PERCEPTION_RUN_CWD || path.resolve(__dirname, '..');
const PERCEPTION_LOG_FILE = process.env.PERCEPTION_LOG_FILE
  || path.resolve(__dirname, '..', 'perception', 'logs', 'control-screen-perception.log');
const PERCEPTION_LOG_TAIL_DEFAULT = Number.parseInt(process.env.PERCEPTION_LOG_TAIL_DEFAULT || '200', 10);
const PERCEPTION_LOG_TAIL_MAX = Number.parseInt(process.env.PERCEPTION_LOG_TAIL_MAX || '4000', 10);
const PERCEPTION_STOP_TIMEOUT_MS = Number.parseInt(process.env.PERCEPTION_STOP_TIMEOUT_MS || '3500', 10);

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
  deadband: buildRange(process.env.LINE_PID_DEADBAND_MIN, process.env.LINE_PID_DEADBAND_MAX, 0, 1),
  rotation_scale: buildRange(process.env.LINE_PID_ROTATION_SCALE_MIN, process.env.LINE_PID_ROTATION_SCALE_MAX, 0, 1),
  line_lag_tau: buildRange(process.env.LINE_PID_LAG_TAU_MIN, process.env.LINE_PID_LAG_TAU_MAX, 0, 2),
  line_lag_enabled: buildRange(process.env.LINE_PID_LAG_ENABLED_MIN, process.env.LINE_PID_LAG_ENABLED_MAX, 0, 1)
};

const perceptionRunner = createPerceptionRunner();

app.use(express.json({ limit: '32kb' }));

// Serve HTML shells before static files so routes like /ops are never shadowed.
app.get(['/', '/pid'], (req, res) => {
  res.sendFile(path.join(__dirname, 'public', 'pid.html'));
});

app.get('/control', (req, res) => {
  res.sendFile(path.join(__dirname, 'public', 'control.html'));
});

app.get('/ops', (req, res) => {
  res.sendFile(path.join(__dirname, 'public', 'ops.html'));
});

app.use(express.static(path.join(__dirname, 'public')));

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
      }
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
    enabled: PERCEPTION_RUNNER_ENABLED,
    perceptionRunnerEnabled: PERCEPTION_RUNNER_ENABLED,
    houghStreamUrl: OPS_HOUGH_STREAM_URL,
    /** Same-origin MJPEG relay — more reliable than pointing <img> at another host/port. */
    mjpegProxyPath: '/api/ops/mjpeg-stream',
    perceptionScript: PERCEPTION_RUN_SCRIPT
  });
});

/**
 * Proxy MJPEG from OPS_HOUGH_STREAM_URL so the browser loads same-origin video
 * (avoids mixed-origin quirks and some browsers mishandling long multipart streams).
 */
app.get('/api/ops/mjpeg-stream', (req, res) => {
  let target;
  try {
    target = new URL(OPS_HOUGH_STREAM_URL);
  } catch (e) {
    res.status(500).type('text/plain').send(`Invalid OPS_HOUGH_STREAM_URL: ${e.message}`);
    return;
  }
  const isHttps = target.protocol === 'https:';
  const lib = isHttps ? https : http;
  const opts = {
    hostname: target.hostname,
    port: target.port || (isHttps ? 443 : 80),
    path: `${target.pathname}${target.search}`,
    method: 'GET',
    timeout: 20000,
    headers: { Connection: 'close' }
  };

  const upstream = lib.request(opts, (upRes) => {
    const code = upRes.statusCode || 502;
    if (code < 200 || code >= 300) {
      if (!res.headersSent) {
        res.status(502).type('text/plain').send(`Upstream MJPEG returned HTTP ${code}`);
      }
      upRes.resume();
      return;
    }
    res.statusCode = 200;
    const ct = upRes.headers['content-type'];
    if (ct) {
      res.setHeader('Content-Type', ct);
    } else {
      res.setHeader('Content-Type', 'multipart/x-mixed-replace; boundary=frame');
    }
    res.setHeader('Cache-Control', 'no-cache, no-store, must-revalidate');
    res.setHeader('Pragma', 'no-cache');
    res.setHeader('X-Accel-Buffering', 'no');
    upRes.pipe(res);
  });

  upstream.on('timeout', () => {
    upstream.destroy();
    if (!res.headersSent) {
      res.status(504).type('text/plain').send('Upstream MJPEG timeout');
    }
  });
  upstream.on('error', (err) => {
    if (!res.headersSent) {
      res.status(502).type('text/plain').send(`Upstream MJPEG error: ${err.message}`);
    } else {
      try {
        res.end();
      } catch (_) {
        /* ignore */
      }
    }
  });
  req.on('close', () => upstream.destroy());
  upstream.end();
});

/** Quick check that OPS_HOUGH_STREAM_URL accepts connections (headers only). */
app.get('/api/ops/mjpeg-health', (req, res) => {
  let target;
  try {
    target = new URL(OPS_HOUGH_STREAM_URL);
  } catch (e) {
    return res.json({
      ok: false,
      error: `invalid OPS_HOUGH_STREAM_URL: ${e.message}`,
      upstreamUrl: OPS_HOUGH_STREAM_URL
    });
  }
  const isHttps = target.protocol === 'https:';
  const lib = isHttps ? https : http;
  const opts = {
    hostname: target.hostname,
    port: target.port || (isHttps ? 443 : 80),
    path: `${target.pathname}${target.search}`,
    method: 'GET',
    timeout: 5000,
    headers: { Connection: 'close' }
  };
  let answered = false;
  const finish = (payload) => {
    if (answered) return;
    answered = true;
    res.json(payload);
  };

  const upstream = lib.request(opts, (upRes) => {
    const code = upRes.statusCode || 0;
    const ok = code >= 200 && code < 300;
    finish({
      ok,
      statusCode: code,
      upstreamUrl: OPS_HOUGH_STREAM_URL,
      contentType: upRes.headers['content-type'] || null
    });
    upRes.resume();
    upstream.destroy();
  });
  upstream.on('timeout', () => {
    upstream.destroy();
    finish({
      ok: false,
      error: 'timeout',
      upstreamUrl: OPS_HOUGH_STREAM_URL
    });
  });
  upstream.on('error', (err) => {
    finish({
      ok: false,
      error: err.message,
      upstreamUrl: OPS_HOUGH_STREAM_URL
    });
  });
  upstream.end();
});

app.get('/api/ops/services', async (req, res) => {
  const status = perceptionRunner.status();
  return res.json({
    services: [
      {
        service: 'perception-runner',
        state: status.running ? 'running' : 'stopped',
        status: status.running ? 'Up' : 'Stopped',
        pid: status.pid || null
      }
    ]
  });
});

app.post('/api/ops/stack/up', async (req, res) => {
  if (!PERCEPTION_RUNNER_ENABLED) {
    return res.status(403).json({ message: 'Perception runner controls are disabled.' });
  }
  const result = perceptionRunner.start();
  if (!result.ok) {
    return res.status(409).json({
      message: result.error || 'Perception is already running.'
    });
  }
  logInsight('perception_runner_start_ok', { pid: result.pid });
  return res.json({
    message: 'Perception runner started.',
    pid: result.pid
  });
});

app.post('/api/ops/stack/stop', async (req, res) => {
  if (!PERCEPTION_RUNNER_ENABLED) {
    return res.status(403).json({ message: 'Perception runner controls are disabled.' });
  }
  const result = await perceptionRunner.stop();
  if (!result.ok) {
    logInsight('perception_runner_stop_error', result);
    return res.status(502).json({
      message: 'Failed to stop perception runner.',
      ...result
    });
  }
  logInsight('perception_runner_stop_ok', result);
  return res.json({
    message: 'Perception runner stopped.',
    ...result
  });
});

app.post('/api/ops/stack/down', async (req, res) => {
  if (!PERCEPTION_RUNNER_ENABLED) {
    return res.status(403).json({ message: 'Perception runner controls are disabled.' });
  }
  const result = await perceptionRunner.stop();
  if (!result.ok) {
    return res.status(502).json({
      message: 'Failed to stop perception runner.',
      ...result
    });
  }
  return res.json({
    message: 'Perception runner stopped.',
    ...result
  });
});

app.post('/api/ops/service/restart', async (req, res) => {
  if (!PERCEPTION_RUNNER_ENABLED) {
    return res.status(403).json({ message: 'Perception runner controls are disabled.' });
  }
  const service = String(req.body?.service || '').trim();
  if (service && service !== 'perception-runner') {
    return res.status(400).json({ message: 'Only perception-runner restart is supported.' });
  }
  const stopResult = await perceptionRunner.stop();
  if (!stopResult.ok) {
    return res.status(502).json({
      message: 'Failed to stop perception runner before restart.',
      ...stopResult
    });
  }
  const startResult = perceptionRunner.start();
  if (!startResult.ok) {
    return res.status(502).json({
      message: 'Failed to restart perception runner.',
      error: startResult.error
    });
  }
  return res.json({
    message: 'perception-runner restarted.',
    pid: startResult.pid
  });
});

app.get('/api/ops/logs', async (req, res) => {
  if (!PERCEPTION_RUNNER_ENABLED) {
    return res.status(403).json({ message: 'Perception runner controls are disabled.' });
  }
  const service = typeof req.query?.service === 'string' ? req.query.service.trim() : 'perception-runner';
  if (service !== 'perception-runner' && service !== 'perception') {
    return res.status(400).json({ message: 'Only perception-runner logs are available on this page.' });
  }

  const rawLines = Number.parseInt(String(req.query?.lines || PERCEPTION_LOG_TAIL_DEFAULT), 10);
  const lines = Number.isFinite(rawLines)
    ? Math.max(1, Math.min(rawLines, PERCEPTION_LOG_TAIL_MAX))
    : PERCEPTION_LOG_TAIL_DEFAULT;
  const result = await perceptionRunner.readLogs(lines);
  return res.json({
    service: 'perception-runner',
    lines,
    ...result
  });
});

app.get('/api/ops/perception/status', (req, res) => {
  if (!PERCEPTION_RUNNER_ENABLED) {
    return res.status(403).json({ message: 'Perception runner controls are disabled.' });
  }
  return res.json(perceptionRunner.status());
});

app.post('/api/ops/tests/neutral-control', async (req, res) => {
  if (!PERCEPTION_RUNNER_ENABLED) {
    return res.status(403).json({ message: 'Perception runner controls are disabled.' });
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
  if (!PERCEPTION_RUNNER_ENABLED) {
    return res.status(403).json({ message: 'Perception runner controls are disabled.' });
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

app.post('/api/ops/tests/turn', async (req, res) => {
  if (!PERCEPTION_RUNNER_ENABLED) {
    return res.status(403).json({ message: 'Perception runner controls are disabled.' });
  }
  const requestedDegrees = Number(req.body?.degrees);
  if (!Number.isFinite(requestedDegrees)) {
    return res.status(400).json({ message: "'degrees' must be numeric." });
  }
  const degrees = clamp(requestedDegrees, -720, 720);
  const turnResult = await runTurnTest(degrees);
  if (!turnResult.ok) {
    return res.status(502).json({
      message: 'Turn test failed.',
      error: turnResult.error,
      detail: turnResult.detail
    });
  }

  const stopResult = await sendRobotStopSignals();
  const stopStatus = stopResult.ok ? 'ok' : 'failed';
  if (!stopResult.ok) {
    logInsight('ops_turn_test_stop_error', { stopResult, degrees });
  }

  return res.json({
    message: `Turn ${degrees}° completed. Auto-stop ${stopStatus}.`,
    turn: turnResult.result,
    stop: stopResult
  });
});

app.post('/api/ops/robot/start', async (req, res) => {
  if (!PERCEPTION_RUNNER_ENABLED) {
    return res.status(403).json({ message: 'Perception runner controls are disabled.' });
  }
  const requestedState = typeof req.body?.state === 'string' && req.body.state.trim()
    ? req.body.state.trim()
    : OPS_ROBOT_START_STATE;
  const stateResult = await setStateMachineState(requestedState);
  if (stateResult.error) {
    logInsight('ops_robot_start_error', { state: requestedState, error: stateResult.error });
    return res.status(502).json({
      message: `Failed to set robot state to ${requestedState}.`,
      error: stateResult.error
    });
  }

  logInsight('ops_robot_start_ok', { state: stateResult.state || requestedState });
  return res.json({
    message: `Robot start command sent (${stateResult.state || requestedState}).`,
    state: stateResult.state || requestedState
  });
});

app.post('/api/ops/robot/stop', async (req, res) => {
  if (!PERCEPTION_RUNNER_ENABLED) {
    return res.status(403).json({ message: 'Perception runner controls are disabled.' });
  }
  const stopResult = await sendRobotStopSignals();
  if (!stopResult.ok) {
    logInsight('ops_robot_stop_error', stopResult);
    return res.status(502).json({
      message: 'Failed to send stop commands to robot services.',
      ...stopResult
    });
  }
  logInsight('ops_robot_stop_ok', stopResult);
  return res.json({
    message: 'Robot stop command sent.',
    ...stopResult
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
  console.log(`Control screen running on http://localhost:${PORT}`);
});

process.on('SIGINT', async () => {
  await perceptionRunner.stop();
  process.exit(0);
});
process.on('SIGTERM', async () => {
  await perceptionRunner.stop();
  process.exit(0);
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

function getControlCommControlUrl() {
  return new URL(CONTROL_COMM_CONTROL_PATH, CONTROL_COMM_BASE_URL).toString();
}

function getControlCommTurnTestUrl() {
  return new URL(CONTROL_COMM_TURN_TEST_PATH, CONTROL_COMM_BASE_URL).toString();
}

function getControlCommLineFollowPidUrl() {
  return new URL(CONTROL_COMM_LINE_FOLLOW_PID_PATH, CONTROL_COMM_BASE_URL).toString();
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

async function sendRobotStopSignals() {
  const inputsUrl = getStateMachineInputsUrl();
  const controlUrl = getControlCommControlUrl();
  const inputsPayload = {
    black_line: { detected: false, vector: { x: 0.0, y: 0.0 } },
    red_line: { detected: false, vector: { x: 0.0, y: 0.0 } },
    target: { detected: false, vector: { x: 0.0, y: 0.0 } }
  };
  const controlPayload = { x: 0.0, y: 0.0, speed: 0.0 };

  const [inputsResult, controlResult] = await Promise.all([
    postJson(inputsUrl, inputsPayload),
    postJson(controlUrl, controlPayload)
  ]);

  const failures = {};
  if (!inputsResult.ok) {
    failures.stateMachineInputs = {
      status: inputsResult.status,
      error: inputsResult.error,
      detail: inputsResult.text
    };
  }
  if (!controlResult.ok) {
    failures.control = {
      status: controlResult.status,
      error: controlResult.error,
      detail: controlResult.text
    };
  }

  return {
    ok: Object.keys(failures).length === 0,
    failures
  };
}

async function runTurnTest(degrees) {
  const url = getControlCommTurnTestUrl();
  try {
    const response = await fetch(url, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        Accept: 'application/json, text/plain'
      },
      body: JSON.stringify({ degrees })
    });
    const bodyText = await response.text();
    const body = safeJsonParse(bodyText);
    if (!response.ok) {
      return {
        ok: false,
        error: `Upstream status ${response.status}`,
        detail: body || bodyText
      };
    }
    if (!body || body.ok !== true || !body.result) {
      return {
        ok: false,
        error: 'Upstream returned invalid turn-test payload.',
        detail: body || bodyText
      };
    }
    return {
      ok: true,
      result: body.result
    };
  } catch (err) {
    return {
      ok: false,
      error: err.message
    };
  }
}

async function postJson(url, payload) {
  try {
    const response = await fetch(url, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        Accept: 'application/json, text/plain'
      },
      body: JSON.stringify(payload)
    });
    const text = await response.text();
    if (!response.ok) {
      return {
        ok: false,
        status: response.status,
        text,
        error: `Upstream status ${response.status}`
      };
    }
    return {
      ok: true,
      status: response.status,
      text
    };
  } catch (err) {
    return {
      ok: false,
      status: 0,
      text: '',
      error: err.message
    };
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

function createPerceptionRunner() {
  let child = null;
  let startedAt = null;
  let lastExitCode = null;
  let lastExitSignal = null;
  const recentLogs = [];

  const appendLog = (chunk) => {
    if (!chunk) return;
    const text = String(chunk);
    if (!text) return;
    recentLogs.push(text);
    if (recentLogs.length > 2000) {
      recentLogs.splice(0, recentLogs.length - 2000);
    }
    try {
      fs.mkdirSync(path.dirname(PERCEPTION_LOG_FILE), { recursive: true });
      fs.appendFileSync(PERCEPTION_LOG_FILE, text, 'utf8');
    } catch (err) {
      // Best effort file logging; keep in-memory logs available.
    }
  };

  const start = () => {
    if (child && !child.killed) {
      return { ok: false, error: 'Perception runner is already running.', pid: child.pid };
    }
    if (!PERCEPTION_RUNNER_ENABLED) {
      return { ok: false, error: 'Perception runner controls are disabled.' };
    }
    if (!fs.existsSync(PERCEPTION_RUN_SCRIPT)) {
      return { ok: false, error: `Perception script not found: ${PERCEPTION_RUN_SCRIPT}` };
    }

    child = spawn('bash', [PERCEPTION_RUN_SCRIPT], {
      cwd: PERCEPTION_RUN_CWD,
      env: process.env,
      stdio: ['ignore', 'pipe', 'pipe']
    });
    startedAt = Date.now();
    lastExitCode = null;
    lastExitSignal = null;

    appendLog(`[control_screen] starting perception script: ${PERCEPTION_RUN_SCRIPT}\n`);
    child.stdout.on('data', appendLog);
    child.stderr.on('data', appendLog);
    child.on('close', (code, signal) => {
      appendLog(`[control_screen] perception exited (code=${code}, signal=${signal || 'none'})\n`);
      lastExitCode = typeof code === 'number' ? code : null;
      lastExitSignal = signal || null;
      child = null;
      startedAt = null;
    });
    child.on('error', (err) => {
      appendLog(`[control_screen] perception runner error: ${err.message}\n`);
      child = null;
      startedAt = null;
    });
    return { ok: true, pid: child.pid };
  };

  const stop = async () => {
    if (!child) {
      return { ok: true, message: 'Perception runner is not running.' };
    }
    const pid = child.pid;
    appendLog('[control_screen] stopping perception runner\n');
    child.kill('SIGTERM');

    const closed = await new Promise((resolve) => {
      let done = false;
      const timeout = setTimeout(() => {
        if (done) return;
        done = true;
        resolve(false);
      }, PERCEPTION_STOP_TIMEOUT_MS);
      child.once('close', () => {
        if (done) return;
        done = true;
        clearTimeout(timeout);
        resolve(true);
      });
    });

    if (!closed && child) {
      child.kill('SIGKILL');
      return { ok: true, message: 'Perception runner force-stopped.', pid };
    }
    return { ok: true, message: 'Perception runner stopped.', pid };
  };

  const status = () => ({
    running: Boolean(child),
    pid: child ? child.pid : null,
    startedAt,
    script: PERCEPTION_RUN_SCRIPT,
    cwd: PERCEPTION_RUN_CWD,
    logFile: PERCEPTION_LOG_FILE,
    lastExitCode,
    lastExitSignal
  });

  const readLogs = async (lines) => {
    const fileTail = await readLogFileTail(PERCEPTION_LOG_FILE, lines);
    let stdout = fileTail;
    if (!stdout.trim()) {
      stdout = recentLogs.join('');
    }
    return {
      ok: true,
      stdout,
      stderr: '',
      running: Boolean(child),
      pid: child ? child.pid : null
    };
  };

  return {
    start,
    stop,
    status,
    readLogs
  };
}

async function readLogFileTail(logFilePath, lines) {
  try {
    const content = await fs.promises.readFile(logFilePath, 'utf8');
    const rows = content.split('\n');
    return rows.slice(Math.max(0, rows.length - lines)).join('\n');
  } catch (err) {
    return '';
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
}
