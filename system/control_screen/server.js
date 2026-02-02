const express = require('express');
const path = require('path');
const fs = require('fs');

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
const LOG_PATH = process.env.LOG_PATH || process.env.INSIGHTS_IPC_PATH;

const DEFAULT_RANGE = { min: -1000, max: 1000 };

const pidRanges = {
  p: buildRange(process.env.PID_P_MIN, process.env.PID_P_MAX),
  i: buildRange(process.env.PID_I_MIN, process.env.PID_I_MAX),
  d: buildRange(process.env.PID_D_MIN, process.env.PID_D_MAX)
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

app.get('/api/config', (req, res) => {
  res.json({
    ranges: pidRanges,
    controlComm: {
      baseUrl: CONTROL_COMM_BASE_URL,
      paths: {
        p: CONTROL_COMM_P_PATH,
        i: CONTROL_COMM_I_PATH,
        d: CONTROL_COMM_D_PATH,
        control: CONTROL_COMM_CONTROL_PATH
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

app.post('/api/control', async (req, res) => {
  const { x, y, speed } = req.body || {};
  const errors = {};

  const parsedX = parseNumberValue(x);
  const parsedY = parseNumberValue(y);
  const parsedSpeed = parseNumberValue(speed);

  if (!Number.isFinite(parsedX)) {
    errors.x = 'X must be a finite number.';
  }
  if (!Number.isFinite(parsedY)) {
    errors.y = 'Y must be a finite number.';
  }
  if (!Number.isFinite(parsedSpeed)) {
    errors.speed = 'Speed must be a finite number.';
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

  if (Object.keys(errors).length > 0) {
    return res.status(400).json({
      message: 'Invalid control vector.',
      errors
    });
  }

  const result = await sendControlCommand({
    x: parsedX,
    y: parsedY,
    speed: parsedSpeed
  });

  if (!result || result.error) {
    logInsight('control_command_error', { error: result?.error });
    return res.status(502).json({
      message: 'Failed to send control command.',
      error: result?.error
    });
  }

  logInsight('control_command_ok', { command: result.command });
  return res.json({ command: result.command });
});

app.listen(PORT, () => {
  logInsight('control_screen_started', { port: PORT });
  logServiceLine(`Control screen running on http://localhost:${PORT}`);
});

function buildRange(minValue, maxValue) {
  const min = parseNumber(minValue, DEFAULT_RANGE.min);
  const max = parseNumber(maxValue, DEFAULT_RANGE.max);

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

async function sendControlCommand(command) {
  const url = new URL(CONTROL_COMM_CONTROL_PATH, CONTROL_COMM_BASE_URL).toString();

  try {
    const response = await fetch(url, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        Accept: 'application/json, text/plain'
      },
      body: JSON.stringify(command)
    });

    const body = await response.text();
    if (!response.ok) {
      return { error: `Upstream status ${response.status}`, detail: body };
    }

    const parsed = safeJsonParse(body);
    if (parsed && parsed.command) {
      return { command: parsed.command };
    }

    return { command };
  } catch (err) {
    return { error: err.message };
  }
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
