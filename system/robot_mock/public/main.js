(() => {
  const canvas = document.getElementById('game');
  const ctx = canvas.getContext('2d');

  const ui = {
    mode: document.getElementById('mode'),
    fsm: document.getElementById('fsm'),
    control: document.getElementById('control'),
    link: document.getElementById('link'),
    robot: document.getElementById('robot'),
    startBtn: document.getElementById('start-btn'),
    pauseBtn: document.getElementById('pause-btn'),
    restartFsmBtn: document.getElementById('restart-fsm-btn'),
    penToggleBtn: document.getElementById('pen-toggle-btn'),
    clearLineBtn: document.getElementById('clear-line-btn'),
    saveCourseBtn: document.getElementById('save-course-btn'),
    loadCourseBtn: document.getElementById('load-course-btn'),
    loadCourseInput: document.getElementById('load-course-input'),
    manualToggle: document.getElementById('manual-toggle'),
    inputsToggle: document.getElementById('inputs-toggle'),
    vectorsToggle: document.getElementById('vectors-toggle'),
    visionDepthInput: document.getElementById('vision-depth'),
    visionWidthInput: document.getElementById('vision-width'),
    visionDepthValue: document.getElementById('vision-depth-value'),
    visionWidthValue: document.getElementById('vision-width-value')
  };

  const WORLD = {
    w: 12,
    h: 7.5
  };

  const CONFIG = {
    maxSpeed: 2,
    maxTurnRate: 3,
    turnInPlaceRate: 2.4,
    turnAggression: 0.45,
    lineDetectWidth: 2,
    visionDepth: 1.6,
    visionWidth: 1.6,
    // Keep these conservative to avoid log spam and upstream hammering.
    sendIntervalMs: 100,
    pollIntervalMs: 100
  };

  const state = {
    // Starts paused; will auto-run once a non-zero control command arrives.
    mode: 'paused',
    timeMs: 0,
    lastSystemPollMs: -1e9,
    lastSendMs: -1e9,
    lastSendOk: null,
    manualPaused: false,
    manualOverride: false,
    sendInputs: true,
    showVectors: true,
    keys: new Set(),
    system: {
      control: { x: 0, y: 0, speed: 0 },
      controlUpdatedAt: null,
      fsm: 'unknown',
      controlSource: 'system',
      lastPollOk: null,
      lastPollError: null
    },
    robot: {
      x: 1.2,
      y: 1.2,
      half: 0.18,
      heading: 0,
      vx: 0,
      vy: 0
    },
    home: { x: 1.2, y: 1.2, r: 0.18 },
    target: { x: 9.8, y: 5.6, r: 0.16 },
    safe: { x: 10.2, y: 1.25, r: 0.2 },
    danger: { x: 6.0, y: 3.6, r: 0.6 },
    path: [
      { x: 1.2, y: 1.2 },
      { x: 9.8, y: 5.6 }
    ],
    penMode: false,
    drawingLine: false,
    penBackupPath: null,
    dragging: null
  };

  const api = {
    configLoaded: false,
    tokenRequired: false,
    token: null
  };

  function clamp(v, lo, hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  function wrapAngle(rad) {
    let a = rad;
    while (a > Math.PI) a -= Math.PI * 2;
    while (a < -Math.PI) a += Math.PI * 2;
    return a;
  }

  function clampWorldPoint(p, margin = 0.02) {
    return {
      x: clamp(p.x, margin, WORLD.w - margin),
      y: clamp(p.y, margin, WORLD.h - margin)
    };
  }

  function parseWorldPoint(data, name) {
    if (!data || typeof data !== 'object') {
      throw new Error(`${name} must be an object with x/y.`);
    }
    const x = Number(data.x);
    const y = Number(data.y);
    if (!Number.isFinite(x) || !Number.isFinite(y)) {
      throw new Error(`${name} must contain finite x/y.`);
    }
    return clampWorldPoint({ x, y });
  }

  function worldToRobotVector(dx, dy, heading) {
    const c = Math.cos(heading);
    const s = Math.sin(heading);
    const right = dx * c + dy * s;
    const forward = dx * s - dy * c;
    return { x: right, y: forward };
  }

  function robotFrontPosition() {
    const frontOffset = state.robot.half || 0;
    return {
      x: state.robot.x + Math.sin(state.robot.heading) * frontOffset,
      y: state.robot.y - Math.cos(state.robot.heading) * frontOffset
    };
  }

  function robotProbePosition(distanceFromFront = 0) {
    const front = robotFrontPosition();
    return {
      x: front.x + Math.sin(state.robot.heading) * distanceFromFront,
      y: front.y - Math.cos(state.robot.heading) * distanceFromFront
    };
  }

  function computeRelativeVectorFromFront(to) {
    const front = robotFrontPosition();
    const dx = to.x - front.x;
    const dy = to.y - front.y;
    const rel = worldToRobotVector(dx, dy, state.robot.heading);
    return { x: rel.x, y: rel.y };
  }

  function withHeadingMagnitude(vector, detected = true) {
    const x = Number(vector?.x || 0);
    const y = Number(vector?.y || 0);
    const magnitude = Math.hypot(x, y);
    const heading = magnitude > 1e-9 ? Math.atan2(x, y) : 0;
    return { detected: !!detected, x, y, heading, magnitude };
  }

  function inVisionSquare(vector) {
    const halfWidth = CONFIG.visionWidth * 0.5;
    return (
      vector.y >= 0
      && vector.y <= CONFIG.visionDepth
      && Math.abs(vector.x) <= halfWidth
    );
  }

  function computeLineFollowObservation(from, lineStart, lineEnd, heading) {
    const segX = lineEnd.x - lineStart.x;
    const segY = lineEnd.y - lineStart.y;
    const segLenSq = segX * segX + segY * segY;
    if (segLenSq <= 1e-9) {
      return {
        detected: false,
        vector: { x: 0, y: 0 },
        _dist: 0,
        _t: 0
      };
    }

    const robotLength = (from.half || 0) * 2;
    const probeOffset = (from.half || 0) + robotLength;
    const frontX = from.x + Math.sin(heading) * probeOffset;
    const frontY = from.y - Math.cos(heading) * probeOffset;

    const px = frontX - lineStart.x;
    const py = frontY - lineStart.y;
    const t = clamp((px * segX + py * segY) / segLenSq, 0, 1);
    const closestX = lineStart.x + segX * t;
    const closestY = lineStart.y + segY * t;

    const toLineX = closestX - frontX;
    const toLineY = closestY - frontY;
    const lineDist = Math.hypot(toLineX, toLineY);

    const rel = worldToRobotVector(toLineX, toLineY, heading);
    const tanNorm = Math.hypot(segX, segY) || 1;
    const tangentRel = worldToRobotVector(segX / tanNorm, segY / tanNorm, heading);
    return {
      detected: lineDist <= CONFIG.lineDetectWidth,
      vector: { x: rel.x, y: rel.y },
      tangent: { x: tangentRel.x, y: tangentRel.y },
      _dist: lineDist,
      _t: t
    };
  }

  function clipSegmentToRect(a, b, minX, maxX, minY, maxY) {
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    let t0 = 0;
    let t1 = 1;
    const p = [-dx, dx, -dy, dy];
    const q = [a.x - minX, maxX - a.x, a.y - minY, maxY - a.y];

    for (let i = 0; i < 4; i += 1) {
      if (Math.abs(p[i]) < 1e-12) {
        if (q[i] < 0) return null;
        continue;
      }
      const r = q[i] / p[i];
      if (p[i] < 0) {
        if (r > t1) return null;
        if (r > t0) t0 = r;
      } else {
        if (r < t0) return null;
        if (r < t1) t1 = r;
      }
    }

    return {
      a: { x: a.x + dx * t0, y: a.y + dy * t0 },
      b: { x: a.x + dx * t1, y: a.y + dy * t1 }
    };
  }

  function closestPointOnSegment(p, a, b) {
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const lenSq = dx * dx + dy * dy;
    if (lenSq <= 1e-12) return { x: a.x, y: a.y, t: 0 };
    const t = clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / lenSq, 0, 1);
    return { x: a.x + dx * t, y: a.y + dy * t, t };
  }

  function computeLineFollowObservationOnPath(from, path, heading) {
    const points = Array.isArray(path) ? path : [];
    if (points.length < 2) {
      return {
        detected: false,
        vector: { x: 0, y: 0 },
        tangent: { x: 0, y: 1 },
        _dist: 0,
        _t: 0
      };
    }

    const front = robotFrontPosition();
    const robotLength = (from.half || 0) * 2;
    const probe = { x: 0, y: robotLength }; // in front-relative robot frame
    const halfWidth = CONFIG.visionWidth * 0.5;
    const minX = -halfWidth;
    const maxX = halfWidth;
    const minY = 0;
    const maxY = CONFIG.visionDepth;

    let best = null;
    for (let i = 0; i < points.length - 1; i += 1) {
      const aWorld = points[i];
      const bWorld = points[i + 1];
      const aRel = worldToRobotVector(aWorld.x - front.x, aWorld.y - front.y, heading);
      const bRel = worldToRobotVector(bWorld.x - front.x, bWorld.y - front.y, heading);
      const clipped = clipSegmentToRect(aRel, bRel, minX, maxX, minY, maxY);
      if (!clipped) continue;

      const closest = closestPointOnSegment(probe, clipped.a, clipped.b);
      const vecX = closest.x - probe.x;
      const vecY = closest.y - probe.y;
      const dist = Math.hypot(vecX, vecY);

      const tanX = clipped.b.x - clipped.a.x;
      const tanY = clipped.b.y - clipped.a.y;
      const tanNorm = Math.hypot(tanX, tanY) || 1;

      const obs = {
        detected: true,
        vector: { x: vecX, y: vecY },
        tangent: { x: tanX / tanNorm, y: tanY / tanNorm },
        _dist: dist,
        _t: closest.t
      };
      if (!best || obs._dist < best._dist) best = obs;
    }
    return best || {
      detected: false,
      vector: { x: 0, y: 0 },
      tangent: { x: 0, y: 1 },
      _dist: 0,
      _t: 0
    };
  }

  function pickManualVelocity() {
    let x = 0;
    let y = 0;
    if (state.keys.has('a') || state.keys.has('arrowleft')) x -= 1;
    if (state.keys.has('d') || state.keys.has('arrowright')) x += 1;
    if (state.keys.has('w') || state.keys.has('arrowup')) y += 1;
    if (state.keys.has('s') || state.keys.has('arrowdown')) y -= 1;
    const mag = Math.hypot(x, y) || 1;
    return { x: x / mag, y: y / mag, speed: state.keys.size ? 0.75 : 0 };
  }

  function controlToVelocity(control, heading, dt) {
    const mag = Math.hypot(control.x, control.y) || 1;
    const speed = clamp(control.speed, 0, 1);
    if (speed <= 1e-4 || mag <= 1e-4) {
      return { vx: 0, vy: 0, heading };
    }

    const cmdX = control.x / mag;
    const cmdY = control.y / mag;
    const desiredOffset = Math.atan2(cmdX, cmdY);
    const desiredHeading = wrapAngle(heading + desiredOffset);
    const headingError = wrapAngle(desiredHeading - heading);
    const behindCommand = Math.abs(desiredOffset) > (Math.PI / 2);

    // If command is behind the robot, rotate in place first to match the
    // requested heading change (including full 180-degree turns).
    if (behindCommand) {
      const turnStep = clamp(
        headingError,
        -CONFIG.turnInPlaceRate * dt,
        CONFIG.turnInPlaceRate * dt
      );
      return { vx: 0, vy: 0, heading: wrapAngle(heading + turnStep) };
    }

    const delta = headingError * CONFIG.turnAggression;
    const maxStep = CONFIG.maxTurnRate * dt;
    const step = clamp(delta, -maxStep, maxStep);
    const nextHeading = wrapAngle(heading + step);
    const remaining = wrapAngle(desiredHeading - nextHeading);
    const forwardScale = Math.max(0, Math.cos(remaining));
    const forwardSpeed = speed * CONFIG.maxSpeed * forwardScale;
    const forwardX = Math.sin(nextHeading);
    const forwardY = -Math.cos(nextHeading);
    return {
      vx: forwardX * forwardSpeed,
      vy: forwardY * forwardSpeed,
      heading: nextHeading
    };
  }

  async function loadConfig() {
    if (api.configLoaded) return;
    try {
      const resp = await fetch('/api/config', { cache: 'no-store' });
      const cfg = await resp.json();
      api.tokenRequired = !!cfg?.token_required;
      api.configLoaded = true;
      if (api.tokenRequired && !api.token) {
        const saved = localStorage.getItem('robot_mock_token');
        if (saved) {
          api.token = saved;
        } else {
          const entered = window.prompt('Robot Mock token required. Enter token:');
          if (entered) {
            api.token = entered;
            localStorage.setItem('robot_mock_token', entered);
          }
        }
      }
    } catch (e) {
      api.configLoaded = true;
    }
  }

  async function apiFetch(url, options = {}) {
    await loadConfig();
    const headers = new Headers(options.headers || {});
    if (api.token) headers.set('x-robot-mock-token', api.token);
    return fetch(url, { ...options, headers });
  }

  async function pollSystem(forceFresh = false) {
    try {
      const systemUrl = forceFresh ? '/api/system?fresh=1' : '/api/system';
      const resp = await apiFetch(systemUrl, { cache: 'no-store' });
      if (resp.status === 401) {
        api.token = null;
        api.configLoaded = false;
        state.system.lastPollOk = false;
        state.system.lastPollError = 'unauthorized';
        return;
      }
      const payload = await resp.json();

      const controlEnvelope = payload?.control;
      const stateEnvelope = payload?.state;
      let controlError = null;
      let stateError = null;

      if (controlEnvelope?.ok) {
        const controlData = controlEnvelope?.data;
        const cmdObj = controlData?.command;
        if (cmdObj && typeof cmdObj.x === 'number' && typeof cmdObj.y === 'number' && typeof cmdObj.speed === 'number') {
          state.system.control = { x: cmdObj.x, y: cmdObj.y, speed: cmdObj.speed };
          state.system.controlUpdatedAt = cmdObj.updated_at || null;
        }
      } else {
        controlError = `control: ${controlEnvelope?.error || controlEnvelope?.status_code || 'error'}`;
      }

      if (stateEnvelope?.ok) {
        const fsm =
          stateEnvelope?.data?.state
          || stateEnvelope?.data?.fsm
          || stateEnvelope?.data?.current_state;
        if (typeof fsm === 'string' && fsm.length) state.system.fsm = fsm;
      } else {
        stateError = `state: ${stateEnvelope?.error || stateEnvelope?.status_code || 'error'}`;
      }

      if (controlError || stateError) {
        state.system.lastPollOk = false;
        state.system.lastPollError = [controlError, stateError].filter(Boolean).join(' | ');
      } else {
        state.system.lastPollOk = true;
        state.system.lastPollError = null;
      }
      if (!state.manualPaused && state.mode === 'paused' && (state.system.control.speed || 0) > 0.02) {
        state.mode = 'running';
      }
    } catch (e) {
      state.system.lastPollOk = false;
      state.system.lastPollError = String(e?.message || e);
    }
  }

  async function sendInputs(payload) {
    try {
      const resp = await apiFetch('/api/state-machine/inputs', {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: JSON.stringify(payload)
      });
      state.lastSendOk = resp.ok;
      await resp.json().catch(() => null);
    } catch (e) {
      state.lastSendOk = false;
    }
  }

  async function restartStateMachine() {
    if (!ui.restartFsmBtn) return;
    ui.restartFsmBtn.disabled = true;
    const oldText = ui.restartFsmBtn.textContent;
    ui.restartFsmBtn.textContent = 'Restarting...';
    try {
      const resp = await apiFetch('/api/state-machine/restart', {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: JSON.stringify({})
      });
      if (!resp.ok) {
        const payload = await resp.json().catch(() => null);
        state.system.lastPollOk = false;
        state.system.lastPollError = payload?.result?.error || `restart failed (${resp.status})`;
      }
      await pollSystem(true);
    } catch (e) {
      state.system.lastPollOk = false;
      state.system.lastPollError = String(e?.message || e);
    } finally {
      ui.restartFsmBtn.disabled = false;
      ui.restartFsmBtn.textContent = oldText;
    }
  }

  function update(dt) {
    state.timeMs += dt * 1000;

    if (!document.hidden && state.timeMs - state.lastSystemPollMs >= CONFIG.pollIntervalMs) {
      state.lastSystemPollMs = state.timeMs;
      pollSystem();
    }

    if (document.hidden) return;
    if (state.mode === 'running') {
      const control = state.manualOverride
        ? pickManualVelocity()
        : state.system.control;
      state.system.controlSource = state.manualOverride
        ? 'manual'
        : 'system';

      const vel = controlToVelocity(control, state.robot.heading, dt);
      state.robot.vx = vel.vx;
      state.robot.vy = vel.vy;
      state.robot.heading = vel.heading;
      const bound = state.robot.half * Math.SQRT2;
      state.robot.x = clamp(state.robot.x + state.robot.vx * dt, bound, WORLD.w - bound);
      state.robot.y = clamp(state.robot.y + state.robot.vy * dt, bound, WORLD.h - bound);
    } else {
      state.robot.vx = 0;
      state.robot.vy = 0;
    }

    const lineObs = computeLineFollowObservationOnPath(
      state.robot,
      state.path,
      state.robot.heading
    );

    const safeVector = computeRelativeVectorFromFront(state.safe);
    const targetVector = computeRelativeVectorFromFront(state.target);
    const dangerVector = computeRelativeVectorFromFront(state.danger);
    const homeVector = computeRelativeVectorFromFront(state.home);
    const targetDetected = inVisionSquare(targetVector);
    const safeDetected = inVisionSquare(safeVector);
    const dangerDetected = inVisionSquare(dangerVector);
    const homeDetected = inVisionSquare(homeVector);
    const lineDetected = !!lineObs.detected;
    const targetPayload = targetDetected ? targetVector : { x: 0, y: 0 };
    const safePayload = safeDetected ? safeVector : { x: 0, y: 0 };
    const dangerPayload = dangerDetected ? dangerVector : { x: 0, y: 0 };
    const homePayload = homeDetected ? homeVector : { x: 0, y: 0 };
    const linePayload = lineDetected ? lineObs.vector : { x: 0, y: 0 };

    const inputsPayload = {
      safe_zone: withHeadingMagnitude(safePayload, safeDetected),
      target: withHeadingMagnitude(targetPayload, targetDetected),
      danger_zone: withHeadingMagnitude(dangerPayload, dangerDetected),
      home: withHeadingMagnitude(homePayload, homeDetected),
      line_error: withHeadingMagnitude(linePayload, lineDetected),
      heading_rad: state.robot.heading,
      speed: Math.hypot(state.robot.vx, state.robot.vy)
    };

    if (state.sendInputs && state.timeMs - state.lastSendMs >= CONFIG.sendIntervalMs) {
      state.lastSendMs = state.timeMs;
      sendInputs(inputsPayload);
    }

    state._lastInputs = inputsPayload;
    state._lastObs = { lineObs };
    state._lastVectors = {
      safe_zone: safeVector,
      target: targetVector,
      danger_zone: dangerVector,
      home: homeVector,
      line_error: linePayload
    };
    state._lastVectorDetected = {
      safe_zone: safeDetected,
      target: targetDetected,
      danger_zone: dangerDetected,
      home: homeDetected,
      line_error: lineDetected
    };
  }

  function worldToPx(x, y) {
    const sx = canvas.width / WORLD.w;
    const sy = canvas.height / WORLD.h;
    return { x: x * sx, y: y * sy };
  }

  function pxToWorld(px, py) {
    const rect = canvas.getBoundingClientRect();
    const x = (px - rect.left) / rect.width;
    const y = (py - rect.top) / rect.height;
    return { x: x * WORLD.w, y: y * WORLD.h };
  }

  function drawGrid() {
    ctx.save();
    ctx.globalAlpha = 0.35;
    ctx.strokeStyle = 'rgba(231, 238, 252, 0.14)';
    ctx.lineWidth = 1;
    const step = 60;
    for (let x = 0; x <= canvas.width; x += step) {
      ctx.beginPath();
      ctx.moveTo(x + 0.5, 0);
      ctx.lineTo(x + 0.5, canvas.height);
      ctx.stroke();
    }
    for (let y = 0; y <= canvas.height; y += step) {
      ctx.beginPath();
      ctx.moveTo(0, y + 0.5);
      ctx.lineTo(canvas.width, y + 0.5);
      ctx.stroke();
    }
    ctx.restore();
  }

  function drawPoint(p, color, label) {
    const pos = worldToPx(p.x, p.y);
    ctx.save();
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(pos.x, pos.y, 12, 0, Math.PI * 2);
    ctx.fill();
    ctx.globalAlpha = 0.9;
    ctx.strokeStyle = 'rgba(255,255,255,0.25)';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.fillStyle = 'rgba(255,255,255,0.85)';
    ctx.font = '12px ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, Helvetica, Arial';
    ctx.fillText(label, pos.x + 14, pos.y - 10);
    ctx.restore();
  }

  function drawCircle(p, r, color, label) {
    const pos = worldToPx(p.x, p.y);
    const sx = canvas.width / WORLD.w;
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.globalAlpha = 0.8;
    ctx.beginPath();
    ctx.arc(pos.x, pos.y, r * sx, 0, Math.PI * 2);
    ctx.stroke();
    ctx.globalAlpha = 0.55;
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(pos.x, pos.y, r * sx, 0, Math.PI * 2);
    ctx.fill();
    ctx.globalAlpha = 0.9;
    ctx.fillStyle = 'rgba(255,255,255,0.75)';
    ctx.font = '12px ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, Helvetica, Arial';
    ctx.fillText(label, pos.x + 14, pos.y - 10);
    ctx.restore();
  }

  function drawRobot() {
    const pos = worldToPx(state.robot.x, state.robot.y);
    const sx = canvas.width / WORLD.w;
    const half = state.robot.half * sx;
    const heading = state.robot.heading;

    ctx.save();
    ctx.translate(pos.x, pos.y);
    ctx.rotate(heading);
    ctx.fillStyle = 'rgba(99, 179, 255, 0.85)';
    ctx.fillRect(-half, -half, half * 2, half * 2);
    ctx.lineWidth = 2;
    ctx.strokeStyle = 'rgba(255,255,255,0.25)';
    ctx.strokeRect(-half, -half, half * 2, half * 2);
    ctx.fillStyle = 'rgba(255, 123, 123, 0.95)';
    ctx.fillRect(-half * 0.55, -half, half * 1.1, half * 0.3);
    ctx.strokeStyle = 'rgba(255,255,255,0.65)';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(0, -half * 1.45);
    ctx.stroke();
    ctx.restore();
  }

  function drawVisionSquare() {
    const front = robotFrontPosition();
    const pxPerWorld = canvas.width / WORLD.w;
    const depthPx = CONFIG.visionDepth * pxPerWorld;
    const widthPx = CONFIG.visionWidth * pxPerWorld;
    const halfWidthPx = widthPx * 0.5;

    ctx.save();
    const frontPx = worldToPx(front.x, front.y);
    ctx.translate(frontPx.x, frontPx.y);
    ctx.rotate(state.robot.heading);
    // In robot local frame, +y is forward and -canvasY is up.
    ctx.translate(0, -depthPx);
    ctx.fillStyle = 'rgba(99, 179, 255, 0.12)';
    ctx.strokeStyle = 'rgba(99, 179, 255, 0.55)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.rect(-halfWidthPx, 0, widthPx, depthPx);
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  }

  function applyVisionControlInputs() {
    const depthRaw = Number(ui.visionDepthInput?.value);
    const widthRaw = Number(ui.visionWidthInput?.value);
    if (Number.isFinite(depthRaw)) {
      CONFIG.visionDepth = clamp(depthRaw, 0.4, 6);
    }
    if (Number.isFinite(widthRaw)) {
      CONFIG.visionWidth = clamp(widthRaw, 0.4, 6);
    }
    if (ui.visionDepthValue) {
      ui.visionDepthValue.textContent = CONFIG.visionDepth.toFixed(2);
    }
    if (ui.visionWidthValue) {
      ui.visionWidthValue.textContent = CONFIG.visionWidth.toFixed(2);
    }
  }

  function robotVectorToWorldOffset(vector) {
    const heading = state.robot.heading;
    const c = Math.cos(heading);
    const s = Math.sin(heading);
    return {
      x: vector.x * c + vector.y * s,
      y: vector.x * s - vector.y * c
    };
  }

  function drawArrowWorld(startWorld, endWorld, color) {
    const start = worldToPx(startWorld.x, startWorld.y);
    const end = worldToPx(endWorld.x, endWorld.y);
    const dx = end.x - start.x;
    const dy = end.y - start.y;
    const dist = Math.hypot(dx, dy);
    if (dist < 2) return;

    const ux = dx / dist;
    const uy = dy / dist;
    const headLen = Math.min(14, Math.max(7, dist * 0.16));
    const wing = headLen * 0.55;

    ctx.save();
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.globalAlpha = 0.95;
    ctx.lineWidth = 2.2;
    ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.moveTo(start.x, start.y);
    ctx.lineTo(end.x, end.y);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(end.x, end.y);
    ctx.lineTo(end.x - ux * headLen - uy * wing, end.y - uy * headLen + ux * wing);
    ctx.lineTo(end.x - ux * headLen + uy * wing, end.y - uy * headLen - ux * wing);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  function drawVectors() {
    if (!state.showVectors) return;
    const vectors = state._lastVectors;
    if (!vectors) return;
    const detected = state._lastVectorDetected || {};

    const front = robotFrontPosition();
    const robotLength = (state.robot.half || 0) * 2;
    const lineProbe = robotProbePosition(robotLength);
    const overlays = [
      { key: 'target', color: 'rgba(255, 209, 102, 0.96)' },
      { key: 'safe_zone', color: 'rgba(110, 231, 183, 0.96)' },
      { key: 'danger_zone', color: 'rgba(255, 107, 107, 0.96)' },
      { key: 'home', color: 'rgba(99, 179, 255, 0.96)' },
      { key: 'line_error', color: 'rgba(167, 139, 250, 0.96)' }
    ];

    for (const item of overlays) {
      const rel = vectors[item.key];
      if (!rel) continue;
      if (!detected[item.key]) continue;
      const origin = item.key === 'line_error' ? lineProbe : front;
      const worldOffset = robotVectorToWorldOffset(rel);
      const end = { x: origin.x + worldOffset.x, y: origin.y + worldOffset.y };
      drawArrowWorld(origin, end, item.color);
    }
  }

  function draw() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    drawGrid();

    drawCircle(state.danger, state.danger.r, 'rgba(255, 107, 107, 0.6)', 'danger');
    if (state.path.length >= 2) {
      ctx.save();
      ctx.strokeStyle = 'rgba(255, 70, 70, 0.9)';
      ctx.lineWidth = 4;
      ctx.lineCap = 'round';
      ctx.lineJoin = 'round';
      ctx.beginPath();
      const startPx = worldToPx(state.path[0].x, state.path[0].y);
      ctx.moveTo(startPx.x, startPx.y);
      for (let i = 1; i < state.path.length; i += 1) {
        const p = worldToPx(state.path[i].x, state.path[i].y);
        ctx.lineTo(p.x, p.y);
      }
      ctx.stroke();
      ctx.restore();
    }
    drawPoint(state.target, 'rgba(255, 209, 102, 0.95)', 'target');
    drawPoint(state.safe, 'rgba(110, 231, 183, 0.95)', 'safe');
    drawPoint(state.home, 'rgba(99, 179, 255, 0.95)', 'home');

    drawVisionSquare();
    drawRobot();
    drawVectors();

    if (state.mode !== 'running') {
      ctx.save();
      ctx.fillStyle = 'rgba(0,0,0,0.35)';
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = 'rgba(231, 238, 252, 0.92)';
      ctx.font = '600 22px ui-sans-serif, system-ui';
      ctx.fillText('Paused', 18, 38);
      ctx.font = '14px ui-sans-serif, system-ui';
      ctx.fillStyle = 'rgba(231, 238, 252, 0.75)';
      ctx.fillText('Press Start or hit p to resume.', 18, 62);
      ctx.restore();
    }
  }

  function renderHud() {
    ui.mode.textContent = state.mode;
    ui.fsm.textContent = state.system.fsm;
    ui.control.textContent = `x=${state.system.control.x.toFixed(2)} y=${state.system.control.y.toFixed(2)} s=${state.system.control.speed.toFixed(2)}`;
    const link = state.system.lastPollOk === null ? 'polling…' : state.system.lastPollOk ? 'ok' : `error (${state.system.lastPollError || 'unknown'})`;
    const controlTs = state.system.controlUpdatedAt ? `, cmd@${state.system.controlUpdatedAt}` : '';
    const source = state.system.controlSource ? `, ctrl=${state.system.controlSource}` : '';
    ui.link.textContent = `${link}${controlTs}${source}`;
    const headingDeg = (state.robot.heading * 180 / Math.PI + 360) % 360;
    ui.robot.textContent = `x=${state.robot.x.toFixed(2)} y=${state.robot.y.toFixed(2)} h=${headingDeg.toFixed(0)}deg`;
  }

  function togglePause() {
    const nextPaused = state.mode === 'running';
    state.mode = nextPaused ? 'paused' : 'running';
    state.manualPaused = nextPaused;
  }

  function toggleFullscreen() {
    if (document.fullscreenElement) document.exitFullscreen();
    else canvas.requestFullscreen().catch(() => null);
  }

  function closestDraggable(world) {
    const items = [
      { key: 'target', obj: state.target },
      { key: 'safe', obj: state.safe },
      { key: 'home', obj: state.home }
    ];
    let best = null;
    let bestD = 1e9;
    for (const it of items) {
      const dx = world.x - it.obj.x;
      const dy = world.y - it.obj.y;
      const d = Math.hypot(dx, dy);
      if (d < 0.35 && d < bestD) {
        best = it;
        bestD = d;
      }
    }
    return best;
  }

  function onPointerDown(ev) {
    const w = pxToWorld(ev.clientX, ev.clientY);
    if (state.penMode) {
      state.drawingLine = true;
      state.dragging = null;
      state.penBackupPath = state.path.map((p) => ({ x: p.x, y: p.y }));
      state.path = [w];
      return;
    }
    const hit = closestDraggable(w);
    if (!hit) return;
    state.dragging = hit.key;
  }

  function onPointerMove(ev) {
    if (state.drawingLine) {
      const w = pxToWorld(ev.clientX, ev.clientY);
      const x = clamp(w.x, 0.02, WORLD.w - 0.02);
      const y = clamp(w.y, 0.02, WORLD.h - 0.02);
      const prev = state.path[state.path.length - 1];
      if (!prev || Math.hypot(prev.x - x, prev.y - y) >= 0.03) {
        state.path.push({ x, y });
      }
      return;
    }
    if (!state.dragging) return;
    const w = pxToWorld(ev.clientX, ev.clientY);
    const p = state[state.dragging];
    p.x = clamp(w.x, 0.4, WORLD.w - 0.4);
    p.y = clamp(w.y, 0.4, WORLD.h - 0.4);
  }

  function onPointerUp() {
    if (state.drawingLine) {
      state.drawingLine = false;
      if (state.path.length < 2) {
        state.path = state.penBackupPath && state.penBackupPath.length >= 2
          ? state.penBackupPath
          : [
            { x: state.home.x, y: state.home.y },
            { x: state.target.x, y: state.target.y }
          ];
      }
      state.penBackupPath = null;
    }
    state.dragging = null;
  }

  function updatePenButton() {
    if (!ui.penToggleBtn) return;
    ui.penToggleBtn.textContent = state.penMode ? 'Pen: On' : 'Pen: Off';
  }

  function togglePenMode() {
    state.penMode = !state.penMode;
    state.drawingLine = false;
    state.penBackupPath = null;
    updatePenButton();
  }

  function resetPathToHomeTarget() {
    state.path = [
      { x: state.home.x, y: state.home.y },
      { x: state.target.x, y: state.target.y }
    ];
  }

  function buildCoursePayload() {
    return {
      schema_version: 1,
      saved_at: new Date().toISOString(),
      world: { width: WORLD.w, height: WORLD.h },
      poi: {
        home: { x: state.home.x, y: state.home.y, r: state.home.r },
        target: { x: state.target.x, y: state.target.y, r: state.target.r },
        safe: { x: state.safe.x, y: state.safe.y, r: state.safe.r },
        danger: { x: state.danger.x, y: state.danger.y, r: state.danger.r }
      },
      path: state.path.map((p) => ({ x: p.x, y: p.y }))
    };
  }

  function saveCourseToJson() {
    const payload = buildCoursePayload();
    const json = JSON.stringify(payload, null, 2);
    const blob = new Blob([json], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    const ts = new Date().toISOString().replace(/[:.]/g, '-');
    a.href = url;
    a.download = `robot-course-${ts}.json`;
    document.body.appendChild(a);
    a.click();
    a.remove();
    URL.revokeObjectURL(url);
  }

  function applyCoursePayload(payload) {
    if (!payload || typeof payload !== 'object') {
      throw new Error('Course JSON must be an object.');
    }
    const poi = payload.poi || {};
    const pathRaw = payload.path;
    if (!Array.isArray(pathRaw) || pathRaw.length < 2) {
      throw new Error('Course path must have at least 2 points.');
    }

    const home = parseWorldPoint(poi.home, 'poi.home');
    const target = parseWorldPoint(poi.target, 'poi.target');
    const safe = parseWorldPoint(poi.safe, 'poi.safe');
    const danger = parseWorldPoint(poi.danger, 'poi.danger');
    const parsedPath = pathRaw.map((p, i) => parseWorldPoint(p, `path[${i}]`));

    state.home.x = home.x;
    state.home.y = home.y;
    state.target.x = target.x;
    state.target.y = target.y;
    state.safe.x = safe.x;
    state.safe.y = safe.y;
    state.danger.x = danger.x;
    state.danger.y = danger.y;

    const dangerR = Number(poi?.danger?.r);
    if (Number.isFinite(dangerR)) {
      state.danger.r = clamp(dangerR, 0.05, 2.5);
    }

    state.path = parsedPath;
    state.drawingLine = false;
    state.penBackupPath = null;
  }

  async function loadCourseFromFile(file) {
    if (!file) return;
    const text = await file.text();
    let payload = null;
    try {
      payload = JSON.parse(text);
    } catch (err) {
      throw new Error('Invalid JSON file.');
    }
    applyCoursePayload(payload);
  }

  function tick() {
    update(1 / 60);
    draw();
    renderHud();
    requestAnimationFrame(tick);
  }

  function renderGameToText() {
    const payload = {
      note: 'World coords: origin=(0,0) top-left; +x right; +y down.',
      mode: state.mode,
      time_ms: Math.round(state.timeMs),
      system: {
        fsm: state.system.fsm,
        control: state.system.control
      },
      robot: {
        x: +state.robot.x.toFixed(3),
        y: +state.robot.y.toFixed(3),
        heading_rad: +state.robot.heading.toFixed(3),
        vx: +state.robot.vx.toFixed(3),
        vy: +state.robot.vy.toFixed(3)
      },
      objects: {
        home: { x: +state.home.x.toFixed(3), y: +state.home.y.toFixed(3) },
        target: { x: +state.target.x.toFixed(3), y: +state.target.y.toFixed(3) },
        safe: { x: +state.safe.x.toFixed(3), y: +state.safe.y.toFixed(3) },
        danger: { x: +state.danger.x.toFixed(3), y: +state.danger.y.toFixed(3), r: state.danger.r }
      },
      vision_fov: {
        depth: +CONFIG.visionDepth.toFixed(3),
        width: +CONFIG.visionWidth.toFixed(3)
      },
      path: state.path.map((p) => ({ x: +p.x.toFixed(3), y: +p.y.toFixed(3) })),
      last_inputs: state._lastInputs
    };
    return JSON.stringify(payload);
  }

  window.render_game_to_text = renderGameToText;
  window.advanceTime = (ms) => {
    const steps = Math.max(1, Math.round(ms / (1000 / 60)));
    for (let i = 0; i < steps; i++) {
      update(1 / 60);
    }
    draw();
    renderHud();
  };

  ui.startBtn.addEventListener('click', () => {
    state.manualPaused = false;
    state.mode = 'running';
  });
  ui.pauseBtn.addEventListener('click', togglePause);
  if (ui.penToggleBtn) ui.penToggleBtn.addEventListener('click', togglePenMode);
  if (ui.clearLineBtn) ui.clearLineBtn.addEventListener('click', resetPathToHomeTarget);
  if (ui.saveCourseBtn) ui.saveCourseBtn.addEventListener('click', saveCourseToJson);
  if (ui.loadCourseBtn && ui.loadCourseInput) {
    ui.loadCourseBtn.addEventListener('click', () => ui.loadCourseInput.click());
    ui.loadCourseInput.addEventListener('change', async (e) => {
      const file = e.target?.files?.[0];
      if (!file) return;
      try {
        await loadCourseFromFile(file);
      } catch (err) {
        alert(`Failed to load course: ${err?.message || err}`);
      } finally {
        ui.loadCourseInput.value = '';
      }
    });
  }
  if (ui.restartFsmBtn) {
    ui.restartFsmBtn.textContent = 'Restart FSM';
    ui.restartFsmBtn.title = 'Reset state-machine to searching';
    ui.restartFsmBtn.addEventListener('click', restartStateMachine);
  }
  ui.manualToggle.addEventListener('change', (e) => {
    state.manualOverride = !!e.target.checked;
  });
  ui.inputsToggle.addEventListener('change', (e) => {
    state.sendInputs = !!e.target.checked;
  });
  if (ui.vectorsToggle) {
    ui.vectorsToggle.addEventListener('change', (e) => {
      state.showVectors = !!e.target.checked;
    });
    ui.vectorsToggle.checked = state.showVectors;
  }
  if (ui.visionDepthInput) {
    ui.visionDepthInput.value = String(CONFIG.visionDepth);
    ui.visionDepthInput.addEventListener('input', applyVisionControlInputs);
    ui.visionDepthInput.addEventListener('change', applyVisionControlInputs);
  }
  if (ui.visionWidthInput) {
    ui.visionWidthInput.value = String(CONFIG.visionWidth);
    ui.visionWidthInput.addEventListener('input', applyVisionControlInputs);
    ui.visionWidthInput.addEventListener('change', applyVisionControlInputs);
  }

  window.addEventListener('keydown', (ev) => {
    const k = ev.key.toLowerCase();
    if (k === 'p') togglePause();
    if (k === 'f') toggleFullscreen();
    if (k === 'l') togglePenMode();
    state.keys.add(k);
  });
  window.addEventListener('keyup', (ev) => {
    state.keys.delete(ev.key.toLowerCase());
  });

  canvas.addEventListener('pointerdown', onPointerDown);
  window.addEventListener('pointermove', onPointerMove);
  window.addEventListener('pointerup', onPointerUp);

  // Initial poll so the HUD isn't blank.
  updatePenButton();
  applyVisionControlInputs();
  resetPathToHomeTarget();
  loadConfig();
  pollSystem();
  requestAnimationFrame(tick);
})();
