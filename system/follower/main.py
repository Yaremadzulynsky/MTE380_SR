#!/usr/bin/env python3
"""
Slim line follower for a 2-wheel differential drive robot.

Architecture (single process, three threads):
  vision thread  →  control thread  →  serial → Arduino
                          ↕
                   Flask web UI on :8080
                     /          live PID tuning (sliders + apply)
                     /stream    SSE status at 20 Hz
                     /video     MJPEG camera feed with detection overlay
                     /logs      SSE log stream
"""

import atexit
import collections
import json
import logging
import os
import signal
import sys
import threading
import time

import cv2
import yaml
from flask import Flask, Response, jsonify, request, send_from_directory

from bridge import SerialBridge
from pid import PID
from vision import LineDetector, VisionResult

# ── Config ────────────────────────────────────────────────────────────────────

_CFG_PATH = os.path.join(os.path.dirname(__file__), 'config.yaml')
with open(_CFG_PATH) as _f:
    cfg = yaml.safe_load(_f)

cam_cfg    = cfg['camera']
vis_cfg    = cfg['vision']
ctrl_cfg   = cfg['control']
serial_cfg = cfg['serial']
web_cfg    = cfg['web']

# ── Logging ───────────────────────────────────────────────────────────────────
# Ring buffer of recent log lines for the /logs SSE endpoint.
# Capped at 200 lines — no file I/O, negligible overhead.

_LOG_BUFFER: collections.deque[str] = collections.deque(maxlen=200)
_LOG_LOCK = threading.Lock()


class _RingHandler(logging.Handler):
    def emit(self, record: logging.LogRecord):
        line = self.format(record)
        with _LOG_LOCK:
            _LOG_BUFFER.append(line)


_ring_handler = _RingHandler()
_ring_handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(name)s: %(message)s'))

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s %(levelname)s %(name)s: %(message)s',
    handlers=[logging.StreamHandler(), _ring_handler],
)
log = logging.getLogger('follower')

# ── Shared state ──────────────────────────────────────────────────────────────

_state = {
    'error':    0.0,
    'detected': False,
    'left':     0.0,
    'right':    0.0,
    'running':  False,
}
_state_lock = threading.Lock()

# Latest vision result — written by vision thread, read by everything else
_latest: VisionResult | None = None
_latest_lock = threading.Lock()

# Latest JPEG bytes for the MJPEG endpoint — encoded once in vision thread
_latest_jpeg: bytes | None = None
_jpeg_lock = threading.Lock()

# ── Components ────────────────────────────────────────────────────────────────

detector = LineDetector(vis_cfg, vis_cfg['roi_bottom_frac'])

pid = PID(
    kp      = ctrl_cfg['kp'],
    ki      = ctrl_cfg['ki'],
    kd      = ctrl_cfg['kd'],
    i_max   = ctrl_cfg['i_max'],
    out_max = ctrl_cfg['out_max'],
)

bridge = SerialBridge(serial_cfg['port'], serial_cfg['baud'])

# ── Camera helpers ────────────────────────────────────────────────────────────

def _open_camera():
    src = cam_cfg['source']
    w, h, fps = cam_cfg['width'], cam_cfg['height'], cam_cfg['fps']

    if src == 'picamera2':
        try:
            from picamera2 import Picamera2  # type: ignore
            pc2 = Picamera2()
            pc2.configure(pc2.create_video_configuration(
                main={'format': 'BGR888', 'size': (w, h)}
            ))
            pc2.start()
            log.info('Camera: picamera2 %dx%d', w, h)
            return pc2
        except Exception as e:
            log.warning('picamera2 failed (%s), falling back to cv2', e)

    cap = cv2.VideoCapture(cam_cfg['device'])
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS,          fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)   # always get the latest frame
    if not cap.isOpened():
        log.error('Could not open camera device %d', cam_cfg['device'])
    else:
        log.info('Camera: cv2 device=%d %dx%d @%dfps', cam_cfg['device'], w, h, fps)
    return cap


def _read_frame(cam):
    try:
        from picamera2 import Picamera2  # type: ignore
        if isinstance(cam, Picamera2):
            frame = cam.capture_array()
            if cam_cfg.get('flip'):
                frame = cv2.flip(frame, -1)
            return frame
    except ImportError:
        pass
    ret, frame = cam.read()
    if not ret:
        return None
    if cam_cfg.get('flip'):
        frame = cv2.flip(frame, -1)
    return frame

# ── Vision thread ─────────────────────────────────────────────────────────────

def _vision_thread():
    cam = _open_camera()
    while True:
        frame = _read_frame(cam)
        if frame is None:
            time.sleep(0.01)
            continue

        result = detector.process(frame)

        # Encode JPEG for the browser feed — done here so it never blocks control
        _, jpeg = cv2.imencode('.jpg', result.debug_frame,
                               [cv2.IMWRITE_JPEG_QUALITY, 70])
        with _jpeg_lock:
            global _latest_jpeg
            _latest_jpeg = jpeg.tobytes()

        with _latest_lock:
            global _latest
            _latest = result

# ── Control thread ────────────────────────────────────────────────────────────

def _control_thread():
    rate         = ctrl_cfg['rate_hz']
    lost_timeout = ctrl_cfg['line_lost_timeout']

    last_detected_at = 0.0
    left = right = 0.0

    while True:
        t0 = time.monotonic()

        with _latest_lock:
            result = _latest

        with _state_lock:
            running    = _state['running']
            base_speed = ctrl_cfg['base_speed']

        if not running or result is None:
            if not running:
                bridge.send_drive(0.0, 0.0)
                pid.reset()
            time.sleep(1.0 / rate)
            continue

        if result.detected:
            last_detected_at = time.monotonic()
            correction = pid.update(result.error)
            left  = max(-1.0, min(1.0, base_speed + correction))
            right = max(-1.0, min(1.0, base_speed - correction))
            bridge.send_drive(left, right)
        else:
            if time.monotonic() - last_detected_at > lost_timeout:
                left = right = 0.0
                pid.reset()
                bridge.send_drive(0.0, 0.0)
            else:
                bridge.send_drive(left, right)   # hold last command briefly

        with _state_lock:
            _state['error']    = result.error
            _state['detected'] = result.detected
            _state['left']     = left
            _state['right']    = right

        elapsed = time.monotonic() - t0
        time.sleep(max(0.0, 1.0 / rate - elapsed))

# ── Flask web UI ──────────────────────────────────────────────────────────────

_static_dir = os.path.join(os.path.dirname(__file__), 'static')
app = Flask(__name__, static_folder=_static_dir)
# Silence Flask/Werkzeug request logs so they don't spam the log stream
logging.getLogger('werkzeug').setLevel(logging.WARNING)


@app.get('/')
def index():
    return send_from_directory(_static_dir, 'index.html')


@app.get('/status')
def get_status():
    with _state_lock:
        return jsonify(dict(_state))


@app.get('/gains')
def get_gains():
    return jsonify({
        'kp':         pid.kp,
        'ki':         pid.ki,
        'kd':         pid.kd,
        'i_max':      pid.i_max,
        'out_max':    pid.out_max,
        'base_speed': ctrl_cfg['base_speed'],
    })


@app.post('/gains')
def post_gains():
    data = request.get_json(silent=True) or {}
    try:
        pid.set_gains(
            kp      = data.get('kp'),
            ki      = data.get('ki'),
            kd      = data.get('kd'),
            i_max   = data.get('i_max'),
            out_max = data.get('out_max'),
        )
        if 'base_speed' in data:
            ctrl_cfg['base_speed'] = float(data['base_speed'])
        log.info('Gains updated → kp=%.3f ki=%.4f kd=%.3f base=%.3f',
                 pid.kp, pid.ki, pid.kd, ctrl_cfg['base_speed'])
        return jsonify({'ok': True})
    except Exception as e:
        return jsonify({'ok': False, 'error': str(e)}), 400


@app.post('/run')
def post_run():
    data = request.get_json(silent=True) or {}
    running = bool(data.get('running', False))
    with _state_lock:
        _state['running'] = running
    if not running:
        pid.reset()
        bridge.send_drive(0.0, 0.0)
    log.info('Robot %s', 'STARTED' if running else 'STOPPED')
    return jsonify({'ok': True, 'running': running})


@app.get('/stream')
def stream():
    """SSE: live status at ~20 Hz. Drives the error chart and status badges."""
    def generate():
        while True:
            with _state_lock:
                payload = {
                    'error':    round(_state['error'],    4),
                    'detected': _state['detected'],
                    'left':     round(_state['left'],     3),
                    'right':    round(_state['right'],    3),
                    'running':  _state['running'],
                }
            yield f'data: {json.dumps(payload)}\n\n'
            time.sleep(0.05)

    return Response(
        generate(),
        mimetype='text/event-stream',
        headers={'Cache-Control': 'no-cache', 'X-Accel-Buffering': 'no'},
    )


@app.get('/video')
def video():
    """MJPEG stream of the camera with detection overlay (~20 fps in browser)."""
    def generate():
        while True:
            with _jpeg_lock:
                frame = _latest_jpeg
            if frame:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                       + frame + b'\r\n')
            time.sleep(0.05)

    return Response(
        generate(),
        mimetype='multipart/x-mixed-replace; boundary=frame',
    )


@app.get('/logs')
def logs():
    """SSE: streams new log lines to the browser as they arrive."""
    def generate():
        # Send the last 20 lines immediately so the browser has context
        with _LOG_LOCK:
            backfill = list(_LOG_BUFFER)[-20:]
        for line in backfill:
            yield f'data: {json.dumps(line)}\n\n'

        sent = len(_LOG_BUFFER)
        while True:
            with _LOG_LOCK:
                current = list(_LOG_BUFFER)
            if len(current) > sent:
                for line in current[sent:]:
                    yield f'data: {json.dumps(line)}\n\n'
                sent = len(current)
            time.sleep(0.25)   # poll at 4 Hz — plenty for log lines

    return Response(
        generate(),
        mimetype='text/event-stream',
        headers={'Cache-Control': 'no-cache', 'X-Accel-Buffering': 'no'},
    )

# ── Shutdown ──────────────────────────────────────────────────────────────────

def _shutdown(signum=None, _frame=None):
    log.info('Shutting down')
    bridge.send_drive(0.0, 0.0)
    bridge.stop()
    sys.exit(0)

atexit.register(_shutdown)
signal.signal(signal.SIGINT,  _shutdown)
signal.signal(signal.SIGTERM, _shutdown)

# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    log.info('Serial: connecting to %s @ %d baud', serial_cfg['port'], serial_cfg['baud'])
    bridge.start()

    threading.Thread(target=_vision_thread,  daemon=True, name='vision').start()
    threading.Thread(target=_control_thread, daemon=True, name='control').start()

    log.info('Web UI → http://0.0.0.0:%d', web_cfg['port'])
    app.run(
        host         = web_cfg['host'],
        port         = web_cfg['port'],
        threaded     = True,
        use_reloader = False,
    )
