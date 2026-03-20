"""
web_server/server.py

Lightweight Flask debug interface for the MTE 380 robot.

Routes
------
  GET /              → Dashboard page (HTML)
  GET /video_feed    → MJPEG camera stream (use in <img> tag)
  GET /events        → Server-Sent Events stream of live debug data (JSON)
  GET /api/status    → One-shot JSON status snapshot

Usage
-----
    from state_machine.machine import StateMachine
    from web_server.server import WebServer

    sm = StateMachine(robot, detector)
    sm.start('idle')

    server = WebServer(state_machine=sm)
    server.start()          # non-blocking — runs in a daemon thread

    # ... rest of your main loop ...
"""

import json
import logging
import threading
import time
from typing import Optional

import cv2
import numpy as np
from flask import Flask, Response, render_template, request, jsonify

import sys as _sys, pathlib as _pathlib
_sys.path.insert(0, str(_pathlib.Path(__file__).parent.parent))
import config as _config

log = logging.getLogger(__name__)

_wc = _config.get()['web_server']
_JPEG_QUALITY = _wc['jpeg_quality']
_STREAM_FPS   = _wc['stream_fps']

# ── No-signal placeholder ──────────────────────────────────────────────────────

def _make_no_signal_jpeg() -> bytes:
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(img, 'NO SIGNAL', (170, 250),
                cv2.FONT_HERSHEY_SIMPLEX, 2.2, (45, 45, 45), 3)
    _, buf = cv2.imencode('.jpg', img)
    return buf.tobytes()


_NO_SIGNAL_JPEG: Optional[bytes] = None   # lazy-init


# ── WebServer ─────────────────────────────────────────────────────────────────

class WebServer:
    """
    Flask-based debug web interface.

    Parameters
    ----------
    robot    : hardware.robot.Robot or None
        Source of hardware telemetry (encoders, heading, motor outputs …).
        Pass None to disable the hardware panel.
    detector : vision.line_detector.LineDetector or None
        Source of camera frames and line-detection results.
        Pass None to disable the camera feed and line panel.
    host     : str
        Bind address. '0.0.0.0' makes the server reachable on the LAN.
    port     : int
        HTTP port (default 8321).
    """

    def __init__(self, *, state_machine=None,
                 host: str = '0.0.0.0', port: int = 8321):
        self._state_machine = state_machine
        self._robot         = state_machine.robot    if state_machine else None
        self._detector      = state_machine.detector if state_machine else None
        self._state_log: list = []   # recent state transitions [(ts, state), ...]
        self._host          = host
        self._port          = port
        self._app           = Flask(__name__)
        self._thread: Optional[threading.Thread] = None
        self._register_routes()

    # ── Lifecycle ──────────────────────────────────────────────────────────────

    def start(self) -> None:
        """Start the HTTP server in a background daemon thread."""
        self._thread = threading.Thread(
            target=self._app.run,
            kwargs=dict(
                host=self._host,
                port=self._port,
                threaded=True,
                use_reloader=False,
            ),
            daemon=True,
            name='web_server',
        )
        self._thread.start()
        log.info('WebServer listening at http://%s:%d', self._host, self._port)

    def stop(self) -> None:
        # Flask's built-in server has no clean-stop API; the daemon thread
        # will exit with the process.
        log.info('WebServer stop requested')

    # ── Route registration ─────────────────────────────────────────────────────

    def _register_routes(self) -> None:
        app = self._app

        @app.route('/')
        def index():
            return render_template('index.html')

        @app.route('/video_feed')
        def video_feed():
            return Response(
                self._mjpeg_stream(),
                mimetype='multipart/x-mixed-replace; boundary=frame',
            )

        @app.route('/events')
        def events():
            return Response(
                self._sse_stream(),
                mimetype='text/event-stream',
                headers={
                    'Cache-Control':    'no-cache',
                    'X-Accel-Buffering': 'no',   # disable nginx buffering if proxied
                },
            )

        @app.route('/api/status')
        def api_status():
            return app.response_class(
                response=json.dumps(self._collect_status()),
                mimetype='application/json',
            )

        @app.route('/api/state', methods=['GET'])
        def get_state():
            sm = self._state_machine
            return jsonify({'state': sm.current_state if sm else None})

        @app.route('/api/state', methods=['POST'])
        def set_state():
            sm = self._state_machine
            if sm is None:
                return jsonify({'error': 'no state machine'}), 503
            body = request.get_json(silent=True) or {}
            state = body.get('state')
            if not state:
                return jsonify({'error': 'missing "state"'}), 400
            sm.transition(state)
            return jsonify({'state': state})

        @app.route('/api/direction', methods=['POST'])
        def set_direction():
            robot = self._robot
            if robot is None:
                return jsonify({'error': 'no robot'}), 503
            body = request.get_json(silent=True) or {}
            x = body.get('x', 0.0)
            y = body.get('y', 0.0)
            robot.add_direction(float(x), float(y))
            return jsonify({'x': x, 'y': y})

        @app.route('/api/speed', methods=['POST'])
        def set_speed():
            robot = self._robot
            if robot is None:
                return jsonify({'error': 'no robot'}), 503
            body = request.get_json(silent=True) or {}
            speed = body.get('speed', 0.0)
            robot.set_speed(float(speed))
            return jsonify({'speed': speed})
        
        @app.route('/api/gains', methods=['GET'])
        def get_gains():
            robot = self._robot
            if robot is None:
                return jsonify({'error': 'no robot'}), 503
            return jsonify(robot.get_gains())

        @app.route('/api/gains', methods=['POST'])
        def set_gains():
            robot = self._robot
            if robot is None:
                return jsonify({'error': 'no robot'}), 503
            body = request.get_json(silent=True) or {}
            pid  = body.get('pid')   # 'heading' or 'speed'
            kp   = body.get('kp')
            ki   = body.get('ki')
            kd   = body.get('kd')
            if pid not in ('heading', 'speed') or None in (kp, ki, kd):
                return jsonify({'error': 'expected pid, kp, ki, kd'}), 400
            if pid == 'heading':
                robot.set_gains(float(kp), float(ki), float(kd))
            else:
                robot.set_speed_gains(float(kp), float(ki), float(kd))
            return jsonify(robot.get_gains())

        @app.route('/api/limits', methods=['GET'])
        def get_limits():
            import state_machine.hardware.robot as _hw
            return jsonify({'max_speed': _hw.MAX_SPEED, 'max_rot_speed': _hw.MAX_ROT_SPEED})

        @app.route('/api/limits', methods=['POST'])
        def set_limits():
            import state_machine.hardware.robot as _hw
            body = request.get_json(silent=True) or {}
            if 'max_speed' in body:
                _hw.MAX_SPEED = max(0.0, min(1.0, float(body['max_speed'])))
            if 'max_rot_speed' in body:
                _hw.MAX_ROT_SPEED = max(0.0, min(1.0, float(body['max_rot_speed'])))
            return jsonify({'max_speed': _hw.MAX_SPEED, 'max_rot_speed': _hw.MAX_ROT_SPEED})

    # ── MJPEG stream ───────────────────────────────────────────────────────────

    def _mjpeg_stream(self):
        global _NO_SIGNAL_JPEG
        if _NO_SIGNAL_JPEG is None:
            _NO_SIGNAL_JPEG = _make_no_signal_jpeg()

        interval = 1.0 / _STREAM_FPS
        while True:
            t0   = time.monotonic()
            jpeg = self._get_jpeg()
            yield (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n'
            )
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, interval - elapsed))

    def _get_jpeg(self) -> bytes:
        if self._detector is not None:
            frame = self._detector.get_frame()
            if frame is not None:
                ok, buf = cv2.imencode(
                    '.jpg', frame,
                    [cv2.IMWRITE_JPEG_QUALITY, _JPEG_QUALITY],
                )
                if ok:
                    return buf.tobytes()
        return _NO_SIGNAL_JPEG  # type: ignore[return-value]

    # ── SSE stream ─────────────────────────────────────────────────────────────

    def _sse_stream(self):
        while True:
            payload = json.dumps(self._collect_status())
            yield f'data: {payload}\n\n'
            time.sleep(0.1)   # push at ~10 Hz

    # ── Status collection ──────────────────────────────────────────────────────

    def _collect_status(self) -> dict:
        import math as _math
        status: dict = {
            'ts':    round(time.time(), 3),
            'state': self._state_machine.current_state if self._state_machine else None,
        }

        sm = self._state_machine
        if sm is not None:
            x, y, heading = sm.odometry.pose()
            status['odometry'] = {
                'x_m':        round(x, 4),
                'y_m':        round(y, 4),
                'heading_deg': round(_math.degrees(heading), 2),
            }

        if self._robot is not None:
            info = self._robot.get_debug_info()
            status['hardware'] = {
                'encoders': {
                    'left':  info['encoders'][0],
                    'right': info['encoders'][1],
                },
                'heading': {
                    'current_deg': round(info['heading_current'], 2),
                    'target_deg':  round(info['heading_target'],  2),
                },
                'speed_scale':      round(info['speed_scale'], 3),
                'linear_speed_mps': round(info['linear_speed_mps'], 3),
                'last_drive': {
                    'left':  round(info['last_drive'][0], 3),
                    'right': round(info['last_drive'][1], 3),
                },
                'angular_z': round(info.get('angular_z', 0.0), 3),
                'heartbeat_age_s': (
                    round(info['heartbeat_age_s'], 2)
                    if info['heartbeat_age_s'] is not None else None
                ),
            }

        if self._detector is not None:
            result = self._detector.get_result()
            if result is not None:
                import config as _cfg
                _vc = _cfg.get()['vision']
                _ppm       = _vc.get('pixels_per_meter', 0)
                _cam_fwd_m = _vc.get('camera_forward_m', 0.0)
                if _ppm and _cam_fwd_m:
                    _lat_m = result.lateral_distance_px / _ppm
                    _delta_rad = _math.atan2(_lat_m, _cam_fwd_m)
                else:
                    _delta_rad = None
                status['line'] = {
                    'found':                True,
                    'angle_deg':            round(result.angle_deg, 2),
                    'lateral_distance_px':  round(result.lateral_distance_px, 1),
                    'lateral_distance_m':   (round(result.lateral_distance_m, 4)
                                             if result.lateral_distance_m is not None
                                             else None),
                    'delta_angle_deg':      (round(_math.degrees(_delta_rad), 2)
                                             if _delta_rad is not None else None),
                    'direction':            [
                        round(result.direction[0], 3),
                        round(result.direction[1], 3),
                    ],
                }
            else:
                status['line'] = {'found': False}

            status['vision'] = self._detector.get_stats()

        # State transition log
        if sm is not None:
            current = sm.current_state
            if not self._state_log or self._state_log[-1]['state'] != current:
                self._state_log.append({'ts': round(time.time(), 2), 'state': current})
                if len(self._state_log) > 30:
                    self._state_log.pop(0)
        status['state_log'] = list(self._state_log)

        return status
