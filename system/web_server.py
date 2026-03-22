"""
web_server.py

Lightweight Flask debug interface for the local mission runner.

Runs as a background daemon thread started from run_local_line_follow.py.
Access from any device on the same network: http://<pi-ip>:5050

Routes
------
  GET /                → redirect to /dashboard
  GET /dashboard       → Dashboard page (camera + telemetry + PID tuner)
  GET /video_feed      → MJPEG annotated camera stream
  GET /video_feed_mask → MJPEG red-mask camera stream
  GET /events          → SSE stream of live telemetry (~10 Hz)
  GET /api/status      → One-shot JSON telemetry snapshot
  GET /api/pid         → Current pid_config.json as JSON
  POST /api/pid        → Validate + save updated config
  POST /api/control    → {"cmd": "go"} or {"cmd": "stop"}
"""
from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
from flask import Flask, Response, jsonify, redirect, request, send_from_directory

# ── PID config constants (mirrors pid_tuner/app.py) ───────────────────────────

DEFAULTS: dict = {
    "steer_kp":                 0.65,
    "steer_ki":                 0.04,
    "steer_kd":                 0.10,
    "steer_out_limit":          0.80,
    "motor_kp":                 0.003125,
    "motor_ki":                 0.0005,
    "motor_kd":                 0.0,
    "base_speed":               0.28,
    "min_speed":                0.16,
    "max_speed":                0.45,
    "search_turn":              0.18,
    "search_turn_max":          0.32,
    "forward_ticks":            800,
    "forward_speed":            0.25,
    "turn_speed":               0.30,
    "turn_duration_s":          2.2,
    "claw_open":                0.0,
    "claw_closed":              90.0,
    "pickup_hold_s":            0.8,
    "geom_enable":              True,
    "geom_lateral_norm_m":      0.10,
    "red_loss_debounce_frames": 4,
    "red_error_ema_alpha":      0.35,
}

INT_KEYS  = {"forward_ticks", "red_loss_debounce_frames"}
BOOL_KEYS = {"geom_enable"}

_JPEG_QUALITY = 75
_STREAM_FPS   = 15


def _make_no_signal_jpeg() -> bytes:
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(img, "NO SIGNAL", (170, 250),
                cv2.FONT_HERSHEY_SIMPLEX, 2.2, (45, 45, 45), 3)
    _, buf = cv2.imencode(".jpg", img)
    return buf.tobytes()


def _validate(data: dict) -> tuple[dict, dict]:
    out, errors = {}, {}
    for key, default in DEFAULTS.items():
        raw = data.get(key)
        if raw is None:
            out[key] = default
            continue
        try:
            if key in BOOL_KEYS:
                if isinstance(raw, bool):
                    val: object = raw
                else:
                    s = str(raw).lower()
                    if   s in ("false", "no",  "0", "off"): val = False
                    elif s in ("true",  "yes", "1", "on" ): val = True
                    else:
                        errors[key] = "Must be a boolean."
                        continue
            elif key in INT_KEYS:
                val = int(raw)
            else:
                val = float(raw)
        except (TypeError, ValueError):
            errors[key] = "Must be a number."
            continue
        if not isinstance(val, bool) and (not isinstance(val, (int, float)) or val != val):
            errors[key] = "Must be a finite number."
            continue
        out[key] = val
    return out, errors


# ── WebServer ─────────────────────────────────────────────────────────────────

class WebServer:
    """
    Flask-based debug + control web interface for the local mission runner.

    Usage
    -----
        ws = WebServer(config_path=cfg_path, port=5050)
        ws.start()

        # Inside main loop each iteration:
        ws.push(frame=annotated_frame, mask=mask_frame, telemetry={...})
        cmd = ws.pop_command()   # returns 'go', 'stop', or None
    """

    def __init__(
        self,
        *,
        config_path: str | Path,
        host: str = "0.0.0.0",
        port: int = 5050,
    ) -> None:
        self._config_path = Path(config_path)
        self._host        = host
        self._port        = port

        self._lock           = threading.Lock()
        self._jpeg: Optional[bytes]      = None
        self._mask_jpeg: Optional[bytes] = None
        self._telemetry: dict            = {}
        self._cmd_queue: list[str]       = []

        self._no_signal = _make_no_signal_jpeg()

        self._app = Flask(__name__)
        self._register_routes()

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start(self) -> None:
        """Start the HTTP server in a background daemon thread."""
        t = threading.Thread(
            target=self._app.run,
            kwargs=dict(host=self._host, port=self._port,
                        threaded=True, use_reloader=False),
            daemon=True,
            name="web_server",
        )
        t.start()
        print(f"[WebServer] http://{self._host}:{self._port}/dashboard", flush=True)

    # ── Main-loop API ─────────────────────────────────────────────────────────

    def push(
        self,
        *,
        frame: Optional[np.ndarray] = None,
        mask:  Optional[np.ndarray] = None,
        telemetry: Optional[dict]   = None,
    ) -> None:
        """Push the latest frame and telemetry from the main loop."""
        jpeg = mask_jpeg = None
        if frame is not None:
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, _JPEG_QUALITY])
            if ok:
                jpeg = buf.tobytes()
        if mask is not None:
            ok, buf = cv2.imencode(".jpg", mask, [cv2.IMWRITE_JPEG_QUALITY, 65])
            if ok:
                mask_jpeg = buf.tobytes()

        with self._lock:
            if jpeg is not None:
                self._jpeg = jpeg
            if mask_jpeg is not None:
                self._mask_jpeg = mask_jpeg
            if telemetry is not None:
                self._telemetry = dict(telemetry)

    def pop_command(self) -> Optional[str]:
        """Return and remove the first pending web command ('go', 'stop'), or None."""
        with self._lock:
            return self._cmd_queue.pop(0) if self._cmd_queue else None

    # ── Route registration ────────────────────────────────────────────────────

    def _register_routes(self) -> None:
        app  = self._app
        HERE = Path(__file__).parent

        @app.get("/")
        def root():
            return redirect("/dashboard")

        @app.get("/dashboard")
        def dashboard():
            return send_from_directory(str(HERE), "web_server_index.html")

        @app.get("/video_feed")
        def video_feed():
            return Response(
                self._mjpeg(mask=False),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        @app.get("/video_feed_mask")
        def video_feed_mask():
            return Response(
                self._mjpeg(mask=True),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        @app.get("/events")
        def events():
            return Response(
                self._sse(),
                mimetype="text/event-stream",
                headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
            )

        @app.get("/api/status")
        def api_status():
            with self._lock:
                return jsonify(self._telemetry)

        @app.get("/api/pid")
        def get_pid():
            return jsonify(self._load_config())

        @app.post("/api/pid")
        def set_pid():
            data = request.get_json(silent=True)
            if not isinstance(data, dict):
                return jsonify({"error": "Expected a JSON object."}), 400
            clean, errors = _validate(data)
            if errors:
                return jsonify({"error": "Validation failed.", "fields": errors}), 400
            self._save_config(clean)
            return jsonify({"ok": True, "saved": clean})

        @app.post("/api/control")
        def api_control():
            body = request.get_json(silent=True) or {}
            cmd  = body.get("cmd")
            if cmd not in ("go", "stop"):
                return jsonify({"error": "cmd must be 'go' or 'stop'"}), 400
            with self._lock:
                self._cmd_queue.append(cmd)
            return jsonify({"cmd": cmd})

    # ── MJPEG stream ──────────────────────────────────────────────────────────

    def _mjpeg(self, mask: bool = False):
        interval = 1.0 / _STREAM_FPS
        while True:
            t0 = time.monotonic()
            with self._lock:
                jpeg = (self._mask_jpeg if mask else self._jpeg) or self._no_signal
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, interval - elapsed))

    # ── SSE telemetry ─────────────────────────────────────────────────────────

    def _sse(self):
        while True:
            with self._lock:
                payload = json.dumps(self._telemetry)
            yield f"data: {payload}\n\n"
            time.sleep(0.1)

    # ── Config I/O ────────────────────────────────────────────────────────────

    def _load_config(self) -> dict:
        try:
            raw = json.loads(self._config_path.read_text())
            return {**DEFAULTS, **{k: v for k, v in raw.items() if k in DEFAULTS}}
        except (FileNotFoundError, json.JSONDecodeError):
            return dict(DEFAULTS)

    def _save_config(self, cfg: dict) -> None:
        self._config_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self._config_path.with_suffix(".tmp")
        tmp.write_text(json.dumps(cfg, indent=2) + "\n")
        tmp.replace(self._config_path)
