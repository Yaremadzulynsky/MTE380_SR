#!/usr/bin/env python3
"""
Web UI — runs on the Pi.

Access from any device on the same network:
    http://<pi-ip>:5050

GET  /                    → serves the control page
GET  /api/pid             → returns config.yaml values as JSON
POST /api/pid             → validates and saves updated values to config.yaml
GET  /api/status          → live telemetry snapshot (requires runner)
POST /api/mission/start   → start line following (requires runner)
POST /api/mission/stop    → stop line following (requires runner)
GET  /stream/camera       → MJPEG stream of annotated camera frames
GET  /stream/mask         → MJPEG stream of red-mask frames

The config file path defaults to system/config.yaml and can be overridden by the
CLI (python -m cli.main serve --config /path/to/config.yaml).
"""
from __future__ import annotations

import dataclasses
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from flask import Flask, Response, jsonify, request, send_from_directory

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent))

import config as _config_module
from config import Config

app = Flask(__name__)

# ── Mission runner (injected by cmd_serve) ────────────────────────────────────

_runner = None  # type: ignore[var-annotated]


def set_runner(runner) -> None:
    """Called by cli.main.cmd_serve to wire in the MissionRunner."""
    global _runner
    _runner = runner


def _config_path() -> Path:
    return _config_module._CONFIG_PATH


# ── Global error handler — always return JSON for /api/* ─────────────────────

@app.errorhandler(Exception)
def _json_error(e):
    import traceback
    code = getattr(e, "code", 500)
    app.logger.error("Unhandled exception: %s", traceback.format_exc())
    return jsonify({"error": str(e)}), code


# ── Camera streaming helpers ──────────────────────────────────────────────────

_NO_SIGNAL_JPEG: bytes | None = None


def _no_signal_jpeg() -> bytes:
    global _NO_SIGNAL_JPEG
    if _NO_SIGNAL_JPEG is None:
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(img, "NO SIGNAL", (160, 250),
                    cv2.FONT_HERSHEY_SIMPLEX, 2.2, (45, 45, 45), 3)
        _, buf = cv2.imencode(".jpg", img)
        _NO_SIGNAL_JPEG = buf.tobytes()
    return _NO_SIGNAL_JPEG


def _get_jpeg(get_frame_fn) -> bytes:
    frame = get_frame_fn()
    if frame is not None:
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            return buf.tobytes()
    return _no_signal_jpeg()


def _mjpeg_stream(get_frame_fn, fps: float = 15.0):
    interval = 1.0 / fps
    while True:
        t0   = time.monotonic()
        jpeg = _get_jpeg(get_frame_fn)
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + jpeg + b"\r\n"
        )
        elapsed = time.monotonic() - t0
        time.sleep(max(0.0, interval - elapsed))


# ── Routes ────────────────────────────────────────────────────────────────────

@app.get("/")
def index():
    return send_from_directory(str(HERE), "index.html")


@app.get("/api/pid")
def get_pid():
    return jsonify(dataclasses.asdict(_config_module.get()))


@app.post("/api/pid")
def set_pid():
    data = request.get_json(silent=True)
    if not isinstance(data, dict):
        return jsonify({"error": "Expected a JSON object."}), 400

    # Build a Config using the same type-coercion logic as config.load():
    # unknown or un-coercible values silently fall back to dataclass defaults.
    defaults = Config()
    kwargs: dict = {}
    for fld in dataclasses.fields(Config):
        raw = data.get(fld.name)
        if raw is None:
            continue
        expected = type(getattr(defaults, fld.name))
        try:
            kwargs[fld.name] = bool(raw) if expected is bool else expected(raw)
        except (TypeError, ValueError):
            pass  # keep dataclass default

    cfg = _config_module.reload()   # start from what's on disk
    for k, v in kwargs.items():
        setattr(cfg, k, v)
    _config_module.save(cfg)
    _config_module.reload()         # refresh singleton
    if _runner is not None:
        _runner.reconfigure(cfg)
    return jsonify({"ok": True, "saved": dataclasses.asdict(cfg)})


@app.get("/api/status")
def get_status():
    if _runner is None:
        return jsonify({"error": "No runner attached."}), 503
    snap = _runner.snapshot()
    det = snap.get("det")
    return jsonify({
        "sm_state":    snap["sm_state"],
        "running":     snap["running"],
        "enc_l":       snap["enc_l"],
        "enc_r":       snap["enc_r"],
        "rpm_l":       snap["rpm_l"],
        "rpm_r":       snap["rpm_r"],
        "pos_current":   snap["pos_current"],
        "pos_target":    snap["pos_target"],
        "pos_error":     snap["pos_error"],
        "pos_tolerance": _config_module.get().pos_tolerance,
        "red_found":   det.red_found       if det else False,
        "red_error":   det.red_error       if det else 0.0,
        "blue_found":  det.blue_found      if det else False,
        "t_junction":  det.t_junction      if det else False,
        "curve":       det.curve_detected  if det else False,
    })


@app.post("/api/mission/start")
def mission_start():
    if _runner is None:
        return jsonify({"error": "No runner attached."}), 503
    _runner.go()
    return jsonify({"ok": True, "state": _runner.state})


@app.post("/api/mission/stop")
def mission_stop():
    if _runner is None:
        return jsonify({"error": "No runner attached."}), 503
    _runner.stop()
    return jsonify({"ok": True, "state": _runner.state})


@app.get("/api/telemetry/history")
def telemetry_history():
    if _runner is None:
        return jsonify({"t": [], "red_error": [], "rpm_l": [], "rpm_r": []})
    return jsonify(_runner.telemetry_history())


@app.post("/api/test/move")
def test_move():
    """Run the position PID for delta_ticks in a background thread."""
    if _runner is None:
        return jsonify({"error": "No runner attached."}), 503
    data = request.get_json(silent=True) or {}
    try:
        delta = int(data.get("delta_ticks", 0))
    except (TypeError, ValueError):
        return jsonify({"error": "delta_ticks must be an integer"}), 400
    if delta == 0:
        return jsonify({"error": "delta_ticks cannot be zero"}), 400
    _runner.run_position_move(delta)
    return jsonify({"ok": True, "delta_ticks": delta})


@app.get("/stream/camera")
def stream_camera():
    get_fn = _runner.get_annotated_frame if _runner is not None else lambda: None
    return Response(
        _mjpeg_stream(get_fn),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.get("/stream/mask")
def stream_mask():
    get_fn = _runner.get_mask_frame if _runner is not None else lambda: None
    return Response(
        _mjpeg_stream(get_fn),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import os
    port = int(os.getenv("PORT", "5050"))
    cfg_path = Path(os.getenv("PID_CONFIG_PATH", str(_config_path())))
    _config_module.set_path(cfg_path)

    if not cfg_path.exists():
        _config_module.save(Config(), cfg_path)
        print(f"Created default config at {cfg_path}", flush=True)

    print(f"PID tuner → http://0.0.0.0:{port}", flush=True)
    print(f"Config file: {cfg_path}", flush=True)
    app.run(host="0.0.0.0", port=port, debug=False)
