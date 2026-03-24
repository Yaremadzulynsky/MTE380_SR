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
        "sm_state":  snap["sm_state"],
        "running":   snap["running"],
        "enc_l":     snap["enc_l"],
        "enc_r":     snap["enc_r"],
        "rpm_l":     snap["rpm_l"],
        "rpm_r":     snap["rpm_r"],
        "red_found":    det.red_found   if det else False,
        "red_error":    det.red_error   if det else 0.0,
        "blue_found":   det.blue_found  if det else False,
        "green_found":  det.green_found if det else False,
        "lateral_turn": snap["lateral_turn"],
        "heading_turn": snap["heading_turn"],
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


@app.get("/api/test/status")
def test_status():
    """Live progress of the active position or rotation move."""
    if _runner is None:
        return jsonify({"active": None})
    return jsonify(_runner.move_status())


@app.post("/api/test/stop")
def test_stop():
    """Immediately cancel any active position/rotation move and idle motors."""
    if _runner is None:
        return jsonify({"error": "No runner attached."}), 503
    _runner.stop_move()
    return jsonify({"ok": True})


@app.post("/api/servo")
def servo():
    """Send a one-shot servo angle command (degrees 0–180)."""
    if _runner is None:
        print("[servo] ERROR: no runner attached", flush=True)
        return jsonify({"error": "No runner attached."}), 503
    data = request.get_json(silent=True) or {}
    try:
        angle = float(data["angle"])
    except (KeyError, TypeError, ValueError):
        return jsonify({"error": "angle must be a number"}), 400
    angle = max(0.0, min(180.0, angle))
    dry = getattr(_runner, "dry_run", "?")
    bridge = getattr(_runner, "bridge", "?")
    print(f"[servo] angle={angle}  dry_run={dry}  bridge={bridge}", flush=True)
    _runner.send_claw(angle)
    print(f"[servo] send_claw done", flush=True)
    return jsonify({"ok": True, "angle": angle})


@app.post("/api/mask/channel")
def mask_channel():
    """Switch the /stream/mask overlay: body = {"channel": "red"|"green"|"blue"}."""
    if _runner is None:
        return jsonify({"error": "No runner attached."}), 503
    data = request.get_json(silent=True) or {}
    channel = data.get("channel", "red")
    if channel not in ("red", "green", "blue"):
        return jsonify({"error": "channel must be red, green, or blue"}), 400
    _runner.set_mask_channel(channel)
    return jsonify({"ok": True, "channel": channel})


@app.post("/api/drive")
def drive():
    """Continuously send direct voltage to motors until /api/drive/stop."""
    if _runner is None:
        return jsonify({"error": "No runner attached."}), 503
    data = request.get_json(silent=True) or {}
    try:
        left  = float(data["left"])
        right = float(data["right"])
    except (KeyError, TypeError, ValueError):
        return jsonify({"error": "left and right must be numbers"}), 400
    _runner.start_drive(left, right)
    return jsonify({"ok": True, "left": left, "right": right})


@app.post("/api/drive/stop")
def drive_stop():
    """Stop the manual drive loop."""
    if _runner is None:
        return jsonify({"error": "No runner attached."}), 503
    _runner.stop_drive()
    return jsonify({"ok": True})


@app.post("/api/test/position")
def test_position():
    """Drive forward/backward by metres using the position PID."""
    if _runner is None:
        return jsonify({"error": "No runner attached."}), 503
    data = request.get_json(silent=True) or {}
    try:
        meters = float(data["meters"])
    except (KeyError, TypeError, ValueError):
        return jsonify({"error": "meters must be a number"}), 400
    _runner.run_position(meters)
    return jsonify({"ok": True, "meters": meters})


@app.post("/api/test/rotation")
def test_rotation():
    """Stop the mission and rotate degrees (positive = CW)."""
    if _runner is None:
        return jsonify({"error": "No runner attached."}), 503
    data = request.get_json(silent=True) or {}
    try:
        degrees = float(data["degrees"])
    except (KeyError, TypeError, ValueError):
        return jsonify({"error": "degrees must be a number"}), 400
    _runner.run_rotation(degrees)
    return jsonify({"ok": True, "degrees": degrees})


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
