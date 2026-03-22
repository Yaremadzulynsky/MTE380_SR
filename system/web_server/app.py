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


# ── Camera streaming helpers ──────────────────────────────────────────────────

def _jpeg_stream(get_frame_fn, fps: float = 15.0):
    """Generator yielding MJPEG frames from a callable that returns a BGR ndarray."""
    period = 1.0 / fps
    while True:
        t0 = time.time()
        frame = get_frame_fn()
        if frame is not None:
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n"
                    + buf.tobytes()
                    + b"\r\n"
                )
        elapsed = time.time() - t0
        remaining = period - elapsed
        if remaining > 0:
            time.sleep(remaining)


# ── Routes ────────────────────────────────────────────────────────────────────

@app.get("/")
def index():
    return send_from_directory(str(HERE), "index.html")


@app.get("/api/pid")
def get_pid():
    cfg = _config_module.load(_config_path())
    return jsonify(dataclasses.asdict(cfg))


@app.post("/api/pid")
def set_pid():
    data = request.get_json(silent=True)
    if not isinstance(data, dict):
        return jsonify({"error": "Expected a JSON object."}), 400

    clean, errors = _validate(data)
    if errors:
        return jsonify({"error": "Validation failed.", "fields": errors}), 400

    cfg = Config(**clean)
    _config_module.save(cfg, _config_path())
    # Refresh the singleton so the next get() reflects the new values
    _config_module.reload()
    return jsonify({"ok": True, "saved": dataclasses.asdict(cfg)})


@app.get("/api/status")
def get_status():
    if _runner is None:
        return jsonify({"error": "No runner attached."}), 503
    snap = _runner.snapshot()
    det = snap.get("det")
    return jsonify({
        "sm_state":   snap["sm_state"],
        "running":    snap["running"],
        "enc_l":      snap["enc_l"],
        "enc_r":      snap["enc_r"],
        "rpm_l":      snap["rpm_l"],
        "rpm_r":      snap["rpm_r"],
        "red_found":  det.red_found  if det else False,
        "red_error":  det.red_error  if det else 0.0,
        "blue_found": det.blue_found if det else False,
        "t_junction": det.t_junction if det else False,
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


@app.get("/stream/camera")
def stream_camera():
    if _runner is None:
        return jsonify({"error": "No runner attached."}), 503
    return Response(
        _jpeg_stream(_runner.get_annotated_frame),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.get("/stream/mask")
def stream_mask():
    if _runner is None:
        return jsonify({"error": "No runner attached."}), 503
    return Response(
        _jpeg_stream(_runner.get_mask_frame),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


# ── Validation ────────────────────────────────────────────────────────────────

def _validate(data: dict) -> tuple[dict, dict]:
    """Return (clean_values, errors). Uses Config field types for coercion."""
    defaults = Config()
    out: dict = {}
    errors: dict = {}

    for fld in dataclasses.fields(Config):
        raw = data.get(fld.name)
        if raw is None:
            out[fld.name] = getattr(defaults, fld.name)
            continue

        expected = type(getattr(defaults, fld.name))
        try:
            if expected is bool:
                if isinstance(raw, bool):
                    val = raw
                else:
                    s = str(raw).lower()
                    if s in ("false", "no", "0", "off"):
                        val = False
                    elif s in ("true", "yes", "on", "1"):
                        val = True
                    else:
                        errors[fld.name] = "Must be a boolean."
                        continue
            elif expected is int:
                val = int(raw)
            else:
                val = float(raw)
                if val != val:   # NaN check
                    raise ValueError("NaN")
        except (TypeError, ValueError):
            errors[fld.name] = f"Must be a {'boolean' if expected is bool else 'number'}."
            continue

        out[fld.name] = val

    return out, errors


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
