#!/usr/bin/env python3
"""
PID tuner web UI — runs on the Pi.

Access from any device on the same network:
    http://<pi-ip>:5050

GET  /         → serves the tuner page
GET  /api/pid  → returns config.yaml values as JSON
POST /api/pid  → validates and saves updated values to config.yaml, returns saved values

The config file path defaults to system/config.yaml and can be overridden by the
CLI (python -m cli.main serve --config /path/to/config.yaml).
"""
from __future__ import annotations

import dataclasses
import sys
from pathlib import Path

from flask import Flask, jsonify, request, send_from_directory

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent))

import config as _config_module
from config import Config

app = Flask(__name__)


def _config_path() -> Path:
    return _config_module._CONFIG_PATH


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
