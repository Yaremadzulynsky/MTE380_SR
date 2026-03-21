# MTE380_SR — `zain/simple` branch

This branch is intentionally **minimal**: no Docker stack, no separate perception/state-machine services, no Grafana/control-screen bundles.

## Contents

| Path | Role |
|------|------|
| `system/run_local_line_follow.py` | Main entry: camera → mission state machine → serial motors |
| `system/local/` | Perception (OpenCV), mission states, motor PID + `SerialBridge` |
| `system/control_communication/robot_control_system/` | Serial bridge + `simple-pid` heading/speed helpers + optional `main.py` CLI |
| `system/pid_config.json` | Tunables for steering + wheel PIDs and speeds |
| `system/pid_tuner/` | Small Flask UI to edit `pid_config.json` |
| `system/requirements-local.txt` | `pip install -r` for the Pi venv |

See **`system/README_LOCAL.md`** for how to run on the Pi.

## Other branches

Use `main` or feature branches for the full multi-container / HTTP architecture.
