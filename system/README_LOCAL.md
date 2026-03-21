# Local Pi runtime (no Docker)

1. OpenCV reads camera frames.
2. HSV finds the red line; optional blue marker and T-junction cues.
3. `MissionStateMachine` turns line error into left/right speed fractions (steering PID via `simple-pid`).
4. `LocalMotorController` runs per-wheel RPM PIDs (`simple-pid`) and sends voltages over serial (`SerialBridge`).

## Install

From the `system/` directory:

```bash
pip install -r requirements-local.txt
```

## Run line follow

```bash
cd system
python run_local_line_follow.py --serial-port /dev/ttyACM0
```

Flags (see `--help`): `--dry-run`, `--no-display`, `--pid-config`, steering/motor gains, ROI tuning.

## PID config

Defaults live in `pid_config.json`. Optional: run `python pid_tuner/app.py` and open the served page to edit the same file (requires Flask).
