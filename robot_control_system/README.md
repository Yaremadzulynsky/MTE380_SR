# MTE 380 Robot Control System

## Setup (Pi)

```bash
# Create venv with access to system opencv/picamera2
python -m venv .venv --system-site-packages
source .venv/bin/activate
pip install -r requirements.txt

# Install system packages if not already present
sudo apt install -y python3-opencv python3-picamera2 rpicam-apps
```

## Run commands

### Full robot (state machine + web server + camera)
```bash
python -m state_machine.main --port /dev/ttyACM1 --camera pi
```

### Full robot without camera
```bash
python -m state_machine.main --port /dev/ttyACM1
```

### Web server + camera only (no robot/serial)
```bash
python -m web_server.main --no-robot --camera pi
```

### Hardware REPL (interactive serial terminal)
```bash
python -m hardware.main --port /dev/ttyACM1
```

## Options

| Flag | Default | Description |
|------|---------|-------------|
| `--port` | `/dev/ttyACM0` | Arduino serial port |
| `--camera pi` | — | Use Pi Camera via rpicam-vid |
| `--camera 0` | — | Use webcam index 0 |
| `--no-robot` | — | Skip serial connection |
| `--web-port` | `8321` | HTTP port |
| `--max-speed` | `1.0` | Cap robot speed (0–1) |
| `--max-rot-speed` | `1.0` | Cap rotation speed (0–1) |
| `--initial-state` | `stopped` | Start in `stopped` or `line_follow` |
| `--debug` | — | Verbose logging + annotated camera overlay |

## CLI

```bash
# Server commands (requires state machine running)
python -m cli.main start                            # → line_follow
python -m cli.main stop                             # → stopped
python -m cli.main state get
python -m cli.main state set line_follow
python -m cli.main status                           # live telemetry stream

# Hardware commands (direct serial)
python -m cli.main --port /dev/ttyACM1 motors 0.3 0.3
python -m cli.main --port /dev/ttyACM1 direction 0 1
python -m cli.main --port /dev/ttyACM1 speed 0.4
python -m cli.main --port /dev/ttyACM1 rotation 0.5
python -m cli.main --port /dev/ttyACM1 drive 500 500
python -m cli.main --port /dev/ttyACM1 wheel left 300
python -m cli.main --port /dev/ttyACM1 servo 90
python -m cli.main --port /dev/ttyACM1 gains heading 2.0 0.1 0.0
python -m cli.main --port /dev/ttyACM1 gains speed 1.0 0.0 0.0
python -m cli.main --port /dev/ttyACM1 heading       # live heading stream
python -m cli.main --port /dev/ttyACM1 encoders      # live encoder stream
```

## Dashboard

Open in browser: `http://<pi-ip>:8321`

- Live camera feed with line overlay
- Hardware telemetry (heading, encoders, motor outputs)
- **PID tuner** — tune heading/speed gains live
- **Manual control** — set direction/speed, turn left/right, stop

## Serial port

```bash
ls /dev/ttyACM*          # find Arduino port
sudo chmod 666 /dev/ttyACM1   # fix permissions if needed
```

## USB passthrough (WSL2 only)

```powershell
# PowerShell as Admin
usbipd list
usbipd attach --wsl --busid <BUSID>
```
