
# Purpose
Allow users to tune the Proportional, Integral, and Derivative (PID) parameters of the control system through a web interface.

# Functions
- Display current PID parameters (P, I, D) on the screen.
- Allow users to input new values for P, I, and D.
- Validate user inputs to ensure they are within acceptable ranges.
- Send updated PID parameters to the control communication endpoint.
- Provide feedback to the user on successful updates or errors.

# Technologies
- HTML
- CSS
- JavaScript
- Express.js (Node.js framework)
- RESTful API

# Ingress
- User inputs via the control tuning screen.
# Egress
Will send HTTP GET/POST requests to the control communication endpoint.
- set Proportional returns set P value and returns 200 OK
- set Integral returns set I value and returns 200 OK
- set Derivative returns set D value and returns 200 OK
- get Proportional returns P
- get Integral returns I
- get Derivative returns D


# Ops Dashboard
- Route: `/ops`
- Purpose: host-side panel for **perception runner control**, perception logs, vision stream preview, and robot test actions.
- This page does **not** manage Docker containers anymore; run your containers manually.
- Perception runner control is enabled by default; disable with `ENABLE_PERCEPTION_RUNNER=false`.
- Script defaults to `../run_perception_rpicam.sh` relative to `control_screen/server.js`.
- Override script path with `PERCEPTION_RUN_SCRIPT=/absolute/path/to/run_perception_rpicam.sh`.
- Override working directory with `PERCEPTION_RUN_CWD=/absolute/path/to/system`.
- **Docker:** `docker-compose.yaml` mounts the repo at `/workspace/system` and sets `PERCEPTION_RUN_*` so the script is not resolved as `/run_perception_rpicam.sh` (broken default when `server.js` lives in `/app` only). Rebuild `control-screen` after changing the Dockerfile (bash is required to run the script).
- Perception logs are written to `../perception/logs/control-screen-perception.log` by default (override with `PERCEPTION_LOG_FILE`).

## Ops API endpoints
- `GET /api/ops/config` returns runner feature flags and defaults.
- `GET /api/ops/services` returns `perception-runner` status.
- `GET /api/ops/perception/status` returns detailed runner status.
- `POST /api/ops/stack/up` starts the perception runner.
- `POST /api/ops/stack/stop` stops the perception runner.
- `POST /api/ops/stack/down` alias for stopping the perception runner.
- `POST /api/ops/service/restart` restarts the perception runner.
- `GET /api/ops/logs?service=perception-runner&lines=<n>` fetches recent runner logs.
- `POST /api/ops/tests/neutral-control` sends a neutral input.
- `POST /api/ops/tests/sample-input` sends a sample movement input.
- `POST /api/ops/tests/turn` runs encoder-validated turn tests (degrees in payload) and then auto-sends robot stop.
- `POST /api/ops/robot/start` sends a state-machine start command (default state `searching`).
- `POST /api/ops/robot/stop` sends explicit stop commands to both state-machine inputs and control-communication control endpoint.

## Security and guardrails
- Only the configured local runner script is started/stopped by `/ops`.
- Container lifecycle is intentionally excluded from the UI/API in this dashboard mode.
- Set `OPS_HOUGH_STREAM_URL` to prefill the Vision Stream URL in the Ops UI (default: `http://localhost:8090/stream.mjpg`).
- **H.264 / WebRTC (recommended):** On the Pi run `docker compose --profile mediamtx up -d mediamtx`, then start the Hough tool with
  `--h264-rtsp-url rtsp://127.0.0.1:8554/hough` (requires `ffmpeg` on the Pi). In Ops, choose viewer **Web / WebRTC (iframe)** and set the URL to
  `http://<pi-tailscale-host>:8889/hough` (MediaMTX embeds the player).
- **H.264 / UDP:** Use `--h264-udp <laptop-tailscale-ip>:5004` and view on the laptop with `ffplay -fflags nobuffer udp://0.0.0.0:5004?localaddr=<laptop-ip>` (or VLC).