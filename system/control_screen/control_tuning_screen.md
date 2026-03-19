
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

- Send data and logs to insights aggregator via IPC pipe.

# Ops Dashboard
- Route: `/ops`
- Purpose: web operations panel for robot stack lifecycle, service status, log viewing, and test actions.
- Requires host execution with Docker CLI access (`docker`) and compose file access.
- Enable with environment variable: `ENABLE_OPS_DASHBOARD=true`.
- Compose file defaults to `../docker-compose.yaml` relative to `control_screen/server.js`.
- Override compose path with `OPS_COMPOSE_FILE=/absolute/path/to/docker-compose.yaml`.

## Ops API endpoints
- `GET /api/ops/config` returns feature flag and service groups.
- `GET /api/ops/services` returns docker compose service states.
- `POST /api/ops/stack/up` starts selected group/services.
- `POST /api/ops/stack/stop` stops selected group/services.
- `POST /api/ops/stack/down` removes selected services or full project.
- `POST /api/ops/service/restart` restarts one service.
- `GET /api/ops/logs?service=<name>&lines=<n>` fetches recent logs.
- `POST /api/ops/tests/neutral-control` sends a neutral input.
- `POST /api/ops/tests/sample-input` sends a sample movement input.
- `POST /api/ops/robot/start` sends a state-machine start command (default state `searching`).
- `POST /api/ops/robot/stop` sends explicit stop commands to both state-machine inputs and control-communication control endpoint.

## Security and guardrails
- Services are restricted by an allowlist.
- Commands are run with fixed compose args and sanitized service names.
- Commands use timeout and max-output limits to avoid hanging or oversized responses.
- On ARM (`process.arch === "arm"` or `process.arch === "arm64"`), the default ops groups automatically exclude `alloy` because compatible images may be unavailable.
- To force-include `alloy` anyway, set `OPS_INCLUDE_ALLOY=true`.
- `core` stack group excludes `robot-mock`.
- Set `OPS_HOUGH_STREAM_URL` to prefill the Vision Stream URL in the Ops UI (default: `http://localhost:8090/stream.mjpg`).