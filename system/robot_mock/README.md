# robot_mock (2D simulator)

Runs a browser-based 2D “virtual world” that:

- **Reads commands** from control-communication (`GET /control`) to move the robot.
- **Reads state-machine state** from control-communication (`GET /state`) to decide when to emit `pick_up_success`, `place_success`, and `at_home`.
- **Sends simulated inputs** to the state-machine (`POST /inputs`) on a fixed interval.

## Run (docker compose)

From the repo root:

```bash
docker compose -f system/docker-compose.yaml up --build robot-mock
```

Open `http://localhost:8200`.

Use the control screen at `http://localhost:3000/control` to send `x/y/speed` commands.

## Smoke test (container)

```bash
docker compose -f system/docker-compose.yaml run --rm robot-mock python smoke_test.py
```

## Run (local, no external IO)

```bash
python system/robot_mock/app.py
```

Set `DISABLE_EXTERNAL_REQUESTS=1` to prevent the backend from calling other services.
