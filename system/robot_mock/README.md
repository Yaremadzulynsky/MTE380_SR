# robot_mock (2D simulator)

Runs a browser-based 2D “virtual world” that:

- **Uses a local neutral control command** (`x=0, y=0, speed=0`) unless manual override is enabled in the UI.
- **Reads state-machine state** from state-machine (`GET /states`) to decide when to emit `pick_up_success`, `place_success`, and `at_home`.
- **Sends simulated inputs** to the state-machine (`POST /inputs`) on a fixed interval.

## Run (docker compose)

From the repo root:

```bash
docker compose -f system/docker-compose.yaml up --build robot-mock
```

Open `http://localhost:8200`.

## Smoke test (container)

```bash
docker compose -f system/docker-compose.yaml run --rm robot-mock python smoke_test.py
```

## Run (local, no external IO)

```bash
python system/robot_mock/app.py
```

Set `DISABLE_EXTERNAL_REQUESTS=1` to prevent the backend from calling other services.
