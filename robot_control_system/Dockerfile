FROM python:3.11-slim

# ── System dependencies ───────────────────────────────────────────────────────
# OpenCV headless runtime
RUN apt-get update && apt-get install -y --no-install-recommends \
        libglib2.0-0 \
        libgl1 \
        libsm6 \
        libxext6 \
    && rm -rf /var/lib/apt/lists/*

# ── Picamera2 + libcamera ─────────────────────────────────────────────────────
# These packages are provided by Raspberry Pi OS (Bullseye / Bookworm).
# On standard Debian they will not be found — only needed on the Pi.
# If unavailable, apt-get exits 0 and the rest of the image still builds;
# the container just won't support --camera pi on non-Pi hosts.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-picamera2 \
        python3-libcamera \
        libcamera-apps \
    2>/dev/null || true \
    && rm -rf /var/lib/apt/lists/*

# Make system picamera2 / libcamera bindings visible to our Python interpreter
ENV PYTHONPATH=/usr/lib/python3/dist-packages

# ── Python dependencies ───────────────────────────────────────────────────────
WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# ── Application code ──────────────────────────────────────────────────────────
COPY . .

EXPOSE 5000

# ── Default: full robot + Pi camera ──────────────────────────────────────────
# Override with env vars:
#   SERIAL_PORT  — serial device for Arduino  (default /dev/ttyACM0)
#   CAMERA       — camera source: "pi", "0", URL (default pi)
#   WEB_PORT     — HTTP port for dashboard    (default 5000)
ENV SERIAL_PORT=/dev/ttyACM0
ENV CAMERA=pi
ENV WEB_PORT=5000

CMD python -m state_machine.main \
        --port    "$SERIAL_PORT" \
        --camera  "$CAMERA"     \
        --web-port "$WEB_PORT"
