FROM debian:bookworm-slim

# ── System dependencies ───────────────────────────────────────────────────────
# python3-opencv from Debian: pre-built for ARM, no compiler needed
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 \
        python3-pip \
        python3-opencv \
        python3-numpy \
        libgl1 \
        libglib2.0-0 \
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

# Make system opencv / picamera2 / libcamera bindings visible to Python
ENV PYTHONPATH=/usr/lib/python3/dist-packages

# ── Python dependencies ───────────────────────────────────────────────────────
WORKDIR /app
COPY requirements.txt .
RUN pip3 install --no-cache-dir --break-system-packages -r requirements.txt

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

CMD python3 -m state_machine.main \
        --port    "$SERIAL_PORT" \
        --camera  "$CAMERA"     \
        --web-port "$WEB_PORT"
