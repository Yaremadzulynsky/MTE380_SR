# Perception node for Raspberry Pi 4 (arm64 / arm32v7).
# Build on the Pi: docker build -t perception .
# For Pi Camera in Docker use Dockerfile.rpicam (perception-rpicam service).
# Or from x86 with buildx: docker buildx build --platform linux/arm64 -t perception .
#
# Uses python3-opencv from Debian (built with GStreamer support).
# Production defaults to RED line tracking; sample replay config tracks BLUE tape.
# Set camera.backend: gstreamer in config for GStreamer capture.
# For Pi Camera Module 2, run perception on the host (run_perception_rpicam.sh), not in Docker.

FROM debian:bookworm-slim

# OpenCV (with GStreamer), GStreamer plugins, Python deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-opencv \
    python3-numpy \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    libgl1 \
    libglib2.0-0 \
    v4l-utils \
    curl \
    ca-certificates \
    gnupg \
    && rm -rf /var/lib/apt/lists/*

# Pi Camera Module 2: do NOT install rpicam-apps in Docker (Debian vs Pi repo conflicts).
# Run perception on the HOST when using the Pi camera - see run_perception_rpicam.sh in system/

WORKDIR /app

# OpenCV from apt; pip only for pyserial, pyyaml, pytest
RUN pip3 install --no-cache-dir --break-system-packages pyserial pyyaml pytest

COPY configs configs
COPY src src
# Mount -v $(pwd)/tests:/app/tests when using test_run.mp4

# Default: production mode, webcam, RED line tracking.
# Uses configs/docker.yaml (backend: gstreamer) by default.
ENV PYTHONUNBUFFERED=1
CMD ["python3", "-m", "src.main", "--mode", "production", "--config", "configs/docker.yaml", "--source", "webcam", "--fps", "30", "--comms", "http"]
