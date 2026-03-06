# Perception node for Raspberry Pi 4 (arm64 / arm32v7).
# Build on the Pi: docker build -t perception .
# Or from x86 with buildx: docker buildx build --platform linux/arm64 -t perception .
#
# Production: tracks RED line (webcam). Test video: tracks BLUE line (test_run.mp4).
# post_vectors sends {"red_line"|"blue_line": {"detected": bool, "vector": {"x", "y"}}} to /inputs.

FROM python:3.11-slim-bookworm

# OpenCV runtime deps (headless; no GUI in container by default)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1 \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY requirements.txt requirements-docker.txt .
RUN pip install --no-cache-dir --default-timeout=1200 -r requirements-docker.txt

COPY configs configs
COPY src src
# Mount -v $(pwd)/tests:/app/tests when using test_run.mp4

# Default: production mode, webcam, RED line tracking.
# Override: --mode test, --source video:/app/tests/test_run.mp4, etc.
ENV PYTHONUNBUFFERED=1
CMD ["python", "-m", "src.main", "--mode", "production", "--config", "configs/default.yaml", "--source", "webcam", "--fps", "30"]
