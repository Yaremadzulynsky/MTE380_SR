#!/usr/bin/env bash
# One-time setup: creates a venv and installs dependencies.
# Run once on the Pi, then use run.sh to start the follower.

set -euo pipefail
cd "$(dirname "$0")/follower"

echo "[install] Creating virtual environment..."
python3 -m venv --system-site-packages .venv
source .venv/bin/activate

echo "[install] Installing dependencies..."
pip install --upgrade pip
pip install -r requirements.txt

# picamera2 is pre-installed system-wide on Pi OS — don't pip install it.
# If it's not available, USB webcam will be used automatically.

echo ""
echo "[install] Done. Start the robot with:"
echo "  ./system/run.sh"
