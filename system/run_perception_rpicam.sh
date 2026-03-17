#!/usr/bin/env bash
# Run perception with Pi Camera Module 2 (rpicam-vid) on the host.
# Use this when the rest of the stack is in Docker and you need the Pi camera.
# Start the stack first: docker compose up robot-mock state-machine control-screen control-communication
# Then run this script on the Pi (in another terminal).

set -e
cd "$(dirname "$0")/perception"

if [ -d ".venv" ]; then
  source .venv/bin/activate
elif [ -d "venv" ]; then
  source venv/bin/activate
fi

# Ensure state machine URL: same host as Docker (localhost when stack runs on this Pi)
export PYTHONPATH="${PYTHONPATH:-}:$(pwd)"
python3 -m src.main \
  --mode production \
  --config configs/docker-rpicam.yaml \
  --source rpicam \
  --fps 30 \
  --comms http
