#!/usr/bin/env bash
# Start the line follower.
# Usage:
#   ./run.sh                   # uses config.yaml defaults
#   SERIAL_PORT=/dev/ttyUSB0 ./run.sh

set -euo pipefail
cd "$(dirname "$0")/follower"

# Activate venv if present
if [ -d ".venv" ]; then
  source .venv/bin/activate
elif [ -d "venv" ]; then
  source venv/bin/activate
fi

# Allow overriding serial port via env var
if [ -n "${SERIAL_PORT:-}" ]; then
  # Patch config on the fly via env — main.py reads cfg['serial']['port']
  # but we can also just pass it directly.
  export FOLLOWER_SERIAL_PORT="$SERIAL_PORT"
fi

echo "[run.sh] Starting line follower — web UI at http://$(hostname -I | awk '{print $1}'):8080"
exec python3 main.py "$@"
