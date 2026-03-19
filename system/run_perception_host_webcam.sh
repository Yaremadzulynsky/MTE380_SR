#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PERCEPTION_DIR="$ROOT_DIR/perception"

cd "$PERCEPTION_DIR"

python3 -m src.main \
  --mode test \
  --no-gui \
  --config configs/docker-webcam.yaml \
  --source "${1:-iphone}"
