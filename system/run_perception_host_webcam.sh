#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PERCEPTION_DIR="$ROOT_DIR/perception"

cd "$PERCEPTION_DIR"

CONFIG_PATH="configs/docker-webcam.yaml"
SOURCE_ARG="${1:-iphone}"
EXTRA_ARGS=()

if [[ "$SOURCE_ARG" == "--udp-stream" || "$SOURCE_ARG" == "udp" || "$SOURCE_ARG" == "udp-stream" ]]; then
  CONFIG_PATH="configs/docker-udp.yaml"
  SOURCE_ARG="udp://0.0.0.0:5000"
  EXTRA_ARGS+=("--udp-stream")
elif [[ "$SOURCE_ARG" == "--rpicam" || "$SOURCE_ARG" == "rpicam" || "$SOURCE_ARG" == "pi-cam" ]]; then
  CONFIG_PATH="configs/docker-rpicam.yaml"
  SOURCE_ARG="rpicam"
elif [[ "$SOURCE_ARG" == "--iphone" || "$SOURCE_ARG" == "iphone" || "$SOURCE_ARG" == "continuity" ]]; then
  SOURCE_ARG="iphone"
elif [[ "$SOURCE_ARG" == "--webcam" || "$SOURCE_ARG" == "webcam" ]]; then
  SOURCE_ARG="webcam"
fi

CMD=(
  python3 -m src.main
  --no-gui
  --config "$CONFIG_PATH"
)

if ((${#EXTRA_ARGS[@]} > 0)); then
  CMD+=("${EXTRA_ARGS[@]}")
fi

CMD+=(
  --source "$SOURCE_ARG"
)

"${CMD[@]}"
