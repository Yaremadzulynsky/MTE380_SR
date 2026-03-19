#!/usr/bin/env bash
# Run perception with Pi Camera Module 2 (rpicam-vid) on the host.
# Use this when the rest of the stack is in Docker and you need the Pi camera.
# Start the stack first: docker compose up state-machine control-screen control-communication
# Then run this script on the Pi (in another terminal).

set -euo pipefail
cd "$(dirname "$0")/perception"

activate_local_venv_if_present() {
  if [ -d ".venv" ]; then
    # shellcheck source=/dev/null
    source .venv/bin/activate
  elif [ -d "venv" ]; then
    # shellcheck source=/dev/null
    source venv/bin/activate
  fi
}

ensure_python_deps() {
  if python3 -c "import yaml, cv2, numpy, serial" >/dev/null 2>&1; then
    return
  fi

  echo "[run_perception_rpicam] missing python dependencies; bootstrapping local venv"
  if [ ! -d ".venv" ]; then
    # Reuse system OpenCV/Numpy on Pi to avoid slow wheel builds.
    python3 -m venv --system-site-packages .venv
  fi

  # shellcheck source=/dev/null
  source .venv/bin/activate
  python3 -m pip install --upgrade pip >/dev/null
  python3 -m pip install pyyaml pyserial pytest >/dev/null

  if ! python3 -c "import yaml, cv2, numpy, serial" >/dev/null 2>&1; then
    echo "[run_perception_rpicam] error: dependencies still missing after install" >&2
    exit 1
  fi
}

activate_local_venv_if_present
ensure_python_deps

export PYTHONPATH="${PYTHONPATH:-}:$(pwd)"

STATE_MACHINE_INPUT_URL="${STATE_MACHINE_INPUT_URL:-http://localhost:8000/inputs}"
CONTROL_COMM_CONTROL_URL="${CONTROL_COMM_CONTROL_URL:-http://localhost:5001/control}"
PERCEPTION_VIEW_MODE="${PERCEPTION_VIEW_MODE:-auto}"
PERCEPTION_LOG_DIR="${PERCEPTION_LOG_DIR:-$(pwd)/logs}"

PERCEPTION_ARGS_PROD=(
  -m src.main
  --mode production
  --config configs/docker-rpicam.yaml
  --source rpicam
  --fps 60
  --comms http
)
PERCEPTION_ARGS_VIEW=(
  -m src.main
  --mode test
  --config configs/docker-rpicam.yaml
  --source rpicam
  --fps 60
  --comms http
)
PERCEPTION_PID=""
PERCEPTION_LOG_FILE=""

show_log_view() {
  local mode="$1"
  local lines="${2:-60}"
  if [ -z "${PERCEPTION_LOG_FILE}" ] || [ ! -f "${PERCEPTION_LOG_FILE}" ]; then
    echo "[run_perception_rpicam] no active log file yet (press g to start perception)"
    return
  fi

  echo "[run_perception_rpicam] log view: ${mode} (${PERCEPTION_LOG_FILE})"
  case "$mode" in
    all)
      tail -n "$lines" "$PERCEPTION_LOG_FILE"
      ;;
    packets)
      if command -v rg >/dev/null 2>&1; then
        rg -i "^\[perception\] send:|vector_send_ok|robot_command" "$PERCEPTION_LOG_FILE" | tail -n "$lines" || true
      else
        tail -n 400 "$PERCEPTION_LOG_FILE"
      fi
      ;;
    errors)
      if command -v rg >/dev/null 2>&1; then
        rg -i "error|failed|exception|traceback|warning|warn" "$PERCEPTION_LOG_FILE" | tail -n "$lines" || true
      else
        tail -n 400 "$PERCEPTION_LOG_FILE"
      fi
      ;;
    *)
      tail -n "$lines" "$PERCEPTION_LOG_FILE"
      ;;
  esac
}

send_zero_vector() {
  local sm_payload control_payload
  sm_payload='{"black_line":{"detected":false,"vector":{"x":0.0,"y":0.0}},"red_line":{"detected":false,"vector":{"x":0.0,"y":0.0}},"target":{"detected":false,"vector":{"x":0.0,"y":0.0}}}'
  control_payload='{"x":0.0,"y":0.0,"speed":0.0}'
  if command -v curl >/dev/null 2>&1; then
    # Retry a few times to survive transient service startup or brief network hiccups.
    for _ in 1 2 3; do
      curl -sS -X POST "$STATE_MACHINE_INPUT_URL" \
        -H "Content-Type: application/json" \
        -d "$sm_payload" >/dev/null || true
      curl -sS -X POST "$CONTROL_COMM_CONTROL_URL" \
        -H "Content-Type: application/json" \
        -d "$control_payload" >/dev/null || true
      sleep 0.1
    done
  else
    echo "[run_perception_rpicam] warning: curl not found; could not send stop commands"
  fi
}

start_perception() {
  local -a args
  local ts
  local use_view_mode
  if [ -n "${PERCEPTION_PID}" ] && kill -0 "${PERCEPTION_PID}" 2>/dev/null; then
    echo "[run_perception_rpicam] perception already running (pid ${PERCEPTION_PID})"
    return
  fi
  mkdir -p "$PERCEPTION_LOG_DIR"
  ts="$(date +%Y%m%d-%H%M%S)"
  PERCEPTION_LOG_FILE="${PERCEPTION_LOG_DIR}/perception-${ts}.log"
  case "$PERCEPTION_VIEW_MODE" in
    1|true|TRUE|yes|YES|on|ON)
      use_view_mode="1"
      ;;
    0|false|FALSE|no|NO|off|OFF)
      use_view_mode="0"
      ;;
    auto|AUTO|"")
      if [ -n "${DISPLAY:-}" ] || [ -n "${WAYLAND_DISPLAY:-}" ]; then
        use_view_mode="1"
      else
        use_view_mode="0"
      fi
      ;;
    *)
      echo "[run_perception_rpicam] warning: unknown PERCEPTION_VIEW_MODE='${PERCEPTION_VIEW_MODE}', using auto"
      if [ -n "${DISPLAY:-}" ] || [ -n "${WAYLAND_DISPLAY:-}" ]; then
        use_view_mode="1"
      else
        use_view_mode="0"
      fi
      ;;
  esac
  if [ "$use_view_mode" = "1" ]; then
    args=("${PERCEPTION_ARGS_VIEW[@]}")
    echo "[run_perception_rpicam] starting in GUI test mode"
  else
    args=("${PERCEPTION_ARGS_PROD[@]}")
    echo "[run_perception_rpicam] starting in headless production mode"
  fi
  python3 "${args[@]}" >>"$PERCEPTION_LOG_FILE" 2>&1 &
  PERCEPTION_PID=$!
  echo "[run_perception_rpicam] started perception (pid ${PERCEPTION_PID})"
  echo "[run_perception_rpicam] log file: ${PERCEPTION_LOG_FILE}"
}

stop_perception() {
  if [ -z "${PERCEPTION_PID}" ] || ! kill -0 "${PERCEPTION_PID}" 2>/dev/null; then
    PERCEPTION_PID=""
    return
  fi
  kill -TERM "${PERCEPTION_PID}" 2>/dev/null || true
  wait "${PERCEPTION_PID}" 2>/dev/null || true
  echo "[run_perception_rpicam] stopped perception"
  PERCEPTION_PID=""
}

cleanup() {
  stop_perception
  send_zero_vector
}
trap cleanup EXIT INT TERM

# Keep old behavior for non-interactive callers (e.g., systemd button controller).
if ! [ -t 0 ]; then
  exec python3 "${PERCEPTION_ARGS_PROD[@]}"
fi

echo "[run_perception_rpicam] Interactive controls:"
echo "  g: start/resume sending perception vectors"
echo "  s: stop sending + send explicit stop commands"
echo "  a: show recent all logs"
echo "  p: show recent packet/control logs"
echo "  e: show recent error/warning logs"
echo "  q: quit script (also stops perception)"
echo "[run_perception_rpicam] view mode: ${PERCEPTION_VIEW_MODE} (1=test GUI, 0=production headless, auto=detect display)"
echo "[run_perception_rpicam] logs dir: ${PERCEPTION_LOG_DIR}"

while true; do
  read -r -s -n 1 key
  case "$key" in
    g|G)
      start_perception
      ;;
    s|S)
      stop_perception
      send_zero_vector
      echo "[run_perception_rpicam] stop command sent to state-machine and control-communication"
      ;;
    a|A)
      show_log_view all 80
      ;;
    p|P)
      show_log_view packets 80
      ;;
    e|E)
      show_log_view errors 80
      ;;
    q|Q)
      stop_perception
      echo "[run_perception_rpicam] exiting"
      break
      ;;
    *)
      ;;
  esac
done
