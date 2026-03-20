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

select_python_bin() {
  if [ -x ".venv/bin/python" ]; then
    PYTHON_BIN=".venv/bin/python"
    return
  fi
  if [ -x "venv/bin/python" ]; then
    PYTHON_BIN="venv/bin/python"
    return
  fi
  if command -v python3 >/dev/null 2>&1; then
    PYTHON_BIN="python3"
    return
  fi
  echo "[run_perception_rpicam] error: python interpreter not found (.venv/bin/python or python3)" >&2
  exit 1
}

ensure_python_deps() {
  if "$PYTHON_BIN" -c "import yaml, cv2, numpy, serial" >/dev/null 2>&1; then
    return
  fi

  echo "[run_perception_rpicam] missing python dependencies; bootstrapping local venv"
  if [ ! -d ".venv" ]; then
    if ! command -v python3 >/dev/null 2>&1; then
      echo "[run_perception_rpicam] error: cannot create .venv because python3 is not installed" >&2
      exit 1
    fi
    # Reuse system OpenCV/Numpy on Pi to avoid slow wheel builds.
    python3 -m venv --system-site-packages .venv
  fi

  # shellcheck source=/dev/null
  source .venv/bin/activate
  PYTHON_BIN=".venv/bin/python"
  "$PYTHON_BIN" -m pip install --upgrade pip >/dev/null
  "$PYTHON_BIN" -m pip install pyyaml pyserial pytest >/dev/null

  if ! "$PYTHON_BIN" -c "import yaml, cv2, numpy, serial" >/dev/null 2>&1; then
    echo "[run_perception_rpicam] error: dependencies still missing after install" >&2
    exit 1
  fi
}

activate_local_venv_if_present
select_python_bin
ensure_python_deps

export PYTHONPATH="${PYTHONPATH:-}:$(pwd)"

STATE_MACHINE_INPUT_URL="${STATE_MACHINE_INPUT_URL:-http://localhost:8000/inputs}"
CONTROL_COMM_CONTROL_URL="${CONTROL_COMM_CONTROL_URL:-http://localhost:5001/control}"
export STATE_MACHINE_INPUT_URL
export CONTROL_COMM_CONTROL_URL
PERCEPTION_VIEW_MODE="${PERCEPTION_VIEW_MODE:-auto}"
PERCEPTION_LOG_DIR="${PERCEPTION_LOG_DIR:-$(pwd)/logs}"
RPICAM_WIDTH="${RPICAM_WIDTH:-480}"
RPICAM_HEIGHT="${RPICAM_HEIGHT:-320}"
RPICAM_FPS="${RPICAM_FPS:-60}"
MEDIAMTX_RTSP_URL="${MEDIAMTX_RTSP_URL:-rtsp://127.0.0.1:8554/rpicam}"
STREAM_PUBLISH_MODE="${STREAM_PUBLISH_MODE:-rtsp}"
RTSP_READY_TIMEOUT_S="${RTSP_READY_TIMEOUT_S:-12}"
STREAMER_PID=""
STREAMER_LOG_FILE=""

PERCEPTION_ARGS_PROD=(
  -m src.main
  --mode production
  --config configs/docker-rpicam.yaml
  --source "${MEDIAMTX_RTSP_URL}"
  --fps "${RPICAM_FPS}"
  --comms http
)
PERCEPTION_ARGS_VIEW=(
  -m src.main
  --mode test
  --config configs/docker-rpicam.yaml
  --source "${MEDIAMTX_RTSP_URL}"
  --fps "${RPICAM_FPS}"
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

ensure_streaming_deps() {
  if ! command -v rpicam-vid >/dev/null 2>&1; then
    echo "[run_perception_rpicam] error: rpicam-vid not found; install rpicam-apps" >&2
    exit 1
  fi
  if ! command -v ffmpeg >/dev/null 2>&1; then
    echo "[run_perception_rpicam] error: ffmpeg not found; required for RTSP publishing" >&2
    exit 1
  fi
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

start_streamer() {
  local ts
  if [ "${STREAM_PUBLISH_MODE}" != "rtsp" ]; then
    return
  fi
  if [ -n "${STREAMER_PID}" ] && kill -0 "${STREAMER_PID}" 2>/dev/null; then
    return
  fi

  ensure_streaming_deps
  mkdir -p "$PERCEPTION_LOG_DIR"
  ts="$(date +%Y%m%d-%H%M%S)"
  STREAMER_LOG_FILE="${PERCEPTION_LOG_DIR}/streamer-${ts}.log"
  echo "[run_perception_rpicam] starting RTSP publisher -> ${MEDIAMTX_RTSP_URL}"
  echo "[run_perception_rpicam] streamer log: ${STREAMER_LOG_FILE}"

  (
    set -euo pipefail
    rpicam-vid \
      -t 0 \
      -n \
      --inline \
      --codec h264 \
      --width "${RPICAM_WIDTH}" \
      --height "${RPICAM_HEIGHT}" \
      --framerate "${RPICAM_FPS}" \
      -o - \
    | ffmpeg \
      -loglevel warning \
      -fflags nobuffer \
      -flags low_delay \
      -f h264 \
      -i - \
      -an \
      -c:v copy \
      -f rtsp \
      -rtsp_transport tcp \
      "${MEDIAMTX_RTSP_URL}"
  ) >>"${STREAMER_LOG_FILE}" 2>&1 &

  STREAMER_PID=$!
  sleep 0.4
  if ! kill -0 "${STREAMER_PID}" 2>/dev/null; then
    echo "[run_perception_rpicam] error: streamer failed to start; check ${STREAMER_LOG_FILE}" >&2
    exit 1
  fi
}

stop_streamer() {
  if [ -z "${STREAMER_PID}" ] || ! kill -0 "${STREAMER_PID}" 2>/dev/null; then
    STREAMER_PID=""
    return
  fi
  kill -TERM "${STREAMER_PID}" 2>/dev/null || true
  wait "${STREAMER_PID}" 2>/dev/null || true
  echo "[run_perception_rpicam] stopped RTSP publisher"
  STREAMER_PID=""
}

wait_for_rtsp_ready() {
  local start now elapsed
  if [ "${STREAM_PUBLISH_MODE}" != "rtsp" ]; then
    return
  fi
  if ! command -v ffprobe >/dev/null 2>&1; then
    echo "[run_perception_rpicam] warning: ffprobe not found; skipping RTSP readiness check"
    return
  fi
  start="$(date +%s)"
  echo "[run_perception_rpicam] waiting for RTSP stream to become ready (${MEDIAMTX_RTSP_URL})"
  while true; do
    if ! kill -0 "${STREAMER_PID}" 2>/dev/null; then
      echo "[run_perception_rpicam] error: streamer exited before RTSP became ready; check ${STREAMER_LOG_FILE}" >&2
      exit 1
    fi
    if ffprobe -v error -rtsp_transport tcp -select_streams v:0 -show_entries stream=codec_name -of csv=p=0 "${MEDIAMTX_RTSP_URL}" >/dev/null 2>&1; then
      echo "[run_perception_rpicam] RTSP stream is ready"
      return
    fi
    now="$(date +%s)"
    elapsed=$((now - start))
    if [ "${elapsed}" -ge "${RTSP_READY_TIMEOUT_S}" ]; then
      echo "[run_perception_rpicam] error: RTSP stream not ready after ${RTSP_READY_TIMEOUT_S}s; check ${STREAMER_LOG_FILE}" >&2
      exit 1
    fi
    sleep 0.3
  done
}

monitor_perception_exit() {
  if [ -z "${PERCEPTION_PID}" ]; then
    return
  fi
  if kill -0 "${PERCEPTION_PID}" 2>/dev/null; then
    return
  fi
  wait "${PERCEPTION_PID}" 2>/dev/null || true
  echo "[run_perception_rpicam] perception exited unexpectedly; sending explicit stop vectors"
  PERCEPTION_PID=""
  send_zero_vector
}

start_perception() {
  local -a args
  local ts
  local use_view_mode
  if [ -n "${PERCEPTION_PID}" ] && kill -0 "${PERCEPTION_PID}" 2>/dev/null; then
    echo "[run_perception_rpicam] perception already running (pid ${PERCEPTION_PID})"
    return
  fi
  start_streamer
  wait_for_rtsp_ready
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
  "$PYTHON_BIN" "${args[@]}" >>"$PERCEPTION_LOG_FILE" 2>&1 &
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
  stop_streamer
  send_zero_vector
}
trap cleanup EXIT INT TERM

# Keep old behavior for non-interactive callers (e.g., systemd button controller).
if ! [ -t 0 ]; then
  start_streamer
  wait_for_rtsp_ready
  trap 'stop_streamer; send_zero_vector' EXIT INT TERM
  exec "$PYTHON_BIN" "${PERCEPTION_ARGS_PROD[@]}"
fi

echo "[run_perception_rpicam] Interactive controls:"
echo "  g: start/resume sending perception vectors"
echo "  s: stop sending + send explicit stop commands"
echo "  a: show recent all logs"
echo "  p: show recent packet/control logs"
echo "  e: show recent error/warning logs"
echo "  q: quit script (also stops perception)"
echo "[run_perception_rpicam] stream mode: ${STREAM_PUBLISH_MODE} (rtsp => camera owned by RTSP publisher)"
echo "[run_perception_rpicam] rtsp url: ${MEDIAMTX_RTSP_URL}"
echo "[run_perception_rpicam] view mode: ${PERCEPTION_VIEW_MODE} (1=test GUI, 0=production headless, auto=detect display)"
echo "[run_perception_rpicam] logs dir: ${PERCEPTION_LOG_DIR}"

while true; do
  monitor_perception_exit
  key=""
  if ! read -r -s -n 1 -t 0.2 key; then
    continue
  fi
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
