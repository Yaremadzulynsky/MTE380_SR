#!/usr/bin/env bash
# Start camera-related processes with logs under perception/logs/.
#
# Modes (env START_MODE):
#   hough       — Hough overlay + MJPEG on port 8090 (Ops dashboard / debugging). Default.
#   perception  — Production perception (rpicam) via run_perception_rpicam.sh, non-interactive.
#
# Examples:
#   ./start_camera_with_logs.sh
#   START_MODE=perception ./start_camera_with_logs.sh
#   CAMERA_SOURCE=webcam MJPEG_PORT=8090 ./start_camera_with_logs.sh
#   BACKGROUND=1 ./start_camera_with_logs.sh    # detach; tail log file printed on stdout
#
# Env (hough mode):
#   CAMERA_SOURCE   webcam | video:/path | http://...  (default: webcam)
#   HOUGH_CONFIG    default configs/docker-rpicam.yaml
#   MJPEG_PORT      default 8090
#   MJPEG_HOST      default 0.0.0.0
#   LOG_DIR         default perception/logs
#
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
PERCEPTION_DIR="${PERCEPTION_DIR:-$ROOT/perception}"
LOG_DIR="${LOG_DIR:-$PERCEPTION_DIR/logs}"
mkdir -p "$LOG_DIR"

START_MODE="${START_MODE:-hough}"
TS="$(date +%Y%m%d-%H%M%S)"
LOG_FILE="${LOG_FILE:-$LOG_DIR/camera-${START_MODE}-${TS}.log}"

usage() {
  sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'
  exit 0
}

[[ "${1:-}" == "-h" || "${1:-}" == "--help" ]] && usage

select_python() {
  if [[ -x "$PERCEPTION_DIR/.venv/bin/python" ]]; then
    echo "$PERCEPTION_DIR/.venv/bin/python"
  elif [[ -x "$PERCEPTION_DIR/venv/bin/python" ]]; then
    echo "$PERCEPTION_DIR/venv/bin/python"
  elif command -v python3 >/dev/null 2>&1; then
    command -v python3
  else
    echo "[start_camera_with_logs] error: no python (.venv/bin/python or python3)" >&2
    exit 1
  fi
}

log_banner() {
  {
    echo "=============================================="
    echo "ts=$(date -Iseconds) event=camera_start mode=${START_MODE}"
    echo "log_file=${LOG_FILE}"
    echo "=============================================="
  } | tee -a "$LOG_FILE"
}

probe_mjpeg() {
  local port="${1:-8090}"
  local code
  if command -v curl >/dev/null 2>&1; then
    code="$(curl -sS -o /dev/null -w '%{http_code}' --max-time 3 "http://127.0.0.1:${port}/stream.mjpg" 2>/dev/null || echo "000")"
    echo "[start_camera_with_logs] probe http://127.0.0.1:${port}/stream.mjpg -> HTTP ${code}"
  else
    echo "[start_camera_with_logs] install curl to auto-probe MJPEG (optional)"
  fi
}

run_hough() {
  local py
  py="$(select_python)"
  cd "$PERCEPTION_DIR"
  export PYTHONPATH="${PYTHONPATH:+${PYTHONPATH}:}$(pwd)"

  local config="${HOUGH_CONFIG:-configs/docker-rpicam.yaml}"
  local source="${CAMERA_SOURCE:-webcam}"
  local port="${MJPEG_PORT:-8090}"
  local host="${MJPEG_HOST:-0.0.0.0}"

  log_banner
  {
    echo "[start_camera_with_logs] python=${py}"
    echo "[start_camera_with_logs] Hough: config=${config} source=${source}"
    echo "[start_camera_with_logs] MJPEG: http://${host}:${port}/stream.mjpg (use http://127.0.0.1:${port}/stream.mjpg on the Pi)"
    echo "[start_camera_with_logs] Ops proxy uses OPS_HOUGH_STREAM_URL on control-screen (default http://localhost:${port}/stream.mjpg)"
  } | tee -a "$LOG_FILE"

  if [[ "${BACKGROUND:-0}" == "1" ]]; then
    echo "[start_camera_with_logs] starting in background → log: $LOG_FILE"
    nohup env PYTHONPATH="$PYTHONPATH" "$py" -u -m src.tools.test_hough_redline \
      --config "$config" \
      --source "$source" \
      --mjpeg-port "$port" \
      --mjpeg-host "$host" \
      --no-display >>"$LOG_FILE" 2>&1 &
    echo "$!" >"$LOG_DIR/camera-hough.pid"
    echo "[start_camera_with_logs] pid=$(cat "$LOG_DIR/camera-hough.pid")"
    sleep 2
    probe_mjpeg "$port" | tee -a "$LOG_FILE"
    echo "[start_camera_with_logs] tail -f \"$LOG_FILE\""
    exit 0
  fi

  echo "[start_camera_with_logs] streaming logs to stdout and $LOG_FILE (Ctrl+C to stop)"
  "$py" -u -m src.tools.test_hough_redline \
    --config "$config" \
    --source "$source" \
    --mjpeg-port "$port" \
    --mjpeg-host "$host" \
    --no-display 2>&1 | tee -a "$LOG_FILE"
}

run_perception() {
  log_banner
  local script="$ROOT/run_perception_rpicam.sh"
  if [[ ! -f "$script" ]]; then
    echo "[start_camera_with_logs] missing $script" | tee -a "$LOG_FILE"
    exit 1
  fi
  echo "[start_camera_with_logs] running non-interactive perception (stdin not a tty → production mode)" | tee -a "$LOG_FILE"
  echo "[start_camera_with_logs] see also PERCEPTION_LOG_DIR under perception/logs from the runner" | tee -a "$LOG_FILE"

  if [[ "${BACKGROUND:-0}" == "1" ]]; then
    echo "[start_camera_with_logs] background perception → log: $LOG_FILE"
    nohup bash "$script" </dev/null >>"$LOG_FILE" 2>&1 &
    echo "$!" >"$LOG_DIR/camera-perception.pid"
    echo "[start_camera_with_logs] pid=$(cat "$LOG_DIR/camera-perception.pid")"
    echo "[start_camera_with_logs] tail -f \"$LOG_FILE\""
    exit 0
  fi

  bash "$script" </dev/null 2>&1 | tee -a "$LOG_FILE"
}

case "$START_MODE" in
  hough)
    run_hough
    ;;
  perception)
    run_perception
    ;;
  *)
    echo "Unknown START_MODE=$START_MODE (use hough or perception)" >&2
    exit 1
    ;;
esac
