#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/.." && pwd)"

DAY_ID="${1:-day01}"
DOMAIN_ID="${2:-211}"
OUT_ROOT="${3:-$HOME/rm65b_visual_$(date +%Y%m%d_%H%M%S)}"
OUT_DIR="$OUT_ROOT/$DAY_ID"

mkdir -p "$OUT_DIR/logs"

source_setup_file() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
}

[[ -f /opt/ros/humble/setup.bash ]] || {
  echo "Missing /opt/ros/humble/setup.bash. Run scripts/bootstrap_vmware_ubuntu.sh --all first." >&2
  exit 1
}
[[ -f "$WORKSPACE/install/setup.bash" ]] || {
  echo "Missing $WORKSPACE/install/setup.bash. Run scripts/bootstrap_vmware_ubuntu.sh --all first." >&2
  exit 1
}
command -v gz >/dev/null 2>&1 || {
  echo "Missing gz command. Run scripts/bootstrap_vmware_ubuntu.sh --all first." >&2
  exit 1
}

source_setup_file /opt/ros/humble/setup.bash
source_setup_file "$WORKSPACE/install/setup.bash"

export ROS_DOMAIN_ID="$DOMAIN_ID"
export GZ_PARTITION="${GZ_PARTITION:-rm65b_visual_${DOMAIN_ID}_$$}"
export RECORD_RVIZ="${RECORD_RVIZ:-0}"
export CAPTURE_TIMEOUT="${CAPTURE_TIMEOUT:-360}"

backend_log="$OUT_DIR/logs/run_day_visual_backend.log"
gui_log="$OUT_DIR/logs/run_day_visual_gz_gui.log"

echo "workspace=$WORKSPACE"
echo "day=$DAY_ID"
echo "ros_domain_id=$ROS_DOMAIN_ID"
echo "gz_partition=$GZ_PARTITION"
echo "output=$OUT_DIR"
echo "backend_log=$backend_log"
echo "gui_log=$gui_log"

bash "$SCRIPT_DIR/play_harmonic_planned_record.sh" "$OUT_DIR" "$WORKSPACE" "$DOMAIN_ID" "$DAY_ID" \
  > "$backend_log" 2>&1 &
backend_pid="$!"

cleanup() {
  if kill -0 "$backend_pid" >/dev/null 2>&1; then
    kill "$backend_pid" >/dev/null 2>&1 || true
  fi
}
trap cleanup INT TERM

sleep "${GUI_START_DELAY:-12}"

if [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]]; then
  echo "Opening Gazebo GUI. Close the GUI window after inspection."
  gz sim -g > "$gui_log" 2>&1 &
  gui_pid="$!"
else
  gui_pid=""
  echo "DISPLAY/WAYLAND_DISPLAY is not set; Gazebo GUI was not opened." | tee -a "$gui_log"
fi

backend_status=0
wait "$backend_pid" || backend_status="$?"

if [[ -n "${gui_pid:-}" ]] && kill -0 "$gui_pid" >/dev/null 2>&1; then
  kill "$gui_pid" >/dev/null 2>&1 || true
fi

if [[ "$backend_status" -ne 0 ]]; then
  echo "Day visual backend failed with exit code $backend_status. See $backend_log" >&2
  exit "$backend_status"
fi

echo "Day visual run complete."
echo "Screenshots/videos/logs are under: $OUT_DIR"
