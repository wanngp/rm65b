#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/.." && pwd)"

DAY_ID="${1:-day01}"
DOMAIN_ID="${2:-211}"
OUT_ROOT="${3:-$HOME/rm65b_mvp_visual_$(date +%Y%m%d_%H%M%S)}"
DURATION="${MVP_DURATION:-45}"
RVIZ_START_DELAY="${RVIZ_START_DELAY:-3}"

source_setup_file() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
}

terminate_tree() {
  local root="$1"
  local child
  for child in $(pgrep -P "$root" 2>/dev/null || true); do
    terminate_tree "$child"
  done
  kill "$root" >/dev/null 2>&1 || true
}

PIDS=()
cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      terminate_tree "$pid"
    fi
  done
}
trap cleanup EXIT INT TERM

case "${DAY_ID,,}" in
  d1|day1) DAY_ID="day01" ;;
  d2|day2) DAY_ID="day02" ;;
  d3|day3) DAY_ID="day03" ;;
  d4|day4) DAY_ID="day04" ;;
  d5|day5) DAY_ID="day05" ;;
esac

OUT_DIR="$OUT_ROOT/$DAY_ID"
mkdir -p "$OUT_DIR/logs"

[[ -f /opt/ros/humble/setup.bash ]] || {
  echo "Missing /opt/ros/humble/setup.bash. Run scripts/bootstrap_vmware_ubuntu.sh --all first." >&2
  exit 1
}
[[ -f "$WORKSPACE/install/setup.bash" ]] || {
  echo "Missing $WORKSPACE/install/setup.bash. Build the workspace first." >&2
  exit 1
}

source_setup_file /opt/ros/humble/setup.bash
source_setup_file "$WORKSPACE/install/setup.bash"

export ROS_DOMAIN_ID="$DOMAIN_ID"

echo "workspace=$WORKSPACE"
echo "day=$DAY_ID"
echo "ros_domain_id=$ROS_DOMAIN_ID"
echo "output=$OUT_DIR"
echo "duration=$DURATION"
echo "visual_stack=MoveIt/RViz only; no Gazebo world, no generated scene, no per-arm spawn" \
  | tee "$OUT_DIR/logs/mvp_visual_mode.txt"

ros2 launch rm65b_dual_arm_moveit_config move_group.launch.py \
  use_sim_time:=false \
  allow_trajectory_execution:=false \
  > "$OUT_DIR/logs/move_group.log" 2>&1 &
PIDS+=("$!")

if [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]]; then
  ros2 launch rm65b_dual_arm_moveit_config moveit_rviz.launch.py \
    use_sim_time:=false \
    > "$OUT_DIR/logs/rviz.log" 2>&1 &
  PIDS+=("$!")
else
  echo "DISPLAY/WAYLAND_DISPLAY is not set; RViz was not opened." | tee "$OUT_DIR/logs/rviz.log"
fi

sleep "$RVIZ_START_DELAY"

python3 "$SCRIPT_DIR/moveit_mvp_visual_replay.py" \
  --day-id "$DAY_ID" \
  --duration "$DURATION" \
  --output-dir "$OUT_DIR/logs" \
  > "$OUT_DIR/logs/moveit_mvp_visual_replay.log" 2>&1

echo "MVP MoveIt/RViz visual complete."
echo "output=$OUT_DIR"
echo "summary=$OUT_DIR/logs/mvp_moveit_summary.json"
