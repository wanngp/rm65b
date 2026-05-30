#!/usr/bin/env bash
set -euo pipefail

OUT_DIR="${1:?output directory required}"
WORKSPACE="${2:-/tmp/rm65b_dual_arm_ws_ascii_verify2}"
DOMAIN_ID="${3:-130}"
DAY_ID="${4:-day05}"

mkdir -p "$OUT_DIR/logs"

export ROS_DOMAIN_ID="$DOMAIN_ID"

set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u

PIDS=()
cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
  sleep 2
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
    wait "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT

ros2 launch rm65b_dual_arm_moveit_config move_group.launch.py allow_trajectory_execution:=false \
  > "$OUT_DIR/logs/dual_move_group.log" 2>&1 &
PIDS+=("$!")

for _ in $(seq 1 60); do
  if ros2 action list 2>/dev/null | grep -qx "/move_action"; then
    break
  fi
  sleep 1
done
ros2 action list > "$OUT_DIR/logs/action_list.txt" 2>&1 || true

ros2 run rm65b_dual_arm_planning dual_moveit_plan_client \
  --day-id "$DAY_ID" \
  --output-dir "$OUT_DIR/logs" \
  > "$OUT_DIR/logs/dual_moveit_plan_client.log" 2>&1

cat "$OUT_DIR/logs/dual_moveit_summary.json"
