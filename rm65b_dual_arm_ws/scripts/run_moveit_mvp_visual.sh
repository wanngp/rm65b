#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/.." && pwd)"

DAY_ID="${1:-day01}"
DOMAIN_ID="${2:-211}"
OUT_ROOT="${3:-$HOME/rm65b_mvp_visual_$(date +%Y%m%d_%H%M%S)}"
DURATION="${MVP_DURATION:-45}"
SPAWN_DELAY="${SPAWN_DELAY:-5}"
GUI_READY_CHECK_DELAY="${GUI_READY_CHECK_DELAY:-2}"

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

[[ -f /opt/ros/humble/setup.bash ]] || {
  echo "Missing /opt/ros/humble/setup.bash. Run scripts/bootstrap_vmware_ubuntu.sh --all first." >&2
  exit 1
}
[[ -f "$WORKSPACE/install/setup.bash" ]] || {
  echo "Missing $WORKSPACE/install/setup.bash. Build the workspace first." >&2
  exit 1
}
command -v gz >/dev/null 2>&1 || {
  echo "Missing gz command." >&2
  exit 1
}

source_setup_file /opt/ros/humble/setup.bash
source_setup_file "$WORKSPACE/install/setup.bash"

case "${DAY_ID,,}" in
  d1|day1) DAY_ID="day01" ;;
  d2|day2) DAY_ID="day02" ;;
  d3|day3) DAY_ID="day03" ;;
  d4|day4) DAY_ID="day04" ;;
  d5|day5) DAY_ID="day05" ;;
esac

OUT_DIR="$OUT_ROOT/$DAY_ID"
mkdir -p "$OUT_DIR/logs"

export ROS_DOMAIN_ID="$DOMAIN_ID"
export GZ_PARTITION="${GZ_PARTITION:-rm65b_mvp_${DAY_ID}_${DOMAIN_ID}_$$}"
export GZ_SIM_RESOURCE_PATH="$WORKSPACE/install/rm_description/share/rm_description:${GZ_SIM_RESOURCE_PATH:-}"

BASE_WORLD="${BASE_WORLD:-$WORKSPACE/harmonic/rm65b_empty_world.sdf}"
WORLD_RUNTIME="$OUT_DIR/logs/rm65b_mvp_world.sdf"
URDF="$WORKSPACE/install/rm_description/share/rm_description/urdf/rm_65.urdf"
URDF_HARMONIC="$OUT_DIR/logs/rm_65_harmonic_file_meshes.urdf"
SDF_DYNAMIC="$OUT_DIR/logs/rm_65_harmonic_dynamic.sdf"
SDF_LEFT="$OUT_DIR/logs/rm_65_mvp_left_controlled.sdf"
SDF_RIGHT="$OUT_DIR/logs/rm_65_mvp_right_controlled.sdf"
MESH_ROOT="$WORKSPACE/install/rm_description/share/rm_description/meshes"
LEFT_INITIAL_POSITIONS="0.000000 -0.350000 0.650000 0.000000 0.900000 0.000000"
RIGHT_INITIAL_POSITIONS="0.000000 -0.350000 0.650000 0.000000 0.900000 0.000000"

echo "workspace=$WORKSPACE"
echo "day=$DAY_ID"
echo "ros_domain_id=$ROS_DOMAIN_ID"
echo "gz_partition=$GZ_PARTITION"
echo "output=$OUT_DIR"
echo "duration=$DURATION"

python3 "$SCRIPT_DIR/generate_day_world.py" \
  --base-world "$BASE_WORLD" \
  --day-id "$DAY_ID" \
  --output "$WORLD_RUNTIME" \
  --notes "$OUT_DIR/logs/day_world_notes.txt"

sed "s#package://rm_description/meshes#file://$MESH_ROOT#g" "$URDF" > "$URDF_HARMONIC"
gz sdf -p "$URDF_HARMONIC" > "$SDF_DYNAMIC"

FORCE_ARGS=()
if [[ "$DAY_ID" == "day02" ]]; then
  FORCE_ARGS=(--force-probe)
fi

python3 "$SCRIPT_DIR/prepare_harmonic_joint_demo_sdf.py" \
  --source "$SDF_DYNAMIC" \
  --output "$SDF_LEFT" \
  --initial-positions "$LEFT_INITIAL_POSITIONS" \
  "${FORCE_ARGS[@]}"

python3 "$SCRIPT_DIR/prepare_harmonic_joint_demo_sdf.py" \
  --source "$SDF_DYNAMIC" \
  --output "$SDF_RIGHT" \
  --initial-positions "$RIGHT_INITIAL_POSITIONS" \
  "${FORCE_ARGS[@]}"

ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
  /model/left_rm65b/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory \
  /model/right_rm65b/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory \
  /rm65b_gripper/upper_finger_cmd@std_msgs/msg/Float64]gz.msgs.Double \
  /rm65b_gripper/lower_finger_cmd@std_msgs/msg/Float64]gz.msgs.Double \
  > "$OUT_DIR/logs/ros_gz_bridge.log" 2>&1 &
PIDS+=("$!")

ros2 launch rm65b_dual_arm_moveit_config move_group.launch.py allow_trajectory_execution:=false \
  > "$OUT_DIR/logs/move_group.log" 2>&1 &
PIDS+=("$!")

gz sim -v 3 -r -s "$WORLD_RUNTIME" > "$OUT_DIR/logs/gz_server.log" 2>&1 &
PIDS+=("$!")

sleep "$SPAWN_DELAY"

spawn_model() {
  local side="$1"
  local file="$2"
  local name="$3"
  local x="$4"
  local yaw="$5"
  local log="$OUT_DIR/logs/spawn_${side}_rm65b.log"
  local attempt
  for attempt in 1 2 3 4; do
    if ros2 run ros_gz_sim create -world rm65b_world -file "$file" -name "$name" -x "$x" -y 0 -z 0.02 -Y "$yaw" \
      > "$log" 2>&1; then
      return 0
    fi
    echo "spawn retry $attempt for $name" >> "$log"
    sleep 2
  done
  echo "Failed to spawn $name. See $log" >&2
  tail -n 80 "$log" >&2 || true
  return 1
}

spawn_model left "$SDF_LEFT" left_rm65b -0.45 1.5708
spawn_model right "$SDF_RIGHT" right_rm65b 0.45 -1.5708

open_gazebo_gui() {
  local gui_log="$OUT_DIR/logs/gz_gui.log"
  if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    echo "DISPLAY/WAYLAND_DISPLAY is not set; Gazebo GUI was not opened." | tee "$gui_log"
    return 0
  fi
  gz sim -g > "$gui_log" 2>&1 &
  local pid="$!"
  sleep "$GUI_READY_CHECK_DELAY"
  if kill -0 "$pid" >/dev/null 2>&1; then
    PIDS+=("$pid")
    echo "Gazebo GUI opened with: gz sim -g"
    return 0
  fi
  if grep -qi "invalid arguments" "$gui_log" 2>/dev/null; then
    echo "gz sim -g failed; trying gz sim --gui-only" >> "$gui_log"
    gz sim --gui-only >> "$gui_log" 2>&1 &
    pid="$!"
    sleep "$GUI_READY_CHECK_DELAY"
    if kill -0 "$pid" >/dev/null 2>&1; then
      PIDS+=("$pid")
      echo "Gazebo GUI opened with: gz sim --gui-only"
      return 0
    fi
  fi
  echo "Gazebo GUI failed. See $gui_log" >&2
  tail -n 80 "$gui_log" >&2 || true
  return 3
}

open_gazebo_gui

sleep "${MOVEIT_START_DELAY:-2}"
python3 "$SCRIPT_DIR/moveit_mvp_visual_replay.py" \
  --day-id "$DAY_ID" \
  --duration "$DURATION" \
  --output-dir "$OUT_DIR/logs" \
  > "$OUT_DIR/logs/moveit_mvp_visual_replay.log" 2>&1

echo "MVP MoveIt visual complete."
echo "output=$OUT_DIR"
echo "summary=$OUT_DIR/logs/mvp_moveit_summary.json"
