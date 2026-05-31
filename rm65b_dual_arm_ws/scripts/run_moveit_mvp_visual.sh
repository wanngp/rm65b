#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/.." && pwd)"

DAY_ID="${1:-day01}"
DOMAIN_ID="${2:-211}"
OUT_ROOT="${3:-$HOME/rm65b_mvp_visual_$(date +%Y%m%d_%H%M%S)}"
DURATION="${MVP_DURATION:-45}"
OPEN_RVIZ="${OPEN_RVIZ:-0}"
START_DELAY="${MVP_START_DELAY:-5}"

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
command -v gz >/dev/null 2>&1 || {
  echo "Missing gz command." >&2
  exit 1
}

source_setup_file /opt/ros/humble/setup.bash
source_setup_file "$WORKSPACE/install/setup.bash"

export ROS_DOMAIN_ID="$DOMAIN_ID"
export GZ_PARTITION="${GZ_PARTITION:-rm65b_mvp_${DAY_ID}_${DOMAIN_ID}_$$}"

DUAL_URDF="$WORKSPACE/install/rm65b_dual_arm_moveit_config/share/rm65b_dual_arm_moveit_config/config/rm65b_dual_arm.urdf"
if [[ ! -f "$DUAL_URDF" ]]; then
  DUAL_URDF="$WORKSPACE/src/rm65b_dual_arm_moveit_config/config/rm65b_dual_arm.urdf"
fi
MESH_ROOT="$WORKSPACE/install/rm_description/share/rm_description/meshes"
if [[ ! -d "$MESH_ROOT" ]]; then
  MESH_ROOT="$WORKSPACE/src/ros2_rm_robot/rm_description/meshes"
fi

DUAL_URDF_RUNTIME="$OUT_DIR/logs/rm65b_dual_arm_mvp_file_meshes.urdf"
DUAL_MODEL_SDF="$OUT_DIR/logs/rm65b_dual_arm_mvp_model.sdf"
WORLD_RUNTIME="$OUT_DIR/logs/rm65b_dual_arm_mvp_world.sdf"

sed "s#package://rm_description/meshes#file://$MESH_ROOT#g" "$DUAL_URDF" > "$DUAL_URDF_RUNTIME"
gz sdf -p "$DUAL_URDF_RUNTIME" > "$DUAL_MODEL_SDF"
python3 "$SCRIPT_DIR/generate_moveit_mvp_gazebo_world.py" \
  --model-sdf "$DUAL_MODEL_SDF" \
  --day-id "$DAY_ID" \
  --output "$WORLD_RUNTIME"

echo "workspace=$WORKSPACE"
echo "day=$DAY_ID"
echo "ros_domain_id=$ROS_DOMAIN_ID"
echo "gz_partition=$GZ_PARTITION"
echo "output=$OUT_DIR"
echo "duration=$DURATION"
echo "visual_stack=Gazebo single dual_rm65b_mvp model; no legacy world generator; no per-arm spawn; D4/D5 use gripper-attached yarn endpoint stubs and no dynamic/static chain" \
  | tee "$OUT_DIR/logs/mvp_visual_mode.txt"

cat > "$OUT_DIR/logs/day_topic_contract.txt" <<EOF
Day visual topic contract
day=$DAY_ID
model=dual_rm65b_mvp
moveit_action=/move_action
robot_state=/joint_states
gazebo_trajectory=/model/dual_rm65b_mvp/joint_trajectory
left_plan=/dual_arm_planning/left_joint_trajectory
right_plan=/dual_arm_planning/right_joint_trajectory
phase=/dual_arm_planning/phase
status=/acceptance/day_status
gripper_left_upper=/model/dual_rm65b_mvp/left_gripper_upper_cmd
gripper_left_lower=/model/dual_rm65b_mvp/left_gripper_lower_cmd
gripper_right_upper=/model/dual_rm65b_mvp/right_gripper_upper_cmd
gripper_right_lower=/model/dual_rm65b_mvp/right_gripper_lower_cmd
d2_force_state=/force_control/state
d2_force_target=/force_control/target_wrench
d2_force_error=/force_control/wrench_error
d2_admittance=/force_control/admittance_offset
d3_camera_image=/right_camera/image_rect
d3_camera_info=/right_camera/camera_info
d3_debug_image=/vision/debug_image
d3_target_pose=/vision/target_pose
d3_status=/vision/status
d3_metrics=/vision/metrics
d3_twist_cmd=/visual_servo/twist_cmd
d3_aligned=/visual_servo/aligned
d3_adapter_state=/visual_servo/gazebo_adapter_state
d3_marker=DICT_4X4_50 id=7 tag_size_m=0.080
d4_weaving_events=/weaving/events
d4_tension_n=/weaving/tension_n
d4_tension_status=/weaving/tension_status
d4_tension_pid_state=/weaving/tension_pid_state
d4_compliance_offset=/weaving/compliance_offset_m
d4_yarn_state=/weaving/yarn_state
d4_primitive_lock=/weaving/primitive_lock
d5_integrated_loop=/weaving/events

Quick checks while the visual run is active:
ros2 action list | grep /move_action
ros2 topic list | grep -E 'dual_arm|joint_states|dual_rm65b_mvp|force_control|vision|visual_servo|right_camera|weaving|acceptance'
ros2 topic echo /acceptance/day_status --once
ros2 topic echo /dual_arm_planning/phase --once
ros2 topic echo /weaving/events --once
EOF
echo "topic_contract=$OUT_DIR/logs/day_topic_contract.txt"

ros2 run ros_gz_bridge parameter_bridge \
  /model/dual_rm65b_mvp/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory \
  /model/dual_rm65b_mvp/left_gripper_upper_cmd@std_msgs/msg/Float64]gz.msgs.Double \
  /model/dual_rm65b_mvp/left_gripper_lower_cmd@std_msgs/msg/Float64]gz.msgs.Double \
  /model/dual_rm65b_mvp/right_gripper_upper_cmd@std_msgs/msg/Float64]gz.msgs.Double \
  /model/dual_rm65b_mvp/right_gripper_lower_cmd@std_msgs/msg/Float64]gz.msgs.Double \
  > "$OUT_DIR/logs/ros_gz_bridge.log" 2>&1 &
PIDS+=("$!")

ros2 launch rm65b_dual_arm_moveit_config move_group.launch.py \
  use_sim_time:=false \
  allow_trajectory_execution:=false \
  > "$OUT_DIR/logs/move_group.log" 2>&1 &
PIDS+=("$!")

if [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]]; then
  gz sim -v 3 -r "$WORLD_RUNTIME" > "$OUT_DIR/logs/gz_sim.log" 2>&1 &
  PIDS+=("$!")
  GZ_PID="$!"
else
  gz sim -v 3 -r -s "$WORLD_RUNTIME" > "$OUT_DIR/logs/gz_sim.log" 2>&1 &
  PIDS+=("$!")
  GZ_PID="$!"
  echo "DISPLAY/WAYLAND_DISPLAY is not set; Gazebo runs server-only." | tee "$OUT_DIR/logs/gz_gui.log"
fi

sleep 2
if ! kill -0 "$GZ_PID" >/dev/null 2>&1; then
  echo "Gazebo exited immediately. See $OUT_DIR/logs/gz_sim.log" >&2
  tail -n 80 "$OUT_DIR/logs/gz_sim.log" >&2 || true
  exit 3
fi

if [[ "$OPEN_RVIZ" == "1" && ( -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ) ]]; then
  ros2 launch rm65b_dual_arm_moveit_config moveit_rviz.launch.py \
    use_sim_time:=false \
    > "$OUT_DIR/logs/rviz.log" 2>&1 &
  PIDS+=("$!")
fi

sleep "$START_DELAY"

python3 "$SCRIPT_DIR/moveit_mvp_visual_replay.py" \
  --day-id "$DAY_ID" \
  --duration "$DURATION" \
  --output-dir "$OUT_DIR/logs" \
  > "$OUT_DIR/logs/moveit_mvp_visual_replay.log" 2>&1

if [[ "$DAY_ID" == "day03" ]]; then
  python3 "$SCRIPT_DIR/d3_hand_eye_calibration.py" \
    --arm-side right \
    --output "$OUT_DIR/logs/d3_hand_eye_matrix.json" \
    > "$OUT_DIR/logs/d3_hand_eye_matrix_stdout.json" 2>&1
fi

echo "MVP MoveIt/Gazebo visual complete."
echo "output=$OUT_DIR"
echo "summary=$OUT_DIR/logs/mvp_moveit_summary.json"
if [[ "$DAY_ID" == "day03" ]]; then
  echo "hand_eye=$OUT_DIR/logs/d3_hand_eye_matrix.json"
fi
