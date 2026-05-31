#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/.." && pwd)"

DURATION="${1:-180}"
DOMAIN_ID="${2:-44}"
OUT_ROOT="${3:-$HOME/rm65b_day03_vision_visual_$(date +%Y%m%d_%H%M%S)}"
OUT_DIR="$OUT_ROOT"
mkdir -p "$OUT_DIR/logs" "$OUT_DIR/left_camera_frames" "$OUT_DIR/vision_debug_frames"

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
export GZ_PARTITION="${GZ_PARTITION:-rm65b_day03_vision_visual_${DOMAIN_ID}_$$}"
export GZ_SIM_RESOURCE_PATH="$WORKSPACE/install/rm_description/share/rm_description:${GZ_SIM_RESOURCE_PATH:-}"

BASE_WORLD="${BASE_WORLD:-$WORKSPACE/harmonic/rm65b_empty_world.sdf}"
WORLD_RUNTIME="$OUT_DIR/logs/rm65b_day03_vision_world.sdf"
URDF="$WORKSPACE/install/rm_description/share/rm_description/urdf/rm_65.urdf"
URDF_HARMONIC="$OUT_DIR/logs/rm_65_harmonic_file_meshes.urdf"
SDF_DYNAMIC="$OUT_DIR/logs/rm_65_harmonic_dynamic.sdf"
SDF_LEFT="$OUT_DIR/logs/rm_65_day03_vision_left_controlled.sdf"
SDF_RIGHT="$OUT_DIR/logs/rm_65_day03_vision_right_controlled.sdf"
MESH_ROOT="$WORKSPACE/install/rm_description/share/rm_description/meshes"
LEFT_INITIAL_POSITIONS="0.046000 -0.840000 1.161000 -0.120000 0.865000 -0.383000"
RIGHT_INITIAL_POSITIONS="0.000000 -0.350000 0.650000 0.000000 0.900000 0.000000"
LEFT_EIH_VISUAL_POSE="${LEFT_EIH_VISUAL_POSE:-0.032 0 0.090 0 0.35 0.38}"
LEFT_EIH_SENSOR_POSE="${LEFT_EIH_SENSOR_POSE:-0.034 0 0.095 0 0.35 0.38}"
LEFT_EIH_HORIZONTAL_FOV="${LEFT_EIH_HORIZONTAL_FOV:-2.20}"

PIDS=()

terminate_tree() {
  local root="$1"
  local child
  for child in $(pgrep -P "$root" 2>/dev/null || true); do
    terminate_tree "$child"
  done
  kill "$root" >/dev/null 2>&1 || true
}

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      terminate_tree "$pid"
    fi
  done
}
trap cleanup EXIT INT TERM

echo "workspace=$WORKSPACE"
echo "duration=$DURATION"
echo "ros_domain_id=$ROS_DOMAIN_ID"
echo "gz_partition=$GZ_PARTITION"
echo "output=$OUT_DIR"

python3 "$SCRIPT_DIR/generate_day_world.py" \
  --base-world "$BASE_WORLD" \
  --day-id day03 \
  --output "$WORLD_RUNTIME" \
  --notes "$OUT_DIR/logs/day03_world_notes.txt"

sed "s#package://rm_description/meshes#file://$MESH_ROOT#g" "$URDF" > "$URDF_HARMONIC"
gz sdf -p "$URDF_HARMONIC" > "$SDF_DYNAMIC"

python3 "$SCRIPT_DIR/prepare_harmonic_joint_demo_sdf.py" \
  --source "$SDF_DYNAMIC" \
  --output "$SDF_LEFT" \
  --initial-positions "$LEFT_INITIAL_POSITIONS" \
  --eye-camera-topic /left_camera/image_rect \
  --eye-camera-name left_eye_in_hand_camera \
  --eye-camera-color "0.16 0.36 0.68 1" \
  --eye-camera-visual-pose "$LEFT_EIH_VISUAL_POSE" \
  --eye-camera-sensor-pose "$LEFT_EIH_SENSOR_POSE" \
  --eye-camera-horizontal-fov "$LEFT_EIH_HORIZONTAL_FOV"

python3 "$SCRIPT_DIR/prepare_harmonic_joint_demo_sdf.py" \
  --source "$SDF_DYNAMIC" \
  --output "$SDF_RIGHT" \
  --initial-positions "$RIGHT_INITIAL_POSITIONS" \
  --eye-camera-topic /right_camera/image_rect \
  --eye-camera-name right_eye_in_hand_camera \
  --eye-camera-color "0.68 0.24 0.18 1" \
  --eye-camera-visual-pose "0.032 0 0.090 0 0.35 0" \
  --eye-camera-sensor-pose "0.034 0 0.095 0 0.35 0" \
  --eye-camera-horizontal-fov "1.65"

ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
  /left_camera/image_rect@sensor_msgs/msg/Image[gz.msgs.Image \
  /right_camera/image_rect@sensor_msgs/msg/Image[gz.msgs.Image \
  /model/left_rm65b/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory \
  /model/right_rm65b/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory \
  /rm65b_gripper/upper_finger_cmd@std_msgs/msg/Float64]gz.msgs.Double \
  /rm65b_gripper/lower_finger_cmd@std_msgs/msg/Float64]gz.msgs.Double \
  > "$OUT_DIR/logs/ros_gz_bridge.log" 2>&1 &
PIDS+=("$!")

ros2 run rm65b_vision_guidance gazebo_camera_info_publisher \
  > "$OUT_DIR/logs/gazebo_camera_info_publisher.log" 2>&1 &
PIDS+=("$!")

ros2 run rm65b_vision_guidance aruco_target_node \
  --ros-args \
  -p image_topic:=/left_camera/image_rect \
  -p camera_info_topic:=/left_camera/camera_info \
  -p debug_image_topic:=/vision/debug_image \
  -p fallback_publish_center:=false \
  > "$OUT_DIR/logs/aruco_target_node.log" 2>&1 &
PIDS+=("$!")

ros2 run rm65b_vision_guidance ibvs_controller \
  > "$OUT_DIR/logs/ibvs_controller.log" 2>&1 &
PIDS+=("$!")

gz sim -v 4 -r -s "$WORLD_RUNTIME" > "$OUT_DIR/logs/gz_server.log" 2>&1 &
PIDS+=("$!")

sleep "${SPAWN_DELAY:-8}"
ros2 run ros_gz_sim create -world rm65b_world -file "$SDF_LEFT" -name left_rm65b -x -0.45 -y 0 -z 0.02 -Y 1.5708 \
  > "$OUT_DIR/logs/spawn_left_rm65b.log" 2>&1
ros2 run ros_gz_sim create -world rm65b_world -file "$SDF_RIGHT" -name right_rm65b -x 0.45 -y 0 -z 0.02 -Y -1.5708 \
  > "$OUT_DIR/logs/spawn_right_rm65b.log" 2>&1

if [[ "${CAPTURE_FRAMES:-0}" != "0" ]]; then
  timeout "${CAPTURE_TIMEOUT:-45}s" python3 "$SCRIPT_DIR/save_ros_images.py" \
    --topic /left_camera/image_rect \
    --output-dir "$OUT_DIR/left_camera_frames" \
    --count "$CAPTURE_FRAMES" \
    > "$OUT_DIR/logs/save_left_gazebo_camera_images.log" 2>&1 &
  PIDS+=("$!")
  timeout "${CAPTURE_TIMEOUT:-45}s" python3 "$SCRIPT_DIR/save_ros_images.py" \
    --topic /vision/debug_image \
    --output-dir "$OUT_DIR/vision_debug_frames" \
    --count "$CAPTURE_FRAMES" \
    > "$OUT_DIR/logs/save_vision_debug_images.log" 2>&1 &
  PIDS+=("$!")
fi

if [[ "${OPEN_GZ_GUI:-1}" == "0" ]]; then
  echo "OPEN_GZ_GUI=0; Gazebo GUI was not opened." | tee "$OUT_DIR/logs/gz_gui.log"
elif [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]]; then
  gz sim -g > "$OUT_DIR/logs/gz_gui.log" 2>&1 &
  PIDS+=("$!")
  echo "Gazebo GUI opened. Watch the left arm camera align with the green Day3 vision target."
  echo "Demo will run for ${DURATION}s; close the window or Ctrl+C to stop early."
else
  echo "DISPLAY/WAYLAND_DISPLAY is not set; Gazebo GUI was not opened." | tee "$OUT_DIR/logs/gz_gui.log"
fi

sleep "${DEMO_START_DELAY:-3}"
python3 "$SCRIPT_DIR/day03_vision_align_demo.py" \
  --duration "$DURATION" \
  > "$OUT_DIR/logs/day03_vision_align_demo.log" 2>&1

timeout 5s ros2 topic echo /vision/status --once > "$OUT_DIR/logs/vision_status_once.txt" 2>&1 || true
timeout 5s ros2 topic echo /vision/target_pose --once > "$OUT_DIR/logs/vision_target_once.txt" 2>&1 || true
timeout 5s ros2 topic echo /visual_servo/twist_cmd --once > "$OUT_DIR/logs/visual_servo_twist_once.txt" 2>&1 || true

echo "Day3 vision visual demo complete. Logs are under: $OUT_DIR/logs"
