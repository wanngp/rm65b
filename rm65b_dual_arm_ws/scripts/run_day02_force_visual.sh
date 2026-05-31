#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/.." && pwd)"

DURATION="${1:-180}"
DOMAIN_ID="${2:-43}"
OUT_ROOT="${3:-$HOME/rm65b_day02_force_visual_$(date +%Y%m%d_%H%M%S)}"

if [[ "${ALLOW_NON_MOVEIT_DAY02_VISUAL:-0}" != "1" ]]; then
  if [[ -f "$SCRIPT_DIR/run_day_visual.sh" ]]; then
    echo "run_day02_force_visual.sh is a direct non-MoveIt diagnostic entry."
    echo "Delegating to the required MoveIt visual path: run_day_visual.sh day02"
    exec bash "$SCRIPT_DIR/run_day_visual.sh" day02 "$DOMAIN_ID" "$OUT_ROOT"
  fi
  echo "run_day02_force_visual.sh direct mode is disabled because D1-D5 must use MoveIt." >&2
  echo "Missing $SCRIPT_DIR/run_day_visual.sh; run scripts/bootstrap_vmware_ubuntu.sh --all or use the current workspace." >&2
  echo "Set ALLOW_NON_MOVEIT_DAY02_VISUAL=1 only for direct force-scene diagnostics." >&2
  exit 2
fi

OUT_DIR="$OUT_ROOT"
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
export GZ_PARTITION="${GZ_PARTITION:-rm65b_day02_force_visual_${DOMAIN_ID}_$$}"
export GZ_SIM_RESOURCE_PATH="$WORKSPACE/install/rm_description/share/rm_description:${GZ_SIM_RESOURCE_PATH:-}"

BASE_WORLD="${BASE_WORLD:-$WORKSPACE/harmonic/rm65b_empty_world.sdf}"
WORLD_RUNTIME="$OUT_DIR/logs/rm65b_day02_force_world.sdf"
URDF="$WORKSPACE/install/rm_description/share/rm_description/urdf/rm_65.urdf"
URDF_HARMONIC="$OUT_DIR/logs/rm_65_harmonic_file_meshes.urdf"
SDF_DYNAMIC="$OUT_DIR/logs/rm_65_harmonic_dynamic.sdf"
SDF_CONTROLLED="$OUT_DIR/logs/rm_65_day02_force_controlled.sdf"
MESH_ROOT="$WORKSPACE/install/rm_description/share/rm_description/meshes"

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
  --day-id day02 \
  --output "$WORLD_RUNTIME" \
  --notes "$OUT_DIR/logs/day02_world_notes.txt"

sed "s#package://rm_description/meshes#file://$MESH_ROOT#g" "$URDF" > "$URDF_HARMONIC"
gz sdf -p "$URDF_HARMONIC" > "$SDF_DYNAMIC"
python3 "$SCRIPT_DIR/prepare_harmonic_joint_demo_sdf.py" \
  --source "$SDF_DYNAMIC" \
  --output "$SDF_CONTROLLED" \
  --force-probe

ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
  /model/left_rm65b/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory \
  /model/right_rm65b/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory \
  /rm65b_gripper/upper_finger_cmd@std_msgs/msg/Float64]gz.msgs.Double \
  /rm65b_gripper/lower_finger_cmd@std_msgs/msg/Float64]gz.msgs.Double \
  > "$OUT_DIR/logs/ros_gz_bridge.log" 2>&1 &
PIDS+=("$!")

gz sim -v 4 -r -s "$WORLD_RUNTIME" > "$OUT_DIR/logs/gz_server.log" 2>&1 &
PIDS+=("$!")

sleep "${SPAWN_DELAY:-8}"
ros2 run ros_gz_sim create -world rm65b_world -file "$SDF_CONTROLLED" -name left_rm65b -x -0.45 -y 0 -z 0.02 -Y 1.5708 \
  > "$OUT_DIR/logs/spawn_left_rm65b.log" 2>&1
ros2 run ros_gz_sim create -world rm65b_world -file "$SDF_CONTROLLED" -name right_rm65b -x 0.45 -y 0 -z 0.02 -Y -1.5708 \
  > "$OUT_DIR/logs/spawn_right_rm65b.log" 2>&1

if [[ "${OPEN_GZ_GUI:-1}" == "0" ]]; then
  echo "OPEN_GZ_GUI=0; Gazebo GUI was not opened." | tee "$OUT_DIR/logs/gz_gui.log"
elif [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]]; then
  gz sim -g > "$OUT_DIR/logs/gz_gui.log" 2>&1 &
  PIDS+=("$!")
  echo "Gazebo GUI opened. Watch the right arm repeatedly press the blue Day2 pad."
  echo "Demo will run for ${DURATION}s; close the window or Ctrl+C to stop early."
else
  echo "DISPLAY/WAYLAND_DISPLAY is not set; Gazebo GUI was not opened." | tee "$OUT_DIR/logs/gz_gui.log"
fi

sleep "${DEMO_START_DELAY:-3}"
python3 "$SCRIPT_DIR/day02_force_press_demo.py" \
  --duration "$DURATION" \
  > "$OUT_DIR/logs/day02_force_press_demo.log" 2>&1

echo "Day2 force visual demo complete. Logs are under: $OUT_DIR/logs"
