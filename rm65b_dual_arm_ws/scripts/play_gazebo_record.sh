#!/usr/bin/env bash
set -euo pipefail

OUT_DIR="${1:?output directory required}"
WORKSPACE="${2:-/tmp/rm65b_dual_arm_ws_ascii_verify2}"
DOMAIN_ID="${3:-65}"

mkdir -p "$OUT_DIR/logs" "$OUT_DIR/screenshots" "$OUT_DIR/videos" "$OUT_DIR/rosbags"

export ROS_DOMAIN_ID="$DOMAIN_ID"
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}"

set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u

echo "OUT_DIR=$OUT_DIR" | tee "$OUT_DIR/logs/playback_env.log"
echo "WORKSPACE=$WORKSPACE" | tee -a "$OUT_DIR/logs/playback_env.log"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" | tee -a "$OUT_DIR/logs/playback_env.log"
echo "DISPLAY=${DISPLAY:-}" | tee -a "$OUT_DIR/logs/playback_env.log"
command -v gazebo | tee -a "$OUT_DIR/logs/playback_env.log"
command -v ffmpeg | tee -a "$OUT_DIR/logs/playback_env.log"

PIDS=()
cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT

SCREEN_SIZE="1280x720"
if command -v xdpyinfo >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
  DETECTED="$(xdpyinfo 2>/dev/null | awk '/dimensions:/ {print $2; exit}' || true)"
  if [ -n "$DETECTED" ]; then
    SCREEN_SIZE="$DETECTED"
  fi
fi
echo "SCREEN_SIZE=$SCREEN_SIZE" | tee -a "$OUT_DIR/logs/playback_env.log"

if [ -n "${DISPLAY:-}" ]; then
  ffmpeg -y -video_size "$SCREEN_SIZE" -framerate 10 -f x11grab -i "$DISPLAY" \
    -t 75 -pix_fmt yuv420p "$OUT_DIR/videos/gazebo_ros_playback.mp4" \
    > "$OUT_DIR/logs/ffmpeg_video.log" 2>&1 &
  PIDS+=("$!")
  sleep 1
fi

ros2 bag record \
  -o "$OUT_DIR/rosbags/rm65b_gazebo_playback" \
  /clock /joint_states /tf /tf_static /parameter_events /rm_group_controller/controller_state \
  > "$OUT_DIR/logs/rosbag_record.log" 2>&1 &
PIDS+=("$!")

ros2 launch rm_gazebo gazebo_65_demo.launch.py \
  > "$OUT_DIR/logs/gazebo_launch.log" 2>&1 &
LAUNCH_PID="$!"
PIDS+=("$LAUNCH_PID")

sleep 30

ros2 topic list > "$OUT_DIR/logs/topic_list_before_motion.txt" 2>&1 || true
ros2 control list_controllers > "$OUT_DIR/logs/controllers_before_motion.txt" 2>&1 || true

if [ -n "${DISPLAY:-}" ]; then
  ffmpeg -y -video_size "$SCREEN_SIZE" -f x11grab -i "$DISPLAY" -frames:v 1 \
    "$OUT_DIR/screenshots/01_gazebo_loaded.png" \
    > "$OUT_DIR/logs/screenshot_01.log" 2>&1 || true
fi

ACTION="/rm_group_controller/follow_joint_trajectory"
for attempt in $(seq 1 30); do
  if ros2 action list 2>/dev/null | grep -qx "$ACTION"; then
    break
  fi
  sleep 1
done
ros2 action list > "$OUT_DIR/logs/action_list.txt" 2>&1 || true

cat > "$OUT_DIR/logs/trajectory_goal.yaml" <<'YAML'
trajectory:
  joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]
  points:
    - positions: [0.0, -0.45, 0.55, 0.0, 0.75, 0.0]
      time_from_start: {sec: 2, nanosec: 0}
    - positions: [0.35, -0.65, 0.95, 0.20, 0.65, 0.25]
      time_from_start: {sec: 6, nanosec: 0}
    - positions: [-0.25, -0.40, 0.70, -0.20, 0.85, -0.20]
      time_from_start: {sec: 10, nanosec: 0}
    - positions: [0.0, -0.45, 0.55, 0.0, 0.75, 0.0]
      time_from_start: {sec: 14, nanosec: 0}
YAML

ros2 action send_goal "$ACTION" control_msgs/action/FollowJointTrajectory \
  "$(cat "$OUT_DIR/logs/trajectory_goal.yaml")" \
  > "$OUT_DIR/logs/trajectory_goal.log" 2>&1 &
GOAL_PID="$!"

sleep 4
if [ -n "${DISPLAY:-}" ]; then
  ffmpeg -y -video_size "$SCREEN_SIZE" -f x11grab -i "$DISPLAY" -frames:v 1 \
    "$OUT_DIR/screenshots/02_motion_mid.png" \
    > "$OUT_DIR/logs/screenshot_02.log" 2>&1 || true
fi

sleep 6
if [ -n "${DISPLAY:-}" ]; then
  ffmpeg -y -video_size "$SCREEN_SIZE" -f x11grab -i "$DISPLAY" -frames:v 1 \
    "$OUT_DIR/screenshots/03_motion_late.png" \
    > "$OUT_DIR/logs/screenshot_03.log" 2>&1 || true
fi

wait "$GOAL_PID" || true
sleep 4
ros2 topic echo /joint_states --once > "$OUT_DIR/logs/joint_states_once.txt" 2>&1 || true
ros2 control list_controllers > "$OUT_DIR/logs/controllers_after_motion.txt" 2>&1 || true

if [ -n "${DISPLAY:-}" ]; then
  ffmpeg -y -video_size "$SCREEN_SIZE" -f x11grab -i "$DISPLAY" -frames:v 1 \
    "$OUT_DIR/screenshots/04_after_motion.png" \
    > "$OUT_DIR/logs/screenshot_04.log" 2>&1 || true
fi

echo "playback_complete" | tee "$OUT_DIR/logs/status.txt"
