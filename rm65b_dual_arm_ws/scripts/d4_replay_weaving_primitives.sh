#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
TRAJECTORY_FILE="${1:-$WORKSPACE/src/rm65b_weaving_primitives/trajectories/weaving_primitives.yaml}"
DRY_RUN="${DRY_RUN:-false}"
BT_RUNTIME="${BT_RUNTIME:-cpp}"

set +u
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi
if [ -f "$WORKSPACE/install/setup.bash" ]; then
  source "$WORKSPACE/install/setup.bash"
fi
set -u

if [ "$BT_RUNTIME" = "cpp" ]; then
  ros2 run rm65b_weaving_primitives weaving_bt_runner \
    --ros-args \
    -p trajectory_file:="$TRAJECTORY_FILE" \
    -p behavior_tree_file:="$WORKSPACE/install/rm65b_weaving_primitives/share/rm65b_weaving_primitives/config/weaving_tree.xml" \
    -p dry_run:="$DRY_RUN" \
    -p playback_stage:=D4_teach_replay
else
  ros2 run rm65b_weaving_primitives weaving_coordinator \
    --ros-args \
    -p trajectory_file:="$TRAJECTORY_FILE" \
    -p behavior_tree_file:="$WORKSPACE/install/rm65b_weaving_primitives/share/rm65b_weaving_primitives/config/weaving_tree.xml" \
    -p dry_run:="$DRY_RUN" \
    -p use_behavior_tree_xml:=true \
    -p playback_stage:=D4_teach_replay
fi
