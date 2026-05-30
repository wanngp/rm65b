#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
colcon build --packages-select rm_ros_interfaces
source install/setup.bash
colcon build
