#!/usr/bin/env bash
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive
sudo apt-get update
sudo apt-get install -y \
  gazebo \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-control-msgs \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers

set +u
source /opt/ros/humble/setup.bash
set -u
command -v gazebo
command -v gzserver
command -v gzclient
ros2 pkg list | grep '^gazebo_ros$'
