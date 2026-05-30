#!/usr/bin/env bash
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive

sudo apt-get install -y \
  gz-harmonic \
  gz-launch7-cli \
  gz-sim8-cli \
  gz-tools2 \
  libgz-fuel-tools9 \
  libgz-fuel-tools9-dev \
  libgz-gui8-dev \
  libgz-launch7 \
  libgz-launch7-dev \
  libgz-physics7-bullet-dev \
  libgz-physics7-dartsim-dev \
  libgz-physics7-dev \
  libgz-physics7-sdf-dev \
  libgz-physics7-tpe-dev \
  libgz-sensors8-air-pressure-dev \
  libgz-sensors8-air-speed-dev \
  libgz-sensors8-altimeter-dev \
  libgz-sensors8-boundingbox-camera-dev \
  libgz-sensors8-camera-dev \
  libgz-sensors8-core-dev \
  libgz-sensors8-depth-camera-dev \
  libgz-sensors8-dev \
  libgz-sensors8-dvl-dev \
  libgz-sensors8-force-torque-dev \
  libgz-sensors8-gpu-lidar-dev \
  libgz-sensors8-imu-dev \
  libgz-sensors8-lidar-dev \
  libgz-sensors8-logical-camera-dev \
  libgz-sensors8-magnetometer-dev \
  libgz-sensors8-navsat-dev \
  libgz-sensors8-rendering-dev \
  libgz-sensors8-rgbd-camera-dev \
  libgz-sensors8-segmentation-camera-dev \
  libgz-sensors8-thermal-camera-dev \
  libgz-sensors8-wide-angle-camera-dev \
  libgz-sim8 \
  libgz-sim8-dev \
  libgz-sim8-plugins \
  libgz-tools2-dev \
  libgz-transport13-core-dev \
  libgz-transport13-dev \
  libgz-transport13-log-dev \
  libgz-transport13-parameters-dev \
  libsdformat14-dev \
  python3-gz-sim8 \
  ros-humble-ros-gzharmonic \
  ros-humble-ros-gzharmonic-bridge \
  ros-humble-ros-gzharmonic-image \
  ros-humble-ros-gzharmonic-sim \
  ros-humble-ros-gzharmonic-sim-demos

gz sim --versions || true
dpkg -l | grep -E 'gz-harmonic|ros-humble-ros-gzharmonic' || true
