# RM65-B dual-arm ROS 2 workspace

This workspace is built around the official RealMan Humble package and a
small layer of experiment packages.

## Packages

- `ros2_rm_robot`: official RealMan ROS 2 Humble source, version V1.6.0 from
  the downloaded humble branch archive.
- `rm65b_dual_arm_bringup`: launch files and unified parameters.
- `rm65b_gripper_control`: `control_msgs/action/GripperCommand` compatible
  gripper server with simulation and protocol placeholders.
- `rm65b_safety`: watchdog, force limit, joint limit, and stop-topic bridge.
- `rm65b_vision_guidance`: ArUco/AprilTag-style target pose and IBVS scaffold.
- `rm65b_weaving_primitives`: weaving primitive trajectories and coordinator.

## Notes

The official RealMan driver publishes topics under `rm_driver/...`. The dual
arm launch starts each driver in a namespace, yielding:

- `/left_rm_driver/rm_driver/...`
- `/right_rm_driver/rm_driver/...`

The official force-position API topic name contains the upstream spelling
`set_force_postion_cmd`; this workspace keeps that spelling for compatibility.

The repository path contains Chinese characters. The custom Python packages
build there, but official `rm_ros_interfaces` can fail in `rosidl` path
handling. Use an ASCII-only workspace path for full official-driver builds.

## VMware Ubuntu bootstrap

Use Ubuntu 22.04 with ROS 2 Humble. VMware Ubuntu is the delivery target; WSL2
Ubuntu 22.04 is acceptable for development and local checks if GUI support is
available.

Expected external software:

- Ubuntu 22.04 Jammy, x86_64.
- ROS 2 Humble deb packages, including ROS base, RViz2, MoveIt2, ros2_control,
  rosbag2, and common build tools.
- Gazebo Harmonic and ROS-Gazebo bridge packages.
- Python 3 with PyYAML, NumPy, and OpenCV.
- `colcon`, CMake, GCC/G++, `ffmpeg`, `x11-utils`, and optional `xdotool`.

Official installation references:

- ROS 2 Humble Ubuntu deb packages:
  <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>
- Gazebo Harmonic Ubuntu binaries:
  <https://gazebosim.org/docs/harmonic/install_ubuntu/>

For a fresh VM, run the all-in-one bootstrap command inside Ubuntu:

```bash
cd ~/rm65b_dual_arm_ws
bash scripts/bootstrap_vmware_ubuntu.sh --all
```

`--all` configures the official ROS2 and Gazebo apt sources, installs available
dependencies through apt, downloads or reuses the official RealMan
`ros2_rm_robot` source, cleans generated build state, checks the environment,
and builds the workspace.

If apt sources are already managed by the VM owner, use:

```bash
bash scripts/bootstrap_vmware_ubuntu.sh --install-apt-deps --clean-build
```

If dependencies are already installed and only a build is needed, use:

```bash
bash scripts/bootstrap_vmware_ubuntu.sh --build
```

The apt step installs available base, ROS2 Humble, MoveIt2, Gazebo/ros_gz,
colcon, Python YAML/NumPy/OpenCV, ffmpeg, and GUI helper packages. If a package
is unavailable from the current apt sources, the script reports it.

If the VM already has RealMan packages in another workspace, reuse them instead
of downloading again:

```bash
bash scripts/bootstrap_vmware_ubuntu.sh \
  --official-source-dir ~/old_ros2_ws/src/ros2_rm_robot \
  --existing-setup ~/old_ros2_ws/install/setup.bash
```

`--official-source-dir` reuses the old source tree. `--existing-setup` sources
the old build as an underlay for environment checks and optional builds. The
current workspace still keeps its own `src/ros2_rm_robot`, because the recording
and playback scripts expect RealMan package files under this workspace after
build.

## Build and run the experiment in VMware Ubuntu

For a fresh VM, let the bootstrap script install missing apt dependencies,
download or reuse the official RealMan source, verify the environment, and
build the workspace:

```bash
cd ~/rm65b_dual_arm_ws
bash scripts/bootstrap_vmware_ubuntu.sh --all
```

If the VM already has a usable RealMan workspace, reuse it as an underlay:

```bash
cd ~/rm65b_dual_arm_ws
bash scripts/bootstrap_vmware_ubuntu.sh \
  --official-source-dir ~/old_ros2_ws/src/ros2_rm_robot \
  --existing-setup ~/old_ros2_ws/install/setup.bash \
  --build
```

Start with Day 01 visual inspection. This opens Gazebo GUI and writes logs and
evidence under `~/rm65b_visual_*`:

```bash
cd ~/rm65b_dual_arm_ws
bash scripts/run_day_visual.sh day01
```

Expected result: a Gazebo window opens with the RM65-B dual-arm scene. The
terminal prints the output directory, for example
`~/rm65b_visual_20260530_173000/day01`. If the GUI does not open, first check
that `echo $DISPLAY` or `echo $WAYLAND_DISPLAY` is non-empty, then inspect
`$OUT/logs/run_day_visual_backend.log` and `$OUT/logs/run_day_visual_gz_gui.log`.
In WSL, GUI support depends on WSLg or an external X server; VMware Ubuntu with
a desktop session is the preferred visual environment.

If you only want generated evidence without opening Gazebo GUI:

```bash
cd ~/rm65b_dual_arm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

export OPS_ROOT=~/rm65b_live_ops_$(date +%Y%m%d_%H%M%S)
export DAY=day01
export DOMAIN=211
export OUT="$OPS_ROOT/$DAY"
mkdir -p "$OUT"

RECORD_RVIZ=0 CAPTURE_TIMEOUT=360 \
  bash scripts/play_harmonic_planned_record.sh "$OUT" "$PWD" "$DOMAIN" "$DAY"
```

The Day 01 run writes videos, screenshots, logs, and rosbag files under
`$OUT/videos`, `$OUT/screenshots`, `$OUT/logs`, and `$OUT/rosbags`.
Use `RECORD_RVIZ=0` for a faster non-interactive run that still records Gazebo
camera evidence.

To run all five simulation days:

```bash
cd ~/rm65b_dual_arm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

export OPS_ROOT=~/rm65b_live_ops_$(date +%Y%m%d_%H%M%S)
RECORD_RVIZ=1 CAPTURE_TIMEOUT=420 \
  bash scripts/record_moveit_days_harmonic.sh "$OPS_ROOT" "$PWD" 211
```

For a quick launch-only system check without recording:

```bash
ros2 launch rm65b_dual_arm_bringup full_system.launch.py use_hardware:=false
```

Do not run with `use_hardware:=true` until robot IPs, TCP frames, gripper
wiring, sensors, joint limits, and emergency stop behavior have been checked on
site.
