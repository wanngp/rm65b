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

Run the bootstrap script inside the Ubuntu VM:

```bash
cd ~/rm65b_dual_arm_ws
bash scripts/bootstrap_vmware_ubuntu.sh
```

The script checks for the official RealMan `ros2_rm_robot` source, downloads it
only when it is missing, copies it to `src/ros2_rm_robot`, and checks the Ubuntu
ROS2 environment.

To let the script install missing system dependencies through apt, add the
explicit install flag:

```bash
bash scripts/bootstrap_vmware_ubuntu.sh --install-apt-deps
```

This runs `sudo apt-get update` and installs available base, ROS2 Humble,
MoveIt2, Gazebo/ros_gz, colcon, Python YAML, and ffmpeg packages. It does not
rewrite apt sources. If ROS2 or Gazebo packages are unavailable from the VM's
current apt sources, the script reports those packages and leaves the source
configuration to the VM owner.

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
bash scripts/bootstrap_vmware_ubuntu.sh --install-apt-deps --build
```

If the VM already has a usable RealMan workspace, reuse it as an underlay:

```bash
cd ~/rm65b_dual_arm_ws
bash scripts/bootstrap_vmware_ubuntu.sh \
  --official-source-dir ~/old_ros2_ws/src/ros2_rm_robot \
  --existing-setup ~/old_ros2_ws/install/setup.bash \
  --build
```

Start with Day 01 to check the visual effect and generated evidence:

```bash
cd ~/rm65b_dual_arm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

export OPS_ROOT=~/rm65b_live_ops_$(date +%Y%m%d_%H%M%S)
export DAY=day01
export DOMAIN=211
export OUT="$OPS_ROOT/$DAY"
mkdir -p "$OUT"

RECORD_RVIZ=1 CAPTURE_TIMEOUT=360 \
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
