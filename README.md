# RM65-B dual-arm weaving experiment workspace

This directory contains the engineering deliverables for the five-day
RealMan RM65-B dual-arm experiment list.

## What is included

- `downloads/ros2_rm_robot_humble.zip`: downloaded official RealMan
  `ros2_rm_robot` Humble package.
- `rm65b_dual_arm_ws/src/ros2_rm_robot`: extracted official source copied
  into the ROS 2 workspace.
- `rm65b_dual_arm_ws/src/rm65b_*`: experiment packages for gripper control,
  static TF/TCP configuration, safety supervision, vision guidance, and
  weaving primitives.
- `experiments/dayXX_*`: day-by-day frozen configs and reports.
- `docs/`: extracted experiment list, acceptance matrix, and field checklist.

## Target robot

- Model: RealMan RM65-B, 6-DOF collaborative arm.
- Official ROS 2 driver arm type: `RM_65`.
- Default official launch family: `rm_65`.
- If the physical arm has a six-axis force sensor, use the official
  `rm_65_6f` or `rm_65_6fb` launch family during site validation.

## Build on Ubuntu 22.04 / ROS 2 Humble

Do not run apt installation while another WSL task is using package manager
locks. After ROS 2 Humble and MoveIt2 are already installed:

```bash
cd /mnt/e/1-项目/睿尔曼/rm65b_dual_arm_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rm_ros_interfaces
source install/setup.bash
colcon build
source install/setup.bash
```

If official `rm_ros_interfaces` fails with a truncated path such as
`/mnt/e/1-`, copy the workspace to an ASCII-only path first:

```bash
mkdir -p /tmp/rm65b_dual_arm_ws/src
cp -a /mnt/e/1-项目/睿尔曼/rm65b_dual_arm_ws/src/. /tmp/rm65b_dual_arm_ws/src/
cd /tmp/rm65b_dual_arm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rm_ros_interfaces rm_driver
source install/setup.bash
colcon build --symlink-install --packages-select rm65b_dual_arm_bringup rm65b_gripper_control rm65b_safety rm65b_vision_guidance rm65b_weaving_primitives
```

Start the offline/simulation-safe experiment stack:

```bash
ros2 launch rm65b_dual_arm_bringup full_system.launch.py use_hardware:=false
```

Start with physical RM65-B drivers only after IPs, TCPs, grippers, sensors,
and emergency stop have been checked:

```bash
ros2 launch rm65b_dual_arm_bringup full_system.launch.py use_hardware:=true
```

## Verification already supported offline

```bash
python rm65b_dual_arm_ws/scripts/verify_project.py
```

Offline verification checks project structure, XML/YAML parseability, and
Python syntax. Hardware acceptance items such as TCP repeatability, gripper
latency, force thresholds, camera calibration, and demo videos still require
on-site execution.
