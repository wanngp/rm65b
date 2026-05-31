# RM65-B dual-arm weaving experiment workspace

This repository contains the engineering deliverables for the five-day
RealMan RM65-B dual-arm experiment list.

## What is included

- Bootstrap scripts that download or reuse the official RealMan
  `ros2_rm_robot` Humble package.
- `rm65b_dual_arm_ws/src/ros2_rm_robot`: ignored third-party source location
  restored by the bootstrap scripts.
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

## Build on VMware Ubuntu 22.04 / ROS 2 Humble

The commands below assume `~/rm65b_dual_arm_ws` is the ROS workspace directory
inside Ubuntu. If you cloned or copied the whole Git repository, first enter its
workspace subdirectory:

```bash
cd ~/realman_project/rm65b_dual_arm_ws
```

Run the environment bootstrap inside the Ubuntu VM. It checks the local
workspace, restores the official RealMan source if missing, can install missing
apt dependencies, and can build the workspace:

```bash
cd ~/rm65b_dual_arm_ws
bash scripts/bootstrap_vmware_ubuntu.sh --all
source install/setup.bash
```

`--all` is the beginner-friendly deployment path: it configures official ROS2
and Gazebo apt sources, installs available dependencies, downloads or reuses the
official RealMan source, cleans old build artifacts, and builds the workspace.
It expects Ubuntu 22.04 with sudo access and network access. The dependency
baseline is ROS 2 Humble, MoveIt2, RViz2, Gazebo Harmonic, ros_gz, colcon,
PyYAML, NumPy, OpenCV, ffmpeg, CMake, and GCC/G++.

Official references:

- ROS 2 Humble Ubuntu deb packages:
  <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>
- Gazebo Harmonic Ubuntu binaries:
  <https://gazebosim.org/docs/harmonic/install_ubuntu/>

If the VM already has RealMan packages built in another workspace, pass that
workspace as an underlay instead of downloading everything again:

```bash
bash scripts/bootstrap_vmware_ubuntu.sh \
  --official-source-dir ~/old_ros2_ws/src/ros2_rm_robot \
  --existing-setup ~/old_ros2_ws/install/setup.bash \
  --build
```

See `rm65b_dual_arm_ws/README.md` for Day 01 and five-day recording commands.

For a quick visual check after bootstrap:

```bash
cd ~/rm65b_dual_arm_ws
bash scripts/run_day_visual.sh day01
```

For one-by-one D1-D5 visual acceptance, run `scripts/run_day_visual.sh day01`
through `day05`. This path uses the dual-arm MoveIt configuration; the older
direct Day 02/Day 03 visual scripts are diagnostic-only wrappers by default.
On the MVP visual branch, `run_day_visual.sh` is a fast MoveIt/Gazebo runner:
it generates one custom Gazebo world containing a single `dual_rm65b_mvp` model,
not two separately spawned arms. Day 04 and Day 05 use a simplified loom scene
where both grippers face the same direction, perpendicular to the left-right
base line, and publish `/weaving/*` inspection topics.

## Simulation-safe launch

Start the offline/simulation-safe experiment stack:

```bash
cd ~/rm65b_dual_arm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rm65b_dual_arm_bringup full_system.launch.py use_hardware:=false
```

Start with physical RM65-B drivers only after IPs, TCPs, grippers, sensors,
joint limits, and emergency stop have been checked:

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
