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

Start with Day 01 visual inspection. On the MVP visual branch this opens
Gazebo, starts MoveIt `move_group`, and writes logs under
`~/rm65b_mvp_visual_*`:

```bash
cd ~/rm65b_dual_arm_ws
bash scripts/run_day_visual.sh day01
```

This command uses the clean MoveIt/Gazebo runner:

- D1: MoveIt plans a clear dual-arm straight-line sweep and Gazebo shows one
  `dual_rm65b_mvp` model moving through a custom lane/collision-zone scene.
  D1 is the node/topic baseline for the later days: it publishes MoveIt
  planning status, dual-arm trajectories, joint states, Gazebo trajectory
  commands, acceptance status, and visible gripper open/close commands. The
  runner asks MoveIt to plan every stage, then sends Gazebo a dense smoothed
  trajectory between the MoveIt-validated stage waypoints for visual playback.
- D2: MoveIt plans approach/press/release stages, with visible compliant relief
  and `/force_control/*` state output. The D2 force-pad scene is placed close
  to the right gripper path so the visual contact is easy to inspect. The right
  gripper holds a small yellow contact block and moves forward/backward along
  the gripper-front direction defined by the right arm's initial pose, pushing
  that block into the front-aligned wall instead of driving the gripper fingers
  directly into the obstacle.
- D3: MoveIt plans right-arm hand-eye viewpoints only: wide view, visual align,
  calibration lock, and retract. The MVP runner does not add physical touch to
  D3; it publishes synthetic `/right_camera/*`, `/vision/*`, and
  `/visual_servo/*` signals while Gazebo shows the eye-in-hand camera and
  a front-facing 80 mm `DICT_4X4_50` ArUco tag with marker id 7. In this D3
  scene the right gripper's forward direction is world `+X`, so the ArUco board
  sits on the right side of the robot and its near face points back toward the
  camera.
- D4: MoveIt plans the five weaving primitives from the experiment list:
  `hook_yarn`, `lift_yarn`, `pull_tight`, `shift`, and `exchange`. For this
  dual-arm weaving scene, the line between the arm bases is world `X`; both
  gripper forward axes are aligned to world `+Y`, so the two arms face the same
  direction and work perpendicular to their connecting line. The scene
  background is intentionally reduced to a black Tian-shaped field frame with a
  32-link red segmented rigid-body yarn chain. The chain starts loosely wrapped
  around the center post, with a bright green right-end handle and a bright blue
  left-end handle. During D4/D5 visual runs, the runner builds the local
  `RuntimeLinkAttacher` Gazebo system and uses fixed `DetachableJoint`
  attachments before playback starts: the right chain end is pinned to the
  right gripper, the left chain end is pinned to the left gripper, and the
  middle chain segment is pinned to the center post. The runner publishes
  `/weaving/*` tension, PID, compliance, primitive-lock, and yarn-state topics.
- D5: MoveIt plans an integrated two-loop sequence that combines the D3 visual
  lock, D2 tension/compliance surrogate, and D4 weaving primitives. Gazebo uses
  the same simplified Tian-frame and pickup-chain scene. This is still a
  simulation MVP for visual/manual inspection, not real yarn physics or
  hardware acceptance.
- The MVP runner does not use the legacy world generator and does not spawn
  left and right arms as separate models.

While Day 01 is running, use another terminal to check the baseline topic
contract:

```bash
cd ~/rm65b_dual_arm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 action list | grep /move_action
ros2 topic list | grep -E 'dual_arm|joint_states|dual_rm65b_mvp|force_control|vision|visual_servo|right_camera|weaving|acceptance'
ros2 topic echo /acceptance/day_status --once
ros2 topic echo /dual_arm_planning/phase --once
ros2 topic echo /weaving/events --once
```

The same command writes the expected topic list to
`$OUT/logs/day_topic_contract.txt`, where `$OUT` is the output directory printed
by the visual runner.

To inspect the Day 02 force-compliance motion through the required MoveIt path:

```bash
cd ~/rm65b_dual_arm_ws
bash scripts/run_day_visual.sh day02
```

In the MVP branch Day 02 is intentionally minimal: Gazebo shows the right arm
approach, press, perform a compliant relief motion, and press again against a
custom force-pad scene. The force/admittance evidence is published on
`/force_control/*`.

To inspect the Day 03 vision-guidance scene through the required MoveIt path:

```bash
cd ~/rm65b_dual_arm_ws
bash scripts/run_day_visual.sh day03
```

In the MVP branch Day 03 uses the same single Gazebo dual-arm model with a
custom ArUco calibration-board scene while MoveIt plans hand-eye camera
viewpoints, visual alignment, calibration lock, and retract stages for the
right arm. The board is placed in the right-arm forward direction, which is
world `+X` for this scene. D3 intentionally avoids contact; the evidence path
is camera image, OpenCV ArUco target pose, visual-servo command, and
aligned-state topics. The simulated marker is `DICT_4X4_50`, id `7`, with
`tag_size_m=0.080`. The run also writes the configured hand-eye result to
`$OUT/logs/d3_hand_eye_matrix.json`.

To inspect the Day 04 weaving primitive MVP:

```bash
cd ~/rm65b_dual_arm_ws
MVP_DURATION=60 bash scripts/run_day_visual.sh day04
```

Watch for both grippers facing the same world `+Y` direction, perpendicular to
the left-right base line, while the five primitive names appear on
`/dual_arm_planning/phase` and `/weaving/events`.

To inspect the Day 05 integrated loop MVP:

```bash
cd ~/rm65b_dual_arm_ws
MVP_DURATION=60 bash scripts/run_day_visual.sh day05
```

Watch for the initial visual-lock stage, two figure-eight weaving loops, and
continuous `/vision/*`, `/force_control/*`, and `/weaving/*` topic output.

`run_day02_force_visual.sh` and `run_day03_vision_visual.sh` are now diagnostic
entrypoints only. By default they delegate to `run_day_visual.sh day02/day03`
so the accepted D1-D5 visual path always uses `rm65b_dual_arm_moveit_config`
and the `dual_arms` MoveIt group.

For a legacy non-MoveIt Gazebo diagnostic, use the joint sweep demo. It is
separate from the MoveIt/Gazebo MVP path:

```bash
cd ~/rm65b_dual_arm_ws
bash scripts/run_joint_sweep_visual.sh 180
```

The first argument is the demo duration in seconds. Use `0` to run until
interrupted.

Expected MVP result: a Gazebo window opens with one `dual_rm65b_mvp` model and
the day-specific custom scene. The terminal prints the output directory, for
example `~/rm65b_visual_20260530_173000/day01`. If Gazebo does not open, first
check that `echo $DISPLAY` or `echo $WAYLAND_DISPLAY` is non-empty, then inspect
`$OUT/logs/gz_sim.log` and `$OUT/logs/moveit_mvp_visual_replay.log`.
In WSL, GUI support depends on WSLg or an external X server; VMware Ubuntu with
a desktop session is the preferred visual environment.

Legacy/full recording path, separate from the MVP visual runner:

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
ros2 launch rm65b_dual_arm_bringup full_system.launch.py \
  use_hardware:=false \
  use_sim_time:=true \
  enable_move_group:=true \
  moveit_allow_trajectory_execution:=false
```

In another terminal, confirm MoveIt is available and generate a dry planning
artifact:

```bash
ros2 action list | grep /move_action
ros2 run rm65b_dual_arm_planning dual_moveit_plan_client \
  --day-id day03 \
  --output-dir outputs/moveit_smoke/day03
```

Do not run with `use_hardware:=true` until robot IPs, TCP frames, gripper
wiring, sensors, joint limits, and emergency stop behavior have been checked on
site.
