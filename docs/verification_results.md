# Verification results

Date: 2026-05-28

## Offline verification

Command:

```bash
python rm65b_dual_arm_ws/scripts/verify_project.py
```

Result:

- Required files present: 23
- YAML parsed: 22
- XML parsed: 9
- Python syntax parsed: 50
- Semantic checks passed
- Offline verification complete

## WSL ROS 2 build verification

Environment:

- WSL distribution: `Ubuntu-22.04-ROS2`
- ROS: Humble from `/opt/ros/humble/setup.bash`
- Gazebo Sim: Harmonic
- ASCII build path: `/tmp/rm65b_dual_arm_ws_ascii_verify2`

Selected build used during the latest verification:

```bash
source /opt/ros/humble/setup.bash
cd /tmp/rm65b_dual_arm_ws_ascii_verify2
colcon build --packages-select rm65b_dual_arm_planning rm65b_weaving_primitives rm65b_dual_arm_bringup
```

BehaviorTree.CPP smoke test passed with the C++ runtime:

```bash
ros2 run rm65b_weaving_primitives weaving_bt_runner --ros-args -p dry_run:=true
```

The direct build from the non-ASCII `/mnt/e` project path is still avoided for
official `rosidl` packages because CMake/rosidl path handling may truncate the
path.

## Gazebo Harmonic and ROS playback evidence

Latest playback output:

```text
outputs/rm65b_final_optimized_days_harmonic_20260528_0025
```

The valid playback uses Gazebo Harmonic only. Trajectories are planned through
MoveIt2 `move_group` using group `dual_arms`; robot bases remain fixed and arm
motion is commanded through Harmonic joint trajectory topics.

Per-day evidence includes screenshots, main MP4, left-camera MP4, ROS bag,
logs, MoveIt plan YAML, generated SDF world, and day-specific notes.

## Non-hardware pass/fail

Command:

```bash
python rm65b_dual_arm_ws/scripts/generate_nonhardware_pass_fail.py --output-root outputs/rm65b_final_optimized_days_harmonic_20260528_0025
```

Result:

```text
status=PASS
failed=0
```

Key checks passing:

- D1-D5 media and Gazebo camera frames.
- D1-D5 movable Gazebo gripper command topics.
- D3/D5 Gazebo camera to ArUco target pose to IBVS to Gazebo trajectory.
- D2/D4/D5 Gazebo contact force to force controller to corrected Gazebo
  trajectory.
- D2/D4/D5 force-loop settling proxy.
- D2/D4/D5 Gazebo contact physics messages.
- D4 live `/joint_states` teach/replay primitive YAML.
- D4/D5 BehaviorTree.CPP runtime events.
- Runtime MoveIt PlanningScene/FCL collision sweep.

## Runtime MoveIt collision sweep

Command:

```bash
wsl -d Ubuntu-22.04-ROS2 -- bash /mnt/e/1-*/睿尔曼/rm65b_dual_arm_ws/scripts/run_runtime_collision_sweep_wsl.sh
```

Output:

```text
outputs/rm65b_final_optimized_days_harmonic_20260528_0025/offline_analysis/runtime_collision_sweep/runtime_collision_sweep_report.json
```

Result:

- samples: 263
- valid: 241
- invalid/collisions: 22
- service errors: 0
- planned trajectory invalid samples: 0

The 22 invalid samples are extreme joint3/joint5 fold-back poses where the
attached gripper contacts Link1/Link2/Link3 on the same arm. This is recorded
as a safety finding; it is not hidden as a clean full-space proof.

## Known limitations

- This is simulation acceptance, not hardware acceptance.
- Real gripper integration, real force sensor calibration, camera hand-eye,
  yarn/material tuning, emergency stop, latency, and repeatability still require
  on-site validation.
- The runtime collision sweep is sampled, not a mathematical continuum proof.
