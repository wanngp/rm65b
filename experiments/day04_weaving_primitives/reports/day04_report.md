# Day 4 report - weaving primitives

## Completed offline

- Added BehaviorTree.CPP XML for a half-cycle weaving demo.
- Added seed primitive trajectories for hook, pick, pull-tight, and shift.
- Added `weaving_coordinator` to dry-run or send arm/gripper actions.
- Added `trajectory_recorder` to capture teach-pendant joint samples.
- Added MoveIt2 playback through `/move_action` for primitive execution.
- Added XML-driven coordinator execution and a simulation tension model.
- Added simulated force-control loop input from yarn tension and six-axis force
  topic.

## Simulation evidence

- Final Harmonic output: `outputs/rm65b_gazebo_camera_force_loop_days_harmonic_20260527_2245/day04`
- Gazebo environment: D4-specific loom rails, warp strips, shuttle lane, and
  tension scale.
- MoveIt2 stages: `pick_yarn`, `pull_tight`, `shift`; true `dual_arms`
  group, 11 playback points, `error_code: 1`.
- Videos: `videos/day04_moveit_harmonic_playback.mp4` and
  `videos/day04_left_gazebo_camera.mp4`, 19.0 s each.
- Screenshots: `01_fixed_bases_loaded.png`, `02_moveit_joint_planning.png`,
  `03_day_result.png`, `04_left_gazebo_camera_view.png`.
- ROS bag topics: `/weaving/events`, `/dual_arm_planning/phase`,
  `/dual_arm_planning/left_joint_trajectory`,
  `/dual_arm_planning/right_joint_trajectory`,
  `/move_action/_action/status`, `/display_planned_path`,
  `/weaving/tension_n`, `/weaving/tension_status`, `/force_control/state`.
- Launch log contains `sequence_start`, `hook_yarn`, `pull_tight`, `shift`, and
  `sequence_complete`.

## Field validation required

- Replace seed trajectories with teach-pendant recordings.
- Smooth and replay trajectories at reduced speed.
- Validate replay repeatability below +/-1 mm.
- Tune compliance/PID under yarn tension and record results.
