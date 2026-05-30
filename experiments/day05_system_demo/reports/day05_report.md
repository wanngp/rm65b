# Day 5 report - integrated system demo

## Completed offline

- Added full-system launch wiring static TF, optional hardware RM drivers,
  grippers, safety, vision, IBVS, dual-arm planning, and weaving coordinator.
- Added safety reset and stop-all services.
- Added dry-run mode for a safe first launch without robot motion.
- Added day-by-day acceptance and limitation documents.
- Added MoveIt2 planned integrated playback through `/move_action`.
- Added true dual-arm MoveIt group playback and simulation tension topic.
- Added day-specific integrated Gazebo world, Gazebo left-camera recording, and
  simulated force-control feedback loop.

## Simulation evidence

- Final Harmonic output: `outputs/rm65b_gazebo_camera_force_loop_days_harmonic_20260527_2245/day05`
- Gazebo environment: D5 integrated scene combining force station, vision
  board, loom, yarn/cloth markers, and status panel.
- MoveIt2 stages: `hook_yarn`, `pick_yarn`, `pull_tight`, `shift`,
  `complete`; true `dual_arms` group, 17 playback points, `error_code: 1`.
- Videos: `videos/day05_moveit_harmonic_playback.mp4` and
  `videos/day05_left_gazebo_camera.mp4`, 26.0 s each.
- Screenshots: `01_fixed_bases_loaded.png`, `02_moveit_joint_planning.png`,
  `03_day_result.png`, `04_left_gazebo_camera_view.png`.
- Contains screenshots, MP4, logs, and a ROS bag covering dual arms, grippers,
  MoveIt action status/feedback, planned joint trajectories, force feedback,
  Gazebo camera vision guidance, weaving events, simulated tension, and
  force-control closed-loop topics.

## Field validation required

- Launch in dry-run mode and inspect topics, TF, and events.
- Enable hardware only after emergency stop, soft limits, and reduced speed are
  confirmed.
- Record rosbag and video for the first weaving half-cycle.
- Validate end-to-end latency below 200 ms.
- Document all skipped items, bad time axes, missing references, and low
  observability windows.
