# Day 1 report - gripper, TF/TCP, dual-arm coordination

## Completed offline

- Built `rm65b_gripper_control` with `GripperCommand` action servers for left
  and right grippers.
- Added left/right gripper parameter files with simulation defaults and
  hardware protocol guardrails.
- Added `dual_arm_frames.yaml` and `static_tf.launch.py` for base, TCP, and
  wrist-camera transforms.
- Added left/right RM65-B driver parameter files using `arm_type: RM_65`.
- Added `rm65b_dual_arm_planning` for left/right joint trajectory planning.
- Added MoveIt2 playback through `/move_action` with the generated true
  dual-arm `dual_arms` group.

## Simulation evidence

- Final Harmonic output: `outputs/rm65b_gazebo_camera_force_loop_days_harmonic_20260527_2245/day01`
- Gazebo environment: D1-specific gripper/TF scene with TCP posts, handoff
  block, and gripper gap gauge.
- Visual: dual RM65-B arms with fixed bases and scaled grippers fixed to
  `Link6`.
- MoveIt2 stage: `gripper_tf_dualarm`, true `dual_arms` group, 6 playback
  points, `error_code: 1`.
- Videos: `videos/day01_moveit_harmonic_playback.mp4` and
  `videos/day01_left_gazebo_camera.mp4`, 14.0 s each.
- Screenshots: `01_fixed_bases_loaded.png`, `02_moveit_joint_planning.png`,
  `03_day_result.png`, `04_left_gazebo_camera_view.png`.
- ROS bag topics: `/joint_states`, `/dual_arm_planning/phase`,
  `/dual_arm_planning/left_joint_trajectory`,
  `/dual_arm_planning/right_joint_trajectory`,
  `/move_action/_action/status`, `/display_planned_path`,
  `/left_gripper_controller/joint_states`,
  `/right_gripper_controller/joint_states`.
- Added dual driver launch with namespaces to prevent `/rm_driver` topic
  collisions.

## Field validation required

- Mount gripper/hook tooling and replace placeholder TCP offsets.
- Measure gripper open/close latency; target is below 100 ms.
- Repeat TCP touch-off test; target repeatability is below +/-0.5 mm.
- Run low-speed dual-arm handoff and record sync error; target is below 50 ms.
- Confirm no mechanical interference in the constrained workspace.
