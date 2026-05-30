# Day 3 report - vision guidance

## Completed offline

- Added `aruco_target_node` for image/camera-info subscription and target pose
  publishing.
- Added `ibvs_controller` for visual-servo twist commands and alignment state.
- Added camera, marker size, and target frame parameters.
- Added TF slots for left and right wrist cameras.
- Added MoveIt2 playback through `/move_action` for vision alignment and
  target pickup.
- Added Gazebo Harmonic camera sensor bridged to `/left_camera/image_rect`.

## Simulation evidence

- Final Harmonic output: `outputs/rm65b_gazebo_camera_force_loop_days_harmonic_20260527_2245/day03`
- Gazebo environment: D3-specific vision board, fiducial-style markers, target
  marker, lighting panel, and Gazebo left camera.
- MoveIt2 stages: `vision_align`, `pick_yarn`; true `dual_arms` group, 8
  playback points, `error_code: 1`.
- Videos: `videos/day03_moveit_harmonic_playback.mp4` and
  `videos/day03_left_gazebo_camera.mp4`, 14.0 s each.
- Screenshots: `01_fixed_bases_loaded.png`, `02_moveit_joint_planning.png`,
  `03_day_result.png`, `04_left_gazebo_camera_view.png`.
- ROS bag topics: `/left_camera/image_rect`, `/left_camera/camera_info`,
  `/vision/target_pose`, `/visual_servo/twist_cmd`,
  `/visual_servo/aligned`, `/move_action/_action/status`,
  `/display_planned_path`.

## Field validation required

- Install and rigidly fix camera.
- Calibrate camera intrinsics and run easy_handeye2.
- Validate target detection at 30 fps or better.
- Validate pixel error below 5 px and robot-frame error below +/-1 mm.
- Run alignment success test from varied initial poses.
