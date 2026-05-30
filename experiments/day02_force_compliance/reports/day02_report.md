# Day 2 report - force feedback and compliance

## Completed offline

- Added force-position topic mapping for the official RealMan ROS 2 driver.
- Added compliance and yarn threshold parameter templates.
- Added `rm65b_safety` supervisor for joint limits, force thresholds,
  watchdog state, and RealMan stop-topic publishing.
- Documented that RM65-B standard hardware is not automatically a 6F/6FB force
  variant; true force validation requires an installed force sensor.
- Added MoveIt2 playback through `/move_action` for the force-pull stage.
- Added simulated closed-loop force/admittance controller topics for target
  wrench, wrench error, admittance offset, state, and corrected right-arm
  trajectory.

## Simulation evidence

- Final Harmonic output: `outputs/rm65b_gazebo_camera_force_loop_days_harmonic_20260527_2245/day02`
- Gazebo environment: simplified press-wall scene. The black target wall is
  the fixed fixture, the blue pad is the intended contact surface, the yellow
  arrow shows the right-arm press direction, and the side gauge represents
  admittance/compliance response.
- MoveIt2 stage: `force_pull_tight`, true `dual_arms` group, 3 playback
  points, `error_code: 1`.
- Videos: `videos/day02_moveit_harmonic_playback.mp4` and
  `videos/day02_left_gazebo_camera.mp4`, 14.0 s each.
- Screenshots: `01_fixed_bases_loaded.png`, `02_moveit_joint_planning.png`,
  `03_day_result.png`, `04_left_gazebo_camera_view.png`.
- ROS bag topics: `/left_rm_driver/rm_driver/udp_six_force`,
  `/right_rm_driver/rm_driver/udp_six_force`, `/safety/state`,
  `/safety/estop`, `/move_action/_action/status`, `/display_planned_path`,
  `/force_control/state`, `/force_control/wrench_error`,
  `/force_control/admittance_offset`,
  `/force_control/corrected_right_joint_trajectory`.

## Field validation required

- Confirm controller firmware and force sensor availability.
- Load gripper mass and save payload/gravity compensation parameters.
- Validate force stream latency below 10 ms.
- Tune compliance so contact motion is smooth and under +/-1 mm tracking error.
- Measure yarn threshold windows per material.
