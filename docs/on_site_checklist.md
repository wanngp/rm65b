# On-site checklist

## Before power-on

- Confirm both arms are RM65-B / `RM_65`, controller firmware is compatible
  with the downloaded `ros2_rm_robot` Humble package.
- Confirm left and right arm IPs match
  `rm65b_dual_arm_bringup/config/rm65b_left_driver.yaml` and
  `rm65b_dual_arm_bringup/config/rm65b_right_driver.yaml`.
- Confirm the physical emergency stop is reachable and tested.
- Confirm gripper wiring protocol: simulation, serial, Modbus TCP, or valve IO.
- Measure the left/right base transform and update `dual_arm_frames.yaml`.
- Measure TCP offsets after the gripper/hook is mounted.

## Bringup sequence

1. Start static TF only and inspect the TF tree.
2. Start one arm driver at a time; check joint states and stop command.
3. Start both arms in namespaces; verify topic separation.
4. Start gripper action servers in simulation mode, then hardware mode.
5. Start safety supervisor and trigger a low-threshold force stop test.
6. Start vision node with camera fixed; verify target pose frame.
7. Run weaving coordinator in `dry_run:=true`.
8. Record D4 teach-pendant primitives with
   `scripts/record_d4_real_teach_replay.py`.
9. Replay the recorded D4 primitive YAML through `weaving_coordinator` at
   reduced speed and force limits.
10. Start MoveIt2 planning with the measured dual-arm base/TCP transforms.

## Data to record

- TF snapshot and calibration values.
- Gripper open/close command timestamp and completion timestamp.
- `/joint_states`, force topic, safety state, target pose, and coordinator
  events in rosbag.
- `/move_action/_action/status`, `/display_planned_path`, and the final
  planned `JointTrajectory` topics in rosbag.
- D4 real primitive YAML from live `/joint_states`, plus teach and replay bags.
- D1-D5 acceptance videos following `docs/daily_acceptance_video_scripts.md`.
- Test video for Day 5 demo.
- Failure cases with exact timestamp, command, and safety state.
