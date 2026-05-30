# RM65-B Day1-Day5 legacy non-hardware evidence results

Date: 2026-05-28

Status note, updated 2026-05-30: this document records a legacy non-hardware
simulation run. It is not the final field acceptance record for the latest
D1-D5 requirements. In particular, it does not contain the new D5 launch audit,
latency record, safety fault-demo JSON, or the latest D1 gripper-only evidence.
Use `docs/technical_experiment_delivery_audit.md` as the current delivery and
gap matrix.

## Legacy valid output

Use this directory only as legacy non-hardware simulation evidence:

```text
outputs/rm65b_final_optimized_days_harmonic_20260528_0025
```

The accepted D1-D5 playback uses Gazebo Harmonic, MoveIt2 `move_group`, group
`dual_arms`, fixed left/right RM65-B bases, joint trajectory controller topics,
Gazebo camera vision, contact-force feedback, C++ BehaviorTree.CPP runtime, and
day-specific SDF worlds.

## Recorded artifacts

- Per-day screenshots in `dayXX/screenshots/`.
- Per-day videos in `dayXX/videos/`.
- Per-day ROS bag in `dayXX/rosbags/harmonic_planned_rosbag`.
- Per-day logs, generated world SDF, MoveIt plan YAML, Gazebo trajectory pbtxt,
  and playback notes under `dayXX/logs/`.
- Offline reports under `offline_analysis/`.

## Legacy non-hardware pass/fail result

```text
outputs/rm65b_final_optimized_days_harmonic_20260528_0025/offline_analysis/nonhardware_pass_fail_metrics.md
Overall: PASS
failed=0
```

Passing evidence includes:

- D1-D5 media and Gazebo camera frame extraction.
- D1-D5 Gazebo gripper command topics.
- D3/D5 Gazebo camera to ArUco target pose to IBVS to Gazebo trajectory.
- D2/D4/D5 Gazebo contact force to force controller to corrected Gazebo
  trajectory.
- D2/D4/D5 force-loop settling proxy.
- D2/D4/D5 Gazebo contact physics messages.
- D4 live `/joint_states` teach/replay primitive YAML.
- D4/D5 C++ BehaviorTree.CPP runtime events.
- Runtime MoveIt PlanningScene/FCL collision sweep.

## Day-by-day result

Day 1 gripper, TF, dual arm: completed in Harmonic simulation with MoveIt2
stage `gripper_tf_dualarm`, fixed bases, scaled grippers, and recorded gripper
commands.

Day 2 force feedback and compliance: completed in Harmonic simulation with
Gazebo contact messages, contact-derived six-axis force input, admittance
controller, corrected trajectory relay, and passing settling proxy.

Day 3 vision guidance: completed in Harmonic simulation with Gazebo camera
images, ArUco target pose, IBVS twist command, and Gazebo trajectory adapter.

Day 4 weaving primitives: completed in Harmonic simulation with C++
BehaviorTree.CPP events, pick/pull/shift stages, tension topics, contact force,
and live `/joint_states` recorder YAML.

Day 5 integrated demo: completed in Harmonic simulation with vision, gripper,
force, BT, MoveIt, videos, ROS bag, metrics, and runtime collision sweep.

## Runtime collision result

Runtime MoveIt `/check_state_validity` sweep:

- samples: 263
- valid: 241
- invalid/collisions: 22
- service errors: 0
- planned trajectory invalid samples: 0

The invalid samples are extreme joint3/joint5 fold-back poses where the
attached gripper contacts Link1/Link2/Link3 on the same arm. This is a safety
finding; it is not a mathematical full-space proof.

## Limits

This is non-hardware simulation acceptance. Physical robot, gripper, force
sensor, camera calibration, yarn tension, emergency stop, repeatability, and
latency must still be validated on site.
