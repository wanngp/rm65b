# RM65-B dual-arm experiment completion report

Date: 2026-05-28

Status note, updated 2026-05-30: this is a non-hardware engineering completion
report, not a final field acceptance certificate. The current authoritative
checklist, missing evidence, and corrected wording are in
`docs/technical_experiment_delivery_audit.md`.

## Summary

The non-hardware engineering work for the five-day RM65-B dual-arm experiment
list is complete in simulation. The latest acceptance evidence is:

```text
outputs/rm65b_final_optimized_days_harmonic_20260528_0025
```

`nonhardware_pass_fail_metrics.json` reports `PASS` with `failed=0`.

This does not claim physical hardware acceptance. Real RM65-B arms, grippers,
force sensor, camera calibration, yarn/materials, emergency stop, latency, and
repeatability still require on-site validation.

## Completed simulation artifacts

- Official RealMan ROS 2 Humble source imported under
  `rm65b_dual_arm_ws/src/ros2_rm_robot`.
- Dual RM65-B driver configs for left and right arms using `arm_type: RM_65`.
- Static TF/TCP configuration and launch.
- Gripper action server and Gazebo gripper command topics.
- Safety supervisor for joint/force/watchdog faults and RealMan stop topics.
- Gazebo camera, ArUco target node, IBVS controller, and Gazebo trajectory
  adapter.
- Gazebo contact-force estimator feeding the simulated six-axis force topic.
- Admittance controller publishing corrected right-arm trajectory back to
  Gazebo.
- C++ BehaviorTree.CPP runtime for D4/D5 weaving events.
- D4 live `/joint_states` recorder output:
  `day04/logs/day04_recorded_primitives_from_live_joint_states.yaml`.
- Dual-arm MoveIt2 configuration with prefixed left/right RM65-B chains, fixed
  bases, attached scaled grippers, and group `dual_arms`.
- Full-system and integrated simulation launch files.
- D1-D5 Gazebo Harmonic playback evidence with day-specific worlds, videos,
  screenshots, logs, ROS bags, and metrics.
- Runtime MoveIt PlanningScene/FCL collision sweep report.

## Runtime collision finding

The runtime sweep used MoveIt `/check_state_validity` and completed with
`service_error_count=0`. It checked 263 sampled states:

- valid samples: 241
- invalid/collision samples: 22
- planned trajectory invalid samples: 0

The invalid samples are extreme joint3/joint5 fold-back poses where the
attached gripper contacts Link1/Link2/Link3 on the same arm. This is a recorded
safety finding, not a hidden pass or a full continuous-space proof.

## Validation

Static verification:

```bash
python rm65b_dual_arm_ws/scripts/verify_project.py
```

Result:

- Required files present: 23
- YAML parsed: 22
- XML parsed: 9
- Python syntax parsed: 50
- Semantic checks passed

Non-hardware metrics:

```bash
python rm65b_dual_arm_ws/scripts/generate_nonhardware_pass_fail.py --output-root outputs/rm65b_final_optimized_days_harmonic_20260528_0025
```

Result:

```text
status=PASS
failed=0
```

## Hardware limitations

- No physical gripper integration or measured open/close latency.
- No measured base/TCP/camera calibration or repeatability.
- No physical RealMan force-mode execution, force latency, gravity
  compensation, or compliance calibration.
- No physical camera hand-eye/pixel/world-error measurement.
- No real yarn/cloth tension calibration.
- No physical emergency-stop/reset validation.
