# RM65-B strict gap audit

Date: 2026-05-28

Status note, updated 2026-05-30: this is a legacy non-hardware gap audit. It
does not include the latest D1 gripper-only scope or the D5 launch audit,
latency record, safety fault-demo JSON, and first weaving Demo evidence. Use
`docs/technical_experiment_delivery_audit.md` as the current checklist.

This document compares the experiment checklist against the current workspace
without treating simulation evidence as hardware acceptance.

## Bottom line

Legacy non-hardware simulation/software evidence was complete for the earlier
scope in this evidence set:

```text
outputs/rm65b_final_optimized_days_harmonic_20260528_0025
```

The generated pass/fail report says `PASS` with `failed=0`:

```text
outputs/rm65b_final_optimized_days_harmonic_20260528_0025/offline_analysis/nonhardware_pass_fail_metrics.md
```

This is still not full hardware acceptance. Physical robot, gripper, force
sensor, camera calibration, yarn/material tuning, emergency stop, and measured
latency/repeatability remain excluded hardware work.

## Non-Hardware Items Now Covered

- Gazebo Harmonic D1-D5 playback videos, screenshots, logs, and ROS bags.
- Day-specific D1-D5 Gazebo worlds.
- Fixed-base dual-arm RM65-B simulation with scaled grippers attached to Link6.
- True dual-arm MoveIt planning through group `dual_arms`.
- Gazebo camera `/left_camera/image_rect` driving ArUco target pose and IBVS.
- IBVS twist output consumed by Gazebo trajectory adapter.
- Gazebo contact-derived force feeding the simulated force controller.
- Corrected force trajectory written back to the right Gazebo arm controller.
- D2/D4/D5 contact physics topics with nonzero messages.
- Movable Gazebo gripper command topics recorded on all days.
- C++ BehaviorTree.CPP runtime for D4/D5 weaving events.
- D4 teach/replay primitive YAML recorded from live `/joint_states`.
- One-launch simulation wiring through `integrated_simulation.launch.py` and
  `full_system.launch.py`.
- Runtime MoveIt PlanningScene/FCL collision sweep through
  `/check_state_validity`.

## Day-by-day strict comparison

| Day | Requirement area | Current non-hardware evidence | Strict status |
| --- | --- | --- | --- |
| D1 | Gripper, TF, dual-arm planning | Gazebo gripper commands, fixed bases, dual-arm MoveIt plan/playback, media evidence | Simulation complete; hardware calibration pending |
| D2 | Force/compliance | Gazebo contact topics feed force estimator, admittance loop publishes corrected trajectory to Gazebo, settling proxy passes | Simulation complete; hardware force mode/calibration pending |
| D3 | Vision guidance | Gazebo camera drives ArUco target pose, IBVS, and robot command adapter | Simulation complete; physical camera/hand-eye metrics pending |
| D4 | Weaving primitives and teach/replay | BehaviorTree.CPP C++ runner records events; live `/joint_states` recorder wrote D4 primitive YAML | Simulation complete; hardware teach/replay repeatability pending |
| D5 | Integrated demo | Vision, gripper, force, BT, MoveIt, Gazebo worlds, videos, metrics, and runtime collision report are present | Simulation complete; hardware demo and safety validation pending |

## Runtime Collision Finding

The runtime MoveIt sweep checked 263 sampled states with
`service_error_count=0`. It found 22 invalid samples at extreme joint3/joint5
fold-back poses where the attached gripper contacts Link1/Link2/Link3 on the
same arm. D1-D5 planned trajectory samples are valid.

This is a strict finding, not a hidden pass. It means MoveIt can detect and
reject those unsafe extreme poses; it does not prove the continuous 12-DOF
workspace is mathematically collision-free.

## Remaining Hardware Work

1. Install and test the physical gripper backend, including latency and stall
   behavior.
2. Measure base, TCP, and camera transforms; produce repeatability reports.
3. Validate RealMan force mode, six-axis force latency, gravity compensation,
   and compliance tracking on hardware.
4. Calibrate camera intrinsics and hand-eye; measure fps, pixel error,
   world-frame error, and IBVS success rate.
5. Run D4 teach/replay on hardware and measure repeatability.
6. Run real yarn/cloth tension and contact-force experiments.
7. Run D1-D5 on hardware with rosbag/video, emergency stop, reset, and latency
   evidence.
