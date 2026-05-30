# BT and Full-Simulation Launch Integration

Date: 2026-05-28

## Implemented launch wiring

`rm65b_dual_arm_bringup/full_system.launch.py` starts the simulation chain when
launched with `use_hardware:=false`:

- `rm65b_vision_guidance/gazebo_camera_info_publisher` publishes
  `/left_camera/camera_info`.
- `rm65b_dual_arm_planning/force_admittance_controller` publishes
  `/force_control/*`.
- `rm65b_dual_arm_planning/gazebo_contact_force_estimator` feeds Gazebo
  contact-derived force into the simulated six-axis force topic.
- `rm65b_weaving_primitives/tension_simulator` publishes
  `/weaving/tension_n` and `/weaving/tension_status`.
- `rm65b_weaving_primitives/weaving_bt_runner` executes the XML with
  BehaviorTree.CPP v3 and publishes events on `/weaving/events`.

The launch remains hardware-gated through `use_hardware`; simulation nodes can
be disabled individually with launch arguments.

## Integrated simulation entry point

`rm65b_dual_arm_bringup/integrated_simulation.launch.py` is the one-command
simulation wrapper for generated Gazebo Harmonic assets. It starts:

- `gz sim` with a supplied day-specific world SDF.
- `ros_gz_bridge` for clock, camera images, evidence camera, gripper commands,
  contact force topics, and robot trajectory topics.
- optional left/right RM65B spawning from a supplied robot SDF.
- MoveIt `move_group`.
- Gazebo trajectory relays and force-contact estimator.
- `full_system.launch.py` with camera info, force admittance, tension, and C++
  BT runtime enabled.

## BT runtime status

The current strict runtime is the C++ `weaving_bt_runner` node, backed by
BehaviorTree.CPP v3. It converts the project XML where needed and emits runtime
events such as:

- `bt_runtime:cpp_behavior_tree_cpp_v3`
- `bt_xml_compat:btcpp_v4_to_v3`
- `bt_action_start:<action>`
- `bt_action_success:<action>`
- `bt_tree_complete:success`

Latest D4/D5 evidence records `/weaving/events` counts of day04 40 and day05
103.

## Remaining limitations

- This is simulation runtime evidence. Hardware execution still requires
  physical gripper, force sensor, yarn, safety, and calibration validation.
- Runtime collision validation is sampled; see
  `offline_analysis/runtime_collision_sweep_summary.md` in the latest output
  directory for the PlanningScene/FCL report and the extreme-pose self-collision
  hazards found by the sweep.
