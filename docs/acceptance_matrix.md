# RM65-B experiment acceptance matrix

The table maps the PDF checklist to delivered engineering artifacts and field
validation items.

| Day | Offline/simulation artifact completed | Field validation still required |
| --- | --- | --- |
| Day 1 gripper, TF, dual arm | Gripper action server, gripper config, static TF launch, RM65-B driver configs, dual-arm bringup, MoveIt2 planned `gripper_tf_dualarm` playback with fixed bases and scaled grippers attached to `Link6` | Mechanical gripper installation, open/close latency under 100 ms, TCP repeatability under +/-0.5 mm, collision-free full-stroke dual-arm motion |
| Day 2 force and compliance | Force-position topic mapping, compliance parameter YAML, safety/force monitor, MoveIt2 planned `force_pull_tight` playback, Gazebo contact-derived force estimator, corrected force trajectory relay to Gazebo, settling proxy passing | Payload calibration, gravity compensation save/restore, physical six-axis force availability, force data latency under 10 ms, force trajectory error under +/-1 mm |
| Day 3 vision guidance | ArUco target node, IBVS controller, Gazebo camera info, visual-servo Gazebo adapter, MoveIt2 planned `vision_align` and `pick_yarn` playback, Gazebo Harmonic camera topic `/left_camera/image_rect` bridged to ROS | Camera mounting, intrinsic calibration, easy_handeye2 sampling, pixel error under 5 px, world error under +/-1 mm, >=95% alignment success |
| Day 4 primitives | BehaviorTree.CPP XML and C++ runtime with interlock, tension wait, and arrival confirmation; standardized `hook_yarn`, `lift_yarn`, `pull_tight`, `shift`, `exchange` trajectory YAML; live `/joint_states` recorder; smoothing/replay scripts; Gazebo materialized loom/yarn/tension model; PID/compliance tuning record | Run the recorder on hardware, replay the smoothed primitive YAML, measure repeatability under +/-1 mm, validate real yarn tension response |
| Day 5 integration | One-launch system stack, launch audit recorder, latency probe, safety supervisor, reset/stop interfaces, MoveIt2 planned `hook_yarn`, `lift_yarn`, `pull_tight`, `shift`, and `exchange` integrated playback with Gazebo camera, vision loop, force loop, BT runtime, tension PID, and fault injection | Single launch has no missing nodes/topics; observed communication chains are under 200 ms; force/joint/collision/hardware-estop faults trigger safety stop and reset; complete video/data for the first weaving demo |

Numeric acceptance criteria require robot, gripper, sensor, camera, and
yarn/cloth fixtures.

Legacy non-hardware simulation evidence exists under
`outputs/rm65b_final_optimized_days_harmonic_20260528_0025`. Treat it as
engineering evidence only, not final field acceptance. It predates the latest
D1 and D5 report changes and does not contain the new D5 launch audit,
latency record, or safety fault-demo JSON. The current authoritative gap list is
`docs/technical_experiment_delivery_audit.md`.
