# D1-D5 acceptance video scripts

Each daily video should show the physical task, the ROS evidence, and the
measured acceptance signal in the same recording window. This file is a field
recording plan, not proof that the measurements already exist. Keep failures,
retries, and safety stops in the video instead of editing them out.

Generate the same manifest from code with:

```bash
python3 rm65b_dual_arm_ws/scripts/daily_acceptance_video_manifest.py
```

## D1 gripper, TF, dual arm

Must show:

- Both RM65-B arms and mounted grippers in one wide shot.
- RViz TF tree or frame overlay for base, Link6, and gripper TCP frames.
- Left and right gripper open/close commands with visible completion.
- Dual-arm cooperative motion with no inter-arm collision.

Record ROS evidence:

- `/tf`
- `/tf_static`
- `/joint_states`
- left and right gripper action/status topics
- `/move_action/_action/status`

Acceptance signal: measured TCP/TF values, gripper latency, and synchronized
dual-arm motion are visible or logged.

## D2 force-control contact

Must show:

- Force sensor zeroing or gravity compensation state.
- Physical contact between the end effector and the fixture or yarn target.
- Force trace, wrench error, and admittance offset during contact.
- Robot settling inside the target force window or safely backing off.

Record ROS evidence:

- `/right_rm_driver/rm_driver/udp_six_force`
- `/force_control/wrench_error`
- `/force_control/admittance_offset`
- `/force_control/state`
- `/safety/state`

Acceptance signal: physical contact is visible and the force window result is
reported with any overshoot or safety stop retained.

## D3 vision IBVS

Must show:

- Camera image with marker, yarn edge, or target overlay.
- Detection fps and pixel error trend.
- Camera or hand-eye frame relationship in RViz.
- Robot motion driven by visual servo output until final alignment.

Record ROS evidence:

- `/left_camera/image_rect`
- `/left_camera/camera_info`
- `/vision/target_pose`
- `/vision/metrics`
- `/visual_servo/twist_cmd`

Acceptance signal: initial and final pixel/world error are both shown.

## D4 weaving teach/replay

Must show:

- Operator teaching `hook_yarn`, `lift_yarn`, `pull_tight`, `shift`, and `exchange`.
- `record_d4_real_teach_replay.py` receiving `/joint_states`.
- The generated YAML metadata with
  `evidence_type: real_teach_replay_primitive_yaml`.
- `d4_smooth_teach_trajectory.py` producing the smoothed replay YAML and quality JSON.
- BehaviorTree.CPP replay with `/weaving/events` visible.
- Yarn tension PID/compliance response during `pull_tight`.

Record command:

```bash
python3 rm65b_dual_arm_ws/scripts/record_d4_real_teach_replay.py \
  --output-file outputs/on_site/day04/logs/day04_real_recorded_primitives.yaml \
  --primitive hook_yarn --primitive lift_yarn --primitive pull_tight --primitive shift --primitive exchange
```

Replay command:

```bash
python3 rm65b_dual_arm_ws/scripts/d4_smooth_teach_trajectory.py \
  --input outputs/on_site/day04/logs/day04_real_recorded_primitives.yaml \
  --output outputs/on_site/day04/logs/day04_smoothed_primitives.yaml \
  --quality-json outputs/on_site/day04/logs/day04_trajectory_quality.json

DRY_RUN=false BT_RUNTIME=cpp bash rm65b_dual_arm_ws/scripts/d4_replay_weaving_primitives.sh \
  outputs/on_site/day04/logs/day04_smoothed_primitives.yaml
```

Acceptance signal: D4 YAML comes from live `/joint_states`, not
`dual_moveit_plans.yaml`; replay uses the recorded YAML path and repeatability is
measured from teach vs replay traces.

## D5 integrated demo

Must show:

- The `integrated_simulation.launch.py` or `full_system.launch.py` command starting
  the system from a clean terminal.
- The node graph audit (`/system/launch_audit`) and generated
  `outputs/d5_system/day05_node_graph.json`.
- The latency record `outputs/d5_system/day05_latency_record.json`, with no
  missing chains and all chains below 200 ms.
- Fault injection or hardware estop evidence: `/safety/fault`, `/safety/estop`,
  `/safety/reset_required`, and reset through recover-home, ack, reset.
- The first weaving demo chain: vision target, dual-arm motion, gripper/yarn
  action, weaving primitive events, and tension pull-tight telemetry.

Record ROS evidence:

- `/acceptance/day_status`
- `/system/launch_audit`
- `/weaving/events`
- `/weaving/tension_n`
- `/weaving/tension_pid_state`
- `/vision/target_pose`
- `/visual_servo/twist_cmd`
- `/force_control/state`
- `/force_control/wrench_error`
- `/safety/state`
- `/safety/fault`
- `/safety/estop`
- `/safety/reset_required`
- `/joint_states`

Acceptance signal: the single launch has no missing nodes/topics, measured
latency has no missing chains and stays below 200 ms, at least one safety fault triggers
and resets correctly, and the final video shows the integrated vision-force-
weaving closed loop rather than isolated arm motion.
