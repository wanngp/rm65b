# D4 real teach/replay recorder evidence

The D4 primitive evidence must be recorded from live ROS joint states while the
operator teaches the physical RM65-B arms. It must not be derived from
`dual_moveit_plans.yaml` or any other offline MoveIt playback file.

## Recorder

Use:

```bash
source rm65b_dual_arm_ws/install/setup.bash
python3 rm65b_dual_arm_ws/scripts/record_d4_real_teach_replay.py \
  --output-file outputs/on_site/day04/logs/day04_real_recorded_primitives.yaml
```

The script starts a ROS node named `d4_teach_replay_recorder`, subscribes to
`/joint_states`, and prompts the operator to record:

- `hook_yarn`
- `lift_yarn`
- `pull_tight`
- `shift`
- `exchange`

The output YAML is compatible with `weaving_coordinator` because it contains
`controllers`, `joint_names`, and `primitives`. Its metadata also includes:

- `evidence_type: real_teach_replay_primitive_yaml`
- `source: live_ros_joint_states`
- `not_offline_moveit_yaml: true`

## Service mode

For operator-controlled starts and stops from another terminal:

```bash
python3 rm65b_dual_arm_ws/scripts/record_d4_real_teach_replay.py \
  --output-file outputs/on_site/day04/logs/day04_real_recorded_primitives.yaml \
  --service-mode
```

Then for each primitive:

```bash
ros2 param set /d4_teach_replay_recorder primitive_name hook_yarn
ros2 service call /d4_teach_replay_recorder/start_recording std_srvs/srv/Trigger {}
ros2 service call /d4_teach_replay_recorder/stop_recording std_srvs/srv/Trigger {}
```

Repeat with `lift_yarn`, `pull_tight`, `shift`, and `exchange`.

Smooth and limit the recorded trajectory before final replay:

```bash
python3 rm65b_dual_arm_ws/scripts/d4_smooth_teach_trajectory.py \
  --input outputs/on_site/day04/logs/day04_real_recorded_primitives.yaml \
  --output outputs/on_site/day04/logs/day04_smoothed_primitives.yaml \
  --quality-json outputs/on_site/day04/logs/day04_trajectory_quality.json
```

## Replay

Replay the smoothed primitive YAML through the BehaviorTree.CPP runtime:

```bash
DRY_RUN=false BT_RUNTIME=cpp bash rm65b_dual_arm_ws/scripts/d4_replay_weaving_primitives.sh \
  outputs/on_site/day04/logs/day04_smoothed_primitives.yaml
```

Record a rosbag during teach and replay:

```bash
ros2 bag record -o outputs/on_site/day04/rosbags/day04_real_teach_replay \
  /joint_states \
  /weaving/events \
  /weaving/tension_n \
  /weaving/tension_status \
  /weaving/tension_pid_state \
  /weaving/compliance_offset_m \
  /safety/state \
  /left_arm_controller/follow_joint_trajectory/_action/status \
  /right_arm_controller/follow_joint_trajectory/_action/status
```

## Acceptance rule

D4 is acceptable only when the recorded YAML and replay video can be traced to
live `/joint_states` from the real teach session. A YAML generated from
`record_simulated_teach_replay.py` remains a simulation proxy and must be labeled
as such.
