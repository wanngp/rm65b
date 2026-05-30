# Offline evidence analysis tools

Added scripts under `rm65b_dual_arm_ws/scripts`:

- `analyze_daily_evidence.py`: finds the latest D1-D5 output and writes `offline_analysis/daily_evidence_report.json` plus `.csv` with topic counts, camera rates, force-loop proxies, trajectory point counts, latency proxies, and media checks.
- `sweep_static_collision_consistency.py`: writes `offline_analysis/static_collision_sweep_report.json` plus sample CSV from URDF/SRDF collision matrix and planned-point limit/time consistency checks.
- `record_simulated_teach_replay.py`: writes D4 `logs/day04_simulated_teach_replay_evidence.yaml` from planned points as explicitly simulated teach/replay evidence.
- `record_d4_real_teach_replay.py`: on-site ROS node that subscribes to live `/joint_states` during teach-pendant motion and writes D4 primitive YAML for real replay through `weaving_coordinator`.
- `daily_acceptance_video_manifest.py`: emits the D1-D5 video shot list and required ROS evidence topics.
- `sweep_runtime_moveit_collision.py`: calls MoveIt `/check_state_validity`
  for sampled states so collision truth comes from the running
  PlanningScene/FCL checker.
- `run_runtime_collision_sweep_wsl.sh`: WSL wrapper that starts a private
  `move_group`, runs the runtime sweep, and shuts down only its own process.

Example:

```bash
python rm65b_dual_arm_ws/scripts/analyze_daily_evidence.py
python rm65b_dual_arm_ws/scripts/sweep_static_collision_consistency.py
python rm65b_dual_arm_ws/scripts/record_simulated_teach_replay.py
python rm65b_dual_arm_ws/scripts/daily_acceptance_video_manifest.py
wsl -d Ubuntu-22.04-ROS2 -- bash /mnt/e/1-*/睿尔曼/rm65b_dual_arm_ws/scripts/run_runtime_collision_sweep_wsl.sh
```

The offline reports are proxies: they do not replace hardware force timing,
real teach-pendant captures, or continuous-space collision proof. D4 field
acceptance should use `record_d4_real_teach_replay.py` on hardware; the latest
non-hardware run already includes a simulation live `/joint_states` recorder
YAML and a runtime FCL sampled sweep.
