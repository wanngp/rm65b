# Runtime MoveIt Collision Sweep

`sweep_runtime_moveit_collision.py` runs after `move_group` is up and queries
MoveIt's `/check_state_validity` service for sampled dual-arm states. URDF/SRDF
files are used only to discover joint names, joint limits, and seed states; the
collision result comes from the running PlanningScene/FCL stack.

## Start MoveIt

```bash
cd /path/to/rm65b_dual_arm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rm65b_dual_arm_moveit_config move_group.launch.py allow_trajectory_execution:=false
```

Use the same `ROS_DOMAIN_ID` in the terminal that runs the sweep.

## Run Sweep

```bash
cd /path/to/repo
source /opt/ros/humble/setup.bash
source rm65b_dual_arm_ws/install/setup.bash
python3 rm65b_dual_arm_ws/scripts/sweep_runtime_moveit_collision.py \
  --output-dir outputs/runtime_collision_sweep
```

Optional planned trajectory validation:

```bash
python3 rm65b_dual_arm_ws/scripts/sweep_runtime_moveit_collision.py \
  --output-dir outputs/runtime_collision_sweep \
  --plan-yaml outputs/<run>/day05/logs/dual_moveit_plans.yaml \
  --plan-stride 5
```

## Outputs

- `runtime_collision_sweep_report.json`: summary, seed state, limits, scenario
  counts, first invalid/collision samples, and contact-pair counts.
- `runtime_collision_sweep_samples.csv`: one row per checked state with
  validity, contact count, contact pairs, varied joints, positions JSON, and
  contacts JSON.

## Coverage

Default sampling includes:

- the SRDF `dual_ready` seed when available;
- every active dual-arm joint swept across its full URDF limit range while the
  other joints stay at the seed state;
- left/right same-index joint pair grids for mutual collision exposure.

Use `--samples-per-joint`, `--samples-per-pair`, and `--random-samples` to
increase coverage. This is a sampled runtime sweep, not a mathematical proof
over the continuous 12-DOF configuration space.
