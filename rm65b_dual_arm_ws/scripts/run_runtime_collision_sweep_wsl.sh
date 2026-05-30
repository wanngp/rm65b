#!/usr/bin/env bash
set -u

ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID_VALUE:-77}"
OUTPUT_ROOT_NAME="${OUTPUT_ROOT_NAME:-rm65b_final_optimized_days_harmonic_20260528_0025}"

repo_root="$(find /mnt/e -maxdepth 3 -type d -name rm65b_dual_arm_ws -printf '%h\n' 2>/dev/null | head -n 1)"
if [[ -z "${repo_root}" ]]; then
  echo "repo_not_found"
  exit 3
fi

ln -sfn "${repo_root}" /tmp/rm65b_repo_current
repo_root="/tmp/rm65b_repo_current"
out_dir="${repo_root}/outputs/${OUTPUT_ROOT_NAME}/offline_analysis/runtime_collision_sweep"
plan_yaml="${repo_root}/outputs/${OUTPUT_ROOT_NAME}/day05/logs/dual_moveit_plans.yaml"

mkdir -p "${out_dir}"

set +u
source /opt/ros/humble/setup.bash
source /tmp/rm65b_dual_arm_ws_ascii_verify2/install/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}"
unset ROS_LOCALHOST_ONLY

ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

ros2 launch rm65b_dual_arm_moveit_config move_group.launch.py \
  allow_trajectory_execution:=false \
  publish_monitored_planning_scene:=true \
  > "${out_dir}/move_group_runtime_sweep.log" 2>&1 &
move_group_pid=$!

{
  echo "repo=${repo_root}"
  echo "ros_domain_id=${ROS_DOMAIN_ID}"
  echo "move_group_pid=${move_group_pid}"
} > "${out_dir}/runtime_collision_sweep_run.log"

cleanup() {
  if kill -0 "${move_group_pid}" 2>/dev/null; then
    kill "${move_group_pid}" 2>/dev/null || true
    wait "${move_group_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

found=0
for _ in $(seq 1 60); do
  ros2 service list > "${out_dir}/ros2_services_latest.txt" 2>/dev/null || true
  if grep -q "/check_state_validity" "${out_dir}/ros2_services_latest.txt"; then
    found=1
    break
  fi
  sleep 1
done

if [[ "${found}" -ne 1 ]]; then
  echo "check_state_validity_not_found" | tee -a "${out_dir}/runtime_collision_sweep_run.log"
  ros2 node list > "${out_dir}/ros2_nodes_on_failure.txt" 2>&1 || true
  ros2 service list > "${out_dir}/ros2_services_on_failure.txt" 2>&1 || true
  tail -80 "${out_dir}/move_group_runtime_sweep.log" | tee -a "${out_dir}/runtime_collision_sweep_run.log"
  exit 2
fi

{
  echo "services_matching:"
  grep "check_state_validity" "${out_dir}/ros2_services_latest.txt"
} | tee -a "${out_dir}/runtime_collision_sweep_run.log"

python3 "${repo_root}/rm65b_dual_arm_ws/scripts/sweep_runtime_moveit_collision.py" \
  --output-dir "${out_dir}" \
  --plan-yaml "${plan_yaml}" \
  --plan-stride 5 \
  --wait-timeout 60 \
  --call-timeout 10 \
  --progress-every 50 2>&1 | tee -a "${out_dir}/runtime_collision_sweep_run.log"
status=${PIPESTATUS[0]}
echo "sweep_status=${status}" | tee -a "${out_dir}/runtime_collision_sweep_run.log"
exit "${status}"
