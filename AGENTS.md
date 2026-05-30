# Project Agent Instructions

## Project Purpose

This repository supports the RealMan RM65-B dual-arm five-day weaving
experiment. The current deliverable is an engineering workspace with ROS 2
Humble packages, MoveIt2/Gazebo simulation, experiment scripts, reports, and
field-recording procedures.

Do not present simulation evidence as hardware acceptance. Field acceptance
still requires real robot, gripper, force sensor, camera, yarn/material,
latency, repeatability, emergency-stop, video, rosbag, JSON, and CSV evidence.

## Environment

- Target runtime: VMware Ubuntu 22.04 with ROS 2 Humble.
- Main bootstrap entrypoint:
  `rm65b_dual_arm_ws/scripts/bootstrap_vmware_ubuntu.sh`.
- Windows PowerShell bootstrap is only a host-side helper:
  `rm65b_dual_arm_ws/scripts/bootstrap_official_sources.ps1`.
- If the official `rm_ros_interfaces` build fails because of a non-ASCII path,
  copy the workspace to an ASCII-only path before a full `colcon build`.
- The official RealMan source under `rm65b_dual_arm_ws/src/ros2_rm_robot` is
  third-party input managed by the bootstrap script.

## Git And Files

- Keep the repository root under Git control.
- Check `git status --short` before and after edits.
- Keep commits small and scoped to one task.
- Do not track generated artifacts, build outputs, downloaded archives, rosbags,
  videos, copied delivery bundles, or local collection directories.
- Update `.gitignore` when new generated directories or large artifacts appear.
- Do not use destructive Git commands such as `git reset --hard` or
  `git checkout --` unless explicitly requested.

## Build And Verification

Preferred bootstrap in the VMware guest:

```bash
cd ~/rm65b_dual_arm_ws
bash scripts/bootstrap_vmware_ubuntu.sh
```

Install missing apt dependencies only when explicitly requested:

```bash
bash scripts/bootstrap_vmware_ubuntu.sh --install-apt-deps
```

Run offline verification when Python dependencies are available:

```bash
python3 scripts/verify_project.py
```

Full build sequence:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rm_ros_interfaces
source install/setup.bash
colcon build --symlink-install
```

## Long-Running Work

- Before long training, export, playback, recording, or evaluation jobs, split
  read-only investigation and independent verification when possible.
- Run non-interactive long jobs in the background with explicit stdout/stderr
  logs, then continue non-conflicting work while polling progress.
- Do not finish while a required job is still running unless the user asked to
  leave it running and you report status, log paths, and remaining work.
- When reporting results, include output directory, key metrics, skipped
  sessions, bad time axes, missing references, and low-observability windows.

## Reporting Rules

- Always distinguish:
  - completed offline/simulation evidence
  - completed software/configuration work
  - pending field/hardware validation
- Use `docs/technical_experiment_delivery_audit.md` as the current gap matrix.
- Legacy non-hardware evidence is useful for engineering context, but is not
  final field acceptance.
