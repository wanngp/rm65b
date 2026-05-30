#!/usr/bin/env python3
"""Offline project verification for the RM65-B experiment workspace."""

from __future__ import annotations

import ast
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

try:
    import yaml
except Exception:  # pragma: no cover - dependency check path
    yaml = None


ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parent

REQUIRED = [
    "src/ros2_rm_robot/rm_driver/config/rm_65_config.yaml",
    "src/ros2_rm_robot/rm_bringup/launch/rm_65_bringup.launch.py",
    "src/ros2_rm_robot/rm_bringup/launch/rm_65_gazebo.launch.py",
    "src/rm65b_dual_arm_bringup/launch/full_system.launch.py",
    "src/rm65b_dual_arm_bringup/config/dual_arm_frames.yaml",
    "src/rm65b_dual_arm_moveit_config/config/rm65b_dual_arm.urdf",
    "src/rm65b_dual_arm_moveit_config/config/rm65b_dual_arm.srdf",
    "src/rm65b_dual_arm_moveit_config/launch/move_group.launch.py",
    "src/rm65b_dual_arm_planning/rm65b_dual_arm_planning/force_admittance_controller.py",
    "src/rm65b_dual_arm_planning/rm65b_dual_arm_planning/gazebo_contact_force_estimator.py",
    "src/rm65b_dual_arm_planning/rm65b_dual_arm_planning/gazebo_trajectory_relays.py",
    "src/rm65b_gripper_control/rm65b_gripper_control/gripper_action_server.py",
    "src/rm65b_safety/rm65b_safety/safety_supervisor.py",
    "src/rm65b_vision_guidance/rm65b_vision_guidance/aruco_target_node.py",
    "src/rm65b_vision_guidance/rm65b_vision_guidance/gazebo_camera_info_publisher.py",
    "src/rm65b_weaving_primitives/config/weaving_tree.xml",
    "src/rm65b_weaving_primitives/rm65b_weaving_primitives/tension_simulator.py",
    "src/rm65b_weaving_primitives/trajectories/weaving_primitives.yaml",
    "scripts/generate_day_world.py",
    "scripts/hardware_acceptance_check.py",
    "../docs/acceptance_matrix.md",
    "../docs/on_site_checklist.md",
    "../docs/strict_gap_audit.md",
]


def fail(message: str) -> None:
    print(f"[FAIL] {message}")
    raise SystemExit(1)


def check_required() -> None:
    missing = [item for item in REQUIRED if not (ROOT / item).exists()]
    if missing:
        fail("missing required files:\n  " + "\n  ".join(missing))
    print(f"[OK] required files present: {len(REQUIRED)}")


def check_yaml() -> None:
    if yaml is None:
        fail("PyYAML is not available; install python3-yaml or pyyaml")
    files = list(ROOT.glob("src/rm65b_*/**/*.yaml")) + list(
        REPO.glob("experiments/day*/config/*.yaml")
    )
    for path in files:
        with path.open("r", encoding="utf-8") as handle:
            yaml.safe_load(handle)
    print(f"[OK] YAML parsed: {len(files)}")


def check_xml() -> None:
    files = list(ROOT.glob("src/rm65b_*/**/*.xml")) + list(ROOT.glob("src/rm65b_*/**/*.srdf"))
    for path in files:
        ET.parse(path)
    print(f"[OK] XML parsed: {len(files)}")


def check_python_syntax() -> None:
    files = [
        path
        for path in ROOT.glob("src/rm65b_*/**/*.py")
        if "build" not in path.parts and "install" not in path.parts
    ]
    files.extend(ROOT.glob("scripts/*.py"))
    for path in files:
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    print(f"[OK] Python syntax parsed: {len(files)}")


def check_semantics() -> None:
    frames = yaml.safe_load(
        (ROOT / "src/rm65b_dual_arm_bringup/config/dual_arm_frames.yaml").read_text(
            encoding="utf-8"
        )
    )
    transforms = frames.get("transforms", [])
    children = {item["child_frame_id"] for item in transforms}
    for required in {"left_base_link", "right_base_link", "left_tcp", "right_tcp"}:
        if required not in children:
            fail(f"required TF child frame not configured: {required}")

    safety = yaml.safe_load(
        (ROOT / "src/rm65b_dual_arm_bringup/config/safety_limits.yaml").read_text(
            encoding="utf-8"
        )
    )
    if safety["safety"]["max_force_n"] <= 0:
        fail("safety.max_force_n must be positive")

    print("[OK] semantic checks passed")


def main() -> int:
    print(f"Workspace: {ROOT}")
    check_required()
    check_yaml()
    check_xml()
    check_python_syntax()
    check_semantics()
    print("[OK] offline verification complete")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
