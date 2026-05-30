#!/usr/bin/env python3
from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path


INITIAL_POSITIONS = (0.0, -0.35, 0.65, 0.0, 0.90, 0.0)
TOOL_COLOR = "0.16 0.22 0.28 1"
FINGER_COLOR = "0.05 0.06 0.07 1"
PROBE_COLOR = "0.95 0.82 0.18 1"


def text(parent: ET.Element, tag: str, value: str) -> ET.Element:
    elem = ET.SubElement(parent, tag)
    elem.text = value
    return elem


def add_inertial(parent: ET.Element, mass: str = "0.050") -> None:
    inertial = ET.SubElement(parent, "inertial")
    text(inertial, "pose", "0 0 0 0 0 0")
    text(inertial, "mass", mass)
    inertia = ET.SubElement(inertial, "inertia")
    for tag, value in {
        "ixx": "0.000020",
        "ixy": "0",
        "ixz": "0",
        "iyy": "0.000020",
        "iyz": "0",
        "izz": "0.000020",
    }.items():
        text(inertia, tag, value)


def add_box(parent: ET.Element, tag: str, name: str, pose: str, size: str, color: str | None = None) -> None:
    elem = ET.SubElement(parent, tag, {"name": name})
    text(elem, "pose", pose)
    geometry = ET.SubElement(elem, "geometry")
    box = ET.SubElement(geometry, "box")
    text(box, "size", size)
    if color is not None:
        material = ET.SubElement(elem, "material")
        text(material, "ambient", color)
        text(material, "diffuse", color)


def add_box_visual(parent: ET.Element, name: str, pose: str, size: str, color: str, collision: bool = True) -> None:
    add_box(parent, "visual", name, pose, size, color)
    if collision:
        add_box(parent, "collision", f"{name}_collision", pose, size)


def add_position_controller(model: ET.Element, joint_name: str, topic: str) -> None:
    plugin = ET.SubElement(
        model,
        "plugin",
        {
            "filename": "gz-sim-joint-position-controller-system",
            "name": "gz::sim::systems::JointPositionController",
        },
    )
    text(plugin, "joint_name", joint_name)
    text(plugin, "topic", topic)
    text(plugin, "p_gain", "80")
    text(plugin, "i_gain", "0.1")
    text(plugin, "d_gain", "2.0")


def add_gripper(model: ET.Element) -> None:
    if model.find("link[@name='Link6']") is None:
        return

    palm = ET.SubElement(model, "link", {"name": "attached_scaled_gripper_palm"})
    text(palm, "pose", "0.018 0 0 0 0 0").set("relative_to", "Link6")
    text(palm, "gravity", "false")
    add_inertial(palm)
    add_box_visual(palm, "flange_adapter", "0 0 0 0 0 0", "0.038 0.038 0.012", TOOL_COLOR)
    add_box_visual(palm, "palm", "0.030 0 0 0 0 0", "0.040 0.026 0.018", TOOL_COLOR)

    for name, y in (
        ("attached_scaled_gripper_upper_finger", 0.006),
        ("attached_scaled_gripper_lower_finger", -0.006),
    ):
        finger = ET.SubElement(model, "link", {"name": name})
        text(finger, "pose", f"0.070 {y:.3f} 0 0 0 0").set("relative_to", "attached_scaled_gripper_palm")
        text(finger, "gravity", "false")
        add_inertial(finger)
        add_box_visual(finger, "finger", "0 0 0 0 0 0", "0.052 0.006 0.010", FINGER_COLOR)
        add_box_visual(finger, "yarn_hook", "0.028 0 0 0 0 0", "0.014 0.014 0.010", FINGER_COLOR)

    fixed = ET.SubElement(model, "joint", {"name": "attached_scaled_gripper_fixed", "type": "fixed"})
    text(fixed, "parent", "Link6")
    text(fixed, "child", "attached_scaled_gripper_palm")

    upper = ET.SubElement(model, "joint", {"name": "gripper_upper_slide", "type": "prismatic"})
    text(upper, "parent", "attached_scaled_gripper_palm")
    text(upper, "child", "attached_scaled_gripper_upper_finger")
    axis = ET.SubElement(upper, "axis")
    text(axis, "xyz", "0 1 0")
    limit = ET.SubElement(axis, "limit")
    text(limit, "lower", "0.000")
    text(limit, "upper", "0.018")

    lower = ET.SubElement(model, "joint", {"name": "gripper_lower_slide", "type": "prismatic"})
    text(lower, "parent", "attached_scaled_gripper_palm")
    text(lower, "child", "attached_scaled_gripper_lower_finger")
    axis = ET.SubElement(lower, "axis")
    text(axis, "xyz", "0 -1 0")
    limit = ET.SubElement(axis, "limit")
    text(limit, "lower", "0.000")
    text(limit, "upper", "0.018")

    add_position_controller(model, "gripper_upper_slide", "/rm65b_gripper/upper_finger_cmd")
    add_position_controller(model, "gripper_lower_slide", "/rm65b_gripper/lower_finger_cmd")


def add_force_probe(model: ET.Element) -> None:
    if model.find("link[@name='attached_scaled_gripper_palm']") is None:
        return
    if model.find("link[@name='d2_force_probe_tip']") is not None:
        return

    probe = ET.SubElement(model, "link", {"name": "d2_force_probe_tip"})
    text(probe, "pose", "0.135 0 0 0 0 0").set("relative_to", "attached_scaled_gripper_palm")
    text(probe, "gravity", "false")
    add_inertial(probe, "0.035")
    add_box_visual(probe, "probe_tip", "0 0 0 0 0 0", "0.032 0.032 0.032", PROBE_COLOR)

    fixed = ET.SubElement(model, "joint", {"name": "d2_force_probe_tip_fixed", "type": "fixed"})
    text(fixed, "parent", "attached_scaled_gripper_palm")
    text(fixed, "child", "d2_force_probe_tip")


def add_joint_trajectory_controller(model: ET.Element) -> None:
    plugin = ET.SubElement(
        model,
        "plugin",
        {
            "filename": "gz-sim-joint-trajectory-controller-system",
            "name": "gz::sim::systems::JointTrajectoryController",
        },
    )
    for idx, initial in enumerate(INITIAL_POSITIONS, start=1):
        text(plugin, "joint_name", f"joint{idx}")
        text(plugin, "initial_position", f"{initial:.6f}")
        text(plugin, "position_p_gain", "45")
        text(plugin, "position_i_gain", "0.1")
        text(plugin, "position_d_gain", "1.5")
        text(plugin, "position_i_min", "-1")
        text(plugin, "position_i_max", "1")
        text(plugin, "position_cmd_min", "-80")
        text(plugin, "position_cmd_max", "80")


def prepare(src: Path, dst: Path, force_probe: bool = False) -> None:
    tree = ET.parse(src)
    root = tree.getroot()
    model = root.find("model")
    if model is None:
        raise RuntimeError(f"{src} has no SDF model")

    model.set("name", "rm65b_joint_sweep_demo")
    static = model.find("static")
    if static is None:
        static = ET.Element("static")
        model.insert(0, static)
    static.text = "false"

    for link in model.findall("link"):
        gravity = link.find("gravity")
        if gravity is None:
            gravity = ET.SubElement(link, "gravity")
        gravity.text = "false"
        for collision in list(link.findall("collision")):
            link.remove(collision)

    world_joint = ET.Element("joint", {"name": "world_fixed_base", "type": "fixed"})
    text(world_joint, "parent", "world")
    text(world_joint, "child", "base_link")
    model.insert(1, world_joint)

    add_gripper(model)
    if force_probe:
        add_force_probe(model)
    add_joint_trajectory_controller(model)
    tree.write(dst, encoding="utf-8", xml_declaration=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--force-probe", action="store_true")
    args = parser.parse_args()
    prepare(Path(args.source), Path(args.output), force_probe=args.force_probe)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
