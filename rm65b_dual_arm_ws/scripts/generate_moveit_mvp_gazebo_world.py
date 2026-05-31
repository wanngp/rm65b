#!/usr/bin/env python3
from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path


DUAL_JOINTS = [f"{side}_joint{i}" for side in ("left", "right") for i in range(1, 7)]
HOME = {
    "left_joint1": 0.0,
    "left_joint2": -0.35,
    "left_joint3": 0.65,
    "left_joint4": 0.0,
    "left_joint5": 0.90,
    "left_joint6": 0.0,
    "right_joint1": 0.0,
    "right_joint2": -0.35,
    "right_joint3": 0.65,
    "right_joint4": 0.0,
    "right_joint5": 0.90,
    "right_joint6": 0.0,
}


def text(parent: ET.Element, tag: str, value: str) -> ET.Element:
    elem = ET.SubElement(parent, tag)
    elem.text = value
    return elem


def indent_xml(elem: ET.Element, level: int = 0) -> None:
    indent = "\n" + level * "  "
    child_indent = "\n" + (level + 1) * "  "
    children = list(elem)
    if children:
        if not elem.text or not elem.text.strip():
            elem.text = child_indent
        for child in children:
            indent_xml(child, level + 1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = indent
    elif level and (not elem.tail or not elem.tail.strip()):
        elem.tail = indent


def add_box_model(
    world: ET.Element,
    name: str,
    pose: str,
    size: str,
    color: str,
    *,
    collision: bool = True,
    static: bool = True,
) -> None:
    model = ET.SubElement(world, "model", {"name": name})
    text(model, "pose", pose)
    text(model, "static", "true" if static else "false")
    link = ET.SubElement(model, "link", {"name": "link"})
    visual = ET.SubElement(link, "visual", {"name": "visual"})
    geometry = ET.SubElement(visual, "geometry")
    box = ET.SubElement(geometry, "box")
    text(box, "size", size)
    material = ET.SubElement(visual, "material")
    text(material, "ambient", color)
    text(material, "diffuse", color)
    if collision:
        coll = ET.SubElement(link, "collision", {"name": "collision"})
        coll_geometry = ET.SubElement(coll, "geometry")
        coll_box = ET.SubElement(coll_geometry, "box")
        text(coll_box, "size", size)


def add_marker_cylinder(world: ET.Element, name: str, pose: str, radius: str, length: str, color: str) -> None:
    model = ET.SubElement(world, "model", {"name": name})
    text(model, "pose", pose)
    text(model, "static", "true")
    link = ET.SubElement(model, "link", {"name": "link"})
    visual = ET.SubElement(link, "visual", {"name": "visual"})
    geometry = ET.SubElement(visual, "geometry")
    cylinder = ET.SubElement(geometry, "cylinder")
    text(cylinder, "radius", radius)
    text(cylinder, "length", length)
    material = ET.SubElement(visual, "material")
    text(material, "ambient", color)
    text(material, "diffuse", color)


def add_inertial(parent: ET.Element, mass: str = "0.025") -> None:
    inertial = ET.SubElement(parent, "inertial")
    text(inertial, "pose", "0 0 0 0 0 0")
    text(inertial, "mass", mass)
    inertia = ET.SubElement(inertial, "inertia")
    text(inertia, "ixx", "0.000010")
    text(inertia, "ixy", "0")
    text(inertia, "ixz", "0")
    text(inertia, "iyy", "0.000010")
    text(inertia, "iyz", "0")
    text(inertia, "izz", "0.000010")


def add_visual_box(parent: ET.Element, name: str, size: str, color: str, pose: str | None = None) -> None:
    visual = ET.SubElement(parent, "visual", {"name": name})
    if pose is not None:
        text(visual, "pose", pose)
    geometry = ET.SubElement(visual, "geometry")
    box = ET.SubElement(geometry, "box")
    text(box, "size", size)
    material = ET.SubElement(visual, "material")
    text(material, "ambient", color)
    text(material, "diffuse", color)


def add_gripper_position_controller(model: ET.Element, joint_name: str, topic: str) -> None:
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
    text(plugin, "p_gain", "90")
    text(plugin, "i_gain", "0.1")
    text(plugin, "d_gain", "2.0")


def add_movable_gripper(model: ET.Element, side: str, color: str) -> None:
    parent_link = f"{side}_attached_scaled_gripper"
    palm = model.find(f"link[@name='{parent_link}']")
    finger_x = 0.070
    if palm is None:
        parent_link = f"{side}_Link6"
        palm = model.find(f"link[@name='{parent_link}']")
        finger_x = 0.088
    if palm is None:
        return

    for visual in list(palm.findall("visual")):
        name = (visual.get("name") or "").lower()
        if "finger" in name or "hook" in name:
            palm.remove(visual)

    specs = (
        ("upper", f"{finger_x:.3f} 0.006 0 0 0 0", "0 1 0"),
        ("lower", f"{finger_x:.3f} -0.006 0 0 0 0", "0 -1 0"),
    )
    for name, pose, axis_xyz in specs:
        link_name = f"{side}_mvp_gripper_{name}_finger"
        joint_name = f"{side}_mvp_gripper_{name}_slide"
        topic = f"/model/dual_rm65b_mvp/{side}_gripper_{name}_cmd"

        finger = ET.SubElement(model, "link", {"name": link_name})
        pose_elem = text(finger, "pose", pose)
        pose_elem.set("relative_to", parent_link)
        text(finger, "gravity", "false")
        add_inertial(finger)
        add_visual_box(finger, "finger", "0.052 0.006 0.010", color)
        add_visual_box(finger, "yarn_hook", "0.014 0.014 0.010", color, "0.028 0 0 0 0 0")

        joint = ET.SubElement(model, "joint", {"name": joint_name, "type": "prismatic"})
        text(joint, "parent", parent_link)
        text(joint, "child", link_name)
        axis = ET.SubElement(joint, "axis")
        text(axis, "xyz", axis_xyz)
        limit = ET.SubElement(axis, "limit")
        text(limit, "lower", "0.000")
        text(limit, "upper", "0.020")

        add_gripper_position_controller(model, joint_name, topic)


def add_controller(model: ET.Element) -> None:
    plugin = ET.SubElement(
        model,
        "plugin",
        {
            "filename": "gz-sim-joint-trajectory-controller-system",
            "name": "gz::sim::systems::JointTrajectoryController",
        },
    )
    text(plugin, "topic", "/model/dual_rm65b_mvp/joint_trajectory")
    for joint in DUAL_JOINTS:
        text(plugin, "joint_name", joint)
        text(plugin, "initial_position", f"{HOME[joint]:.6f}")
        text(plugin, "position_p_gain", "45")
        text(plugin, "position_i_gain", "0.1")
        text(plugin, "position_d_gain", "1.5")
        text(plugin, "position_i_min", "-1")
        text(plugin, "position_i_max", "1")
        text(plugin, "position_cmd_min", "-80")
        text(plugin, "position_cmd_max", "80")


def load_model(model_sdf: Path) -> ET.Element:
    root = ET.parse(model_sdf).getroot()
    model = root.find("model") if root.tag == "sdf" else root
    if model is None or model.tag != "model":
        raise RuntimeError(f"{model_sdf} does not contain an SDF model")
    model.set("name", "dual_rm65b_mvp")

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

    add_controller(model)
    add_movable_gripper(model, "left", "0.05 0.10 0.95 1")
    add_movable_gripper(model, "right", "0.95 0.16 0.08 1")
    return model


def add_day_scene(world: ET.Element, day_id: str) -> None:
    day = day_id.lower()
    add_box_model(world, "work_table", "0 0 -0.035 0 0 0", "1.80 1.10 0.04", "0.18 0.20 0.22 1", collision=False)
    add_marker_cylinder(world, "left_base_marker", "-0.45 0 0.005 0 0 0", "0.12", "0.01", "0.10 0.38 0.90 1")
    add_marker_cylinder(world, "right_base_marker", "0.45 0 0.005 0 0 0", "0.12", "0.01", "0.90 0.25 0.12 1")

    if day in {"day01", "d1"}:
        add_box_model(world, "d1_motion_lane", "0 0.18 0.42 0 0 0", "1.05 0.03 0.03", "0.10 0.70 0.95 1", collision=False)
        add_box_model(world, "d1_forbidden_collision_zone", "0 -0.10 0.55 0 0 0", "0.18 0.55 0.36", "0.95 0.08 0.06 0.55")
        add_box_model(world, "d1_goal_gate", "0 0.35 0.60 0 0 0", "0.75 0.035 0.55", "0.08 0.80 0.25 0.45", collision=False)
    elif day in {"day02", "d2"}:
        add_box_model(world, "d2_force_wall", "0.58 -0.23 0.55 0 0 0", "0.34 0.06 0.48", "0.10 0.12 0.14 1")
        add_box_model(world, "d2_compliance_pad", "0.58 -0.285 0.55 0 0 0", "0.20 0.035 0.20", "0.10 0.35 0.95 1")
        add_box_model(world, "d2_relief_window", "0.28 -0.26 0.68 0 0 0", "0.22 0.025 0.22", "0.10 0.85 0.35 0.55", collision=False)
    elif day in {"day03", "d3"}:
        add_box_model(world, "d3_vision_board", "-0.34 0.38 0.68 0 0 0", "0.34 0.035 0.30", "0.05 0.09 0.12 1")
        add_box_model(world, "d3_green_target", "-0.34 0.35 0.70 0 0 0", "0.10 0.018 0.10", "0.05 0.95 0.25 1")
    elif day in {"day04", "d4"}:
        add_box_model(world, "d4_upper_loom_rail", "0 0.36 0.74 0 0 0", "1.00 0.035 0.035", "0.78 0.64 0.20 1")
        add_box_model(world, "d4_lower_loom_rail", "0 0.36 0.46 0 0 0", "1.00 0.035 0.035", "0.78 0.64 0.20 1")
        add_box_model(world, "d4_weft_yarn", "0 0.31 0.60 0 0 0", "0.90 0.018 0.018", "0.92 0.12 0.16 1", collision=False)
    else:
        add_box_model(world, "d5_force_wall", "0.58 -0.23 0.55 0 0 0", "0.34 0.06 0.48", "0.10 0.12 0.14 1")
        add_box_model(world, "d5_vision_target", "-0.34 0.35 0.70 0 0 0", "0.10 0.018 0.10", "0.05 0.95 0.25 1")
        add_box_model(world, "d5_loom_lane", "0 0.36 0.58 0 0 0", "1.00 0.035 0.25", "0.78 0.64 0.20 0.45", collision=False)


def build_world(model: ET.Element, day_id: str, output: Path) -> None:
    sdf = ET.Element("sdf", {"version": "1.9"})
    world = ET.SubElement(sdf, "world", {"name": "rm65b_mvp_world"})

    ET.SubElement(world, "plugin", {"filename": "gz-sim-physics-system", "name": "gz::sim::systems::Physics"})
    ET.SubElement(world, "plugin", {"filename": "gz-sim-user-commands-system", "name": "gz::sim::systems::UserCommands"})
    ET.SubElement(world, "plugin", {"filename": "gz-sim-scene-broadcaster-system", "name": "gz::sim::systems::SceneBroadcaster"})
    text(world, "gravity", "0 0 -9.8")
    physics = ET.SubElement(world, "physics", {"name": "fast", "type": "ode"})
    text(physics, "max_step_size", "0.004")
    text(physics, "real_time_factor", "1.0")

    light = ET.SubElement(world, "light", {"name": "sun", "type": "directional"})
    text(light, "pose", "0 0 5 0 0 0")
    text(light, "cast_shadows", "true")
    diffuse = ET.SubElement(light, "diffuse")
    diffuse.text = "0.8 0.8 0.8 1"
    direction = ET.SubElement(light, "direction")
    direction.text = "-0.5 0.2 -1"

    world.append(model)
    add_day_scene(world, day_id)

    indent_xml(sdf)
    ET.ElementTree(sdf).write(output, encoding="utf-8", xml_declaration=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model-sdf", required=True)
    parser.add_argument("--day-id", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    model = load_model(Path(args.model_sdf))
    build_world(model, args.day_id, Path(args.output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
