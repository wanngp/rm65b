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


def add_eye_camera(
    model: ET.Element,
    *,
    topic: str,
    sensor_name: str,
    color: str,
    visual_pose: str,
    sensor_pose: str,
    horizontal_fov: str,
) -> None:
    palm = model.find("link[@name='attached_scaled_gripper_palm']")
    if palm is None:
        raise RuntimeError("arm SDF has no attached_scaled_gripper_palm link for eye-in-hand camera")
    if palm.find(f"sensor[@name='{sensor_name}']") is not None:
        return

    visual = ET.SubElement(palm, "visual", {"name": f"{sensor_name}_body"})
    text(visual, "pose", visual_pose)
    geometry = ET.SubElement(visual, "geometry")
    box = ET.SubElement(geometry, "box")
    text(box, "size", "0.024 0.018 0.014")
    material = ET.SubElement(visual, "material")
    text(material, "ambient", color)
    text(material, "diffuse", color)

    sensor = ET.SubElement(palm, "sensor", {"name": sensor_name, "type": "camera"})
    text(sensor, "pose", sensor_pose)
    text(sensor, "topic", topic)
    text(sensor, "always_on", "true")
    text(sensor, "update_rate", "30")
    camera = ET.SubElement(sensor, "camera")
    text(camera, "horizontal_fov", horizontal_fov)
    image = ET.SubElement(camera, "image")
    text(image, "width", "640")
    text(image, "height", "480")
    text(image, "format", "R8G8B8")
    clip = ET.SubElement(camera, "clip")
    text(clip, "near", "0.02")
    text(clip, "far", "6")


def parse_positions(value: str | None) -> tuple[float, ...]:
    if not value:
        return INITIAL_POSITIONS
    parts = tuple(float(part) for part in value.split())
    if len(parts) != 6:
        raise ValueError("--initial-positions must contain exactly 6 numeric joint values")
    return parts


def add_joint_trajectory_controller(model: ET.Element, initial_positions: tuple[float, ...]) -> None:
    plugin = ET.SubElement(
        model,
        "plugin",
        {
            "filename": "gz-sim-joint-trajectory-controller-system",
            "name": "gz::sim::systems::JointTrajectoryController",
        },
    )
    for idx, initial in enumerate(initial_positions, start=1):
        text(plugin, "joint_name", f"joint{idx}")
        text(plugin, "initial_position", f"{initial:.6f}")
        text(plugin, "position_p_gain", "45")
        text(plugin, "position_i_gain", "0.1")
        text(plugin, "position_d_gain", "1.5")
        text(plugin, "position_i_min", "-1")
        text(plugin, "position_i_max", "1")
        text(plugin, "position_cmd_min", "-80")
        text(plugin, "position_cmd_max", "80")


def prepare(
    src: Path,
    dst: Path,
    *,
    force_probe: bool = False,
    initial_positions: tuple[float, ...] = INITIAL_POSITIONS,
    eye_camera_topic: str | None = None,
    eye_camera_name: str = "eye_in_hand_camera",
    eye_camera_color: str = "0.16 0.36 0.68 1",
    eye_camera_visual_pose: str = "0.032 0 0.090 0 0.35 0",
    eye_camera_sensor_pose: str = "0.034 0 0.095 0 0.35 0",
    eye_camera_horizontal_fov: str = "1.65",
) -> None:
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
    if eye_camera_topic:
        add_eye_camera(
            model,
            topic=eye_camera_topic,
            sensor_name=eye_camera_name,
            color=eye_camera_color,
            visual_pose=eye_camera_visual_pose,
            sensor_pose=eye_camera_sensor_pose,
            horizontal_fov=eye_camera_horizontal_fov,
        )
    add_joint_trajectory_controller(model, initial_positions)
    tree.write(dst, encoding="utf-8", xml_declaration=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--force-probe", action="store_true")
    parser.add_argument("--initial-positions")
    parser.add_argument("--eye-camera-topic")
    parser.add_argument("--eye-camera-name", default="eye_in_hand_camera")
    parser.add_argument("--eye-camera-color", default="0.16 0.36 0.68 1")
    parser.add_argument("--eye-camera-visual-pose", default="0.032 0 0.090 0 0.35 0")
    parser.add_argument("--eye-camera-sensor-pose", default="0.034 0 0.095 0 0.35 0")
    parser.add_argument("--eye-camera-horizontal-fov", default="1.65")
    args = parser.parse_args()
    prepare(
        Path(args.source),
        Path(args.output),
        force_probe=args.force_probe,
        initial_positions=parse_positions(args.initial_positions),
        eye_camera_topic=args.eye_camera_topic,
        eye_camera_name=args.eye_camera_name,
        eye_camera_color=args.eye_camera_color,
        eye_camera_visual_pose=args.eye_camera_visual_pose,
        eye_camera_sensor_pose=args.eye_camera_sensor_pose,
        eye_camera_horizontal_fov=args.eye_camera_horizontal_fov,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
