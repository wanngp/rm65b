#!/usr/bin/env python3
from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path


DAY_NOTES = {
    "day01": [
        "D1 gripper, TF, and dual-arm setup scene.",
        "Includes offset TCP reference markers and compact gripper calibration markers.",
    ],
    "day02": [
        "D2 force-compliance scene.",
        "Includes contact bars, compliance spring marker, and force target panel.",
    ],
    "day03": [
        "D3 vision-guidance scene.",
        "Includes Gazebo left camera, fiducial-style vision board, target marker, and lighting panel.",
    ],
    "day04": [
        "D4 weaving primitive scene.",
        "Includes loom rails, warp strips, shuttle lane, yarn tension scale, and primitive targets.",
    ],
    "day05": [
        "D5 integrated demo scene.",
        "Combines dual arms, grippers, Gazebo camera, force-control station, loom, yarn, and status markers.",
    ],
}


def _text(parent: ET.Element, tag: str, value: str) -> ET.Element:
    child = ET.SubElement(parent, tag)
    child.text = value
    return child


def _material(parent: ET.Element, rgba: str) -> None:
    material = ET.SubElement(parent, "material")
    _text(material, "ambient", rgba)
    _text(material, "diffuse", rgba)


def add_box_model(
    world: ET.Element,
    name: str,
    pose: str,
    size: str,
    rgba: str,
    collision: bool = False,
    contact_sensor: bool = False,
    static: bool = True,
    gravity: bool = False,
    mass: str = "0.050",
    collision_size: str | None = None,
) -> None:
    model = ET.SubElement(world, "model", {"name": name})
    _text(model, "static", "true" if static else "false")
    _text(model, "pose", pose)
    link = ET.SubElement(model, "link", {"name": "link"})
    if not static:
        _text(link, "gravity", "true" if gravity else "false")
        inertial = ET.SubElement(link, "inertial")
        _text(inertial, "mass", mass)
        inertia = ET.SubElement(inertial, "inertia")
        for tag, value in {
            "ixx": "0.00002",
            "ixy": "0",
            "ixz": "0",
            "iyy": "0.00002",
            "iyz": "0",
            "izz": "0.00002",
        }.items():
            _text(inertia, tag, value)
    if collision:
        col = ET.SubElement(link, "collision", {"name": "collision"})
        geometry = ET.SubElement(col, "geometry")
        box = ET.SubElement(geometry, "box")
        _text(box, "size", collision_size or size)
        surface = ET.SubElement(col, "surface")
        friction = ET.SubElement(surface, "friction")
        ode = ET.SubElement(friction, "ode")
        _text(ode, "mu", "1.4")
        _text(ode, "mu2", "1.4")
        if contact_sensor:
            sensor = ET.SubElement(link, "sensor", {"name": f"{name}_contact", "type": "contact"})
            _text(sensor, "topic", f"/contacts/{name}")
            _text(sensor, "always_on", "true")
            _text(sensor, "update_rate", "30")
            contact = ET.SubElement(sensor, "contact")
            _text(contact, "collision", "collision")
    visual = ET.SubElement(link, "visual", {"name": "visual"})
    geometry = ET.SubElement(visual, "geometry")
    box = ET.SubElement(geometry, "box")
    _text(box, "size", size)
    _material(visual, rgba)


def add_contact_probe(
    world: ET.Element,
    name: str,
    pose: str,
    size: str,
    rgba: str = "0.95 0.95 0.20 1",
    mass: str = "0.030",
) -> None:
    model = ET.SubElement(world, "model", {"name": name})
    _text(model, "static", "false")
    _text(model, "pose", pose)
    link = ET.SubElement(model, "link", {"name": "link"})
    _text(link, "gravity", "false")
    inertial = ET.SubElement(link, "inertial")
    _text(inertial, "mass", mass)
    inertia = ET.SubElement(inertial, "inertia")
    for tag, value in {
        "ixx": "0.00001",
        "ixy": "0",
        "ixz": "0",
        "iyy": "0.00001",
        "iyz": "0",
        "izz": "0.00001",
    }.items():
        _text(inertia, tag, value)
    col = ET.SubElement(link, "collision", {"name": "collision"})
    geometry = ET.SubElement(col, "geometry")
    box = ET.SubElement(geometry, "box")
    _text(box, "size", size)
    visual = ET.SubElement(link, "visual", {"name": "visual"})
    geometry = ET.SubElement(visual, "geometry")
    box = ET.SubElement(geometry, "box")
    _text(box, "size", size)
    _material(visual, rgba)


def set_model_pose(world: ET.Element, name: str, pose: str) -> None:
    model = world.find(f"model[@name='{name}']")
    if model is None:
        return
    pose_el = model.find("pose")
    if pose_el is None:
        pose_el = ET.SubElement(model, "pose")
    pose_el.text = pose


def set_camera_fov(world: ET.Element, model_name: str, horizontal_fov: str) -> None:
    fov = world.find(f"model[@name='{model_name}']/link/sensor/camera/horizontal_fov")
    if fov is not None:
        fov.text = horizontal_fov


def add_vision_target(world: ET.Element, pose: str = "-0.08 0.402 0.36 0 0 0") -> None:
    model = ET.SubElement(world, "model", {"name": "vision_target"})
    _text(model, "static", "true")
    _text(model, "pose", pose)
    link = ET.SubElement(model, "link", {"name": "target_link"})
    collision = ET.SubElement(link, "collision", {"name": "target_collision"})
    geometry = ET.SubElement(collision, "geometry")
    box = ET.SubElement(geometry, "box")
    _text(box, "size", "0.18 0.080 0.18")
    sensor = ET.SubElement(link, "sensor", {"name": "vision_target_contact", "type": "contact"})
    _text(sensor, "topic", "/contacts/vision_target")
    _text(sensor, "always_on", "true")
    _text(sensor, "update_rate", "30")
    contact = ET.SubElement(sensor, "contact")
    _text(contact, "collision", "target_collision")
    visual = ET.SubElement(link, "visual", {"name": "target_plate"})
    geometry = ET.SubElement(visual, "geometry")
    box = ET.SubElement(geometry, "box")
    _text(box, "size", "0.16 0.012 0.16")
    _material(visual, "0.03 0.04 0.05 1")
    center = ET.SubElement(link, "visual", {"name": "target_center"})
    _text(center, "pose", "0 0.014 0 0 0 0")
    geometry = ET.SubElement(center, "geometry")
    box = ET.SubElement(geometry, "box")
    _text(box, "size", "0.090 0.006 0.090")
    _material(center, "0.12 1.00 0.48 1")
    back_center = ET.SubElement(link, "visual", {"name": "target_back_center"})
    _text(back_center, "pose", "0 -0.014 0 0 0 0")
    geometry = ET.SubElement(back_center, "geometry")
    box = ET.SubElement(geometry, "box")
    _text(box, "size", "0.090 0.006 0.090")
    _material(back_center, "0.12 1.00 0.48 1")
    top_center = ET.SubElement(link, "visual", {"name": "target_top_center"})
    _text(top_center, "pose", "0 0 0.083 0 0 0")
    geometry = ET.SubElement(top_center, "geometry")
    box = ET.SubElement(geometry, "box")
    _text(box, "size", "0.060 0.050 0.006")
    _material(top_center, "0.12 1.00 0.48 1")


def add_weft_yarn(world: ET.Element, pose: str = "0.00 0.47 0.35 0 0 0") -> None:
    add_box_model(
        world,
        "weft_yarn",
        pose,
        "0.66 0.018 0.018",
        "0.88 0.12 0.16 1",
        True,
        False,
        static=False,
        gravity=False,
        mass="0.020",
    )


def ensure_contact_system(world: ET.Element) -> None:
    for plugin in world.findall("plugin"):
        if plugin.get("filename") == "gz-sim-contact-system":
            return
    contact_plugin = ET.Element(
        "plugin",
        {
            "filename": "gz-sim-contact-system",
            "name": "gz::sim::systems::Contact",
        },
    )
    world.insert(1, contact_plugin)


def ensure_detachable_joint_system(world: ET.Element) -> None:
    """Intentionally empty.

    Gazebo's DetachableJoint system must be attached to the parent model, not
    to the world. The recording script injects one plugin per arm/object pair
    into the generated arm SDFs before spawning the arms.
    """
    return


def add_day01(world: ET.Element) -> None:
    add_box_model(world, "d1_tcp_left_target", "-0.390 -0.340 0.575 0 0 0", "0.030 0.030 0.075", "0.16 0.36 0.68 0.72")
    add_box_model(world, "d1_tcp_right_target", "0.390 -0.340 0.575 0 0 0", "0.030 0.030 0.075", "0.68 0.24 0.18 0.72")
    add_box_model(world, "d1_sync_motion_left_lane", "-0.245 -0.340 0.525 0 0 0", "0.22 0.012 0.014", "0.16 0.36 0.68 0.65")
    add_box_model(world, "d1_sync_motion_right_lane", "0.245 -0.340 0.525 0 0 0", "0.22 0.012 0.014", "0.68 0.24 0.18 0.65")
    add_box_model(world, "d1_center_clearance_marker", "0.000 -0.340 0.555 0 0 0", "0.030 0.030 0.055", "0.72 0.72 0.68 0.65")
    add_box_model(world, "d1_gripper_gap_gauge", "0.000 -0.390 0.510 0 0 0", "0.18 0.010 0.016", "0.82 0.68 0.18 0.70")


def add_day02(world: ET.Element) -> None:
    add_box_model(world, "d2_force_target_panel", "0.542 -0.060 0.760 0 0 0", "0.32 0.070 0.36", "0.15 0.18 0.22 1", True, True)
    add_box_model(world, "d2_contact_pad", "0.542 -0.130 0.760 0 0 0", "0.22 0.090 0.22", "0.12 0.52 0.74 1", True, True)
    add_box_model(world, "d2_left_compliance_bar", "0.452 -0.130 0.760 0 0 0", "0.045 0.060 0.34", "0.10 0.60 0.24 1", True, True)
    add_box_model(world, "d2_right_compliance_bar", "0.632 -0.130 0.760 0 0 0", "0.045 0.060 0.34", "0.88 0.48 0.12 1", True, True)
    add_contact_probe(world, "d2_force_contact_probe", "0.542 -0.235 0.760 0 0 0", "0.080 0.045 0.080")
    for idx, x in enumerate([0.472, 0.507, 0.542, 0.577, 0.612]):
        add_box_model(world, f"d2_spring_coil_{idx}", f"{x:.3f} -0.180 0.940 0 0 0", "0.024 0.036 0.024", "0.18 0.52 0.95 1")


def add_day03(world: ET.Element) -> None:
    add_box_model(world, "d3_vision_board", "-0.509 -0.190 0.531 0 0 0", "0.38 0.030 0.30", "0.92 0.92 0.86 1", True, False)
    add_box_model(world, "d3_marker_center", "-0.509 -0.206 0.531 0 0 0", "0.130 0.018 0.130", "0.12 1.00 0.48 1", False, False)
    add_box_model(world, "d3_marker_left", "-0.594 -0.206 0.596 0 0 0", "0.055 0.014 0.055", "0.08 0.32 0.90 1")
    add_box_model(world, "d3_marker_right", "-0.424 -0.206 0.466 0 0 0", "0.055 0.014 0.055", "0.88 0.18 0.12 1")
    add_box_model(world, "d3_lighting_panel", "-0.509 -0.355 0.771 0 0 0", "0.52 0.035 0.045", "1.00 0.94 0.68 1")
    add_vision_target(world, "-0.509 -0.165 0.531 0 0 0")


def add_day04(world: ET.Element) -> None:
    add_box_model(world, "d4_upper_loom_rail", "0.088 0.077 0.805 0 0 0", "0.58 0.040 0.030", "0.16 0.18 0.20 1", True, True)
    add_box_model(world, "d4_lower_loom_rail", "0.088 0.077 0.595 0 0 0", "0.58 0.040 0.030", "0.16 0.18 0.20 1", True, True)
    for idx, x in enumerate([-0.132, -0.077, -0.022, 0.033, 0.088, 0.143, 0.198, 0.253, 0.308]):
        add_box_model(world, f"d4_warp_{idx}", f"{x:.3f} 0.072 0.700 0 0 0", "0.010 0.018 0.210", "0.82 0.82 0.78 1", True, False)
    for idx, z in enumerate([0.630, 0.665, 0.700, 0.735, 0.770]):
        add_box_model(world, f"d4_weft_preview_{idx}", f"0.088 0.054 {z:.3f} 0 0 0", "0.560 0.010 0.008", "0.88 0.12 0.16 1", True, False)
    add_box_model(
        world,
        "d4_shuttle_lane",
        "0.088 0.030 0.790 0 0 0",
        "0.60 0.030 0.030",
        "0.84 0.10 0.16 1",
        True,
        True,
        collision_size="0.70 0.52 0.62",
    )
    add_box_model(world, "d4_tension_scale", "0.542 -0.091 0.702 0 0 0", "0.095 0.035 0.150", "0.12 0.44 0.72 1", True, True)
    add_box_model(world, "d4_tension_spring_anchor", "0.542 -0.168 0.702 0 0 0", "0.070 0.026 0.070", "0.10 0.10 0.12 1", True, True)
    for idx, y in enumerate([-0.150, -0.137, -0.124, -0.111, -0.098]):
        add_box_model(world, f"d4_tension_spring_mass_{idx}", f"0.542 {y:.3f} 0.702 0 0 0", "0.018 0.008 0.018", "0.95 0.78 0.18 1", False, False, static=False, gravity=False, mass="0.006")
    add_box_model(world, "d4_hook_target_marker", "0.500 -0.028 0.689 0 0 0", "0.045 0.012 0.045", "0.10 0.62 0.32 1", True, False)
    add_box_model(world, "d4_lift_target_marker", "0.535 -0.018 0.728 0 0 0", "0.040 0.012 0.040", "0.95 0.70 0.12 1", True, False)
    add_box_model(world, "d4_exchange_target_marker", "0.446 0.020 0.690 0 0 0", "0.050 0.012 0.050", "0.35 0.22 0.84 1", True, False)
    add_weft_yarn(world, "0.564 -0.028 0.689 0 0 0")
    add_contact_probe(world, "d4_shuttle_contact_probe", "0.564 -0.055 0.689 0 0 0", "0.080 0.030 0.030")
    add_contact_probe(world, "d4_tension_contact_probe", "0.542 -0.120 0.702 0 0 0", "0.055 0.030 0.055")


def add_day05(world: ET.Element) -> None:
    add_day02(world)
    add_day03(world)
    add_day04(world)
    add_box_model(world, "d5_demo_status_panel", "0.120 0.030 0.300 0 0 0", "0.10 0.030 0.24", "0.42 0.16 0.65 1")
    add_box_model(world, "d5_cloth_result_strip", "0.450 0.205 0.340 0 0 0", "0.44 0.018 0.13", "0.20 0.56 0.48 1")


CAMERA_LAYOUTS = {
    "day01": {
        "evidence": "0.95 -0.78 1.05 0 0.58 2.45",
        "evidence_fov": "1.35",
    },
    "day02": {
        "evidence": "1.18 -0.86 0.74 0 0.38 2.18",
    },
    "day03": {
        "evidence": "0.12 0.72 0.74 0 0.34 -2.15",
    },
    "day04": {
        "evidence": "1.20 -0.90 0.78 0 0.38 2.18",
    },
    "day05": {
        "evidence": "1.25 -0.96 0.82 0 0.40 2.20",
    },
}


def normalize_day(day_id: str) -> str:
    mapping = {"d1": "day01", "d2": "day02", "d3": "day03", "d4": "day04", "d5": "day05"}
    return mapping.get(day_id, day_id)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-world", required=True)
    parser.add_argument("--day-id", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--notes", required=True)
    args = parser.parse_args()

    day = normalize_day(args.day_id)
    tree = ET.parse(args.base_world)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise SystemExit("base SDF has no world element")

    ensure_contact_system(world)
    camera_layout = CAMERA_LAYOUTS[day]
    set_model_pose(world, "evidence_camera", camera_layout["evidence"])
    if "evidence_fov" in camera_layout:
        set_camera_fov(world, "evidence_camera", camera_layout["evidence_fov"])
    if day == "day01":
        add_day01(world)
    elif day == "day02":
        add_day02(world)
    elif day == "day03":
        add_day03(world)
    elif day == "day04":
        add_day04(world)
    elif day == "day05":
        add_day05(world)
    else:
        raise SystemExit(f"unsupported day id: {args.day_id}")

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if hasattr(ET, "indent"):
        ET.indent(tree, space="  ")
    tree.write(output, encoding="unicode", xml_declaration=True)

    notes = Path(args.notes)
    notes.parent.mkdir(parents=True, exist_ok=True)
    notes.write_text(
        "\n".join(
            [
                f"day_id: {day}",
                "gazebo_left_eye_in_hand_camera_topic: /left_camera/image_rect",
                "gazebo_right_eye_in_hand_camera_topic: /right_camera/image_rect",
                "evidence_camera_topic: /rm65b/evidence_camera/image",
                "contact_physics: D2/D4/D5 include dynamic probe bodies that contact instrumented collision sensors.",
                *DAY_NOTES[day],
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
