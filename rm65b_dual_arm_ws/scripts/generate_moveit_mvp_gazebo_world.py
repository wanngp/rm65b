#!/usr/bin/env python3
from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path


DUAL_JOINTS = [f"{side}_joint{i}" for side in ("left", "right") for i in range(1, 7)]
ARUCO_4X4_50_ID7 = (
    (0, 0, 0, 0, 0, 0),
    (0, 1, 1, 0, 0, 0),
    (0, 0, 1, 0, 0, 0),
    (0, 1, 1, 1, 1, 0),
    (0, 0, 0, 1, 0, 0),
    (0, 0, 0, 0, 0, 0),
)
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


def add_aruco_marker_y_plane(
    world: ET.Element,
    *,
    name_prefix: str,
    center_x: float,
    face_y: float,
    center_z: float,
    marker_size_m: float = 0.080,
) -> None:
    cell = marker_size_m / 6.0
    for row, values in enumerate(ARUCO_4X4_50_ID7):
        for col, value in enumerate(values):
            if value:
                continue
            x = center_x + (col - 2.5) * cell
            z = center_z + (2.5 - row) * cell
            add_box_model(
                world,
                f"{name_prefix}_black_{row}_{col}",
                f"{x:.4f} {face_y:.4f} {z:.4f} 0 0 0",
                f"{cell:.4f} 0.004 {cell:.4f}",
                "0.01 0.01 0.01 1",
                collision=False,
            )


def add_aruco_marker_x_plane(
    world: ET.Element,
    *,
    name_prefix: str,
    face_x: float,
    center_y: float,
    center_z: float,
    marker_size_m: float = 0.080,
) -> None:
    cell = marker_size_m / 6.0
    for row, values in enumerate(ARUCO_4X4_50_ID7):
        for col, value in enumerate(values):
            if value:
                continue
            y = center_y + (col - 2.5) * cell
            z = center_z + (2.5 - row) * cell
            add_box_model(
                world,
                f"{name_prefix}_black_{row}_{col}",
                f"{face_x:.4f} {y:.4f} {z:.4f} 0 0 0",
                f"0.004 {cell:.4f} {cell:.4f}",
                "0.01 0.01 0.01 1",
                collision=False,
            )


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


def add_visual_box(
    parent: ET.Element,
    name: str,
    size: str,
    color: str,
    pose: str | None = None,
    *,
    collision: bool = False,
) -> None:
    visual = ET.SubElement(parent, "visual", {"name": name})
    if pose is not None:
        text(visual, "pose", pose)
    geometry = ET.SubElement(visual, "geometry")
    box = ET.SubElement(geometry, "box")
    text(box, "size", size)
    material = ET.SubElement(visual, "material")
    text(material, "ambient", color)
    text(material, "diffuse", color)
    if collision:
        coll = ET.SubElement(parent, "collision", {"name": name})
        if pose is not None:
            text(coll, "pose", pose)
        coll_geometry = ET.SubElement(coll, "geometry")
        coll_box = ET.SubElement(coll_geometry, "box")
        text(coll_box, "size", size)


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
        add_visual_box(finger, "finger", "0.052 0.006 0.010", color, collision=True)
        add_visual_box(
            finger,
            "yarn_hook",
            "0.014 0.014 0.010",
            color,
            "0.028 0 0 0 0 0",
            collision=True,
        )

        joint = ET.SubElement(model, "joint", {"name": joint_name, "type": "prismatic"})
        text(joint, "parent", parent_link)
        text(joint, "child", link_name)
        axis = ET.SubElement(joint, "axis")
        text(axis, "xyz", axis_xyz)
        limit = ET.SubElement(axis, "limit")
        text(limit, "lower", "0.000")
        text(limit, "upper", "0.020")

        add_gripper_position_controller(model, joint_name, topic)


def add_held_contact_block(model: ET.Element, side: str) -> None:
    parent_link = f"{side}_attached_scaled_gripper"
    if model.find(f"link[@name='{parent_link}']") is None:
        parent_link = f"{side}_Link6"
    if model.find(f"link[@name='{parent_link}']") is None:
        return

    block = ET.SubElement(model, "link", {"name": f"{side}_mvp_held_contact_block"})
    pose = text(block, "pose", "0.150 0 0 0 0 0")
    pose.set("relative_to", parent_link)
    text(block, "gravity", "false")
    add_inertial(block, "0.08")
    add_visual_box(block, "held_block", "0.038 0.038 0.038", "0.96 0.78 0.16 1", collision=True)

    joint = ET.SubElement(model, "joint", {"name": f"{side}_mvp_held_contact_block_fixed", "type": "fixed"})
    text(joint, "parent", parent_link)
    text(joint, "child", f"{side}_mvp_held_contact_block")


def add_gripper_yarn_visual(model: ET.Element, side: str) -> None:
    parent_link = f"{side}_attached_scaled_gripper"
    palm = model.find(f"link[@name='{parent_link}']")
    if palm is None:
        parent_link = f"{side}_Link6"
        palm = model.find(f"link[@name='{parent_link}']")
    if palm is None:
        return

    add_visual_box(
        palm,
        "moving_yarn_segment",
        "0.260 0.022 0.022",
        "1.00 0.05 0.04 1",
        "0.130 0 0 0 0 0",
    )
    add_visual_box(
        palm,
        "moving_yarn_highlight",
        "0.260 0.007 0.007",
        "1.00 0.90 0.08 1",
        "0.130 0 0.016 0 0 0",
    )
    add_visual_box(
        palm,
        "moving_yarn_grip_knot",
        "0.040 0.040 0.040",
        "1.00 0.05 0.04 1",
        "0.045 0 0 0 0 0",
    )


def add_eye_in_hand_camera(model: ET.Element, side: str) -> None:
    parent_link = f"{side}_attached_scaled_gripper"
    if model.find(f"link[@name='{parent_link}']") is None:
        parent_link = f"{side}_Link6"
    if model.find(f"link[@name='{parent_link}']") is None:
        return

    camera = ET.SubElement(model, "link", {"name": f"{side}_mvp_eye_in_hand_camera"})
    pose = text(camera, "pose", "0.064 0 0.045 0 0.35 0")
    pose.set("relative_to", parent_link)
    text(camera, "gravity", "false")
    add_inertial(camera, "0.04")
    add_visual_box(camera, "camera_body", "0.045 0.032 0.026", "0.06 0.16 0.30 1")
    add_visual_box(camera, "camera_lens", "0.014 0.022 0.022", "0.02 0.02 0.025 1", "0.028 0 0 0 0 0")
    add_visual_box(camera, "view_frustum", "0.090 0.080 0.006", "0.20 0.65 1.00 0.28", "0.088 0 0 0 0 0")

    joint = ET.SubElement(model, "joint", {"name": f"{side}_mvp_eye_in_hand_camera_fixed", "type": "fixed"})
    text(joint, "parent", parent_link)
    text(joint, "child", f"{side}_mvp_eye_in_hand_camera")


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


def load_model(model_sdf: Path, day_id: str) -> ET.Element:
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
    day = day_id.lower()
    if day in {"day02", "d2"}:
        add_held_contact_block(model, "right")
    if day in {"day04", "d4", "day05", "d5"}:
        add_gripper_yarn_visual(model, "left")
        add_gripper_yarn_visual(model, "right")
    if day in {"day03", "d3", "day04", "d4", "day05", "d5"}:
        add_eye_in_hand_camera(model, "right")
    return model


def add_weaving_mvp_scene(world: ET.Element, prefix: str) -> None:
    add_box_model(world, f"{prefix}_upper_loom_rail", "0 0.36 0.74 0 0 0", "1.08 0.035 0.035", "0.78 0.64 0.20 1")
    add_box_model(world, f"{prefix}_lower_loom_rail", "0 0.36 0.46 0 0 0", "1.08 0.035 0.035", "0.78 0.64 0.20 1")
    add_box_model(world, f"{prefix}_left_loom_post", "-0.56 0.36 0.60 0 0 0", "0.035 0.035 0.34", "0.56 0.42 0.18 1")
    add_box_model(world, f"{prefix}_right_loom_post", "0.56 0.36 0.60 0 0 0", "0.035 0.035 0.34", "0.56 0.42 0.18 1")
    add_box_model(world, f"{prefix}_weft_yarn", "0 0.275 0.60 0 0 0", "0.94 0.030 0.030", "1.00 0.06 0.05 1", collision=False)
    add_box_model(world, f"{prefix}_weft_yarn_highlight", "0 0.248 0.602 0 0 0", "0.94 0.010 0.010", "1.00 0.92 0.12 1", collision=False)
    add_box_model(world, f"{prefix}_shuttle_lane", "0 0.285 0.60 0 0 0", "1.00 0.024 0.060", "0.10 0.65 0.95 0.62", collision=False)
    add_box_model(world, f"{prefix}_tension_scale", "0.67 0.31 0.64 0 0 0", "0.055 0.020 0.24", "0.08 0.12 0.15 1")
    add_box_model(world, f"{prefix}_tension_target_band", "0.672 0.292 0.61 0 0 0", "0.060 0.006 0.070", "0.10 0.85 0.35 0.85", collision=False)
    add_box_model(world, f"{prefix}_tension_spring", "0.61 0.305 0.60 0 0 0", "0.095 0.010 0.010", "0.96 0.78 0.16 1", collision=False)
    add_box_model(world, f"{prefix}_tension_mass", "0.56 0.305 0.60 0 0 0", "0.035 0.035 0.035", "0.96 0.78 0.16 1", collision=False)
    add_box_model(world, f"{prefix}_hook_target_marker", "0.43 0.300 0.61 0 0 0", "0.055 0.012 0.055", "0.95 0.16 0.10 0.70", collision=False)
    add_box_model(world, f"{prefix}_lift_target_marker", "-0.30 0.300 0.68 0 0 0", "0.070 0.012 0.030", "0.20 0.65 1.00 0.68", collision=False)
    add_box_model(world, f"{prefix}_exchange_target_marker", "0 0.300 0.60 0 0 0", "0.060 0.012 0.060", "0.18 0.90 0.40 0.68", collision=False)
    for idx, x in enumerate((-0.48, -0.36, -0.24, -0.12, 0.0, 0.12, 0.24, 0.36, 0.48)):
        add_box_model(
            world,
            f"{prefix}_warp_thread_{idx:02d}",
            f"{x:.3f} 0.252 0.60 0 0 0",
            "0.012 0.012 0.300",
            "0.98 0.96 0.76 1",
            collision=False,
        )
    add_box_model(world, f"{prefix}_left_yarn_knot", "-0.47 0.248 0.60 0 0 0", "0.045 0.045 0.045", "1.00 0.06 0.05 1", collision=False)
    add_box_model(world, f"{prefix}_right_yarn_knot", "0.47 0.248 0.60 0 0 0", "0.045 0.045 0.045", "1.00 0.06 0.05 1", collision=False)


def add_figure_eight_markers(world: ET.Element, prefix: str) -> None:
    for name, cx in (("left_loop", -0.22), ("right_loop", 0.22)):
        add_box_model(world, f"{prefix}_{name}_top", f"{cx:.3f} 0.245 0.685 0 0 0", "0.250 0.024 0.024", "0.45 0.18 0.95 1", collision=False)
        add_box_model(world, f"{prefix}_{name}_bottom", f"{cx:.3f} 0.245 0.515 0 0 0", "0.250 0.024 0.024", "0.45 0.18 0.95 1", collision=False)
        add_box_model(world, f"{prefix}_{name}_outer", f"{cx - 0.125:.3f} 0.245 0.600 0 0 0", "0.024 0.024 0.170", "0.45 0.18 0.95 1", collision=False)
        add_box_model(world, f"{prefix}_{name}_inner", f"{cx + 0.125:.3f} 0.245 0.600 0 0 0", "0.024 0.024 0.170", "0.45 0.18 0.95 1", collision=False)
    add_box_model(world, f"{prefix}_figure_eight_cross", "0 0.240 0.600 0 0 0", "0.070 0.026 0.070", "0.98 0.84 0.20 1", collision=False)


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
        front_aligned_pose = "3.1416 1.1999 1.5708"
        add_box_model(world, "d2_force_wall", f"0.450 0.229 0.425 {front_aligned_pose}", "0.060 0.42 0.32", "0.10 0.12 0.14 1")
        add_box_model(world, "d2_compliance_pad", f"0.450 0.212 0.467 {front_aligned_pose}", "0.018 0.30 0.24", "0.10 0.35 0.95 1")
        add_box_model(world, "d2_contact_face", f"0.450 0.210 0.471 {front_aligned_pose}", "0.008 0.34 0.28", "0.95 0.16 0.10 0.70", collision=False)
        add_box_model(world, "d2_probe_path_marker", f"0.450 0.184 0.555 {front_aligned_pose}", "0.170 0.018 0.018", "0.96 0.78 0.16 0.65", collision=False)
        add_box_model(world, "d2_relief_window", f"0.450 0.198 0.520 {front_aligned_pose}", "0.080 0.26 0.018", "0.10 0.85 0.35 0.55", collision=False)
    elif day in {"day03", "d3"}:
        add_box_model(world, "d3_calibration_board", "0.860 0.150 0.590 0 0 0", "0.025 0.220 0.220", "0.96 0.96 0.92 1", collision=False)
        add_aruco_marker_x_plane(
            world,
            name_prefix="d3_aruco_4x4_50_id7",
            face_x=0.846,
            center_y=0.150,
            center_z=0.590,
            marker_size_m=0.080,
        )
        add_box_model(world, "d3_board_size_reference_y", "0.843 0.150 0.715 0 0 0", "0.004 0.080 0.010", "0.20 0.65 1.00 0.70", collision=False)
        add_box_model(world, "d3_camera_alignment_lane", "0.685 0.150 0.660 0 0 0", "0.300 0.018 0.018", "0.20 0.65 1.00 0.55", collision=False)
    elif day in {"day04", "d4"}:
        add_weaving_mvp_scene(world, "d4")
    else:
        add_weaving_mvp_scene(world, "d5")
        add_figure_eight_markers(world, "d5")
        add_box_model(world, "d5_vision_lock_marker", "0.10 0.292 0.60 0 0 0", "0.060 0.008 0.060", "0.05 0.95 0.25 0.85", collision=False)
        add_aruco_marker_y_plane(
            world,
            name_prefix="d5_aruco_4x4_50_id7",
            center_x=0.10,
            face_y=0.286,
            center_z=0.60,
            marker_size_m=0.080,
        )


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

    model = load_model(Path(args.model_sdf), args.day_id)
    build_world(model, args.day_id, Path(args.output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
