#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
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
        surface = ET.SubElement(coll, "surface")
        friction = ET.SubElement(surface, "friction")
        ode_friction = ET.SubElement(friction, "ode")
        text(ode_friction, "mu", "1.6")
        text(ode_friction, "mu2", "1.6")
        contact = ET.SubElement(surface, "contact")
        ode_contact = ET.SubElement(contact, "ode")
        text(ode_contact, "kp", "50000")
        text(ode_contact, "kd", "5")


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
    if day in {"day03", "d3", "day04", "d4", "day05", "d5"}:
        add_eye_in_hand_camera(model, "right")
    return model


def add_pickup_yarn_chain(world: ET.Element, prefix: str) -> None:
    model = ET.SubElement(world, "model", {"name": f"{prefix}_pickup_yarn_chain"})
    text(model, "pose", "0 0 0 0 0 0")
    text(model, "static", "false")
    text(model, "self_collide", "false")

    # Thirty-two short rigid links are initialized as a loose wrap around the
    # center post; Gazebo joints then provide semi-flexible motion.
    chain_points = [
        (0.450, 0.115, 0.737),
        (0.430, 0.160, 0.715),
        (0.390, 0.205, 0.690),
        (0.330, 0.250, 0.665),
        (0.250, 0.300, 0.635),
        (0.160, 0.340, 0.605),
        (0.070, 0.360, 0.575),
        (0.000, 0.360, 0.555),
        (-0.065, 0.360, 0.570),
        (-0.105, 0.360, 0.615),
        (-0.095, 0.360, 0.665),
        (-0.045, 0.360, 0.705),
        (0.025, 0.360, 0.710),
        (0.085, 0.360, 0.675),
        (0.105, 0.360, 0.620),
        (0.075, 0.360, 0.575),
        (0.010, 0.360, 0.555),
        (-0.060, 0.360, 0.575),
        (-0.100, 0.360, 0.625),
        (-0.085, 0.360, 0.675),
        (-0.035, 0.360, 0.705),
        (0.020, 0.360, 0.690),
        (0.045, 0.360, 0.635),
        (0.020, 0.350, 0.590),
        (-0.055, 0.335, 0.600),
        (-0.135, 0.305, 0.620),
        (-0.220, 0.260, 0.650),
        (-0.300, 0.210, 0.680),
        (-0.370, 0.155, 0.710),
        (-0.420, 0.085, 0.730),
        (-0.455, 0.015, 0.740),
        (-0.465, -0.060, 0.735),
        (-0.450, -0.115, 0.737),
    ]
    for idx, (start, end) in enumerate(zip(chain_points[:-1], chain_points[1:])):
        sx, sy, sz = start
        ex, ey, ez = end
        dx = ex - sx
        dy = ey - sy
        dz = ez - sz
        length = (dx * dx + dy * dy + dz * dz) ** 0.5
        yaw = math.atan2(dy, dx)
        pitch = math.atan2(-dz, (dx * dx + dy * dy) ** 0.5)
        link_name = f"yarn_link_{idx:02d}"
        link = ET.SubElement(model, "link", {"name": link_name})
        text(
            link,
            "pose",
            f"{(sx + ex) / 2.0:.3f} {(sy + ey) / 2.0:.3f} {(sz + ez) / 2.0:.3f} 0 {pitch:.4f} {yaw:.4f}",
        )
        text(link, "gravity", "true")
        text(link, "self_collide", "false")
        add_inertial(link, "0.010")
        add_visual_box(link, "yarn_segment", f"{length:.3f} 0.016 0.016", "0.86 0.06 0.04 1", collision=True)
        if idx == 0:
            add_visual_box(
                link,
                "right_bright_grasp_handle",
                "0.055 0.055 0.055",
                "0.05 1.00 0.28 1",
                f"{-length / 2.0:.3f} 0 0.010 0 0 0",
                collision=True,
            )
        if idx == len(chain_points) - 2:
            add_visual_box(
                link,
                "left_bright_grasp_handle",
                "0.050 0.050 0.050",
                "0.05 0.65 1.00 1",
                f"{length / 2.0:.3f} 0 0 0 0 0",
                collision=True,
            )

    for idx, hinge in enumerate(chain_points[1:-1], start=1):
        joint = ET.SubElement(model, "joint", {"name": f"yarn_joint_{idx:02d}", "type": "revolute"})
        parent_link = f"yarn_link_{idx - 1:02d}"
        text(joint, "parent", parent_link)
        text(joint, "child", f"yarn_link_{idx:02d}")
        text(joint, "pose", f"{hinge[0]:.3f} {hinge[1]:.3f} {hinge[2]:.3f} 0 0 0")
        axis = ET.SubElement(joint, "axis")
        text(axis, "xyz", "0 1 0" if idx % 2 == 0 else "0 0 1")
        limit = ET.SubElement(axis, "limit")
        text(limit, "lower", "-1.05")
        text(limit, "upper", "1.05")
        text(limit, "effort", "0.35")
        text(limit, "velocity", "2.0")
        dynamics = ET.SubElement(axis, "dynamics")
        text(dynamics, "damping", "0.12")
        text(dynamics, "friction", "0.03")


def add_weaving_mvp_scene(world: ET.Element, prefix: str) -> None:
    frame_color = "0.01 0.01 0.01 1"
    add_box_model(world, f"{prefix}_field_frame_top", "0 0.360 0.740 0 0 0", "1.080 0.030 0.030", frame_color)
    add_box_model(world, f"{prefix}_field_frame_bottom", "0 0.360 0.460 0 0 0", "1.080 0.030 0.030", frame_color)
    add_box_model(world, f"{prefix}_field_frame_left", "-0.540 0.360 0.600 0 0 0", "0.030 0.030 0.310", frame_color)
    add_box_model(world, f"{prefix}_field_frame_right", "0.540 0.360 0.600 0 0 0", "0.030 0.030 0.310", frame_color)
    add_box_model(world, f"{prefix}_field_frame_mid_vertical", "0 0.360 0.600 0 0 0", "0.026 0.026 0.310", frame_color)
    add_box_model(world, f"{prefix}_field_frame_mid_horizontal", "0 0.360 0.600 0 0 0", "1.080 0.026 0.026", frame_color, collision=False)
    add_pickup_yarn_chain(world, prefix)


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
