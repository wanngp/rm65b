#!/usr/bin/env python3
from __future__ import annotations

import copy
import textwrap
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
OFFICIAL_MOVEIT = SRC / "ros2_rm_robot" / "rm_moveit2_config" / "rm_65_config"
OFFICIAL_URDF = SRC / "ros2_rm_robot" / "rm_description" / "urdf" / "rm_65.urdf"
OFFICIAL_SRDF = OFFICIAL_MOVEIT / "config" / "rm_65_description.srdf"
OUT = SRC / "rm65b_dual_arm_moveit_config"


def _indent(tree: ET.ElementTree) -> None:
    try:
        ET.indent(tree, space="  ")
    except AttributeError:
        pass


def _prefix_model_children(source_robot: ET.Element, prefix: str) -> list[ET.Element]:
    children: list[ET.Element] = []
    for child in list(source_robot):
        cloned = copy.deepcopy(child)
        _prefix_element(cloned, prefix)
        children.append(cloned)
    return children


def _prefix_element(element: ET.Element, prefix: str) -> None:
    if element.tag in {"link", "joint", "transmission"} and "name" in element.attrib:
        element.set("name", prefix + element.get("name", ""))
    if element.tag in {"parent", "child"} and "link" in element.attrib:
        element.set("link", prefix + element.get("link", ""))
    if element.tag == "mimic" and "joint" in element.attrib:
        element.set("joint", prefix + element.get("joint", ""))
    for child in list(element):
        _prefix_element(child, prefix)


def _add_world_fixed_joint(robot: ET.Element, side: str, xyz: str, yaw: str) -> None:
    joint = ET.SubElement(robot, "joint", {"name": f"{side}_world_to_base", "type": "fixed"})
    ET.SubElement(joint, "origin", {"xyz": xyz, "rpy": f"0 0 {yaw}"})
    ET.SubElement(joint, "parent", {"link": "world"})
    ET.SubElement(joint, "child", {"link": f"{side}_base_link"})


def _add_box(parent: ET.Element, tag: str, name: str, xyz: str, size: str) -> None:
    element = ET.SubElement(parent, tag, {"name": name})
    ET.SubElement(element, "origin", {"xyz": xyz, "rpy": "0 0 0"})
    geometry = ET.SubElement(element, "geometry")
    ET.SubElement(geometry, "box", {"size": size})


GRIPPER_BOXES = (
    ("flange_adapter", "0 0 0", "0.038 0.038 0.012"),
    ("palm", "0.030 0 0", "0.040 0.026 0.018"),
    ("finger_upper", "0.070 0.018 0", "0.052 0.006 0.010"),
    ("finger_lower", "0.070 -0.018 0", "0.052 0.006 0.010"),
    ("upper_yarn_hook", "0.098 0.018 0", "0.014 0.014 0.010"),
    ("lower_yarn_hook", "0.098 -0.018 0", "0.014 0.014 0.010"),
)


def _add_gripper(robot: ET.Element, side: str) -> None:
    link = ET.SubElement(robot, "link", {"name": f"{side}_attached_scaled_gripper"})
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": "0.040 0 0", "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": "0.05"})
    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": "0.00002",
            "ixy": "0",
            "ixz": "0",
            "iyy": "0.00002",
            "iyz": "0",
            "izz": "0.00002",
        },
    )
    for tag in ("visual", "collision"):
        for name, xyz, size in GRIPPER_BOXES:
            _add_box(link, tag, name, xyz, size)

    joint = ET.SubElement(robot, "joint", {"name": f"{side}_attached_scaled_gripper_fixed", "type": "fixed"})
    ET.SubElement(joint, "origin", {"xyz": "0.018 0 0", "rpy": "0 0 0"})
    ET.SubElement(joint, "parent", {"link": f"{side}_Link6"})
    ET.SubElement(joint, "child", {"link": f"{side}_attached_scaled_gripper"})


def write_urdf() -> None:
    source_root = ET.parse(OFFICIAL_URDF).getroot()
    robot = ET.Element("robot", {"name": "rm65b_dual_arm"})
    ET.SubElement(robot, "link", {"name": "world"})
    _add_world_fixed_joint(robot, "left", "-0.45 0 0.02", "1.5708")
    _add_world_fixed_joint(robot, "right", "0.45 0 0.02", "-1.5708")
    for side in ("left", "right"):
        for child in _prefix_model_children(source_root, side + "_"):
            robot.append(child)
        _add_gripper(robot, side)
    tree = ET.ElementTree(robot)
    _indent(tree)
    tree.write(OUT / "config" / "rm65b_dual_arm.urdf", encoding="utf-8", xml_declaration=True)


def write_srdf() -> None:
    source = ET.parse(OFFICIAL_SRDF).getroot()
    robot = ET.Element("robot", {"name": "rm65b_dual_arm"})
    for side in ("left", "right"):
        group = ET.SubElement(robot, "group", {"name": f"{side}_arm"})
        ET.SubElement(group, "chain", {"base_link": f"{side}_base_link", "tip_link": f"{side}_Link6"})
    dual = ET.SubElement(robot, "group", {"name": "dual_arms"})
    ET.SubElement(dual, "group", {"name": "left_arm"})
    ET.SubElement(dual, "group", {"name": "right_arm"})
    for side in ("left", "right"):
        state = ET.SubElement(robot, "group_state", {"name": "ready", "group": f"{side}_arm"})
        for name, value in {
            "joint1": "0",
            "joint2": "-0.35",
            "joint3": "0.65",
            "joint4": "0",
            "joint5": "0.90",
            "joint6": "0",
        }.items():
            ET.SubElement(state, "joint", {"name": f"{side}_{name}", "value": value})
    dual_state = ET.SubElement(robot, "group_state", {"name": "dual_ready", "group": "dual_arms"})
    for side in ("left", "right"):
        for name, value in {
            "joint1": "0",
            "joint2": "-0.35",
            "joint3": "0.65",
            "joint4": "0",
            "joint5": "0.90",
            "joint6": "0",
        }.items():
            ET.SubElement(dual_state, "joint", {"name": f"{side}_{name}", "value": value})
    for disabled in source.findall("disable_collisions"):
        for side in ("left", "right"):
            ET.SubElement(
                robot,
                "disable_collisions",
                {
                    "link1": side + "_" + disabled.get("link1", ""),
                    "link2": side + "_" + disabled.get("link2", ""),
                    "reason": disabled.get("reason", "Adjacent"),
                },
            )
    for side in ("left", "right"):
        for link in (f"{side}_Link5", f"{side}_Link6"):
            ET.SubElement(
                robot,
                "disable_collisions",
                {
                    "link1": link,
                    "link2": f"{side}_attached_scaled_gripper",
                    "reason": "AdjacentTool",
                },
            )
        ET.SubElement(
            robot,
            "disable_collisions",
            {
                "link1": "world",
                "link2": "left_" + disabled.get("link1", ""),
                "reason": "FixedBase",
            },
        )
        ET.SubElement(
            robot,
            "disable_collisions",
            {
                "link1": "world",
                "link2": "right_" + disabled.get("link1", ""),
                "reason": "FixedBase",
            },
        )
    tree = ET.ElementTree(robot)
    _indent(tree)
    tree.write(OUT / "config" / "rm65b_dual_arm.srdf", encoding="utf-8", xml_declaration=True)


def write_yaml() -> None:
    official_limits = yaml.safe_load((OFFICIAL_MOVEIT / "config" / "joint_limits.yaml").read_text(encoding="utf-8"))
    joint_limits = {}
    for side in ("left", "right"):
        for name, limits in official_limits["joint_limits"].items():
            joint_limits[f"{side}_{name}"] = limits
    (OUT / "config" / "joint_limits.yaml").write_text(
        yaml.safe_dump(
            {
                "default_velocity_scaling_factor": 0.1,
                "default_acceleration_scaling_factor": 0.1,
                "joint_limits": joint_limits,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    kinematics = {
        "left_arm": {
            "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
            "kinematics_solver_search_resolution": 0.005,
            "kinematics_solver_timeout": 0.05,
        },
        "right_arm": {
            "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
            "kinematics_solver_search_resolution": 0.005,
            "kinematics_solver_timeout": 0.05,
        },
    }
    (OUT / "config" / "kinematics.yaml").write_text(yaml.safe_dump(kinematics, sort_keys=False), encoding="utf-8")
    controllers = {
        "moveit_simple_controller_manager": {
            "controller_names": ["left_arm_controller", "right_arm_controller"],
            "left_arm_controller": {
                "type": "FollowJointTrajectory",
                "action_ns": "follow_joint_trajectory",
                "default": True,
                "joints": [f"left_joint{i}" for i in range(1, 7)],
            },
            "right_arm_controller": {
                "type": "FollowJointTrajectory",
                "action_ns": "follow_joint_trajectory",
                "default": True,
                "joints": [f"right_joint{i}" for i in range(1, 7)],
            },
        }
    }
    (OUT / "config" / "moveit_controllers.yaml").write_text(
        yaml.safe_dump(controllers, sort_keys=False), encoding="utf-8"
    )
    (OUT / "config" / "ompl_planning.yaml").write_text(
        textwrap.dedent(
            """\
            planning_plugin: ompl_interface/OMPLPlanner
            request_adapters: >-
              default_planner_request_adapters/ResolveConstraintFrames
              default_planner_request_adapters/FixWorkspaceBounds
              default_planner_request_adapters/FixStartStateBounds
              default_planner_request_adapters/FixStartStateCollision
            start_state_max_bounds_error: 0.1
            planner_configs:
              RRTConnectkConfigDefault:
                type: geometric::RRTConnect
                range: 0.0
            left_arm:
              planner_configs:
                - RRTConnectkConfigDefault
            right_arm:
              planner_configs:
                - RRTConnectkConfigDefault
            dual_arms:
              planner_configs:
                - RRTConnectkConfigDefault
              projection_evaluator: joints(left_joint1,left_joint2,right_joint1,right_joint2)
            """
        ),
        encoding="utf-8",
    )


def write_launch_and_manifest() -> None:
    (OUT / "package.xml").write_text(
        textwrap.dedent(
            """\
            <?xml version="1.0"?>
            <package format="3">
              <name>rm65b_dual_arm_moveit_config</name>
              <version>0.1.0</version>
              <description>Dual RM65-B MoveIt2 configuration with fixed bases and attached scaled grippers.</description>
              <maintainer email="rm65b@example.invalid">RM65-B Experiment</maintainer>
              <license>BSD</license>
              <buildtool_depend>ament_cmake</buildtool_depend>
              <exec_depend>moveit_configs_utils</exec_depend>
              <exec_depend>moveit_ros_move_group</exec_depend>
              <exec_depend>moveit_ros_visualization</exec_depend>
              <exec_depend>moveit_kinematics</exec_depend>
              <exec_depend>moveit_planners_ompl</exec_depend>
              <exec_depend>robot_state_publisher</exec_depend>
              <exec_depend>rviz2</exec_depend>
              <exec_depend>rm_description</exec_depend>
              <exec_depend>xacro</exec_depend>
              <export>
                <build_type>ament_cmake</build_type>
              </export>
            </package>
            """
        ),
        encoding="utf-8",
    )
    (OUT / "CMakeLists.txt").write_text(
        textwrap.dedent(
            """\
            cmake_minimum_required(VERSION 3.22)
            project(rm65b_dual_arm_moveit_config)

            find_package(ament_cmake REQUIRED)

            install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

            ament_package()
            """
        ),
        encoding="utf-8",
    )
    (OUT / "launch" / "move_group.launch.py").write_text(
        textwrap.dedent(
            """\
            from moveit_configs_utils import MoveItConfigsBuilder
            from moveit_configs_utils.launches import generate_move_group_launch


            def generate_launch_description():
                moveit_config = (
                    MoveItConfigsBuilder("rm65b_dual_arm", package_name="rm65b_dual_arm_moveit_config")
                    .robot_description(file_path="config/rm65b_dual_arm.urdf")
                    .robot_description_semantic(file_path="config/rm65b_dual_arm.srdf")
                    .robot_description_kinematics(file_path="config/kinematics.yaml")
                    .joint_limits(file_path="config/joint_limits.yaml")
                    .trajectory_execution(file_path="config/moveit_controllers.yaml")
                    .planning_pipelines(pipelines=["ompl"])
                    .to_moveit_configs()
                )
                return generate_move_group_launch(moveit_config)
            """
        ),
        encoding="utf-8",
    )
    (OUT / "launch" / "demo.launch.py").write_text(
        textwrap.dedent(
            """\
            from launch import LaunchDescription
            from launch_ros.actions import Node
            from moveit_configs_utils import MoveItConfigsBuilder


            def generate_launch_description():
                moveit_config = (
                    MoveItConfigsBuilder("rm65b_dual_arm", package_name="rm65b_dual_arm_moveit_config")
                    .robot_description(file_path="config/rm65b_dual_arm.urdf")
                    .robot_description_semantic(file_path="config/rm65b_dual_arm.srdf")
                    .robot_description_kinematics(file_path="config/kinematics.yaml")
                    .joint_limits(file_path="config/joint_limits.yaml")
                    .planning_pipelines(pipelines=["ompl"])
                    .to_moveit_configs()
                )
                return LaunchDescription([
                    Node(
                        package="robot_state_publisher",
                        executable="robot_state_publisher",
                        parameters=[moveit_config.robot_description],
                        output="screen",
                    ),
                    Node(
                        package="moveit_ros_move_group",
                        executable="move_group",
                        output="screen",
                        parameters=[moveit_config.to_dict()],
                    ),
                ])
            """
        ),
        encoding="utf-8",
    )


def main() -> int:
    (OUT / "config").mkdir(parents=True, exist_ok=True)
    (OUT / "launch").mkdir(parents=True, exist_ok=True)
    write_urdf()
    write_srdf()
    write_yaml()
    write_launch_and_manifest()
    print(f"generated {OUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
