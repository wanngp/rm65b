#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path


EXPECTED_NODES = [
    "/static_transform_publisher_*",
    "/left_rm_driver",
    "/right_rm_driver",
    "/left_gripper_controller/gripper_action_server",
    "/right_gripper_controller/gripper_action_server",
    "/safety_supervisor",
    "/launch_audit_recorder",
    "/d5_latency_probe",
    "/gazebo_camera_info_publisher",
    "/left_vision/left_aruco_target_node",
    "/ibvs_controller",
    "/move_group",
    "/dual_arm_planner",
    "/force_admittance_controller",
    "/gazebo_contact_force_estimator",
    "/force_gazebo_trajectory_relay",
    "/visual_servo_gazebo_adapter",
    "/weaving_bt_runner",
    "/tension_simulator",
]

EXPECTED_EDGES = [
    ("Gazebo left eye-in-hand camera", "/left_camera/image_rect", "/left_vision/left_aruco_target_node"),
    ("/left_vision/left_aruco_target_node", "/vision/target_pose", "/ibvs_controller"),
    ("/ibvs_controller", "/visual_servo/twist_cmd", "/visual_servo_gazebo_adapter"),
    ("/visual_servo_gazebo_adapter", "/visual_servo/left_corrected_joint_trajectory", "Gazebo left arm"),
    ("Gazebo contacts", "/contacts/d2_* and /contacts/d4_*", "/gazebo_contact_force_estimator"),
    ("/gazebo_contact_force_estimator", "/right_rm_driver/rm_driver/udp_six_force", "/force_admittance_controller"),
    ("/force_admittance_controller", "/force_control/corrected_right_joint_trajectory", "/force_gazebo_trajectory_relay"),
    ("/dual_arm_planner", "/joint_states", "RViz RobotModel"),
    ("/weaving_bt_runner", "/weaving/events", "/tension_simulator"),
    ("/tension_simulator", "/weaving/tension_n", "/weaving_bt_runner"),
    ("/safety_supervisor", "/safety/estop and stop topics", "left/right rm_driver"),
    ("/safety_supervisor", "/safety/state /fault /reset_required", "recording terminal"),
    ("/launch_audit_recorder", "/system/launch_audit", "recording terminal"),
    ("/d5_latency_probe", "day05_latency_record.json", "report"),
]


def run_ros2(args: list[str], timeout_s: float = 4.0) -> tuple[bool, str]:
    try:
        result = subprocess.run(
            args,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout_s,
        )
    except Exception as exc:
        return False, str(exc)
    if result.returncode != 0:
        return False, result.stderr.strip()
    return True, result.stdout.strip()


def capture_live_graph() -> dict:
    ok_nodes, node_text = run_ros2(["ros2", "node", "list"])
    ok_topics, topic_text = run_ros2(["ros2", "topic", "list", "-t"])
    if not ok_nodes or not ok_topics:
        return {
            "mode": "expected_fallback",
            "live_error": {
                "nodes": "" if ok_nodes else node_text,
                "topics": "" if ok_topics else topic_text,
            },
            "nodes": EXPECTED_NODES,
            "edges": EXPECTED_EDGES,
        }
    topics = []
    for line in topic_text.splitlines():
        line = line.strip()
        if not line:
            continue
        if " [" in line and line.endswith("]"):
            topic, type_text = line.rsplit(" [", 1)
            topics.append({"name": topic, "type": type_text[:-1]})
        else:
            topics.append({"name": line, "type": ""})
    return {
        "mode": "live",
        "nodes": sorted(line.strip() for line in node_text.splitlines() if line.strip()),
        "topics": topics,
        "edges": EXPECTED_EDGES,
    }


def graph_to_dot(graph: dict) -> str:
    lines = ["digraph day05_system {", "  rankdir=LR;", "  node [shape=box, style=rounded];"]
    node_ids: dict[str, str] = {}

    def node_id(label: str) -> str:
        if label not in node_ids:
            node_ids[label] = f"n{len(node_ids) + 1}"
            lines.append(f'  {node_ids[label]} [label="{label}"];')
        return node_ids[label]

    for src, topic, dst in graph.get("edges", []):
        lines.append(f'  {node_id(src)} -> {node_id(dst)} [label="{topic}"];')
    lines.append("}")
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description="Capture or generate the D5 ROS2 node graph.")
    parser.add_argument("--output-json", type=Path, default=Path("outputs/d5_system/day05_node_graph.json"))
    parser.add_argument("--output-dot", type=Path, default=Path("outputs/d5_system/day05_node_graph.dot"))
    parser.add_argument("--expected-only", action="store_true")
    args = parser.parse_args()

    graph = {
        "mode": "expected",
        "nodes": EXPECTED_NODES,
        "edges": EXPECTED_EDGES,
    } if args.expected_only else capture_live_graph()

    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(graph, indent=2, ensure_ascii=False), encoding="utf-8")
    args.output_dot.write_text(graph_to_dot(graph), encoding="utf-8")
    print(f"wrote {args.output_json}")
    print(f"wrote {args.output_dot}")


if __name__ == "__main__":
    main()
