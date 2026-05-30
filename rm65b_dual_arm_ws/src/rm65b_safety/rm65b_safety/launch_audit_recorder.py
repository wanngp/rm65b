#!/usr/bin/env python3
from __future__ import annotations

import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


EXPECTED_D5_NODES = [
    "/safety_supervisor",
    "/d5_latency_probe",
    "/launch_audit_recorder",
    "/left_gripper_controller/gripper_action_server",
    "/right_gripper_controller/gripper_action_server",
    "/left_vision/left_aruco_target_node",
    "/ibvs_controller",
    "/dual_arm_planner",
    "/force_admittance_controller",
    "/gazebo_contact_force_estimator",
    "/force_gazebo_trajectory_relay",
    "/visual_servo_gazebo_adapter",
    "/weaving_bt_runner",
    "/tension_simulator",
]

EXPECTED_D5_TOPICS = [
    "/joint_states",
    "/left_camera/image_rect",
    "/vision/debug_image",
    "/vision/target_pose",
    "/visual_servo/twist_cmd",
    "/visual_servo/gazebo_adapter_state",
    "/force_control/target_wrench",
    "/force_control/wrench_error",
    "/force_control/admittance_offset",
    "/force_control/corrected_right_joint_states",
    "/weaving/events",
    "/weaving/tension_n",
    "/weaving/tension_status",
    "/safety/state",
    "/safety/fault",
    "/safety/estop",
    "/safety/reset_required",
    "/safety/recovery_command",
]


class LaunchAuditRecorder(Node):
    def __init__(self) -> None:
        super().__init__("launch_audit_recorder")
        self.declare_parameter("output_json", "outputs/d5_system/day05_launch_audit.json")
        self.declare_parameter("write_period_s", 1.0)
        self.output_json = Path(str(self.get_parameter("output_json").value))
        self.pub = self.create_publisher(String, "/system/launch_audit", 10)
        self.create_timer(float(self.get_parameter("write_period_s").value), self._write_record)
        self.get_logger().info(f"Launch audit recorder writing {self.output_json}")

    def _write_record(self) -> None:
        nodes = sorted(
            f"{namespace.rstrip('/')}/{name}".replace("//", "/")
            for name, namespace in self.get_node_names_and_namespaces()
        )
        topics = {
            topic: types
            for topic, types in sorted(self.get_topic_names_and_types())
        }
        missing_nodes = [node for node in EXPECTED_D5_NODES if node not in nodes]
        missing_topics = [topic for topic in EXPECTED_D5_TOPICS if topic not in topics]
        payload = {
            "node": self.get_name(),
            "expected_nodes": EXPECTED_D5_NODES,
            "expected_topics": EXPECTED_D5_TOPICS,
            "observed_nodes": nodes,
            "observed_topics": topics,
            "missing_nodes": missing_nodes,
            "missing_topics": missing_topics,
            "acceptance_pass": not missing_nodes and not missing_topics,
        }
        self.output_json.parent.mkdir(parents=True, exist_ok=True)
        self.output_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
        self.pub.publish(String(data=json.dumps({
            "acceptance_pass": payload["acceptance_pass"],
            "missing_nodes": missing_nodes,
            "missing_topics": missing_topics,
        }, sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaunchAuditRecorder()
    try:
        rclpy.spin(node)
    finally:
        node._write_record()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
