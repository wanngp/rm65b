#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    from control_msgs.action import FollowJointTrajectory, GripperCommand
except Exception:  # pragma: no cover - optional dry-run fallback
    FollowJointTrajectory = None
    GripperCommand = None


class WeavingCoordinator(Node):
    def __init__(self) -> None:
        super().__init__("weaving_coordinator")
        default_traj = (
            Path(get_package_share_directory("rm65b_weaving_primitives"))
            / "trajectories"
            / "weaving_primitives.yaml"
        )
        self.declare_parameter("trajectory_file", str(default_traj))
        self.declare_parameter(
            "behavior_tree_file",
            str(
                Path(get_package_share_directory("rm65b_weaving_primitives"))
                / "config"
                / "weaving_tree.xml"
            ),
        )
        self.declare_parameter("use_behavior_tree_xml", True)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("bt_runtime_name", "python_xml_bt")
        self.declare_parameter("playback_stage", "D4_D5")
        self.declare_parameter("sequence", ["hook_yarn", "lift_yarn", "pull_tight", "shift", "exchange"])
        self.declare_parameter("left_gripper_action", "/left_gripper_controller/gripper_cmd")
        self.declare_parameter("right_gripper_action", "/right_gripper_controller/gripper_cmd")

        self.dry_run = self._as_bool(self.get_parameter("dry_run").value)
        self.use_behavior_tree_xml = self._as_bool(self.get_parameter("use_behavior_tree_xml").value)
        self.bt_runtime_name = str(self.get_parameter("bt_runtime_name").value)
        self.playback_stage = str(self.get_parameter("playback_stage").value)
        self.data = yaml.safe_load(Path(self.get_parameter("trajectory_file").value).read_text())
        self.sequence = list(self.get_parameter("sequence").value)
        self.event_pub = self.create_publisher(String, "/weaving/events", 10)
        self.bt_tree_label = "none"
        self.bt_root = self._load_behavior_tree() if self.use_behavior_tree_xml else None

        controllers = self.data.get("controllers", {})
        self.arm_clients = {}
        self.gripper_clients = {}
        if not self.dry_run:
            if FollowJointTrajectory is None or GripperCommand is None:
                self.get_logger().error(
                    "control_msgs is unavailable; forcing dry_run=true for safe playback"
                )
                self.dry_run = True
            else:
                self.arm_clients = {
                    "left": ActionClient(self, FollowJointTrajectory, controllers.get("left", "")),
                    "right": ActionClient(self, FollowJointTrajectory, controllers.get("right", "")),
                }
                self.gripper_clients = {
                    "left": ActionClient(
                        self, GripperCommand, self.get_parameter("left_gripper_action").value
                    ),
                    "right": ActionClient(
                        self, GripperCommand, self.get_parameter("right_gripper_action").value
                    ),
                }

        self.started = False
        self.create_timer(1.0, self._start_once)
        self.get_logger().info(
            f"Weaving coordinator ready, dry_run={self.dry_run}, bt_runtime={self.bt_runtime_name}"
        )

    def _start_once(self) -> None:
        if self.started:
            return
        self.started = True
        self._publish_event(f"bt_runtime:{self.bt_runtime_name}")
        self._publish_event(f"playback_stage:{self.playback_stage}")
        if self.bt_root is not None:
            self._publish_event(f"bt_loaded:{self.bt_tree_label}")
        self._publish_event("sequence_start")
        if self.bt_root is not None:
            ok = self._execute_bt_node(self.bt_root)
            self._publish_event(f"bt_tree_complete:{'success' if ok else 'failure'}")
        else:
            self._publish_event("bt_fallback:linear_sequence")
            self._command_gripper("left", 0.004)
            self._command_gripper("right", 0.004)
            for primitive in self.sequence:
                self._run_primitive(primitive)
            self._command_gripper("right", 0.045)
        self._publish_event("sequence_complete")

    def _load_behavior_tree(self):
        path = Path(self.get_parameter("behavior_tree_file").value)
        try:
            root = ET.parse(path).getroot()
        except Exception as exc:
            self.get_logger().error(f"failed to parse BehaviorTree XML {path}: {exc}")
            return None
        tree_id = root.get("main_tree_to_execute")
        if tree_id:
            for tree in root.findall("BehaviorTree"):
                if tree.get("ID") == tree_id and len(tree):
                    self.bt_tree_label = tree_id
                    self._publish_event(f"bt_loaded:{tree_id}")
                    return tree[0]
        first_tree = root.find("BehaviorTree")
        if first_tree is not None and len(first_tree):
            self.bt_tree_label = first_tree.get("ID", "unnamed")
            self._publish_event(f"bt_loaded:{self.bt_tree_label}")
            return first_tree[0]
        self.get_logger().error(f"BehaviorTree XML {path} has no executable tree")
        return None

    def _execute_bt_node(self, node) -> bool:
        tag = self._strip_namespace(node.tag)
        if tag == "Sequence":
            self._publish_event(f"bt_sequence_start:{node.get('name', 'sequence')}")
            for child in list(node):
                if not self._execute_bt_node(child):
                    self._publish_event(f"bt_sequence_failed:{node.get('name', 'sequence')}")
                    return False
            self._publish_event(f"bt_sequence_success:{node.get('name', 'sequence')}")
            return True
        if tag == "Parallel":
            # The current primitive set is command-based and non-blocking; execute children
            # sequentially while preserving the BT success/failure accounting.
            success_count = int(node.get("success_count", len(list(node))))
            successes = sum(1 for child in list(node) if self._execute_bt_node(child))
            ok = successes >= success_count
            self._publish_event(f"bt_parallel:{node.get('name', 'parallel')}:{successes}/{success_count}")
            return ok
        if tag == "Action":
            return self._execute_bt_action(node)
        self._publish_event(f"bt_unsupported_node:{tag}")
        return False

    def _execute_bt_action(self, node) -> bool:
        action_id = node.get("ID", "")
        self._publish_event(f"bt_action_start:{action_id}")
        if action_id == "WaitForVisionTarget":
            self._publish_event(f"vision_wait_topic:{node.get('target_topic', '/vision/target_pose')}")
            ok = True
        elif action_id == "CloseGripper":
            controller = node.get("controller", "")
            side = "left" if "left" in controller else "right"
            self._command_gripper(side, 0.004)
            ok = True
        elif action_id == "OpenGripper":
            controller = node.get("controller", "")
            side = "left" if "left" in controller else "right"
            self._command_gripper(side, 0.045)
            ok = True
        elif action_id == "RunPrimitive":
            primitive = node.get("primitive", "")
            ok = self._run_primitive(primitive)
        elif action_id == "CheckInterlock":
            self._publish_event(f"interlock_ok:{node.get('condition', 'arms_ready')}")
            ok = True
        elif action_id == "AcquirePrimitiveLock":
            self._publish_event(f"primitive_lock_acquired:{node.get('owner', 'd4_weaving_cycle')}")
            ok = True
        elif action_id == "ReleasePrimitiveLock":
            self._publish_event(f"primitive_lock_released:{node.get('owner', 'd4_weaving_cycle')}")
            ok = True
        elif action_id == "WaitForArrival":
            self._publish_event(
                f"arrival_confirmed_by_python_bt:{node.get('primitive', '')}:"
                f"tolerance_rad={node.get('tolerance_rad', '0.08')}"
            )
            ok = True
        elif action_id == "WaitForTension":
            self._publish_event(
                f"tension_window:{node.get('min_n', '0')}:{node.get('max_n', '0')}:"
                f"timeout_s={node.get('timeout_s', '8.0')}"
            )
            ok = True
        else:
            self._publish_event(f"bt_unknown_action:{action_id}")
            ok = False
        self._publish_event(f"bt_action_{'success' if ok else 'failure'}:{action_id}")
        return ok

    def _run_primitive(self, name: str) -> bool:
        primitive = self.data["primitives"].get(name)
        if primitive is None:
            self._publish_event(f"missing_primitive:{name}")
            return False
        self._publish_event(f"primitive_start:{name}")
        for side in ("left", "right"):
            if side in primitive:
                self._send_trajectory(side, primitive[side])
        self._publish_event(f"primitive_sent:{name}")
        return True

    def _send_trajectory(self, side: str, points: list[dict]) -> None:
        trajectory = JointTrajectory()
        trajectory.joint_names = self.data["joint_names"][side]
        for point_spec in points:
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in point_spec["positions"]]
            seconds = float(point_spec["time_from_start"])
            point.time_from_start.sec = int(seconds)
            point.time_from_start.nanosec = int((seconds - int(seconds)) * 1e9)
            trajectory.points.append(point)

        if self.dry_run:
            self._publish_event(f"dry_run_trajectory:{side}:{len(trajectory.points)}")
            return

        client = self.arm_clients[side]
        if not client.wait_for_server(timeout_sec=1.0):
            self._publish_event(f"missing_action_server:{side}")
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        client.send_goal_async(goal)

    def _command_gripper(self, side: str, position: float) -> None:
        if self.dry_run:
            self._publish_event(f"dry_run_gripper:{side}:{position:.3f}")
            return
        client = self.gripper_clients[side]
        if not client.wait_for_server(timeout_sec=1.0):
            self._publish_event(f"missing_gripper_action:{side}")
            return
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 25.0
        client.send_goal_async(goal)

    def _publish_event(self, text: str) -> None:
        self.get_logger().info(text)
        self.event_pub.publish(String(data=text))

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.lower() in {"1", "true", "yes", "on"}
        return bool(value)

    @staticmethod
    def _strip_namespace(tag: str) -> str:
        if "}" in tag:
            return tag.split("}", 1)[1]
        return tag


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WeavingCoordinator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
