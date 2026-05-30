#!/usr/bin/env python3
from __future__ import annotations

import math
import time
from dataclasses import dataclass

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

try:
    from control_msgs.action import GripperCommand
except Exception:  # pragma: no cover - optional dry-run fallback
    GripperCommand = None


@dataclass
class GripperState:
    position: float
    effort: float
    moving: bool = False


class GripperActionServer(Node):
    """Simulation-first gripper server with hardware protocol guardrails.

    Hardware writes are intentionally disabled unless `hardware_enabled` is
    true. The concrete serial/Modbus/valve protocol must be supplied in the
    YAML before site use.
    """

    def __init__(self) -> None:
        super().__init__("gripper_action_server")
        self.declare_parameter("gripper_name", "rm65b_gripper")
        self.declare_parameter("joint_name", "finger_joint")
        self.declare_parameter("backend", "simulation")
        self.declare_parameter("hardware_enabled", False)
        self.declare_parameter("open_position_m", 0.045)
        self.declare_parameter("closed_position_m", 0.0)
        self.declare_parameter("max_effort_n", 30.0)
        self.declare_parameter("position_tolerance_m", 0.001)
        self.declare_parameter("simulated_latency_s", 0.05)
        self.declare_parameter("serial_port", "")
        self.declare_parameter("modbus_host", "")
        self.declare_parameter("modbus_port", 502)
        self.declare_parameter("valve_open_channel", 0)
        self.declare_parameter("valve_close_channel", 1)

        self.gripper_name = self.get_parameter("gripper_name").value
        self.joint_name = self.get_parameter("joint_name").value
        self.backend = self.get_parameter("backend").value
        self.hardware_enabled = self._as_bool(self.get_parameter("hardware_enabled").value)
        self.open_position = float(self.get_parameter("open_position_m").value)
        self.closed_position = float(self.get_parameter("closed_position_m").value)
        self.max_effort = float(self.get_parameter("max_effort_n").value)
        self.position_tolerance = float(self.get_parameter("position_tolerance_m").value)
        self.simulated_latency = float(self.get_parameter("simulated_latency_s").value)

        self.state = GripperState(position=self.open_position, effort=0.0)
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.timer = self.create_timer(0.05, self._publish_joint_state)

        self.action_server = None
        if GripperCommand is not None:
            self.action_server = ActionServer(
                self,
                GripperCommand,
                "gripper_cmd",
                execute_callback=self._execute,
                goal_callback=self._goal_callback,
                cancel_callback=self._cancel_callback,
            )
        else:
            self.get_logger().warn(
                "control_msgs is unavailable; gripper is running in service-only dry-run mode"
            )
        self.create_service(Trigger, "open", self._open_service)
        self.create_service(Trigger, "close", self._close_service)

        self.get_logger().info(
            f"{self.gripper_name} ready, backend={self.backend}, "
            f"hardware_enabled={self.hardware_enabled}"
        )

    def _goal_callback(self, goal_request: GripperCommand.Goal) -> GoalResponse:
        position = float(goal_request.command.position)
        if not math.isfinite(position):
            self.get_logger().error("Rejecting gripper goal with non-finite position")
            return GoalResponse.REJECT
        lower = min(self.closed_position, self.open_position) - self.position_tolerance
        upper = max(self.closed_position, self.open_position) + self.position_tolerance
        if not lower <= position <= upper:
            self.get_logger().error(
                f"Rejecting gripper goal {position:.4f}; allowed [{lower:.4f}, {upper:.4f}]"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        self.state.moving = False
        self.get_logger().warn("Gripper goal cancelled")
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle) -> GripperCommand.Result:
        command = goal_handle.request.command
        target = float(command.position)
        effort = min(abs(float(command.max_effort or self.max_effort)), self.max_effort)
        self.state.moving = True
        start = time.monotonic()

        feedback = GripperCommand.Feedback()
        feedback.position = self.state.position
        feedback.effort = self.state.effort
        feedback.stalled = False
        feedback.reached_goal = False
        goal_handle.publish_feedback(feedback)

        ok, message = self._send_to_backend(target, effort)
        if not ok:
            goal_handle.abort()
            self.state.moving = False
            result = GripperCommand.Result()
            result.position = self.state.position
            result.effort = self.state.effort
            result.stalled = True
            result.reached_goal = False
            self.get_logger().error(message)
            return result

        while time.monotonic() - start < self.simulated_latency:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.state.moving = False
                result = GripperCommand.Result()
                result.position = self.state.position
                result.effort = self.state.effort
                result.stalled = False
                result.reached_goal = False
                return result
            time.sleep(0.005)

        self.state.position = target
        self.state.effort = effort
        self.state.moving = False
        goal_handle.succeed()

        result = GripperCommand.Result()
        result.position = self.state.position
        result.effort = self.state.effort
        result.stalled = False
        result.reached_goal = True
        return result

    def _send_to_backend(self, target: float, effort: float) -> tuple[bool, str]:
        if self.backend == "simulation":
            return True, "simulation command accepted"

        if not self.hardware_enabled:
            return (
                False,
                "hardware backend requested but hardware_enabled=false; "
                "set and validate protocol YAML before real gripper motion",
            )

        if self.backend == "serial" and not self.get_parameter("serial_port").value:
            return False, "serial backend requires serial_port"
        if self.backend == "modbus_tcp" and not self.get_parameter("modbus_host").value:
            return False, "modbus_tcp backend requires modbus_host"
        if self.backend == "valve_io":
            return True, f"valve_io command target={target:.4f}, effort={effort:.2f}"

        return True, f"{self.backend} command target={target:.4f}, effort={effort:.2f}"

    def _publish_joint_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.joint_name]
        msg.position = [self.state.position]
        msg.effort = [self.state.effort]
        self.joint_pub.publish(msg)

    def _open_service(self, _request, response):
        self.state.position = self.open_position
        response.success = True
        response.message = "gripper opened in local state"
        return response

    def _close_service(self, _request, response):
        self.state.position = self.closed_position
        response.success = True
        response.message = "gripper closed in local state"
        return response

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.lower() in {"1", "true", "yes", "on"}
        return bool(value)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperActionServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
