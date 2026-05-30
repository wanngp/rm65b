#!/usr/bin/env python3
from __future__ import annotations

import importlib
import json
import math
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger


def _resolve_msg_type(type_name: str):
    parts = str(type_name).split("/")
    if len(parts) == 2:
        module_name, class_name = parts
    elif len(parts) == 3 and parts[1] == "msg":
        module_name, _, class_name = parts
    else:
        raise ValueError(f"unsupported ROS message type name: {type_name}")
    module = importlib.import_module(f"{module_name}.msg")
    return getattr(module, class_name)


class SafetySupervisor(Node):
    def __init__(self) -> None:
        super().__init__("safety_supervisor")
        self.declare_parameter("watchdog_timeout_s", 0.5)
        self.declare_parameter("max_force_n", 35.0)
        self.declare_parameter("max_torque_nm", 4.0)
        self.declare_parameter("max_joint_velocity_rad_s", 1.2)
        self.declare_parameter("stop_topics", ["/left_rm_driver/rm_driver/move_stop_cmd"])
        self.declare_parameter("force_topics", ["/left_rm_driver/rm_driver/udp_six_force"])
        self.declare_parameter("force_message_type", "rm_ros_interfaces/Sixforce")
        self.declare_parameter("collision_topics", [])
        self.declare_parameter("collision_message_type", "ros_gz_interfaces/Contacts")
        self.declare_parameter("collision_stop_min_contacts", 1)
        self.declare_parameter("hardware_estop_topic", "/safety/hardware_estop")
        self.declare_parameter("fault_injection_topic", "/safety/inject_fault")
        self.declare_parameter("joint_limit_names_csv", "")
        self.declare_parameter("joint_limit_low_csv", "")
        self.declare_parameter("joint_limit_high_csv", "")
        self.declare_parameter("reset_requires_operator_ack", True)
        self.declare_parameter("recovery_home_required", True)

        self.watchdog_timeout_s = float(self.get_parameter("watchdog_timeout_s").value)
        self.max_force_n = float(self.get_parameter("max_force_n").value)
        self.max_torque_nm = float(self.get_parameter("max_torque_nm").value)
        self.max_joint_velocity_rad_s = float(self.get_parameter("max_joint_velocity_rad_s").value)
        self.collision_stop_min_contacts = int(
            self.get_parameter("collision_stop_min_contacts").value
        )
        self.reset_requires_operator_ack = bool(
            self.get_parameter("reset_requires_operator_ack").value
        )
        self.recovery_home_required = bool(self.get_parameter("recovery_home_required").value)
        self.joint_limits = self._load_joint_limits()

        self.estopped = False
        self.reset_required = False
        self.operator_ack = not self.reset_requires_operator_ack
        self.fault_type = ""
        self.fault_reason = ""
        self.last_joint_time = self.get_clock().now()
        self.last_joint_positions: dict[str, float] = {}
        self.last_joint_stamp_s: float | None = None

        self.estop_pub = self.create_publisher(Bool, "/safety/estop", 10)
        self.state_pub = self.create_publisher(String, "/safety/state", 10)
        self.fault_pub = self.create_publisher(String, "/safety/fault", 10)
        self.reset_required_pub = self.create_publisher(Bool, "/safety/reset_required", 10)
        self.recovery_pub = self.create_publisher(String, "/safety/recovery_command", 10)
        self.stop_publishers = [
            self.create_publisher(Empty, topic, 10)
            for topic in self.get_parameter("stop_topics").value
        ]

        self.create_subscription(JointState, "/joint_states", self._joint_callback, 10)
        self.create_subscription(
            Bool,
            str(self.get_parameter("hardware_estop_topic").value),
            self._hardware_estop_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("fault_injection_topic").value),
            self._fault_injection_callback,
            10,
        )
        self._create_force_subscribers()
        self._create_collision_subscribers()
        self.create_service(Trigger, "/safety/reset", self._reset_callback)
        self.create_service(Trigger, "/safety/ack_reset", self._ack_reset_callback)
        self.create_service(Trigger, "/safety/recover_home", self._recover_home_callback)
        self.create_service(Trigger, "/safety/stop_all", self._stop_all_callback)
        self.create_timer(0.1, self._watchdog)
        self.get_logger().info("Safety supervisor armed")

    def _create_force_subscribers(self) -> None:
        type_name = self.get_parameter("force_message_type").value
        topics = self.get_parameter("force_topics").value
        try:
            msg_type = _resolve_msg_type(type_name)
        except Exception as exc:
            self.get_logger().warn(
                f"Force message type {type_name} unavailable ({exc}); force checks disabled"
            )
            return
        for topic in topics:
            self.create_subscription(msg_type, topic, self._force_callback, 10)

    def _create_collision_subscribers(self) -> None:
        type_name = self.get_parameter("collision_message_type").value
        topics = self.get_parameter("collision_topics").value
        if not topics:
            return
        try:
            msg_type = _resolve_msg_type(type_name)
        except Exception as exc:
            self.get_logger().warn(
                f"Collision message type {type_name} unavailable ({exc}); collision checks disabled"
            )
            return
        for topic in topics:
            self.create_subscription(msg_type, topic, self._collision_callback, 10)

    def _load_joint_limits(self) -> dict[str, tuple[float, float]]:
        names = self._split_csv(self.get_parameter("joint_limit_names_csv").value)
        lows = self._split_csv(self.get_parameter("joint_limit_low_csv").value)
        highs = self._split_csv(self.get_parameter("joint_limit_high_csv").value)
        if not names:
            return {}
        if not (len(names) == len(lows) == len(highs)):
            self.get_logger().warn("Joint limit arrays differ in length; joint checks disabled")
            return {}
        return {
            str(name): (float(low), float(high))
            for name, low, high in zip(names, lows, highs)
        }

    @staticmethod
    def _split_csv(value: str) -> list[str]:
        if not value:
            return []
        return [item.strip() for item in str(value).split(",") if item.strip()]

    def _joint_callback(self, msg: JointState) -> None:
        self.last_joint_time = self.get_clock().now()
        now_s = self.last_joint_time.nanoseconds / 1e9
        velocity_by_name = self._joint_velocity_by_name(msg, now_s)
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_limits:
                low, high = self.joint_limits[name]
                if position < low or position > high:
                    self._trigger_estop(
                        "joint_limit",
                        f"joint limit exceeded: {name}={position:.3f} rad, limit=[{low:.3f},{high:.3f}]",
                    )
            velocity = velocity_by_name.get(name)
            if velocity is not None and abs(velocity) > self.max_joint_velocity_rad_s:
                self._trigger_estop(
                    "joint_velocity",
                    f"joint velocity exceeded: {name}={velocity:.3f} rad/s",
                )
        self.last_joint_positions = {
            name: float(position) for name, position in zip(msg.name, msg.position)
        }
        self.last_joint_stamp_s = now_s

    def _joint_velocity_by_name(self, msg: JointState, now_s: float) -> dict[str, float]:
        if len(msg.velocity) == len(msg.name):
            return {name: float(velocity) for name, velocity in zip(msg.name, msg.velocity)}
        if self.last_joint_stamp_s is None:
            return {}
        dt = max(1e-6, now_s - self.last_joint_stamp_s)
        velocities: dict[str, float] = {}
        for name, position in zip(msg.name, msg.position):
            if name in self.last_joint_positions:
                velocities[name] = (float(position) - self.last_joint_positions[name]) / dt
        return velocities

    def _force_callback(self, msg: Any) -> None:
        values = [
            float(getattr(msg, "force_fx", 0.0)),
            float(getattr(msg, "force_fy", 0.0)),
            float(getattr(msg, "force_fz", 0.0)),
            float(getattr(msg, "force_mx", 0.0)),
            float(getattr(msg, "force_my", 0.0)),
            float(getattr(msg, "force_mz", 0.0)),
        ]
        force_norm = math.sqrt(sum(v * v for v in values[:3]))
        torque_norm = math.sqrt(sum(v * v for v in values[3:]))
        if force_norm > self.max_force_n:
            self._trigger_estop("force_limit", f"force limit exceeded: {force_norm:.2f} N")
        if torque_norm > self.max_torque_nm:
            self._trigger_estop("torque_limit", f"torque limit exceeded: {torque_norm:.2f} Nm")

    def _collision_callback(self, msg: Any) -> None:
        contacts = getattr(msg, "contacts", [])
        contact_count = len(contacts) if contacts is not None else 0
        if contact_count >= self.collision_stop_min_contacts:
            self._trigger_estop("collision", f"collision contact count={contact_count}")

    def _hardware_estop_callback(self, msg: Bool) -> None:
        if msg.data:
            self._trigger_estop("hardware_estop", "hardware estop input asserted")

    def _fault_injection_callback(self, msg: String) -> None:
        text = msg.data.strip() or "manual fault injection"
        lowered = text.lower()
        if "force" in lowered:
            self._trigger_estop("force_limit", f"injected fault: {text}")
        elif "joint" in lowered:
            self._trigger_estop("joint_limit", f"injected fault: {text}")
        elif "collision" in lowered:
            self._trigger_estop("collision", f"injected fault: {text}")
        elif "hardware" in lowered or "estop" in lowered:
            self._trigger_estop("hardware_estop", f"injected fault: {text}")
        else:
            self._trigger_estop("manual_fault", f"injected fault: {text}")

    def _watchdog(self) -> None:
        age = (self.get_clock().now() - self.last_joint_time).nanoseconds / 1e9
        if age > self.watchdog_timeout_s:
            self._publish_state("WARN", f"joint state stale {age:.2f}s")
        else:
            self._publish_state("ESTOP" if self.estopped else "RUNNING", self.fault_reason)
        self.estop_pub.publish(Bool(data=self.estopped))
        self.reset_required_pub.publish(Bool(data=self.reset_required))

    def _trigger_estop(self, fault_type: str, reason: str) -> None:
        if not self.estopped:
            self.get_logger().error(f"Emergency stop triggered: {fault_type}: {reason}")
        self.estopped = True
        self.reset_required = True
        self.operator_ack = False if self.reset_requires_operator_ack else True
        self.fault_type = fault_type
        self.fault_reason = reason
        for publisher in self.stop_publishers:
            publisher.publish(Empty())
        if self.recovery_home_required:
            self.recovery_pub.publish(String(data="return_home_after_ack"))
        self.fault_pub.publish(String(data=self._state_json("ESTOP", reason)))
        self._publish_state("ESTOP", reason)
        self.estop_pub.publish(Bool(data=True))
        self.reset_required_pub.publish(Bool(data=True))

    def _state_json(self, mode: str, message: str) -> str:
        stamp = self.get_clock().now().nanoseconds / 1e9
        payload = {
            "stamp_s": round(stamp, 6),
            "mode": mode,
            "estopped": self.estopped,
            "fault_type": self.fault_type,
            "reason": message,
            "reset_required": self.reset_required,
            "operator_ack": self.operator_ack,
            "limits": {
                "force_n": self.max_force_n,
                "torque_nm": self.max_torque_nm,
                "joint_velocity_rad_s": self.max_joint_velocity_rad_s,
            },
        }
        return json.dumps(payload, sort_keys=True)

    def _publish_state(self, mode: str, message: str) -> None:
        self.state_pub.publish(String(data=self._state_json(mode, message)))

    def _ack_reset_callback(self, _request, response):
        self.operator_ack = True
        self._publish_state("ACK", "operator reset acknowledgement received")
        response.success = True
        response.message = "operator acknowledgement recorded"
        return response

    def _recover_home_callback(self, _request, response):
        self.recovery_pub.publish(String(data="return_home"))
        self._publish_state("RECOVERY", "return_home recovery command published")
        response.success = True
        response.message = "return_home command published on /safety/recovery_command"
        return response

    def _reset_callback(self, _request, response):
        if self.reset_requires_operator_ack and not self.operator_ack:
            self._publish_state("RESET_BLOCKED", "operator acknowledgement required")
            response.success = False
            response.message = "call /safety/ack_reset before /safety/reset"
            return response
        self.estopped = False
        self.reset_required = False
        self.fault_type = ""
        self.fault_reason = ""
        self.operator_ack = not self.reset_requires_operator_ack
        self._publish_state("RESET", "software estop flag reset")
        self.estop_pub.publish(Bool(data=False))
        self.reset_required_pub.publish(Bool(data=False))
        response.success = True
        response.message = "software estop flag reset"
        return response

    def _stop_all_callback(self, _request, response):
        self._trigger_estop("manual_stop", "manual stop_all service")
        response.success = True
        response.message = "stop commands published"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
