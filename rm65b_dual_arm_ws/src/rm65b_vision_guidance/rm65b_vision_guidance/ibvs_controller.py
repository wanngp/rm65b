#!/usr/bin/env python3
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


class IbvsController(Node):
    def __init__(self) -> None:
        super().__init__("ibvs_controller")
        self.declare_parameter("target_pose_topic", "/vision/target_pose")
        self.declare_parameter("twist_topic", "/visual_servo/twist_cmd")
        self.declare_parameter("gain_xy", 0.8)
        self.declare_parameter("gain_z", 0.5)
        self.declare_parameter("max_linear_mps", 0.03)
        self.declare_parameter("target_tolerance_m", 0.001)

        self.gain_xy = float(self.get_parameter("gain_xy").value)
        self.gain_z = float(self.get_parameter("gain_z").value)
        self.max_linear = float(self.get_parameter("max_linear_mps").value)
        self.tolerance = float(self.get_parameter("target_tolerance_m").value)

        self.twist_pub = self.create_publisher(TwistStamped, self.get_parameter("twist_topic").value, 10)
        self.aligned_pub = self.create_publisher(Bool, "/visual_servo/aligned", 10)
        self.create_subscription(
            PoseStamped,
            self.get_parameter("target_pose_topic").value,
            self._target_callback,
            10,
        )

    def _target_callback(self, msg: PoseStamped) -> None:
        error_y = msg.pose.position.y
        error_z = msg.pose.position.z
        norm = math.sqrt(error_y * error_y + error_z * error_z)
        aligned = norm <= self.tolerance

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = msg.header.frame_id
        twist.twist.linear.y = self._clamp(-self.gain_xy * error_y)
        twist.twist.linear.z = self._clamp(-self.gain_z * error_z)
        if aligned:
            twist.twist.linear.y = 0.0
            twist.twist.linear.z = 0.0
        self.twist_pub.publish(twist)
        self.aligned_pub.publish(Bool(data=aligned))

    def _clamp(self, value: float) -> float:
        return max(-self.max_linear, min(self.max_linear, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IbvsController()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
