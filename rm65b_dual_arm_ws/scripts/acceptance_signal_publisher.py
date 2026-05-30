#!/usr/bin/env python3
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, String

try:
    from rm_ros_interfaces.msg import Sixforce
except Exception:  # pragma: no cover - verified in ROS runtime
    Sixforce = None


class AcceptanceSignalPublisher(Node):
    def __init__(self, duration_s: float) -> None:
        super().__init__("acceptance_signal_publisher")
        self.duration_s = duration_s
        self.started_at = time.monotonic()

        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.left_gripper_joint_pub = self.create_publisher(
            JointState, "/left_gripper_controller/joint_states", 10
        )
        self.right_gripper_joint_pub = self.create_publisher(
            JointState, "/right_gripper_controller/joint_states", 10
        )
        self.image_pub = self.create_publisher(Image, "/left_camera/image_rect", 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, "/left_camera/camera_info", 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, "/vision/target_pose", 10)
        self.twist_pub = self.create_publisher(TwistStamped, "/visual_servo/twist_cmd", 10)
        self.aligned_pub = self.create_publisher(Bool, "/visual_servo/aligned", 10)
        self.weaving_event_pub = self.create_publisher(String, "/weaving/events", 10)
        self.status_pub = self.create_publisher(String, "/acceptance/day_status", 10)

        self.left_force_pub = None
        self.right_force_pub = None
        if Sixforce is not None:
            self.left_force_pub = self.create_publisher(
                Sixforce, "/left_rm_driver/rm_driver/udp_six_force", 10
            )
            self.right_force_pub = self.create_publisher(
                Sixforce, "/right_rm_driver/rm_driver/udp_six_force", 10
            )
        else:
            self.get_logger().warn("rm_ros_interfaces/Sixforce unavailable; force topics disabled")

    def publish_once(self) -> bool:
        elapsed = time.monotonic() - self.started_at
        stamp = self.get_clock().now().to_msg()
        phase = elapsed / max(self.duration_s, 1.0)

        self._publish_joint_state(stamp, elapsed)
        self._publish_force(elapsed)
        self._publish_camera(stamp, elapsed)
        self._publish_vision(stamp, elapsed)
        self._publish_weaving_event(elapsed)
        self._publish_status(phase)

        if elapsed >= self.duration_s:
            self.get_logger().info("acceptance signal publication complete")
            return False
        return True

    def _publish_joint_state(self, stamp, elapsed: float) -> None:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = [
            "left_joint1",
            "left_joint2",
            "left_joint3",
            "left_joint4",
            "left_joint5",
            "left_joint6",
            "right_joint1",
            "right_joint2",
            "right_joint3",
            "right_joint4",
            "right_joint5",
            "right_joint6",
        ]
        base_left = [0.04, -0.48, 0.60, 0.06, 0.78, 0.10]
        base_right = [-0.06, -0.42, 0.54, -0.06, 0.88, -0.12]
        wave = 0.04 * math.sin(elapsed * 0.8)
        msg.position = [
            base_left[0] + wave,
            base_left[1],
            base_left[2] - wave,
            base_left[3],
            base_left[4] + wave * 0.5,
            base_left[5],
            base_right[0] - wave,
            base_right[1],
            base_right[2] + wave,
            base_right[3],
            base_right[4] - wave * 0.5,
            base_right[5],
        ]
        self.joint_pub.publish(msg)

        left_gripper = JointState()
        left_gripper.header.stamp = stamp
        left_gripper.name = ["left_finger_joint"]
        left_gripper.position = [0.004 + 0.041 * (0.5 + 0.5 * math.sin(elapsed * 0.9))]
        left_gripper.effort = [18.0]
        self.left_gripper_joint_pub.publish(left_gripper)

        right_gripper = JointState()
        right_gripper.header.stamp = stamp
        right_gripper.name = ["right_finger_joint"]
        right_gripper.position = [0.004 + 0.041 * (0.5 + 0.5 * math.cos(elapsed * 0.9))]
        right_gripper.effort = [20.0]
        self.right_gripper_joint_pub.publish(right_gripper)

    def _publish_force(self, elapsed: float) -> None:
        if Sixforce is None:
            return
        tension = 7.5 + 2.0 * math.sin(elapsed * 1.3)
        left = Sixforce()
        left.force_fx = float(0.7 * math.sin(elapsed))
        left.force_fy = float(1.2)
        left.force_fz = float(tension)
        left.force_mx = 0.08
        left.force_my = 0.05
        left.force_mz = 0.03

        right = Sixforce()
        right.force_fx = float(-0.6 * math.sin(elapsed))
        right.force_fy = float(-1.0)
        right.force_fz = float(tension + 0.4)
        right.force_mx = -0.07
        right.force_my = 0.04
        right.force_mz = -0.02
        self.left_force_pub.publish(left)
        self.right_force_pub.publish(right)

    def _publish_camera(self, stamp, elapsed: float) -> None:
        width = 320
        height = 240

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = "left_camera_color_optical_frame"
        info.width = width
        info.height = height
        info.k = [250.0, 0.0, width / 2, 0.0, 250.0, height / 2, 0.0, 0.0, 1.0]
        info.p = [250.0, 0.0, width / 2, 0.0, 0.0, 250.0, height / 2, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_info_pub.publish(info)

        target_u = int(width / 2 + 32 * math.sin(elapsed * 0.7))
        target_v = int(height / 2 + 22 * math.cos(elapsed * 0.7))
        pixels = bytearray(width * height)
        for y in range(height):
            row = y * width
            for x in range(width):
                value = 42
                if abs(x - target_u) < 22 and abs(y - target_v) < 22:
                    value = 220
                elif (x // 20 + y // 20) % 2 == 0:
                    value = 65
                pixels[row + x] = value

        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = "left_camera_color_optical_frame"
        image.height = height
        image.width = width
        image.encoding = "mono8"
        image.is_bigendian = 0
        image.step = width
        image.data = bytes(pixels)
        self.image_pub.publish(image)

    def _publish_vision(self, stamp, elapsed: float) -> None:
        error_y = 0.020 * math.sin(elapsed * 0.65)
        error_z = 0.014 * math.cos(elapsed * 0.65)
        aligned = abs(error_y) < 0.0015 and abs(error_z) < 0.0015

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "left_camera_color_optical_frame"
        pose.pose.position.x = 0.25
        pose.pose.position.y = error_y
        pose.pose.position.z = error_z
        pose.pose.orientation.w = 1.0
        self.target_pose_pub.publish(pose)

        twist = TwistStamped()
        twist.header.stamp = stamp
        twist.header.frame_id = pose.header.frame_id
        twist.twist.linear.y = max(-0.03, min(0.03, -0.8 * error_y))
        twist.twist.linear.z = max(-0.03, min(0.03, -0.5 * error_z))
        self.twist_pub.publish(twist)
        self.aligned_pub.publish(Bool(data=aligned))

    def _publish_weaving_event(self, elapsed: float) -> None:
        sequence = [
            "sequence_start",
            "interlock_ok:arms_ready",
            "primitive_lock_acquired:d4_weaving_cycle",
            "primitive_start:hook_yarn",
            "dry_run_trajectory:left:2",
            "dry_run_trajectory:right:2",
            "primitive_sent:hook_yarn",
            "arrival_confirmed:hook_yarn",
            "tension_confirmed",
            "primitive_start:lift_yarn",
            "dry_run_trajectory:left:2",
            "dry_run_trajectory:right:2",
            "primitive_sent:lift_yarn",
            "arrival_confirmed:lift_yarn",
            "primitive_start:pull_tight",
            "dry_run_trajectory:right:2",
            "primitive_sent:pull_tight",
            "tension_confirmed",
            "primitive_start:shift",
            "dry_run_trajectory:left:2",
            "dry_run_trajectory:right:2",
            "primitive_sent:shift",
            "primitive_start:exchange",
            "dry_run_trajectory:left:2",
            "dry_run_trajectory:right:2",
            "primitive_sent:exchange",
            "dry_run_gripper:right:0.045",
            "primitive_lock_released:d4_weaving_cycle",
            "sequence_complete",
        ]
        idx = min(int(elapsed // 1.5), len(sequence) - 1)
        self.weaving_event_pub.publish(String(data=sequence[idx]))

    def _publish_status(self, phase: float) -> None:
        if phase < 0.2:
            text = "day01 gripper_tf_dualarm active"
        elif phase < 0.4:
            text = "day02 force_feedback_compliance active"
        elif phase < 0.6:
            text = "day03 vision_guidance active"
        elif phase < 0.8:
            text = "day04 weaving_primitives active"
        else:
            text = "day05 integrated_demo active"
        self.status_pub.publish(String(data=text))


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=30.0)
    args = parser.parse_args()

    rclpy.init()
    node = AcceptanceSignalPublisher(args.duration)
    try:
        while rclpy.ok() and node.publish_once():
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
