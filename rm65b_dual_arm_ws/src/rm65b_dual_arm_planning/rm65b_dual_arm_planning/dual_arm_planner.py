from __future__ import annotations

import math
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, Float64, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .rm65b_kinematics import build_keyframes, load_weaving_yaml, sample_program

try:
    from rm_ros_interfaces.msg import Sixforce
except Exception:  # pragma: no cover - verified in ROS runtime
    Sixforce = None


class DualArmPlanner(Node):
    def __init__(self) -> None:
        super().__init__("dual_arm_planner")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("loop", True)
        self.declare_parameter("day_id", "day05")
        self.declare_parameter("trajectory_file", "")
        self.declare_parameter("moveit_plan_file", "")
        self.declare_parameter("publish_synthetic_camera", True)
        self.declare_parameter("publish_synthetic_vision", True)
        self.declare_parameter("publish_simulated_force", True)
        trajectory_file = self.get_parameter("trajectory_file").get_parameter_value().string_value
        moveit_plan_file = self.get_parameter("moveit_plan_file").get_parameter_value().string_value
        self.day_id = self.get_parameter("day_id").get_parameter_value().string_value
        self.publish_synthetic_camera = (
            self.get_parameter("publish_synthetic_camera").get_parameter_value().bool_value
        )
        self.publish_simulated_force = (
            self.get_parameter("publish_simulated_force").get_parameter_value().bool_value
        )
        self.publish_synthetic_vision = (
            self.get_parameter("publish_synthetic_vision").get_parameter_value().bool_value
        )
        self.keyframes = build_keyframes(load_weaving_yaml(trajectory_file or None))
        self.moveit_points = self._load_moveit_points(moveit_plan_file)
        self.loop = self.get_parameter("loop").get_parameter_value().bool_value
        self.started_at = time.monotonic()
        self.force_admittance_offset = 0.0
        self.current_phase = self.day_id

        self.phase_pub = self.create_publisher(String, "/dual_arm_planning/phase", 10)
        self.event_pub = self.create_publisher(String, "/weaving/events", 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.left_traj_pub = self.create_publisher(
            JointTrajectory, "/dual_arm_planning/left_joint_trajectory", 10
        )
        self.right_traj_pub = self.create_publisher(
            JointTrajectory, "/dual_arm_planning/right_joint_trajectory", 10
        )
        self.status_pub = self.create_publisher(String, "/acceptance/day_status", 10)
        self.left_gripper_pub = self.create_publisher(
            JointState, "/left_gripper_controller/joint_states", 10
        )
        self.right_gripper_pub = self.create_publisher(
            JointState, "/right_gripper_controller/joint_states", 10
        )
        self.image_pub = None
        self.camera_info_pub = None
        if self.publish_synthetic_camera:
            self.image_pub = self.create_publisher(Image, "/left_camera/image_rect", 10)
            self.camera_info_pub = self.create_publisher(CameraInfo, "/left_camera/camera_info", 10)
        self.target_pub = None
        self.twist_pub = None
        self.aligned_pub = None
        if self.publish_synthetic_vision:
            self.target_pub = self.create_publisher(PoseStamped, "/vision/target_pose", 10)
            self.twist_pub = self.create_publisher(TwistStamped, "/visual_servo/twist_cmd", 10)
            self.aligned_pub = self.create_publisher(Bool, "/visual_servo/aligned", 10)
        self.create_subscription(
            Float64,
            "/force_control/admittance_offset",
            self._force_offset_callback,
            10,
        )
        self.left_force_pub = None
        self.right_force_pub = None
        if Sixforce is not None:
            self.left_force_pub = self.create_publisher(
                Sixforce, "/left_rm_driver/rm_driver/udp_six_force", 10
            )
            self.right_force_pub = self.create_publisher(
                Sixforce, "/right_rm_driver/rm_driver/udp_six_force", 10
            )

        self._publish_full_trajectories()
        self.last_trajectory_publish = time.monotonic()
        rate = max(self.get_parameter("publish_rate_hz").get_parameter_value().double_value, 1.0)
        self.timer = self.create_timer(1.0 / rate, self._tick)

    def _publish_full_trajectories(self) -> None:
        self.left_traj_pub.publish(self._trajectory("left"))
        self.right_traj_pub.publish(self._trajectory("right"))

    def _trajectory(self, side: str) -> JointTrajectory:
        msg = JointTrajectory()
        msg.joint_names = [f"{side}_joint{i}" for i in range(1, 7)]
        if self.moveit_points:
            for frame in self.moveit_points:
                point = JointTrajectoryPoint()
                point.positions = list(frame[side])
                point.time_from_start.sec = int(frame["time_from_start"])
                point.time_from_start.nanosec = int(
                    (frame["time_from_start"] - int(frame["time_from_start"])) * 1e9
                )
                msg.points.append(point)
            return msg
        for frame in self.keyframes:
            point = JointTrajectoryPoint()
            point.positions = list(frame.left if side == "left" else frame.right)
            point.time_from_start.sec = int(frame.time_s)
            point.time_from_start.nanosec = int((frame.time_s - int(frame.time_s)) * 1e9)
            msg.points.append(point)
        return msg

    def _load_moveit_points(self, plan_file: str) -> list[dict]:
        if not plan_file:
            return []
        path = Path(plan_file)
        if not path.exists():
            self.get_logger().warn(f"MoveIt plan file does not exist: {path}")
            return []
        with path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        points = data.get("points", [])
        self.get_logger().info(f"loaded {len(points)} MoveIt planned dual-arm points from {path}")
        return points

    def _sample_moveit(self, elapsed: float):
        duration = float(self.moveit_points[-1]["time_from_start"])
        t = elapsed % duration if self.loop and duration > 0.0 else min(max(elapsed, 0.0), duration)
        previous = self.moveit_points[0]
        current = self.moveit_points[-1]
        for idx in range(1, len(self.moveit_points)):
            if t <= float(self.moveit_points[idx]["time_from_start"]):
                previous = self.moveit_points[idx - 1]
                current = self.moveit_points[idx]
                break
        start_t = float(previous["time_from_start"])
        end_t = float(current["time_from_start"])
        alpha = 0.0 if end_t <= start_t else min(max((t - start_t) / (end_t - start_t), 0.0), 1.0)
        left = [
            float(a) + (float(b) - float(a)) * alpha
            for a, b in zip(previous["left"], current["left"])
        ]
        right = [
            float(a) + (float(b) - float(a)) * alpha
            for a, b in zip(previous["right"], current["right"])
        ]
        return {
            "phase": current.get("phase", self.day_id),
            "left": left,
            "right": right,
            "left_gripper": 0.006 if current.get("phase") in {"pick_yarn", "pull_tight"} else 0.028,
            "right_gripper": 0.008 if current.get("phase") in {"pull_tight", "shift"} else 0.028,
            "target_xyz": (0.08, 0.20, 0.36),
        }

    def _tick(self) -> None:
        elapsed = time.monotonic() - self.started_at
        now = time.monotonic()
        if now - self.last_trajectory_publish > 2.0:
            self._publish_full_trajectories()
            self.last_trajectory_publish = now
        sample = self._sample_moveit(elapsed) if self.moveit_points else sample_program(self.keyframes, elapsed, self.loop)
        stamp = self.get_clock().now().to_msg()

        joint = JointState()
        joint.header.stamp = stamp
        joint.name = [f"left_joint{i}" for i in range(1, 7)] + [
            f"right_joint{i}" for i in range(1, 7)
        ]
        left_position = list(sample["left"] if isinstance(sample, dict) else sample.left)
        right_position = list(sample["right"] if isinstance(sample, dict) else sample.right)
        if right_position:
            right_position[-1] += self.force_admittance_offset
        joint.position = left_position + right_position
        self.joint_pub.publish(joint)

        phase = sample["phase"] if isinstance(sample, dict) else sample.phase
        self.current_phase = phase
        left_gripper = sample["left_gripper"] if isinstance(sample, dict) else sample.left_gripper
        right_gripper = sample["right_gripper"] if isinstance(sample, dict) else sample.right_gripper
        self.phase_pub.publish(String(data=phase))
        self.event_pub.publish(String(data=f"moveit_planned_phase:{phase}:bases_fixed:true"))
        self.status_pub.publish(String(data=f"{self.day_id} active: moveit_dual_arm_planning"))
        self._publish_grippers(stamp, left_gripper, right_gripper)
        if self.publish_synthetic_camera:
            self._publish_camera(stamp, elapsed)
        if self.publish_synthetic_vision:
            self._publish_vision(stamp, sample)
        if self.publish_simulated_force:
            self._publish_force(elapsed)

    def _force_offset_callback(self, msg: Float64) -> None:
        self.force_admittance_offset = max(-0.08, min(0.08, float(msg.data)))

    def _publish_grippers(self, stamp, left_position: float, right_position: float) -> None:
        left = JointState()
        left.header.stamp = stamp
        left.name = ["left_finger_joint"]
        left.position = [float(left_position)]
        left.effort = [18.0]
        self.left_gripper_pub.publish(left)

        right = JointState()
        right.header.stamp = stamp
        right.name = ["right_finger_joint"]
        right.position = [float(right_position)]
        right.effort = [20.0]
        self.right_gripper_pub.publish(right)

    def _publish_camera(self, stamp, elapsed: float) -> None:
        if self.image_pub is None or self.camera_info_pub is None:
            return
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

        target_u = int(width / 2 + 34 * math.sin(elapsed * 0.45))
        target_v = int(height / 2 + 20 * math.cos(elapsed * 0.45))
        pixels = bytearray(width * height)
        for y in range(height):
            row = y * width
            for x in range(width):
                value = 44
                if abs(x - target_u) < 22 and abs(y - target_v) < 22:
                    value = 225
                elif (x // 18 + y // 18) % 2 == 0:
                    value = 68
                pixels[row + x] = value
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = info.header.frame_id
        image.height = height
        image.width = width
        image.encoding = "mono8"
        image.is_bigendian = 0
        image.step = width
        image.data = bytes(pixels)
        self.image_pub.publish(image)

    def _publish_vision(self, stamp, sample) -> None:
        if self.target_pub is None or self.twist_pub is None or self.aligned_pub is None:
            return
        target_xyz = sample["target_xyz"] if isinstance(sample, dict) else sample.target_xyz
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "left_camera_color_optical_frame"
        pose.pose.position.x = 0.25
        pose.pose.position.y = target_xyz[1] - 0.20
        pose.pose.position.z = target_xyz[2] - 0.36
        pose.pose.orientation.w = 1.0
        self.target_pub.publish(pose)

        twist = TwistStamped()
        twist.header.stamp = stamp
        twist.header.frame_id = pose.header.frame_id
        twist.twist.linear.y = max(-0.03, min(0.03, -0.8 * pose.pose.position.y))
        twist.twist.linear.z = max(-0.03, min(0.03, -0.6 * pose.pose.position.z))
        self.twist_pub.publish(twist)
        self.aligned_pub.publish(
            Bool(data=abs(pose.pose.position.y) < 0.003 and abs(pose.pose.position.z) < 0.003)
        )

    def _publish_force(self, elapsed: float) -> None:
        if Sixforce is None or self.left_force_pub is None or self.right_force_pub is None:
            return
        phase_bias = {
            "hook_yarn": 0.6,
            "pick_yarn": 1.1,
            "pull_tight": 2.2,
            "shift": 1.6,
            "complete": 0.2,
        }.get(self.current_phase, 0.8)
        plant_response = 22.0 * self.force_admittance_offset
        tension = 5.0 + phase_bias + plant_response + 0.6 * math.sin(elapsed * 0.55)
        left = Sixforce()
        left.force_fx = float(0.4 * math.sin(elapsed))
        left.force_fy = 1.1
        left.force_fz = float(tension)
        left.force_mx = 0.07
        left.force_my = 0.04
        left.force_mz = 0.02
        right = Sixforce()
        right.force_fx = float(-0.3 * math.sin(elapsed))
        right.force_fy = -1.0
        right.force_fz = float(tension + 0.6)
        right.force_mx = -0.06
        right.force_my = 0.04
        right.force_mz = -0.02
        self.left_force_pub.publish(left)
        self.right_force_pub.publish(right)


def main() -> None:
    rclpy.init()
    node = DualArmPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
