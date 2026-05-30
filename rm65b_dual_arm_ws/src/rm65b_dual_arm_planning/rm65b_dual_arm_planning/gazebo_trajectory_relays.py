from __future__ import annotations

import copy
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def _rename_for_gazebo(msg: JointTrajectory, prefix: str) -> JointTrajectory:
    out = copy.deepcopy(msg)
    names = []
    for name in out.joint_names:
        if name.startswith(prefix):
            names.append(name[len(prefix) :])
        else:
            names.append(name)
    out.joint_names = names
    return out


class ForceGazeboTrajectoryRelay(Node):
    """Send force-corrected right-arm trajectory back to Gazebo."""

    def __init__(self) -> None:
        super().__init__("force_gazebo_trajectory_relay")
        self.pub = self.create_publisher(
            JointTrajectory, "/model/right_rm65b/joint_trajectory", 10
        )
        self.state_pub = self.create_publisher(String, "/force_control/gazebo_relay_state", 10)
        self.create_subscription(
            JointTrajectory,
            "/force_control/corrected_right_joint_trajectory",
            self._trajectory_callback,
            10,
        )
        self.forwarded = 0

    def _trajectory_callback(self, msg: JointTrajectory) -> None:
        out = _rename_for_gazebo(msg, "right_")
        self.pub.publish(out)
        self.forwarded += 1
        self.state_pub.publish(
            String(data=f"forwarded_force_corrected_right_trajectory:{self.forwarded}")
        )


class VisualServoGazeboAdapter(Node):
    """Convert IBVS twist commands into small left-arm Gazebo trajectory updates."""

    def __init__(self) -> None:
        super().__init__("visual_servo_gazebo_adapter")
        self.declare_parameter("gain_joint1_per_mps", 0.55)
        self.declare_parameter("gain_joint2_per_mps", 0.45)
        self.declare_parameter("max_step_rad", 0.030)
        self.declare_parameter("publish_rate_hz", 15.0)
        self.gain_j1 = float(self.get_parameter("gain_joint1_per_mps").value)
        self.gain_j2 = float(self.get_parameter("gain_joint2_per_mps").value)
        self.max_step = float(self.get_parameter("max_step_rad").value)
        self.latest_left = [0.0, -0.35, 0.65, 0.0, 0.90, 0.0]
        self.latest_twist: TwistStamped | None = None
        self.last_command_time = time.monotonic()

        self.pub = self.create_publisher(
            JointTrajectory, "/model/left_rm65b/joint_trajectory", 10
        )
        self.corrected_pub = self.create_publisher(
            JointTrajectory, "/visual_servo/left_corrected_joint_trajectory", 10
        )
        self.state_pub = self.create_publisher(String, "/visual_servo/gazebo_adapter_state", 10)
        self.create_subscription(JointState, "/joint_states", self._joint_state_callback, 10)
        self.create_subscription(TwistStamped, "/visual_servo/twist_cmd", self._twist_callback, 10)
        rate = max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self.create_timer(1.0 / rate, self._tick)

    def _joint_state_callback(self, msg: JointState) -> None:
        values = []
        for idx in range(1, 7):
            name = f"left_joint{idx}"
            if name in msg.name:
                values.append(float(msg.position[msg.name.index(name)]))
        if len(values) == 6:
            self.latest_left = values

    def _twist_callback(self, msg: TwistStamped) -> None:
        self.latest_twist = msg

    def _tick(self) -> None:
        if self.latest_twist is None:
            return
        now = time.monotonic()
        if now - self.last_command_time < 0.04:
            return
        self.last_command_time = now
        dy = float(self.latest_twist.twist.linear.y)
        dz = float(self.latest_twist.twist.linear.z)
        positions = list(self.latest_left)
        positions[0] += self._clamp(dy * self.gain_j1)
        positions[1] += self._clamp(dz * self.gain_j2)

        traj = JointTrajectory()
        traj.joint_names = [f"joint{i}" for i in range(1, 7)]
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 1
        traj.points.append(point)
        self.pub.publish(traj)

        ros_traj = copy.deepcopy(traj)
        ros_traj.joint_names = [f"left_joint{i}" for i in range(1, 7)]
        self.corrected_pub.publish(ros_traj)
        self.state_pub.publish(
            String(
                data=(
                    "ibvs_to_gazebo "
                    f"twist_y={dy:.5f} twist_z={dz:.5f} "
                    f"joint1={positions[0]:.5f} joint2={positions[1]:.5f}"
                )
            )
        )

    def _clamp(self, value: float) -> float:
        return max(-self.max_step, min(self.max_step, value))


def force_relay_main(args=None) -> None:
    rclpy.init(args=args)
    node = ForceGazeboTrajectoryRelay()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def visual_servo_adapter_main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualServoGazeboAdapter()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
