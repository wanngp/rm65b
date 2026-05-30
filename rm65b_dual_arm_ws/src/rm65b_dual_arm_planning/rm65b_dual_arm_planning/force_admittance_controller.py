from __future__ import annotations

import copy
import time

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64, String
from trajectory_msgs.msg import JointTrajectory

try:
    from rm_ros_interfaces.msg import Sixforce
except Exception:  # pragma: no cover - verified in ROS runtime
    Sixforce = None


class ForceAdmittanceController(Node):
    """Closed-loop simulated force controller for the acceptance playback.

    The loop is intentionally observable in ROS bags: measured force enters from
    the simulated RealMan force topic, target and error are published as
    WrenchStamped messages, and the admittance correction is sent back to the
    planner plus a corrected trajectory topic.
    """

    def __init__(self) -> None:
        super().__init__("force_admittance_controller")
        self.declare_parameter("day_id", "day05")
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("kp", 0.0065)
        self.declare_parameter("ki", 0.0009)
        self.declare_parameter("max_offset_rad", 0.08)
        self.declare_parameter("deadband_n", 0.015)
        self.declare_parameter("nominal_force_n", 6.0)
        self.declare_parameter("virtual_mass_kg", 1.20)
        self.declare_parameter("virtual_damping_ns_per_m", 38.0)
        self.declare_parameter("virtual_stiffness_n_per_m", 180.0)
        self.declare_parameter("joint6_rad_per_m", 1.60)
        self.day_id = self.get_parameter("day_id").get_parameter_value().string_value
        self.kp = float(self.get_parameter("kp").value)
        self.ki = float(self.get_parameter("ki").value)
        self.max_offset = float(self.get_parameter("max_offset_rad").value)
        self.deadband = float(self.get_parameter("deadband_n").value)
        self.nominal_force = float(self.get_parameter("nominal_force_n").value)
        self.virtual_mass = float(self.get_parameter("virtual_mass_kg").value)
        self.virtual_damping = float(self.get_parameter("virtual_damping_ns_per_m").value)
        self.virtual_stiffness = float(self.get_parameter("virtual_stiffness_n_per_m").value)
        self.joint6_rad_per_m = float(self.get_parameter("joint6_rad_per_m").value)

        self.phase = self.day_id
        self.tension_n = 0.0
        self.measured_force_z = 0.0
        self.integral = 0.0
        self.offset = 0.0
        self.displacement_m = 0.0
        self.velocity_mps = 0.0
        self.acceleration_mps2 = 0.0
        self.last_update = time.monotonic()
        self.latest_right_trajectory: JointTrajectory | None = None

        self.target_pub = self.create_publisher(WrenchStamped, "/force_control/target_wrench", 10)
        self.error_pub = self.create_publisher(WrenchStamped, "/force_control/wrench_error", 10)
        self.impedance_pub = self.create_publisher(WrenchStamped, "/force_control/impedance_wrench", 10)
        self.offset_pub = self.create_publisher(Float64, "/force_control/admittance_offset", 10)
        self.displacement_pub = self.create_publisher(Float64, "/force_control/admittance_displacement_m", 10)
        self.velocity_pub = self.create_publisher(Float64, "/force_control/admittance_velocity_mps", 10)
        self.state_pub = self.create_publisher(String, "/force_control/state", 10)
        self.impedance_state_pub = self.create_publisher(String, "/force_control/impedance_state", 10)
        self.corrected_joint_state_pub = self.create_publisher(
            JointState,
            "/force_control/corrected_right_joint_states",
            10,
        )
        self.corrected_pub = self.create_publisher(
            JointTrajectory,
            "/force_control/corrected_right_joint_trajectory",
            10,
        )

        if Sixforce is not None:
            self.create_subscription(
                Sixforce,
                "/right_rm_driver/rm_driver/udp_six_force",
                self._force_callback,
                20,
            )
        self.create_subscription(Float32, "/weaving/tension_n", self._tension_callback, 20)
        self.create_subscription(String, "/dual_arm_planning/phase", self._phase_callback, 20)
        self.create_subscription(
            JointTrajectory,
            "/dual_arm_planning/right_joint_trajectory",
            self._trajectory_callback,
            10,
        )
        rate = max(float(self.get_parameter("control_rate_hz").value), 1.0)
        self.create_timer(1.0 / rate, self._update)
        self.get_logger().info("closed-loop simulated force controller ready")

    def _target_force_n(self) -> float:
        phase_targets = {
            "day01": 4.5,
            "day02": 7.5,
            "day03": 5.0,
            "day04": 6.5,
            "day05": 7.0,
            "hook_yarn": 5.0,
            "pick_yarn": 6.0,
            "pull_tight": 7.8,
            "shift": 6.8,
            "complete": 4.5,
        }
        base = phase_targets.get(self.phase, phase_targets.get(self.day_id, self.nominal_force))
        return float(base + 0.18 * self.tension_n)

    def _force_callback(self, msg) -> None:
        self.measured_force_z = float(getattr(msg, "force_fz", 0.0))

    def _tension_callback(self, msg: Float32) -> None:
        self.tension_n = float(msg.data)

    def _phase_callback(self, msg: String) -> None:
        text = msg.data.strip()
        if text:
            self.phase = text.split(":")[-1] if ":" in text else text

    def _trajectory_callback(self, msg: JointTrajectory) -> None:
        self.latest_right_trajectory = msg
        self._publish_corrected_trajectory()

    def _update(self) -> None:
        now = time.monotonic()
        dt = max(min(now - self.last_update, 0.1), 0.001)
        self.last_update = now

        target = self._target_force_n()
        raw_error = target - self.measured_force_z
        error = 0.0 if abs(raw_error) < self.deadband else raw_error
        self.acceleration_mps2 = (
            error
            - self.virtual_damping * self.velocity_mps
            - self.virtual_stiffness * self.displacement_m
        ) / max(self.virtual_mass, 1e-6)
        self.velocity_mps += self.acceleration_mps2 * dt
        self.displacement_m += self.velocity_mps * dt
        max_displacement = self.max_offset / max(abs(self.joint6_rad_per_m), 1e-6)
        self.displacement_m = max(-max_displacement, min(max_displacement, self.displacement_m))
        self.offset = max(
            -self.max_offset,
            min(self.max_offset, self.displacement_m * self.joint6_rad_per_m),
        )
        impedance_force = (
            self.virtual_mass * self.acceleration_mps2
            + self.virtual_damping * self.velocity_mps
            + self.virtual_stiffness * self.displacement_m
        )

        stamp = self.get_clock().now().to_msg()
        target_msg = WrenchStamped()
        target_msg.header.stamp = stamp
        target_msg.header.frame_id = "right_tcp"
        target_msg.wrench.force.z = target
        self.target_pub.publish(target_msg)

        error_msg = WrenchStamped()
        error_msg.header.stamp = stamp
        error_msg.header.frame_id = "right_tcp"
        error_msg.wrench.force.z = raw_error
        self.error_pub.publish(error_msg)

        impedance_msg = WrenchStamped()
        impedance_msg.header.stamp = stamp
        impedance_msg.header.frame_id = "right_tcp"
        impedance_msg.wrench.force.z = float(impedance_force)
        self.impedance_pub.publish(impedance_msg)

        self.offset_pub.publish(Float64(data=float(self.offset)))
        self.displacement_pub.publish(Float64(data=float(self.displacement_m)))
        self.velocity_pub.publish(Float64(data=float(self.velocity_mps)))
        self.state_pub.publish(
            String(
                data=(
                    f"admittance day={self.day_id} phase={self.phase} "
                    f"target_fz={target:.3f} measured_fz={self.measured_force_z:.3f} "
                    f"error_fz={error:.3f} x_m={self.displacement_m:.5f} "
                    f"xdot_mps={self.velocity_mps:.5f} offset_rad={self.offset:.5f}"
                )
            )
        )
        self.impedance_state_pub.publish(
            String(
                data=(
                    f"impedance_observer day={self.day_id} phase={self.phase} "
                    f"M={self.virtual_mass:.3f} B={self.virtual_damping:.3f} "
                    f"K={self.virtual_stiffness:.3f} "
                    f"estimated_fz={impedance_force:.3f}"
                )
            )
        )
        self._publish_corrected_trajectory()

    def _publish_corrected_trajectory(self) -> None:
        if self.latest_right_trajectory is None:
            return
        corrected = copy.deepcopy(self.latest_right_trajectory)
        for point in corrected.points:
            if point.positions:
                positions = list(point.positions)
                positions[-1] = float(positions[-1]) + self.offset
                point.positions = positions
        self.corrected_pub.publish(corrected)
        if corrected.points:
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = list(corrected.joint_names)
            joint_state.position = list(corrected.points[-1].positions)
            self.corrected_joint_state_pub.publish(joint_state)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ForceAdmittanceController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
