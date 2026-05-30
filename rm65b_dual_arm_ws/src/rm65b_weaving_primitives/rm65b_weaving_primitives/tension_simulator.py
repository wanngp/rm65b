#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from trajectory_msgs.msg import JointTrajectory

try:
    import yaml
except Exception:  # pragma: no cover - optional on field laptops
    yaml = None


class TensionSimulator(Node):
    """Spring-mass-damper yarn tension model with PID/compliance telemetry."""

    def __init__(self) -> None:
        super().__init__("tension_simulator")
        self.declare_parameter("config_file", "")
        self.declare_parameter("baseline_n", 0.25)
        self.declare_parameter("target_tension_n", 1.65)
        self.declare_parameter("min_ok_n", 0.8)
        self.declare_parameter("max_ok_n", 3.4)
        self.declare_parameter("yarn_mass_kg", 0.020)
        self.declare_parameter("spring_k_n_per_m", 95.0)
        self.declare_parameter("damping_n_s_per_m", 1.85)
        self.declare_parameter("joint_to_yarn_m_per_rad", 0.018)
        self.declare_parameter("max_elongation_m", 0.035)
        self.declare_parameter("pid_kp", 0.42)
        self.declare_parameter("pid_ki", 0.08)
        self.declare_parameter("pid_kd", 0.018)
        self.declare_parameter("integral_limit_n_s", 4.0)
        self.declare_parameter("derivative_filter_alpha", 0.25)
        self.declare_parameter("compliance_gain_m_per_n", 0.0035)
        self.declare_parameter("max_compliance_offset_m", 0.010)

        self._load_config_file()

        self.baseline = float(self.get_parameter("baseline_n").value)
        self.target_tension = float(self.get_parameter("target_tension_n").value)
        self.min_ok = float(self.get_parameter("min_ok_n").value)
        self.max_ok = float(self.get_parameter("max_ok_n").value)
        self.mass = max(float(self.get_parameter("yarn_mass_kg").value), 1.0e-4)
        self.spring_k = float(self.get_parameter("spring_k_n_per_m").value)
        self.damping = float(self.get_parameter("damping_n_s_per_m").value)
        self.joint_to_yarn = float(self.get_parameter("joint_to_yarn_m_per_rad").value)
        self.max_elongation = float(self.get_parameter("max_elongation_m").value)
        self.kp = float(self.get_parameter("pid_kp").value)
        self.ki = float(self.get_parameter("pid_ki").value)
        self.kd = float(self.get_parameter("pid_kd").value)
        self.integral_limit = float(self.get_parameter("integral_limit_n_s").value)
        self.derivative_alpha = float(self.get_parameter("derivative_filter_alpha").value)
        self.compliance_gain = float(self.get_parameter("compliance_gain_m_per_n").value)
        self.max_compliance_offset = float(self.get_parameter("max_compliance_offset_m").value)

        self.elongation_m = 0.0
        self.velocity_mps = 0.0
        self.commanded_elongation_m = 0.0
        self.integral_error = 0.0
        self.filtered_derivative = 0.0
        self.previous_error = 0.0
        self.tension = self.baseline
        self.last_update = self.get_clock().now()
        self.last_right_position: list[float] | None = None

        self.tension_pub = self.create_publisher(Float32, "/weaving/tension_n", 10)
        self.status_pub = self.create_publisher(String, "/weaving/tension_status", 10)
        self.pid_state_pub = self.create_publisher(String, "/weaving/tension_pid_state", 10)
        self.params_pub = self.create_publisher(String, "/weaving/tension_pid_params", 1)
        self.compliance_pub = self.create_publisher(Float32, "/weaving/compliance_offset_m", 10)
        self.create_subscription(
            JointTrajectory,
            "/dual_arm_planning/right_joint_trajectory",
            self._trajectory_callback,
            10,
        )
        self.create_subscription(String, "/weaving/events", self._event_callback, 10)
        self.create_timer(0.02, self._update)
        self.create_timer(1.0, self._publish_params)
        self.get_logger().info("D4 spring-mass-damper tension model with PID ready")

    def _load_config_file(self) -> None:
        path_text = str(self.get_parameter("config_file").value or "")
        if not path_text or yaml is None:
            return
        path = Path(path_text)
        if not path.exists():
            self.get_logger().warn(f"tension config file not found: {path}")
            return
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        model = data.get("tension_model", {})
        pid = data.get("pid", {})
        compliance = data.get("compliance", {})
        mapping = {
            "baseline_n": model.get("baseline_n"),
            "target_tension_n": model.get("target_tension_n"),
            "min_ok_n": model.get("min_ok_n"),
            "max_ok_n": model.get("max_ok_n"),
            "yarn_mass_kg": model.get("yarn_mass_kg"),
            "spring_k_n_per_m": model.get("spring_k_n_per_m"),
            "damping_n_s_per_m": model.get("damping_n_s_per_m"),
            "joint_to_yarn_m_per_rad": model.get("joint_to_yarn_m_per_rad"),
            "max_elongation_m": model.get("max_elongation_m"),
            "pid_kp": pid.get("kp"),
            "pid_ki": pid.get("ki"),
            "pid_kd": pid.get("kd"),
            "integral_limit_n_s": pid.get("integral_limit_n_s"),
            "derivative_filter_alpha": pid.get("derivative_filter_alpha"),
            "compliance_gain_m_per_n": compliance.get("compliance_gain_m_per_n"),
            "max_compliance_offset_m": compliance.get("max_offset_m"),
        }
        for name, value in mapping.items():
            if value is not None:
                self.set_parameters([rclpy.parameter.Parameter(name, value=value)])

    def _trajectory_callback(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return
        current = [float(v) for v in msg.points[-1].positions[:6]]
        if self.last_right_position is None:
            self.last_right_position = current
            return
        delta = math.sqrt(
            sum((a - b) * (a - b) for a, b in zip(current, self.last_right_position))
        )
        self.last_right_position = current
        self.commanded_elongation_m = min(
            self.max_elongation,
            max(0.0, self.commanded_elongation_m + delta * self.joint_to_yarn),
        )

    def _event_callback(self, msg: String) -> None:
        text = msg.data
        if "primitive_start:hook_yarn" in text:
            self.commanded_elongation_m = max(self.commanded_elongation_m, 0.004)
        elif "primitive_start:lift_yarn" in text:
            self.commanded_elongation_m = max(self.commanded_elongation_m, 0.009)
        elif "primitive_start:pull_tight" in text:
            self.commanded_elongation_m = max(self.commanded_elongation_m, 0.018)
        elif "primitive_start:shift" in text:
            self.commanded_elongation_m = max(self.commanded_elongation_m, 0.015)
        elif "primitive_start:exchange" in text:
            self.commanded_elongation_m = min(self.commanded_elongation_m, 0.010)
        elif "sequence_complete" in text:
            self.commanded_elongation_m = 0.0
            self.integral_error = 0.0

    def _update(self) -> None:
        now = self.get_clock().now()
        dt = max((now - self.last_update).nanoseconds / 1.0e9, 1.0e-3)
        self.last_update = now

        error = self.target_tension - self.tension
        self.integral_error = self._clamp(
            self.integral_error + error * dt,
            -self.integral_limit,
            self.integral_limit,
        )
        raw_derivative = (error - self.previous_error) / dt
        self.filtered_derivative = (
            self.derivative_alpha * raw_derivative
            + (1.0 - self.derivative_alpha) * self.filtered_derivative
        )
        self.previous_error = error
        pid_output = self.kp * error + self.ki * self.integral_error + self.kd * self.filtered_derivative
        compliance_offset = self._clamp(
            pid_output * self.compliance_gain,
            -self.max_compliance_offset,
            self.max_compliance_offset,
        )

        equilibrium = self._clamp(
            self.commanded_elongation_m + compliance_offset,
            0.0,
            self.max_elongation,
        )
        acceleration = (
            self.spring_k * (equilibrium - self.elongation_m) - self.damping * self.velocity_mps
        ) / self.mass
        self.velocity_mps += acceleration * dt
        self.elongation_m = self._clamp(
            self.elongation_m + self.velocity_mps * dt,
            0.0,
            self.max_elongation,
        )
        self.tension = max(0.0, self.baseline + self.spring_k * self.elongation_m)

        if self.tension < self.min_ok:
            status = "LOW"
        elif self.tension > self.max_ok:
            status = "HIGH"
        else:
            status = "OK"
        self.tension_pub.publish(Float32(data=float(self.tension)))
        self.status_pub.publish(String(data=status))
        self.compliance_pub.publish(Float32(data=float(compliance_offset)))
        self.pid_state_pub.publish(
            String(
                data=json.dumps(
                    {
                        "tension_n": self.tension,
                        "target_tension_n": self.target_tension,
                        "status": status,
                        "elongation_m": self.elongation_m,
                        "velocity_mps": self.velocity_mps,
                        "commanded_elongation_m": self.commanded_elongation_m,
                        "pid_output": pid_output,
                        "integral_error": self.integral_error,
                        "derivative": self.filtered_derivative,
                        "compliance_offset_m": compliance_offset,
                    },
                    separators=(",", ":"),
                )
            )
        )

    def _publish_params(self) -> None:
        self.params_pub.publish(
            String(
                data=json.dumps(
                    {
                        "baseline_n": self.baseline,
                        "target_tension_n": self.target_tension,
                        "min_ok_n": self.min_ok,
                        "max_ok_n": self.max_ok,
                        "yarn_mass_kg": self.mass,
                        "spring_k_n_per_m": self.spring_k,
                        "damping_n_s_per_m": self.damping,
                        "pid": {"kp": self.kp, "ki": self.ki, "kd": self.kd},
                        "compliance_gain_m_per_n": self.compliance_gain,
                        "max_compliance_offset_m": self.max_compliance_offset,
                    },
                    separators=(",", ":"),
                )
            )
        )

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TensionSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
