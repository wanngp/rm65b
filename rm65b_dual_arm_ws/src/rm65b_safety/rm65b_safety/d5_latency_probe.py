#!/usr/bin/env python3
from __future__ import annotations

import json
import time
from pathlib import Path
from statistics import mean

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Float32, Float64, String


class D5LatencyProbe(Node):
    def __init__(self) -> None:
        super().__init__("d5_latency_probe")
        self.declare_parameter("output_json", "outputs/d5_system/day05_latency_record.json")
        self.declare_parameter("write_period_s", 1.0)
        self.declare_parameter("acceptance_limit_ms", 200.0)
        self.declare_parameter("camera_topic", "/left_camera/image_rect")
        self.declare_parameter("vision_pose_topic", "/vision/target_pose")
        self.declare_parameter("twist_topic", "/visual_servo/twist_cmd")
        self.declare_parameter("visual_adapter_topic", "/visual_servo/gazebo_adapter_state")
        self.declare_parameter("force_input_topic", "/force_control/target_wrench")
        self.declare_parameter("force_error_topic", "/force_control/wrench_error")
        self.declare_parameter("admittance_offset_topic", "/force_control/admittance_offset")
        self.declare_parameter("joint_topic", "/joint_states")
        self.declare_parameter("weaving_event_topic", "/weaving/events")
        self.declare_parameter("tension_topic", "/weaving/tension_n")
        self.declare_parameter("safety_fault_topic", "/safety/fault")
        self.declare_parameter("safety_estop_topic", "/safety/estop")

        self.output_json = Path(str(self.get_parameter("output_json").value))
        self.acceptance_limit_ms = float(self.get_parameter("acceptance_limit_ms").value)
        self.last_seen_s: dict[str, float] = {}
        self.samples: dict[str, list[float]] = {}
        self.counts: dict[str, int] = {}

        self._subscribe("camera", Image, "camera_topic")
        self._subscribe("vision_pose", PoseStamped, "vision_pose_topic")
        self._subscribe("twist_cmd", TwistStamped, "twist_topic")
        self._subscribe("visual_adapter", String, "visual_adapter_topic")
        self._subscribe("force_input", WrenchStamped, "force_input_topic")
        self._subscribe("force_error", WrenchStamped, "force_error_topic")
        self._subscribe("admittance_offset", Float64, "admittance_offset_topic")
        self._subscribe("joint_states", JointState, "joint_topic")
        self._subscribe("weaving_event", String, "weaving_event_topic")
        self._subscribe("tension", Float32, "tension_topic")
        self._subscribe("safety_fault", String, "safety_fault_topic")
        self._subscribe("safety_estop", Bool, "safety_estop_topic")

        self.edge_map = {
            "camera_to_vision_pose_ms": ("camera", "vision_pose"),
            "vision_pose_to_twist_ms": ("vision_pose", "twist_cmd"),
            "twist_to_visual_adapter_ms": ("twist_cmd", "visual_adapter"),
            "force_input_to_error_ms": ("force_input", "force_error"),
            "force_error_to_admittance_ms": ("force_error", "admittance_offset"),
            "admittance_to_joint_state_ms": ("admittance_offset", "joint_states"),
            "weaving_event_to_tension_ms": ("weaving_event", "tension"),
            "safety_fault_to_estop_ms": ("safety_fault", "safety_estop"),
        }
        self.create_timer(float(self.get_parameter("write_period_s").value), self._write_record)
        self.get_logger().info(f"D5 latency probe writing {self.output_json}")

    def _subscribe(self, key: str, msg_type, parameter_name: str) -> None:
        topic = str(self.get_parameter(parameter_name).value)
        self.create_subscription(msg_type, topic, lambda msg, k=key: self._record(k), 10)

    def _record(self, key: str) -> None:
        now_s = time.perf_counter()
        self.last_seen_s[key] = now_s
        self.counts[key] = self.counts.get(key, 0) + 1
        for edge_name, (src, dst) in self.edge_map.items():
            if dst != key or src not in self.last_seen_s:
                continue
            latency_ms = (now_s - self.last_seen_s[src]) * 1000.0
            if latency_ms >= 0.0:
                self.samples.setdefault(edge_name, []).append(latency_ms)

    @staticmethod
    def _stats(values: list[float]) -> dict[str, float | int | bool]:
        if not values:
            return {"samples": 0, "mean_ms": 0.0, "max_ms": 0.0, "p95_ms": 0.0}
        sorted_values = sorted(values)
        p95_index = min(len(sorted_values) - 1, int(round(0.95 * (len(sorted_values) - 1))))
        return {
            "samples": len(values),
            "mean_ms": round(mean(values), 3),
            "max_ms": round(max(values), 3),
            "p95_ms": round(sorted_values[p95_index], 3),
        }

    def _write_record(self) -> None:
        chains = {
            name: self._stats(values)
            for name, values in sorted(self.samples.items())
        }
        observed = {name: stats for name, stats in chains.items() if stats["samples"] > 0}
        missing_chains = [
            name for name in sorted(self.edge_map) if name not in observed
        ]
        observed_pass = all(
            float(stats["max_ms"]) <= self.acceptance_limit_ms
            for stats in observed.values()
        )
        payload = {
            "node": self.get_name(),
            "acceptance_limit_ms": self.acceptance_limit_ms,
            "acceptance_pass": bool(observed and not missing_chains and observed_pass),
            "observed_topic_counts": dict(sorted(self.counts.items())),
            "chains": chains,
            "missing_chains": missing_chains,
        }
        self.output_json.parent.mkdir(parents=True, exist_ok=True)
        self.output_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = D5LatencyProbe()
    try:
        rclpy.spin(node)
    finally:
        node._write_record()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
