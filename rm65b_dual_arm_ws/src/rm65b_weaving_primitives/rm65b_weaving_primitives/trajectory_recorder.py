#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


class TrajectoryRecorder(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_recorder")
        self.declare_parameter("output_file", "recorded_primitive.yaml")
        self.declare_parameter("joint_prefix", "left_")
        self.samples: list[dict] = []
        self.recording = False
        self.start_time = None
        self.create_subscription(JointState, "/joint_states", self._joint_callback, 10)
        self.create_service(Trigger, "start_recording", self._start)
        self.create_service(Trigger, "stop_recording", self._stop)

    def _start(self, _request, response):
        self.samples.clear()
        self.recording = True
        self.start_time = self.get_clock().now()
        response.success = True
        response.message = "recording started"
        return response

    def _stop(self, _request, response):
        self.recording = False
        output = Path(self.get_parameter("output_file").value)
        output.write_text(yaml.safe_dump({"points": self.samples}, sort_keys=False), encoding="utf-8")
        response.success = True
        response.message = f"wrote {len(self.samples)} samples to {output}"
        return response

    def _joint_callback(self, msg: JointState) -> None:
        if not self.recording or self.start_time is None:
            return
        prefix = self.get_parameter("joint_prefix").value
        pairs = [
            (name, pos)
            for name, pos in zip(msg.name, msg.position)
            if name.startswith(prefix)
        ]
        if not pairs:
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.samples.append(
            {
                "time_from_start": round(elapsed, 4),
                "names": [name for name, _ in pairs],
                "positions": [float(pos) for _, pos in pairs],
            }
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
