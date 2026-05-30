#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import statistics
import time
from pathlib import Path
from typing import Any

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    from control_msgs.action import GripperCommand
except Exception:  # pragma: no cover
    GripperCommand = None

try:
    from rm_ros_interfaces.msg import Sixforce
except Exception:  # pragma: no cover
    Sixforce = None


class HardwareAcceptanceCheck(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("rm65b_hardware_acceptance_check")
        self.args = args
        self.force_stamps: list[float] = []
        self.image_stamps: list[float] = []
        if Sixforce is not None:
            self.create_subscription(Sixforce, args.force_topic, self._force_callback, 50)
        self.create_subscription(Image, args.image_topic, self._image_callback, 20)

    def _force_callback(self, _msg: Any) -> None:
        self.force_stamps.append(time.monotonic())

    def _image_callback(self, _msg: Image) -> None:
        self.image_stamps.append(time.monotonic())

    def run(self) -> dict:
        report = {
            "strict_note": "This script reports FAIL when real hardware/topics are absent. It does not fabricate acceptance.",
            "checks": {},
        }
        if self.args.check_gripper:
            report["checks"]["gripper"] = self._check_gripper()
        if self.args.check_force:
            report["checks"]["force_latency"] = self._check_period(
                self.force_stamps,
                self.args.force_sample_s,
                self.args.force_latency_ms,
                "force",
            )
        if self.args.check_vision:
            report["checks"]["vision_fps"] = self._check_fps(
                self.image_stamps,
                self.args.vision_sample_s,
                self.args.vision_min_fps,
            )
        report["overall_pass"] = all(item.get("pass", False) for item in report["checks"].values())
        return report

    def _spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def _check_gripper(self) -> dict:
        if GripperCommand is None:
            return {"pass": False, "reason": "control_msgs/GripperCommand unavailable"}
        client = ActionClient(self, GripperCommand, self.args.gripper_action)
        if not client.wait_for_server(timeout_sec=self.args.action_timeout_s):
            return {"pass": False, "reason": f"missing action server {self.args.gripper_action}"}
        results = []
        for target in (self.args.gripper_open_m, self.args.gripper_closed_m):
            goal = GripperCommand.Goal()
            goal.command.position = float(target)
            goal.command.max_effort = float(self.args.gripper_effort_n)
            started = time.monotonic()
            future = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=self.args.action_timeout_s)
            handle = future.result()
            if handle is None or not handle.accepted:
                return {"pass": False, "reason": f"gripper goal rejected at {target}"}
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.args.action_timeout_s)
            wrapped = result_future.result()
            if wrapped is None:
                return {"pass": False, "reason": f"no gripper result at {target}"}
            duration_ms = (time.monotonic() - started) * 1000.0
            reached = bool(wrapped.result.reached_goal)
            results.append({"target_m": target, "duration_ms": duration_ms, "reached_goal": reached})
        worst = max(item["duration_ms"] for item in results)
        return {
            "pass": worst <= self.args.gripper_latency_ms and all(item["reached_goal"] for item in results),
            "threshold_ms": self.args.gripper_latency_ms,
            "worst_duration_ms": worst,
            "samples": results,
        }

    def _check_period(self, stamps: list[float], sample_s: float, threshold_ms: float, label: str) -> dict:
        before = len(stamps)
        self._spin_for(sample_s)
        sample = stamps[before:]
        if len(sample) < 2:
            return {"pass": False, "reason": f"insufficient {label} messages", "count": len(sample)}
        periods = [(b - a) * 1000.0 for a, b in zip(sample, sample[1:])]
        worst = max(periods)
        return {
            "pass": worst <= threshold_ms,
            "count": len(sample),
            "threshold_ms": threshold_ms,
            "mean_period_ms": statistics.mean(periods),
            "worst_period_ms": worst,
        }

    def _check_fps(self, stamps: list[float], sample_s: float, min_fps: float) -> dict:
        before = len(stamps)
        self._spin_for(sample_s)
        count = len(stamps) - before
        fps = count / sample_s if sample_s > 0 else 0.0
        return {"pass": fps >= min_fps, "count": count, "fps": fps, "threshold_fps": min_fps}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--check-gripper", action="store_true")
    parser.add_argument("--check-force", action="store_true")
    parser.add_argument("--check-vision", action="store_true")
    parser.add_argument("--gripper-action", default="/left_gripper_controller/gripper_cmd")
    parser.add_argument("--gripper-open-m", type=float, default=0.045)
    parser.add_argument("--gripper-closed-m", type=float, default=0.0)
    parser.add_argument("--gripper-effort-n", type=float, default=25.0)
    parser.add_argument("--gripper-latency-ms", type=float, default=100.0)
    parser.add_argument("--action-timeout-s", type=float, default=5.0)
    parser.add_argument("--force-topic", default="/left_rm_driver/rm_driver/udp_six_force")
    parser.add_argument("--force-sample-s", type=float, default=3.0)
    parser.add_argument("--force-latency-ms", type=float, default=10.0)
    parser.add_argument("--image-topic", default="/left_camera/image_rect")
    parser.add_argument("--vision-sample-s", type=float, default=3.0)
    parser.add_argument("--vision-min-fps", type=float, default=30.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = HardwareAcceptanceCheck(args)
    try:
        report = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["overall_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
