#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


LOCAL_JOINTS = [f"joint{i}" for i in range(1, 7)]


def load_points(plan_file: Path) -> list[dict]:
    with plan_file.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    points = data.get("points") or []
    if len(points) < 2:
        raise RuntimeError(f"MoveIt plan has too few points: {plan_file}")
    return points


def build_trajectory(points: list[dict], side: str, time_offset: float) -> JointTrajectory:
    msg = JointTrajectory()
    msg.joint_names = list(LOCAL_JOINTS)
    for frame in points:
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in frame[side]]
        seconds = max(float(frame["time_from_start"]) + time_offset, 0.0)
        point.time_from_start.sec = int(seconds)
        point.time_from_start.nanosec = int((seconds - int(seconds)) * 1e9)
        msg.points.append(point)
    return msg


class GazeboTrajectoryPublisher(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("rm65b_gazebo_trajectory_publisher")
        self.args = args
        self.points = load_points(Path(args.plan_file))
        self.left_pub = self.create_publisher(JointTrajectory, "/model/left_rm65b/joint_trajectory", 10)
        self.right_pub = self.create_publisher(JointTrajectory, "/model/right_rm65b/joint_trajectory", 10)
        self.left_msg = build_trajectory(self.points, "left", args.time_offset)
        self.right_msg = build_trajectory(self.points, "right", args.time_offset)

    def publish(self) -> None:
        for index in range(max(self.args.repeats, 1)):
            self.left_pub.publish(self.left_msg)
            self.right_pub.publish(self.right_msg)
            self.get_logger().info(
                f"published Gazebo JointTrajectory pair {index + 1}/{self.args.repeats} "
                f"points={len(self.left_msg.points)} offset={self.args.time_offset:.3f}s"
            )
            if index + 1 < self.args.repeats:
                time.sleep(max(self.args.interval, 0.0))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan-file", required=True)
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--interval", type=float, default=0.2)
    parser.add_argument("--time-offset", type=float, default=0.25)
    parser.add_argument("--settle", type=float, default=0.35)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = GazeboTrajectoryPublisher(args)
    try:
        time.sleep(max(args.settle, 0.0))
        node.publish()
        rclpy.spin_once(node, timeout_sec=max(args.interval, 0.1))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
