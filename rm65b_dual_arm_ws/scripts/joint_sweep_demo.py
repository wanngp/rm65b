#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINTS = [f"joint{i}" for i in range(1, 7)]
HOME = (0.0, -0.35, 0.65, 0.0, 0.90, 0.0)
LEFT_SWEEP = (
    HOME,
    (0.70, -0.80, 1.10, 0.60, 1.18, -0.55),
    (-0.68, -0.22, 0.40, -0.60, 0.60, 0.55),
    (0.42, -0.72, 1.32, -0.42, 1.08, 0.92),
    (-0.42, -0.32, 0.52, 0.42, 0.72, -0.92),
    HOME,
)


def mirror_for_right(point: tuple[float, ...]) -> tuple[float, ...]:
    q1, q2, q3, q4, q5, q6 = point
    return (-q1, q2, q3, -q4, q5, -q6)


def make_trajectory(points: tuple[tuple[float, ...], ...], cycle_duration: float) -> JointTrajectory:
    msg = JointTrajectory()
    msg.joint_names = list(JOINTS)
    step = cycle_duration / max(len(points) - 1, 1)
    for idx, positions in enumerate(points):
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        seconds = max(idx * step, 0.35)
        point.time_from_start.sec = int(seconds)
        point.time_from_start.nanosec = int((seconds - int(seconds)) * 1e9)
        msg.points.append(point)
    return msg


def trajectory_to_gz_pbtxt(msg: JointTrajectory) -> str:
    lines = [f'joint_names: "{name}"' for name in msg.joint_names]
    for point in msg.points:
        parts = [f"positions: {value:.6f}" for value in point.positions]
        parts.append(
            "time_from_start { "
            f"sec: {int(point.time_from_start.sec)} "
            f"nsec: {int(point.time_from_start.nanosec)} "
            "}"
        )
        lines.append("points { " + " ".join(parts) + " }")
    return "\n".join(lines)


class JointSweepDemo(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("rm65b_joint_sweep_demo")
        self.args = args
        self.left_pub = self.create_publisher(JointTrajectory, "/model/left_rm65b/joint_trajectory", 10)
        self.right_pub = self.create_publisher(JointTrajectory, "/model/right_rm65b/joint_trajectory", 10)
        self.upper_pub = self.create_publisher(Float64, "/rm65b_gripper/upper_finger_cmd", 10)
        self.lower_pub = self.create_publisher(Float64, "/rm65b_gripper/lower_finger_cmd", 10)
        self.left_msg = make_trajectory(LEFT_SWEEP, args.cycle_duration)
        self.right_msg = make_trajectory(tuple(mirror_for_right(point) for point in LEFT_SWEEP), args.cycle_duration)

    def publish_trajectory(self) -> None:
        if self.args.direct_gz:
            self.publish_direct_gz("/model/left_rm65b/joint_trajectory", self.left_msg)
            self.publish_direct_gz("/model/right_rm65b/joint_trajectory", self.right_msg)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and rclpy.ok():
            self.left_pub.publish(self.left_msg)
            self.right_pub.publish(self.right_msg)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.08)
        self.get_logger().info(
            f"published joint-space sweep trajectory: points={len(self.left_msg.points)} "
            f"cycle_duration={self.args.cycle_duration:.1f}s"
        )

    def publish_direct_gz(self, topic: str, msg: JointTrajectory) -> None:
        try:
            subprocess.run(
                [
                    "gz",
                    "topic",
                    "-t",
                    topic,
                    "-m",
                    "gz.msgs.JointTrajectory",
                    "-p",
                    trajectory_to_gz_pbtxt(msg),
                ],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=3.0,
            )
        except Exception as exc:
            self.get_logger().warning(f"direct gz publish failed for {topic}: {exc}")

    def publish_gripper(self, position: float) -> None:
        msg = Float64()
        msg.data = float(position)
        self.upper_pub.publish(msg)
        self.lower_pub.publish(msg)
        if self.args.direct_gz:
            for topic in ("/rm65b_gripper/upper_finger_cmd", "/rm65b_gripper/lower_finger_cmd"):
                try:
                    subprocess.run(
                        ["gz", "topic", "-t", topic, "-m", "gz.msgs.Double", "-p", f"data: {position:.6f}"],
                        check=False,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        timeout=1.5,
                    )
                except Exception as exc:
                    self.get_logger().warning(f"direct gz gripper publish failed for {topic}: {exc}")


    def run(self) -> None:
        start = time.monotonic()
        cycle = 0
        self.publish_gripper(self.args.open_position)
        time.sleep(max(self.args.settle, 0.0))

        while rclpy.ok():
            if self.args.duration > 0 and time.monotonic() - start >= self.args.duration:
                break
            cycle += 1
            self.publish_trajectory()
            cycle_start = time.monotonic()
            next_toggle = cycle_start
            open_gripper = cycle % 2 == 1
            while rclpy.ok() and time.monotonic() - cycle_start < self.args.cycle_duration:
                if self.args.duration > 0 and time.monotonic() - start >= self.args.duration:
                    break
                now = time.monotonic()
                if now >= next_toggle:
                    position = self.args.open_position if open_gripper else self.args.closed_position
                    self.publish_gripper(position)
                    open_gripper = not open_gripper
                    next_toggle = now + self.args.gripper_period
                rclpy.spin_once(self, timeout_sec=0.02)
                time.sleep(0.05)

        self.publish_gripper(self.args.open_position)
        self.get_logger().info("joint sweep demo finished")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=180.0, help="seconds; use 0 to run until Ctrl+C")
    parser.add_argument("--cycle-duration", type=float, default=18.0)
    parser.add_argument("--gripper-period", type=float, default=2.0)
    parser.add_argument("--open-position", type=float, default=0.018)
    parser.add_argument("--closed-position", type=float, default=0.002)
    parser.add_argument("--settle", type=float, default=1.0)
    parser.add_argument("--no-direct-gz", dest="direct_gz", action="store_false")
    parser.set_defaults(direct_gz=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = JointSweepDemo(args)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
