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
GRIPPER_TOPICS = ("/rm65b_gripper/upper_finger_cmd", "/rm65b_gripper/lower_finger_cmd")

LEFT_INITIAL_VIEW = (0.046, -0.840, 1.161, -0.120, 0.865, -0.383)
LEFT_SEARCH_OFFSET = (-0.030, -0.825, 1.140, -0.105, 0.875, -0.300)
LEFT_VISUAL_ALIGN = (0.150, -0.735, 1.265, -0.175, 0.868, -0.500)
LEFT_TARGET_CENTER = (0.255, -0.650, 1.345, -0.225, 0.862, -0.610)
LEFT_NEAR_TARGET = (0.356, -0.567, 1.408, -0.263, 0.860, -0.705)

RIGHT_OBSERVER = (0.000, -0.350, 0.650, 0.000, 0.900, 0.000)

LEFT_VISION_CYCLE = (
    LEFT_INITIAL_VIEW,
    LEFT_SEARCH_OFFSET,
    LEFT_VISUAL_ALIGN,
    LEFT_TARGET_CENTER,
    LEFT_NEAR_TARGET,
    LEFT_TARGET_CENTER,
    LEFT_INITIAL_VIEW,
)


def smooth_step(alpha: float) -> float:
    return alpha * alpha * (3.0 - 2.0 * alpha)


def interpolate_point(a: tuple[float, ...], b: tuple[float, ...], alpha: float) -> tuple[float, ...]:
    eased = smooth_step(alpha)
    return tuple(float(x) + (float(y) - float(x)) * eased for x, y in zip(a, b))


def densify_points(
    points: tuple[tuple[float, ...], ...],
    samples_per_segment: int,
) -> list[tuple[float, ...]]:
    samples = max(samples_per_segment, 1)
    dense: list[tuple[float, ...]] = []
    for idx in range(len(points) - 1):
        start = points[idx]
        end = points[idx + 1]
        for sample in range(samples):
            if idx > 0 and sample == 0:
                continue
            dense.append(interpolate_point(start, end, sample / samples))
    dense.append(points[-1])
    return dense


def make_trajectory(
    points: tuple[tuple[float, ...], ...],
    cycle_duration: float,
    samples_per_segment: int,
) -> JointTrajectory:
    dense_points = densify_points(points, samples_per_segment)
    msg = JointTrajectory()
    msg.joint_names = list(JOINTS)
    step = cycle_duration / max(len(dense_points) - 1, 1)
    for idx, positions in enumerate(dense_points):
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        seconds = max(idx * step, 0.30)
        point.time_from_start.sec = int(seconds)
        point.time_from_start.nanosec = int((seconds - int(seconds)) * 1e9)
        msg.points.append(point)
    return msg


def hold_trajectory(positions: tuple[float, ...], duration: float = 1.0) -> JointTrajectory:
    return make_trajectory((positions, positions), duration, samples_per_segment=1)


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


class Day03VisionAlignDemo(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("rm65b_day03_vision_align_demo")
        self.args = args
        self.left_pub = self.create_publisher(JointTrajectory, "/model/left_rm65b/joint_trajectory", 10)
        self.right_pub = self.create_publisher(JointTrajectory, "/model/right_rm65b/joint_trajectory", 10)
        self.upper_pub = self.create_publisher(Float64, "/rm65b_gripper/upper_finger_cmd", 10)
        self.lower_pub = self.create_publisher(Float64, "/rm65b_gripper/lower_finger_cmd", 10)
        self.left_cycle = make_trajectory(
            LEFT_VISION_CYCLE,
            args.cycle_duration,
            args.samples_per_segment,
        )
        self.right_hold = hold_trajectory(RIGHT_OBSERVER)

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
            self.get_logger().warning(f"direct gz trajectory publish failed for {topic}: {exc}")

    def publish_trajectory(self) -> None:
        if self.args.direct_gz:
            self.publish_direct_gz("/model/left_rm65b/joint_trajectory", self.left_cycle)
            self.publish_direct_gz("/model/right_rm65b/joint_trajectory", self.right_hold)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and rclpy.ok():
            self.left_pub.publish(self.left_cycle)
            self.right_pub.publish(self.right_hold)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.08)
        self.get_logger().info(
            f"published Day3 vision alignment cycle: points={len(self.left_cycle.points)} "
            f"cycle_duration={self.args.cycle_duration:.1f}s"
        )

    def publish_gripper(self, position: float) -> None:
        msg = Float64()
        msg.data = float(position)
        if self.args.direct_gz:
            for topic in GRIPPER_TOPICS:
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
        deadline = time.monotonic() + max(self.args.gripper_command_burst_s, 0.0)
        while time.monotonic() <= deadline and rclpy.ok():
            self.upper_pub.publish(msg)
            self.lower_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(1.0 / max(self.args.gripper_command_rate_hz, 1.0))

    def run(self) -> None:
        start = time.monotonic()
        self.publish_gripper(self.args.closed_position)
        self.get_logger().info("Day3 gripper closed; demo focuses on camera target alignment")
        time.sleep(max(self.args.settle, 0.0))

        while rclpy.ok():
            if self.args.duration > 0 and time.monotonic() - start >= self.args.duration:
                break
            self.publish_trajectory()
            self.publish_gripper(self.args.closed_position)
            cycle_start = time.monotonic()
            next_gripper_hold = cycle_start + self.args.gripper_hold_period
            while rclpy.ok() and time.monotonic() - cycle_start < self.args.cycle_duration:
                if self.args.duration > 0 and time.monotonic() - start >= self.args.duration:
                    break
                now = time.monotonic()
                if now >= next_gripper_hold:
                    self.publish_gripper(self.args.closed_position)
                    next_gripper_hold = now + self.args.gripper_hold_period
                rclpy.spin_once(self, timeout_sec=0.02)
                time.sleep(0.05)

        self.publish_gripper(self.args.closed_position)
        self.get_logger().info("Day3 vision alignment demo finished")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=180.0, help="seconds; use 0 to run until Ctrl+C")
    parser.add_argument("--cycle-duration", type=float, default=14.0)
    parser.add_argument("--samples-per-segment", type=int, default=6)
    parser.add_argument("--closed-position", type=float, default=0.000)
    parser.add_argument("--gripper-hold-period", type=float, default=3.0)
    parser.add_argument("--gripper-command-burst-s", type=float, default=0.45)
    parser.add_argument("--gripper-command-rate-hz", type=float, default=25.0)
    parser.add_argument("--settle", type=float, default=1.0)
    parser.add_argument("--no-direct-gz", dest="direct_gz", action="store_false")
    parser.set_defaults(direct_gz=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = Day03VisionAlignDemo(args)
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
