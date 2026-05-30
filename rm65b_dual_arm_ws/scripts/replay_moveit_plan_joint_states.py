#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


LEFT_JOINTS = [f"left_joint{i}" for i in range(1, 7)]
RIGHT_JOINTS = [f"right_joint{i}" for i in range(1, 7)]


def load_points(plan_file: Path) -> list[dict]:
    with plan_file.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    points = data.get("points", [])
    if len(points) < 2:
        raise RuntimeError(f"MoveIt plan has too few points: {plan_file}")
    return points


def sample(points: list[dict], elapsed: float) -> dict:
    duration = float(points[-1]["time_from_start"])
    t = min(max(elapsed, 0.0), duration)
    previous = points[0]
    current = points[-1]
    for idx in range(1, len(points)):
        if t <= float(points[idx]["time_from_start"]):
            previous = points[idx - 1]
            current = points[idx]
            break
    t0 = float(previous["time_from_start"])
    t1 = float(current["time_from_start"])
    alpha = 0.0 if t1 <= t0 else min(max((t - t0) / (t1 - t0), 0.0), 1.0)
    left = [float(a) + (float(b) - float(a)) * alpha for a, b in zip(previous["left"], current["left"])]
    right = [float(a) + (float(b) - float(a)) * alpha for a, b in zip(previous["right"], current["right"])]
    return {
        "phase": current.get("phase", ""),
        "left": left,
        "right": right,
    }


class MoveItPlanJointStateReplay(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("moveit_plan_joint_state_replay")
        self.args = args
        self.points = load_points(Path(args.plan_file))
        self.started_at = time.monotonic() + args.start_delay
        self.finished_at = self.started_at + args.duration
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.phase_pub = self.create_publisher(String, "/dual_arm_planning/phase", 10)
        self.status_pub = self.create_publisher(String, "/acceptance/day_status", 10)
        self.left_traj_pub = self.create_publisher(JointTrajectory, "/dual_arm_planning/left_joint_trajectory", 10)
        self.right_traj_pub = self.create_publisher(JointTrajectory, "/dual_arm_planning/right_joint_trajectory", 10)
        self.left_gripper_pub = self.create_publisher(JointState, "/left_gripper_controller/joint_states", 10)
        self.right_gripper_pub = self.create_publisher(JointState, "/right_gripper_controller/joint_states", 10)
        self._publish_trajectories()
        self.timer = self.create_timer(1.0 / args.rate_hz, self._tick)

    def _trajectory(self, side: str) -> JointTrajectory:
        msg = JointTrajectory()
        msg.joint_names = LEFT_JOINTS if side == "left" else RIGHT_JOINTS
        for frame in self.points:
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in frame[side]]
            seconds = float(frame["time_from_start"])
            point.time_from_start.sec = int(seconds)
            point.time_from_start.nanosec = int((seconds - int(seconds)) * 1e9)
            msg.points.append(point)
        return msg

    def _publish_trajectories(self) -> None:
        self.left_traj_pub.publish(self._trajectory("left"))
        self.right_traj_pub.publish(self._trajectory("right"))

    def _tick(self) -> None:
        now = time.monotonic()
        if now < self.started_at:
            elapsed = 0.0
        else:
            elapsed = now - self.started_at
        state = sample(self.points, elapsed)
        stamp = self.get_clock().now().to_msg()

        joint = JointState()
        joint.header.stamp = stamp
        joint.name = LEFT_JOINTS + RIGHT_JOINTS
        joint.position = state["left"] + state["right"]
        self.joint_pub.publish(joint)

        phase = state["phase"] or self.args.day_id
        self.phase_pub.publish(String(data=phase))
        self.status_pub.publish(String(data=f"{self.args.day_id} synchronized_moveit_replay:{phase}"))
        self._publish_gripper_state(stamp, phase)

        if now >= self.finished_at:
            raise SystemExit

    def _publish_gripper_state(self, stamp, phase: str) -> None:
        close_keywords = (
            "align",
            "contact",
            "force",
            "grasp",
            "handoff",
            "pick",
            "place",
            "pull",
            "receive",
            "shuttle",
            "tension",
            "transfer",
        )
        closed = any(keyword in phase for keyword in close_keywords)
        position = 0.000 if closed else 0.018
        for pub, name in (
            (self.left_gripper_pub, "left_finger_joint"),
            (self.right_gripper_pub, "right_finger_joint"),
        ):
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = [name]
            msg.position = [position]
            pub.publish(msg)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan-file", required=True)
    parser.add_argument("--day-id", required=True)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--start-delay", type=float, default=0.25)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = MoveItPlanJointStateReplay(args)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
