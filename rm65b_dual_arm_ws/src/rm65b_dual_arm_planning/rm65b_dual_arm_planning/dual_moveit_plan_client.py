from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .moveit_plan_client import _to_ros_points, _write_gz_trajectory, day_stage_targets


LEFT_JOINTS = [f"left_joint{i}" for i in range(1, 7)]
RIGHT_JOINTS = [f"right_joint{i}" for i in range(1, 7)]
DUAL_JOINTS = LEFT_JOINTS + RIGHT_JOINTS


class DualMoveItPlanClient(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("rm65b_dual_moveit_plan_client")
        self.args = args
        self.client = ActionClient(self, MoveGroup, "/move_action")
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)

    def wait(self) -> None:
        if not self.client.wait_for_server(timeout_sec=self.args.wait_timeout):
            raise RuntimeError("/move_action is not available; dual-arm move_group did not start")

    def plan(self, stage: str, start: tuple[float, ...], target: tuple[float, ...]) -> dict:
        self._publish_start_state(start)
        goal = MoveGroup.Goal()
        goal.request.pipeline_id = self.args.pipeline_id
        goal.request.planner_id = self.args.planner_id
        goal.request.group_name = self.args.group_name
        goal.request.num_planning_attempts = self.args.attempts
        goal.request.allowed_planning_time = self.args.allowed_planning_time
        goal.request.max_velocity_scaling_factor = self.args.velocity_scaling
        goal.request.max_acceleration_scaling_factor = self.args.acceleration_scaling
        goal.request.start_state.joint_state.name = list(DUAL_JOINTS)
        goal.request.start_state.joint_state.position = list(start)
        goal.request.start_state.is_diff = False
        goal.request.goal_constraints.append(self._joint_goal(stage, target))
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.args.allowed_planning_time + 10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"MoveIt rejected dual-arm plan for {stage}")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.args.allowed_planning_time + 20.0)
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError(f"MoveIt returned no dual-arm result for {stage}")
        result = wrapped.result
        trajectory = result.planned_trajectory.joint_trajectory
        if result.error_code.val != MoveItErrorCodes.SUCCESS or not trajectory.points:
            raise RuntimeError(
                f"MoveIt failed dual-arm {stage}: error={result.error_code.val} status={wrapped.status}"
            )
        return {
            "stage": stage,
            "error_code": int(result.error_code.val),
            "status": int(wrapped.status),
            "planning_time": float(result.planning_time),
            "moveit_points": len(trajectory.points),
            "start": list(start),
            "target": list(target),
            "trajectory_points": [
                [float(value) for value in point.positions[:12]]
                for point in trajectory.points
            ],
        }

    def _joint_goal(self, stage: str, target: tuple[float, ...]) -> Constraints:
        constraints = Constraints()
        constraints.name = stage
        for name, position in zip(DUAL_JOINTS, target):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(position)
            joint.tolerance_above = self.args.joint_tolerance
            joint.tolerance_below = self.args.joint_tolerance
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        return constraints

    def _publish_start_state(self, start: tuple[float, ...]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(DUAL_JOINTS)
        msg.position = list(start)
        end = time.monotonic() + 0.3
        while time.monotonic() < end and rclpy.ok():
            self.joint_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)


def _interpolate(points: list[dict], segment: dict, elapsed: float, min_duration: float) -> float:
    duration = max(float(len(segment["trajectory_points"]) - 1) * 0.25, min_duration)
    for idx, positions in enumerate(segment["trajectory_points"]):
        if points and idx == 0:
            continue
        t = elapsed + duration * idx / max(len(segment["trajectory_points"]) - 1, 1)
        points.append(
            {
                "time_from_start": round(t, 4),
                "phase": segment["stage"],
                "day_id": "",
                "left": [round(v, 6) for v in positions[:6]],
                "right": [round(v, 6) for v in positions[6:12]],
            }
        )
    return points[-1]["time_from_start"] if points else elapsed


def _write_outputs(out: Path, day_id: str, segments: list[dict], args: argparse.Namespace) -> None:
    points: list[dict] = []
    elapsed = 0.0
    for segment in segments:
        elapsed = _interpolate(points, segment, elapsed, args.segment_duration)
    for point in points:
        point["day_id"] = day_id
    plan_yaml = {
        "metadata": {
            "source": "MoveIt2 /move_action true dual_arms group",
            "pipeline_id": args.pipeline_id,
            "planner_id": args.planner_id,
            "group_name": args.group_name,
            "day_id": day_id,
            "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        },
        "joint_names": {"left": LEFT_JOINTS, "right": RIGHT_JOINTS, "moveit": DUAL_JOINTS},
        "points": points,
        "segments": [
            {
                key: value
                for key, value in segment.items()
                if key != "trajectory_points"
            }
            for segment in segments
        ],
    }
    out.mkdir(parents=True, exist_ok=True)
    (out / "dual_moveit_plans.yaml").write_text(yaml.safe_dump(plan_yaml, sort_keys=False), encoding="utf-8")
    summary = {
        "day_id": day_id,
        "source": "MoveIt2 /move_action true dual_arms group",
        "pipeline_id": args.pipeline_id,
        "planner_id": args.planner_id,
        "group_name": args.group_name,
        "combined_points": len(points),
        "duration_s": points[-1]["time_from_start"] if points else 0.0,
        "segments": [
            {
                "stage": segment["stage"],
                "error_code": segment["error_code"],
                "status": segment["status"],
                "moveit_points": segment["moveit_points"],
                "planning_time": segment["planning_time"],
            }
            for segment in segments
        ],
    }
    (out / "dual_moveit_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    _write_gz_trajectory(out / "dual_moveit_left_gz_trajectory.pbtxt", points, "left")
    _write_gz_trajectory(out / "dual_moveit_right_gz_trajectory.pbtxt", points, "right")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--day-id", default="day05")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--wait-timeout", type=float, default=30.0)
    parser.add_argument("--allowed-planning-time", type=float, default=10.0)
    parser.add_argument("--attempts", type=int, default=8)
    parser.add_argument("--pipeline-id", default="ompl")
    parser.add_argument("--planner-id", default="RRTConnectkConfigDefault")
    parser.add_argument("--group-name", default="dual_arms")
    parser.add_argument("--velocity-scaling", type=float, default=0.25)
    parser.add_argument("--acceleration-scaling", type=float, default=0.25)
    parser.add_argument("--joint-tolerance", type=float, default=0.01)
    parser.add_argument("--segment-duration", type=float, default=8.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    targets = day_stage_targets(args.day_id)
    rclpy.init()
    node = DualMoveItPlanClient(args)
    try:
        node.wait()
        segments = []
        for idx in range(1, len(targets)):
            stage = targets[idx][0]
            start = tuple(targets[idx - 1][1] + targets[idx - 1][2])
            target = tuple(targets[idx][1] + targets[idx][2])
            segments.append(node.plan(stage, start, target))
        _write_outputs(Path(args.output_dir), args.day_id, segments, args)
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
