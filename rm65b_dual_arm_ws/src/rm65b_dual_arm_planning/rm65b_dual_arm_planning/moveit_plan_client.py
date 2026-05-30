from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .rm65b_kinematics import build_keyframes, load_weaving_yaml


JOINTS = [f"joint{i}" for i in range(1, 7)]
LEFT_JOINTS = [f"left_joint{i}" for i in range(1, 7)]
RIGHT_JOINTS = [f"right_joint{i}" for i in range(1, 7)]


@dataclass
class ArmPlan:
    side: str
    stage: str
    start: tuple[float, ...]
    target: tuple[float, ...]
    points: list[tuple[float, ...]]
    times: list[float]
    planning_time: float
    error_code: int
    status: int


def _duration_to_float(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) / 1e9


def _float_to_duration(value: float):
    sec = int(value)
    nanosec = int((value - sec) * 1e9)
    return sec, nanosec


def _lerp(a: Iterable[float], b: Iterable[float], alpha: float) -> tuple[float, ...]:
    return tuple(float(x) + (float(y) - float(x)) * alpha for x, y in zip(a, b))


def _sample_points(points: list[tuple[float, ...]], count: int) -> list[tuple[float, ...]]:
    if not points:
        return []
    if len(points) == 1:
        return [points[0] for _ in range(count)]
    out: list[tuple[float, ...]] = []
    for idx in range(count):
        pos = idx * (len(points) - 1) / max(count - 1, 1)
        lo = int(math.floor(pos))
        hi = min(lo + 1, len(points) - 1)
        alpha = pos - lo
        out.append(_lerp(points[lo], points[hi], alpha))
    return out


def _same_state(a: Iterable[float], b: Iterable[float]) -> bool:
    return max(abs(float(x) - float(y)) for x, y in zip(a, b)) < 1e-4


def day_stage_targets(day_id: str) -> list[tuple[str, tuple[float, ...], tuple[float, ...]]]:
    frames = {frame.name: frame for frame in build_keyframes(load_weaving_yaml(None))}
    home = frames["home"]
    hook = frames["hook_yarn"]
    lift = frames.get("lift_yarn", frames["pick_yarn"])
    pick = frames["pick_yarn"]
    pull = frames["pull_tight"]
    shift = frames["shift"]
    exchange = frames.get("exchange", frames["complete"])
    complete = frames["complete"]
    day = day_id.lower()
    right_receive = _lerp(hook.right, pull.right, 0.45)
    right_pre_contact = _lerp(hook.right, pull.right, 0.60)
    right_pre_shift = _lerp(hook.right, shift.right, 0.35)
    left_pre_contact = _lerp(hook.left, pick.left, 0.60)
    left_pre_shift = _lerp(pick.left, shift.left, 0.35)
    if day in {"day01", "d1"}:
        # D1 is a gripper/TF/dual-arm capability check. It intentionally avoids
        # object pick/place so the video focuses on gripper open-close states and
        # true dual-arm synchronous/alternating MoveIt planning.
        sync_spread_left = hook.left
        sync_spread_right = hook.right
        sync_shift_left = shift.left
        sync_shift_right = shift.right
        left_only_target = pick.left
        right_only_target = pull.right
        return [
            ("home", home.left, home.right),
            ("d1_sync_spread_both_arms", sync_spread_left, sync_spread_right),
            ("d1_sync_shift_both_arms", sync_shift_left, sync_shift_right),
            ("d1_alternate_left_moves_right_holds", left_only_target, sync_shift_right),
            ("d1_alternate_right_moves_left_holds", left_only_target, right_only_target),
            ("d1_return_dual_ready", home.left, home.right),
        ]
    if day in {"day02", "d2"}:
        return [
            ("home", home.left, home.right),
            ("d2_dual_observe", hook.left, hook.right),
            ("d2_contact_approach", left_pre_contact, right_pre_contact),
            ("d2_force_press", pick.left, pull.right),
            ("d2_release", shift.left, complete.right),
        ]
    if day in {"day03", "d3"}:
        d3_initial_view = (0.046, -0.840, 1.161, -0.120, 0.865, -0.383)
        d3_pre_touch = (0.201, -0.704, 1.285, -0.192, 0.863, -0.544)
        d3_touch_target = (0.356, -0.567, 1.408, -0.263, 0.860, -0.705)
        return [
            ("d3_initial_camera_sees_target", d3_initial_view, home.right),
            ("d3_visual_follow_approach", d3_pre_touch, home.right),
            ("d3_touch_target", d3_touch_target, home.right),
            ("d3_retract_after_touch", d3_initial_view, home.right),
        ]
    if day in {"day04", "d4"}:
        return [
            ("home", home.left, home.right),
            ("d4_hook_yarn", hook.left, hook.right),
            ("d4_lift_yarn", lift.left, lift.right),
            ("d4_pull_tight", left_pre_shift, pull.right),
            ("d4_shift", shift.left, shift.right),
            ("d4_exchange", exchange.left, exchange.right),
        ]
    return [
        ("home", home.left, home.right),
        ("d5_hook_yarn", hook.left, hook.right),
        ("d5_lift_yarn", lift.left, lift.right),
        ("d5_force_pull", left_pre_shift, pull.right),
        ("d5_shift", shift.left, shift.right),
        ("d5_exchange", exchange.left, exchange.right),
    ]


class MoveItPlanClient(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("rm65b_moveit_plan_client")
        self.args = args
        self.client = ActionClient(self, MoveGroup, "/move_action")
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)

    def wait(self) -> None:
        if not self.client.wait_for_server(timeout_sec=self.args.wait_timeout):
            raise RuntimeError("/move_action is not available; move_group did not start")

    def plan(self, side: str, stage: str, start: tuple[float, ...], target: tuple[float, ...]) -> ArmPlan:
        if _same_state(start, target):
            return ArmPlan(side, stage, start, target, [start, target], [0.0, 0.5], 0.0, MoveItErrorCodes.SUCCESS, GoalStatus.STATUS_SUCCEEDED)

        self._publish_start_state(start)
        goal = MoveGroup.Goal()
        goal.request.pipeline_id = self.args.pipeline_id
        goal.request.planner_id = self.args.planner_id
        goal.request.group_name = "rm_group"
        goal.request.num_planning_attempts = self.args.attempts
        goal.request.allowed_planning_time = self.args.allowed_planning_time
        goal.request.max_velocity_scaling_factor = self.args.velocity_scaling
        goal.request.max_acceleration_scaling_factor = self.args.acceleration_scaling
        goal.request.start_state.joint_state.name = list(JOINTS)
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
            raise RuntimeError(f"MoveIt rejected {side} {stage} plan")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.args.allowed_planning_time + 15.0)
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError(f"MoveIt returned no result for {side} {stage}")
        result = wrapped.result
        traj = result.planned_trajectory.joint_trajectory
        points = [tuple(float(v) for v in point.positions[:6]) for point in traj.points]
        times = [_duration_to_float(point.time_from_start) for point in traj.points]
        if result.error_code.val != MoveItErrorCodes.SUCCESS or not points:
            raise RuntimeError(
                f"MoveIt failed {side} {stage}: error={result.error_code.val} status={wrapped.status}"
            )
        if points[0] != start:
            points.insert(0, start)
            times.insert(0, 0.0)
        return ArmPlan(
            side=side,
            stage=stage,
            start=start,
            target=target,
            points=points,
            times=times,
            planning_time=float(result.planning_time),
            error_code=int(result.error_code.val),
            status=int(wrapped.status),
        )

    def _joint_goal(self, stage: str, target: tuple[float, ...]) -> Constraints:
        constraints = Constraints()
        constraints.name = stage
        for name, position in zip(JOINTS, target):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(position)
            jc.tolerance_above = self.args.joint_tolerance
            jc.tolerance_below = self.args.joint_tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        return constraints

    def _publish_start_state(self, start: tuple[float, ...]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINTS)
        msg.position = list(start)
        end = time.monotonic() + 0.3
        while time.monotonic() < end and rclpy.ok():
            self.joint_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)


def _to_ros_points(day_id: str, segment_plans: list[tuple[str, ArmPlan, ArmPlan]]) -> list[dict]:
    out: list[dict] = []
    elapsed = 0.0
    for stage, left_plan, right_plan in segment_plans:
        count = max(len(left_plan.points), len(right_plan.points), 3)
        left_samples = _sample_points(left_plan.points, count)
        right_samples = _sample_points(right_plan.points, count)
        duration = max(left_plan.times[-1] if left_plan.times else 0.0, right_plan.times[-1] if right_plan.times else 0.0, 2.0)
        duration = max(duration, 2.5)
        for idx in range(count):
            if out and idx == 0:
                continue
            t = elapsed + duration * idx / max(count - 1, 1)
            out.append(
                {
                    "time_from_start": round(t, 4),
                    "phase": stage,
                    "day_id": day_id,
                    "left": [round(v, 6) for v in left_samples[idx]],
                    "right": [round(v, 6) for v in right_samples[idx]],
                }
            )
        elapsed = out[-1]["time_from_start"]
    return out


def _write_gz_trajectory(path: Path, points: list[dict], side: str) -> None:
    lines = [f'joint_names: "{name}"' for name in JOINTS]
    previous_sec = -1
    for point in points:
        positions = point[side]
        sec, nanosec = _float_to_duration(float(point["time_from_start"]) + 0.25)
        if sec == previous_sec and nanosec == 0:
            nanosec = 1
        previous_sec = sec
        fields = " ".join(f"positions: {value:.6f}" for value in positions)
        lines.append(f"points {{ {fields} time_from_start {{ sec: {sec} nsec: {nanosec} }} }}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_plan_yaml(path: Path, day_id: str, points: list[dict], plans: list[ArmPlan], args: argparse.Namespace) -> None:
    data = {
        "metadata": {
            "source": "MoveIt2 /move_action",
            "pipeline_id": args.pipeline_id,
            "planner_id": args.planner_id,
            "group_name": "rm_group",
            "day_id": day_id,
            "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        },
        "joint_names": {"left": LEFT_JOINTS, "right": RIGHT_JOINTS, "moveit": JOINTS},
        "points": points,
        "segments": [
            {
                "side": plan.side,
                "stage": plan.stage,
                "moveit_points": len(plan.points),
                "planning_time": plan.planning_time,
                "error_code": plan.error_code,
                "status": plan.status,
                "start": [round(v, 6) for v in plan.start],
                "target": [round(v, 6) for v in plan.target],
            }
            for plan in plans
        ],
    }
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")


def _write_summary(path: Path, day_id: str, points: list[dict], plans: list[ArmPlan], args: argparse.Namespace) -> None:
    summary = {
        "day_id": day_id,
        "source": "MoveIt2 /move_action",
        "pipeline_id": args.pipeline_id,
        "planner_id": args.planner_id,
        "combined_points": len(points),
        "duration_s": points[-1]["time_from_start"] if points else 0.0,
        "segments": [
            {
                "side": plan.side,
                "stage": plan.stage,
                "error_code": plan.error_code,
                "status": plan.status,
                "moveit_points": len(plan.points),
                "planning_time": plan.planning_time,
            }
            for plan in plans
        ],
    }
    path.write_text(json.dumps(summary, indent=2), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--day-id", default="day05")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--wait-timeout", type=float, default=30.0)
    parser.add_argument("--allowed-planning-time", type=float, default=5.0)
    parser.add_argument("--attempts", type=int, default=8)
    parser.add_argument("--pipeline-id", default="ompl")
    parser.add_argument("--planner-id", default="RRTConnectkConfigDefault")
    parser.add_argument("--velocity-scaling", type=float, default=0.35)
    parser.add_argument("--acceleration-scaling", type=float, default=0.35)
    parser.add_argument("--joint-tolerance", type=float, default=0.01)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = MoveItPlanClient(args)
    try:
        node.wait()
        targets = day_stage_targets(args.day_id)
        plans: list[ArmPlan] = []
        segments: list[tuple[str, ArmPlan, ArmPlan]] = []
        for idx in range(1, len(targets)):
            stage = targets[idx][0]
            left_plan = node.plan("left", stage, targets[idx - 1][1], targets[idx][1])
            right_plan = node.plan("right", stage, targets[idx - 1][2], targets[idx][2])
            plans.extend([left_plan, right_plan])
            segments.append((stage, left_plan, right_plan))
        points = _to_ros_points(args.day_id, segments)
        _write_plan_yaml(out / "moveit_plans.yaml", args.day_id, points, plans, args)
        _write_summary(out / "moveit_summary.json", args.day_id, points, plans, args)
        _write_gz_trajectory(out / "moveit_left_gz_trajectory.pbtxt", points, "left")
        _write_gz_trajectory(out / "moveit_right_gz_trajectory.pbtxt", points, "right")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
