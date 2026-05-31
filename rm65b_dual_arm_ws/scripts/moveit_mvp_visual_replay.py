#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


LEFT_JOINTS = [f"left_joint{i}" for i in range(1, 7)]
RIGHT_JOINTS = [f"right_joint{i}" for i in range(1, 7)]
DUAL_JOINTS = LEFT_JOINTS + RIGHT_JOINTS
LOCAL_JOINTS = [f"joint{i}" for i in range(1, 7)]

HOME = (0.0, -0.35, 0.65, 0.0, 0.90, 0.0)

D1_LINE_A = (0.0, -0.64, 1.02, 0.0, 0.72, 0.0)
D1_LINE_B = (0.0, -0.18, 0.36, 0.0, 1.05, 0.0)

D2_LEFT_OBSERVER = (0.260, -0.580, 0.780, 0.100, 0.720, 0.200)
D2_RIGHT_READY = (-0.280, -0.540, 0.760, -0.120, 0.740, -0.220)
D2_RIGHT_APPROACH = (-0.560, -0.705, 1.010, -0.355, 0.525, -0.520)
D2_RIGHT_PRESS = (-0.780, -0.780, 1.100, -0.500, 0.440, -0.680)
D2_RIGHT_RELIEF = (-0.650, -0.725, 1.040, -0.430, 0.500, -0.580)


def _lerp(a: tuple[float, ...], b: tuple[float, ...], alpha: float) -> tuple[float, ...]:
    return tuple(float(x) + (float(y) - float(x)) * alpha for x, y in zip(a, b))


def _mirror(point: tuple[float, ...]) -> tuple[float, ...]:
    q1, q2, q3, q4, q5, q6 = point
    return (-q1, q2, q3, -q4, q5, -q6)


def stage_targets(day_id: str) -> list[tuple[str, tuple[float, ...], tuple[float, ...]]]:
    day = day_id.lower()
    if day in {"day01", "d1"}:
        return [
            ("home", HOME, HOME),
            ("d1_line_start", D1_LINE_A, _mirror(D1_LINE_A)),
            ("d1_straight_line_out", D1_LINE_B, _mirror(D1_LINE_B)),
            ("d1_straight_line_back", D1_LINE_A, _mirror(D1_LINE_A)),
            ("d1_return_home", HOME, HOME),
        ]
    if day in {"day02", "d2"}:
        return [
            ("home", HOME, HOME),
            ("d2_ready_force_station", D2_LEFT_OBSERVER, D2_RIGHT_READY),
            ("d2_approach_pad", D2_LEFT_OBSERVER, D2_RIGHT_APPROACH),
            ("d2_force_press", D2_LEFT_OBSERVER, D2_RIGHT_PRESS),
            ("d2_compliance_relief", D2_LEFT_OBSERVER, D2_RIGHT_RELIEF),
            ("d2_force_press_settle", D2_LEFT_OBSERVER, D2_RIGHT_PRESS),
            ("d2_release", D2_LEFT_OBSERVER, D2_RIGHT_READY),
        ]
    if day in {"day03", "d3"}:
        left_view = (0.046, -0.840, 1.161, -0.120, 0.865, -0.383)
        left_align = (0.260, -0.700, 1.300, -0.210, 0.860, -0.610)
        left_touch = (0.356, -0.567, 1.408, -0.263, 0.860, -0.705)
        return [
            ("d3_camera_view", left_view, HOME),
            ("d3_visual_align", left_align, HOME),
            ("d3_touch_target", left_touch, HOME),
            ("d3_retract", left_view, HOME),
        ]
    if day in {"day04", "d4"}:
        return [
            ("home", HOME, HOME),
            ("d4_hook_yarn", D1_LINE_A, D2_RIGHT_READY),
            ("d4_pull_tight", D1_LINE_B, D2_RIGHT_PRESS),
            ("d4_release", HOME, HOME),
        ]
    return [
        ("home", HOME, HOME),
        ("d5_vision_pick", D1_LINE_A, D2_RIGHT_READY),
        ("d5_force_pull", D1_LINE_B, D2_RIGHT_PRESS),
        ("d5_exchange", _lerp(D1_LINE_A, D1_LINE_B, 0.5), D2_RIGHT_RELIEF),
        ("d5_done", HOME, HOME),
    ]


def _duration_to_float(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) / 1e9


def _trajectory_msg(points: list[dict], side: str, time_offset: float) -> JointTrajectory:
    msg = JointTrajectory()
    msg.joint_names = list(LOCAL_JOINTS)
    for frame in points:
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in frame[side]]
        seconds = max(float(frame["time_from_start"]) + time_offset, 0.25)
        point.time_from_start.sec = int(seconds)
        point.time_from_start.nanosec = int((seconds - int(seconds)) * 1e9)
        msg.points.append(point)
    return msg


def _trajectory_to_gz_pbtxt(msg: JointTrajectory) -> str:
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


def _sample(points: list[dict], elapsed: float) -> dict:
    if elapsed <= 0:
        return points[0]
    if elapsed >= float(points[-1]["time_from_start"]):
        return points[-1]
    previous = points[0]
    current = points[-1]
    for idx in range(1, len(points)):
        if elapsed <= float(points[idx]["time_from_start"]):
            previous = points[idx - 1]
            current = points[idx]
            break
    t0 = float(previous["time_from_start"])
    t1 = float(current["time_from_start"])
    alpha = 0.0 if t1 <= t0 else (elapsed - t0) / (t1 - t0)
    return {
        "time_from_start": elapsed,
        "phase": current.get("phase", ""),
        "left": [float(a) + (float(b) - float(a)) * alpha for a, b in zip(previous["left"], current["left"])],
        "right": [float(a) + (float(b) - float(a)) * alpha for a, b in zip(previous["right"], current["right"])],
    }


class MoveItMvpReplay(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("rm65b_moveit_mvp_visual_replay")
        self.args = args
        self.client = ActionClient(self, MoveGroup, "/move_action")
        self.left_pub = self.create_publisher(JointTrajectory, "/model/left_rm65b/joint_trajectory", 10)
        self.right_pub = self.create_publisher(JointTrajectory, "/model/right_rm65b/joint_trajectory", 10)
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.phase_pub = self.create_publisher(String, "/dual_arm_planning/phase", 10)
        self.day_status_pub = self.create_publisher(String, "/acceptance/day_status", 10)
        self.force_state_pub = self.create_publisher(String, "/force_control/state", 10)
        self.force_target_pub = self.create_publisher(WrenchStamped, "/force_control/target_wrench", 10)
        self.force_error_pub = self.create_publisher(WrenchStamped, "/force_control/wrench_error", 10)
        self.admittance_pub = self.create_publisher(Float64, "/force_control/admittance_offset", 10)
        self.upper_pub = self.create_publisher(Float64, "/rm65b_gripper/upper_finger_cmd", 10)
        self.lower_pub = self.create_publisher(Float64, "/rm65b_gripper/lower_finger_cmd", 10)

    def wait_for_moveit(self) -> None:
        if not self.client.wait_for_server(timeout_sec=self.args.wait_timeout):
            raise RuntimeError("/move_action is not available")

    def plan_segment(
        self,
        stage: str,
        start: tuple[float, ...],
        target: tuple[float, ...],
    ) -> dict:
        self._publish_start_state(start)
        goal = MoveGroup.Goal()
        goal.request.pipeline_id = self.args.pipeline_id
        goal.request.planner_id = self.args.planner_id
        goal.request.group_name = "dual_arms"
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
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.args.allowed_planning_time + 8.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"MoveIt rejected {stage}")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.args.allowed_planning_time + 12.0)
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError(f"MoveIt returned no result for {stage}")
        result = wrapped.result
        trajectory = result.planned_trajectory.joint_trajectory
        if result.error_code.val != MoveItErrorCodes.SUCCESS or not trajectory.points:
            raise RuntimeError(f"MoveIt failed {stage}: error={result.error_code.val}")

        names = list(trajectory.joint_names)
        planned_points = []
        for point in trajectory.points:
            values = []
            for idx, name in enumerate(DUAL_JOINTS):
                if name in names:
                    values.append(float(point.positions[names.index(name)]))
                else:
                    values.append(float(target[idx]))
            planned_points.append(values)
        if not planned_points or max(abs(a - b) for a, b in zip(planned_points[0], start)) > 1e-4:
            planned_points.insert(0, list(start))

        return {
            "stage": stage,
            "start": list(start),
            "target": list(target),
            "planning_time": float(result.planning_time),
            "moveit_points": len(trajectory.points),
            "trajectory_points": planned_points,
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
        deadline = time.monotonic() + 0.35
        while time.monotonic() < deadline and rclpy.ok():
            self.joint_state_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)

    def build_plan(self) -> tuple[list[dict], list[dict]]:
        targets = stage_targets(self.args.day_id)
        segments = []
        frames: list[dict] = []
        elapsed = 0.0
        for idx in range(1, len(targets)):
            stage = targets[idx][0]
            start = tuple(targets[idx - 1][1] + targets[idx - 1][2])
            target = tuple(targets[idx][1] + targets[idx][2])
            segment = self.plan_segment(stage, start, target)
            segments.append(segment)

            raw = segment["trajectory_points"]
            duration = max(self.args.segment_duration, 0.5 * max(len(raw) - 1, 1))
            for pidx, positions in enumerate(raw):
                if frames and pidx == 0:
                    continue
                t = elapsed + duration * pidx / max(len(raw) - 1, 1)
                frames.append(
                    {
                        "time_from_start": round(t, 4),
                        "phase": stage,
                        "left": [round(v, 6) for v in positions[:6]],
                        "right": [round(v, 6) for v in positions[6:12]],
                    }
                )
            elapsed = float(frames[-1]["time_from_start"])
        return frames, segments

    def write_outputs(self, points: list[dict], segments: list[dict]) -> None:
        out = Path(self.args.output_dir)
        out.mkdir(parents=True, exist_ok=True)
        plan = {
            "metadata": {
                "source": "MVP MoveIt2 /move_action dual_arms visual replay",
                "day_id": self.args.day_id,
                "group_name": "dual_arms",
                "pipeline_id": self.args.pipeline_id,
                "planner_id": self.args.planner_id,
            },
            "joint_names": {"left": LEFT_JOINTS, "right": RIGHT_JOINTS, "moveit": DUAL_JOINTS},
            "points": points,
            "segments": [
                {k: v for k, v in segment.items() if k != "trajectory_points"} for segment in segments
            ],
        }
        (out / "mvp_moveit_plan.yaml").write_text(yaml.safe_dump(plan, sort_keys=False), encoding="utf-8")
        summary = {
            "day_id": self.args.day_id,
            "source": "MoveIt2 /move_action dual_arms",
            "duration_s": points[-1]["time_from_start"] if points else 0.0,
            "segments": [
                {
                    "stage": s["stage"],
                    "moveit_points": s["moveit_points"],
                    "planning_time": s["planning_time"],
                }
                for s in segments
            ],
        }
        (out / "mvp_moveit_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    def replay(self, points: list[dict]) -> None:
        left = _trajectory_msg(points, "left", self.args.time_offset)
        right = _trajectory_msg(points, "right", self.args.time_offset)
        plan_duration = float(points[-1]["time_from_start"]) + self.args.time_offset
        run_duration = self.args.duration if self.args.duration > 0 else plan_duration + 2.0
        started = time.monotonic()
        next_send = started
        send_interval = max(plan_duration + 0.5, 1.0)

        while rclpy.ok() and time.monotonic() - started <= run_duration:
            now = time.monotonic()
            if now >= next_send:
                self._send_trajectories(left, right)
                next_send = now + send_interval
            loop_elapsed = (now - started) % max(plan_duration, 0.1)
            state = _sample(points, max(loop_elapsed - self.args.time_offset, 0.0))
            self._publish_observable_state(state, loop_elapsed)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(1.0 / max(self.args.state_rate_hz, 1.0))

    def _send_trajectories(self, left: JointTrajectory, right: JointTrajectory) -> None:
        if self.args.direct_gz:
            self._send_direct_gz("/model/left_rm65b/joint_trajectory", left)
            self._send_direct_gz("/model/right_rm65b/joint_trajectory", right)
            self._send_direct_gripper(0.0 if self.args.day_id.lower() in {"day02", "d2"} else 0.018)
        deadline = time.monotonic() + self.args.publish_burst_s
        while time.monotonic() < deadline and rclpy.ok():
            self.left_pub.publish(left)
            self.right_pub.publish(right)
            self._publish_gripper_ros(0.0 if self.args.day_id.lower() in {"day02", "d2"} else 0.018)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.05)

    def _send_direct_gz(self, topic: str, msg: JointTrajectory) -> None:
        subprocess.run(
            ["gz", "topic", "-t", topic, "-m", "gz.msgs.JointTrajectory", "-p", _trajectory_to_gz_pbtxt(msg)],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=3.0,
        )

    def _publish_gripper_ros(self, position: float) -> None:
        msg = Float64(data=float(position))
        self.upper_pub.publish(msg)
        self.lower_pub.publish(msg)

    def _send_direct_gripper(self, position: float) -> None:
        for topic in ("/rm65b_gripper/upper_finger_cmd", "/rm65b_gripper/lower_finger_cmd"):
            subprocess.run(
                ["gz", "topic", "-t", topic, "-m", "gz.msgs.Double", "-p", f"data: {position:.6f}"],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=1.0,
            )

    def _publish_observable_state(self, state: dict, elapsed: float) -> None:
        stamp = self.get_clock().now().to_msg()
        joint = JointState()
        joint.header.stamp = stamp
        joint.name = list(DUAL_JOINTS)
        joint.position = list(state["left"] + state["right"])
        self.joint_state_pub.publish(joint)

        phase = str(state.get("phase", self.args.day_id))
        self.phase_pub.publish(String(data=phase))
        self.day_status_pub.publish(String(data=f"{self.args.day_id} mvp_moveit:{phase}"))

        if self.args.day_id.lower() in {"day02", "d2"}:
            target = 7.5
            if "press" in phase:
                measured = 7.1 + 0.25 * min(elapsed, 1.0)
                offset = -0.025
            elif "relief" in phase:
                measured = 7.7
                offset = 0.035
            else:
                measured = 3.0
                offset = 0.0
            error = target - measured
            self.admittance_pub.publish(Float64(data=float(offset)))
            self.force_state_pub.publish(
                String(data=f"mvp_admittance phase={phase} target_fz={target:.2f} measured_fz={measured:.2f} offset_rad={offset:.4f}")
            )
            target_msg = WrenchStamped()
            target_msg.header.stamp = stamp
            target_msg.header.frame_id = "right_tcp"
            target_msg.wrench.force.z = target
            self.force_target_pub.publish(target_msg)

            error_msg = WrenchStamped()
            error_msg.header.stamp = stamp
            error_msg.header.frame_id = "right_tcp"
            error_msg.wrench.force.z = error
            self.force_error_pub.publish(error_msg)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--day-id", default="day01")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--wait-timeout", type=float, default=30.0)
    parser.add_argument("--allowed-planning-time", type=float, default=5.0)
    parser.add_argument("--attempts", type=int, default=4)
    parser.add_argument("--pipeline-id", default="ompl")
    parser.add_argument("--planner-id", default="RRTConnectkConfigDefault")
    parser.add_argument("--velocity-scaling", type=float, default=0.45)
    parser.add_argument("--acceleration-scaling", type=float, default=0.45)
    parser.add_argument("--joint-tolerance", type=float, default=0.015)
    parser.add_argument("--segment-duration", type=float, default=3.0)
    parser.add_argument("--time-offset", type=float, default=0.30)
    parser.add_argument("--publish-burst-s", type=float, default=1.0)
    parser.add_argument("--state-rate-hz", type=float, default=20.0)
    parser.add_argument("--no-direct-gz", dest="direct_gz", action="store_false")
    parser.set_defaults(direct_gz=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = MoveItMvpReplay(args)
    try:
        node.wait_for_moveit()
        points, segments = node.build_plan()
        node.write_outputs(points, segments)
        node.replay(points)
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
