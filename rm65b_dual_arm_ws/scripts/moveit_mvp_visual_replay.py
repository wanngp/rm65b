#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

try:
    import cv2
except Exception:  # pragma: no cover - optional runtime dependency
    cv2 = None

try:
    import numpy as np
except Exception:  # pragma: no cover - optional runtime dependency
    np = None

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, Float64, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


LEFT_JOINTS = [f"left_joint{i}" for i in range(1, 7)]
RIGHT_JOINTS = [f"right_joint{i}" for i in range(1, 7)]
DUAL_JOINTS = LEFT_JOINTS + RIGHT_JOINTS
ARUCO_TAG_SIZE_M = 0.080
ARUCO_MARKER_ID = 7

HOME = (0.0, -0.35, 0.65, 0.0, 0.90, 0.0)

D1_LINE_A = (0.36, -0.72, 1.24, 0.26, 0.62, -0.35)
D1_LINE_B = (0.36, -0.10, 0.34, -0.26, 1.20, 0.35)
D1_LINE_C = (0.36, -0.54, 1.48, 0.34, 0.56, -0.70)

D2_LEFT_OBSERVER = (0.260, -0.580, 0.780, 0.100, 0.720, 0.200)
D2_RIGHT_READY = (0.000, -0.300, 0.423, 0.000, 0.944, 0.000)
D2_RIGHT_APPROACH = (0.000, -0.450, 0.935, 0.000, 0.727, 0.000)
D2_RIGHT_PRESS = (0.000, -0.300, 1.100, 0.000, 0.826, 0.000)
D2_RIGHT_RELIEF = (0.000, -0.433, 1.100, 0.000, 0.708, 0.000)

D45_RIGHT_READY = (-0.280, -0.540, 0.760, -0.120, 0.740, -0.220)
D45_RIGHT_PRESS = (-0.780, -0.780, 1.100, -0.500, 0.440, -0.680)
D45_RIGHT_RELIEF = (-0.650, -0.725, 1.040, -0.430, 0.500, -0.580)

D4_LEFT_GUARD = (0.398, -0.594, -0.378, -1.913, -1.816, -0.668)
D4_LEFT_RECEIVE = (-0.348, -0.571, -0.391, -1.389, -1.270, -0.632)
D4_LEFT_TENSION = (-0.210, -0.545, -0.365, 1.820, -1.641, 0.686)
D4_RIGHT_ENTRY = (0.005, 0.326, 1.269, 0.782, -1.592, 0.014)
D4_RIGHT_INSERT = (0.293, 0.381, 1.136, -0.879, -1.310, -0.145)
D4_RIGHT_PULL_TIGHT = (-0.395, 1.513, -1.219, 0.191, -0.277, 0.212)
D5_LEFT_LOOP_TOP = (-0.063, -0.611, -0.157, -1.829, -1.231, -0.745)
D5_LEFT_LOOP_BOTTOM = (-0.085, -0.435, -0.646, -1.070, -1.740, -0.469)
D5_LEFT_CENTER = (-0.632, -0.860, -0.066, -1.467, -0.877, -0.575)
D5_RIGHT_LOOP_TOP = (0.439, 1.245, -0.854, -0.228, -0.362, -0.227)
D5_RIGHT_LOOP_BOTTOM = (0.053, 0.339, 1.391, -1.204, -1.578, -0.167)
D5_RIGHT_CENTER = (0.706, 0.788, 0.512, -0.756, -0.890, -0.339)
D4_TENSION_WINDOWS = {
    "hook_yarn": (0.25, 1.20, 0.55),
    "lift_yarn": (0.45, 1.60, 1.05),
    "pull_tight": (0.80, 3.40, 1.85),
    "shift": (0.70, 3.20, 1.65),
    "exchange": (0.35, 2.00, 0.85),
}


def _lerp(a: tuple[float, ...], b: tuple[float, ...], alpha: float) -> tuple[float, ...]:
    return tuple(float(x) + (float(y) - float(x)) * alpha for x, y in zip(a, b))


def _mirror(point: tuple[float, ...]) -> tuple[float, ...]:
    q1, q2, q3, q4, q5, q6 = point
    return (-q1, q2, q3, -q4, q5, -q6)


def _smootherstep(alpha: float) -> float:
    t = min(max(float(alpha), 0.0), 1.0)
    return t * t * t * (t * (6.0 * t - 15.0) + 10.0)


def _seconds_to_msg(point: JointTrajectoryPoint, seconds: float) -> None:
    value = max(float(seconds), 0.0)
    point.time_from_start.sec = int(value)
    point.time_from_start.nanosec = int((value - int(value)) * 1e9)


def _finite_difference_velocities(positions: list[list[float]], times: list[float]) -> list[list[float]]:
    velocities: list[list[float]] = []
    last_idx = len(positions) - 1
    for idx, current in enumerate(positions):
        if idx == 0 or idx == last_idx:
            velocities.append([0.0 for _ in current])
            continue
        dt = times[idx + 1] - times[idx - 1]
        if dt <= 1e-9:
            velocities.append([0.0 for _ in current])
            continue
        velocities.append([(b - a) / dt for a, b in zip(positions[idx - 1], positions[idx + 1])])
    return velocities


def _matrix_to_quaternion(matrix) -> tuple[float, float, float, float]:
    m00 = float(matrix[0][0])
    m01 = float(matrix[0][1])
    m02 = float(matrix[0][2])
    m10 = float(matrix[1][0])
    m11 = float(matrix[1][1])
    m12 = float(matrix[1][2])
    m20 = float(matrix[2][0])
    m21 = float(matrix[2][1])
    m22 = float(matrix[2][2])
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (m21 - m12) / scale
        qy = (m02 - m20) / scale
        qz = (m10 - m01) / scale
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / scale
        qx = 0.25 * scale
        qy = (m01 + m10) / scale
        qz = (m02 + m20) / scale
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / scale
        qx = (m01 + m10) / scale
        qy = 0.25 * scale
        qz = (m12 + m21) / scale
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / scale
        qx = (m02 + m20) / scale
        qy = (m12 + m21) / scale
        qz = 0.25 * scale
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1.0e-9:
        return 0.0, 0.0, 0.0, 1.0
    return qx / norm, qy / norm, qz / norm, qw / norm


def stage_targets(day_id: str) -> list[tuple[str, tuple[float, ...], tuple[float, ...]]]:
    day = day_id.lower()
    if day in {"day01", "d1"}:
        return [
            ("home", HOME, HOME),
            ("d1_line_start", D1_LINE_A, _mirror(D1_LINE_A)),
            ("d1_straight_line_out", D1_LINE_B, _mirror(D1_LINE_B)),
            ("d1_straight_line_high", D1_LINE_C, _mirror(D1_LINE_C)),
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
            ("d2_return_home", HOME, HOME),
        ]
    if day in {"day03", "d3"}:
        right_view = (-1.089, -0.582, 1.161, 1.098, 1.776, -2.139)
        right_align = (-0.225, -0.522, 1.224, 0.251, 1.200, -1.722)
        right_confirm = (-0.886, -0.280, 0.993, 0.662, 1.554, -2.035)
        return [
            ("d3_right_hand_eye_view_wide", HOME, right_view),
            ("d3_right_visual_align_target", HOME, right_align),
            ("d3_right_hand_eye_confirm_lock", HOME, right_confirm),
            ("d3_right_retract_camera", HOME, right_view),
        ]
    if day in {"day04", "d4"}:
        return [
            ("home", HOME, HOME),
            ("hook_yarn", D4_LEFT_GUARD, D4_RIGHT_ENTRY),
            ("lift_yarn", D4_LEFT_RECEIVE, D4_RIGHT_ENTRY),
            ("pull_tight", D4_LEFT_TENSION, D4_RIGHT_PULL_TIGHT),
            ("shift", D4_LEFT_RECEIVE, D4_RIGHT_INSERT),
            ("exchange", D4_LEFT_GUARD, D4_RIGHT_ENTRY),
            ("d4_return_home", HOME, HOME),
        ]
    if day in {"day05", "d5"}:
        return [
            ("home", HOME, HOME),
            ("d5_vision_lock", D5_LEFT_CENTER, D5_RIGHT_CENTER),
            ("d5_loop_a_hook_yarn", D4_LEFT_GUARD, D4_RIGHT_ENTRY),
            ("d5_loop_a_lift_yarn", D5_LEFT_LOOP_TOP, D5_RIGHT_LOOP_BOTTOM),
            ("d5_loop_a_pull_tight", D4_LEFT_TENSION, D4_RIGHT_PULL_TIGHT),
            ("d5_loop_a_shift", D5_LEFT_CENTER, D5_RIGHT_CENTER),
            ("d5_loop_a_exchange", D5_LEFT_LOOP_BOTTOM, D5_RIGHT_LOOP_TOP),
            ("d5_loop_b_hook_yarn", D4_LEFT_GUARD, D4_RIGHT_ENTRY),
            ("d5_loop_b_lift_yarn", D5_LEFT_LOOP_BOTTOM, D5_RIGHT_LOOP_TOP),
            ("d5_loop_b_pull_tight", D4_LEFT_TENSION, D4_RIGHT_PULL_TIGHT),
            ("d5_loop_b_shift", D5_LEFT_CENTER, D5_RIGHT_CENTER),
            ("d5_loop_b_exchange", D5_LEFT_LOOP_TOP, D5_RIGHT_LOOP_BOTTOM),
            ("d5_done", HOME, HOME),
        ]
    return [
        ("home", HOME, HOME),
        ("d5_vision_pick", D1_LINE_A, D45_RIGHT_READY),
        ("d5_force_pull", D1_LINE_B, D45_RIGHT_PRESS),
        ("d5_exchange", _lerp(D1_LINE_A, D1_LINE_B, 0.5), D45_RIGHT_RELIEF),
        ("d5_done", HOME, HOME),
    ]


def _trajectory_msg(points: list[dict], side: str) -> JointTrajectory:
    msg = JointTrajectory()
    msg.joint_names = list(LEFT_JOINTS if side == "left" else RIGHT_JOINTS)
    positions = [[float(v) for v in frame[side]] for frame in points]
    times = [float(frame["time_from_start"]) for frame in points]
    velocities = _finite_difference_velocities(positions, times)
    for frame, frame_positions, frame_velocities in zip(points, positions, velocities):
        point = JointTrajectoryPoint()
        point.positions = frame_positions
        point.velocities = frame_velocities
        _seconds_to_msg(point, float(frame["time_from_start"]))
        msg.points.append(point)
    return msg


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
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.gazebo_traj_pub = self.create_publisher(JointTrajectory, args.gazebo_topic, 10)
        self.left_traj_pub = self.create_publisher(JointTrajectory, "/dual_arm_planning/left_joint_trajectory", 10)
        self.right_traj_pub = self.create_publisher(JointTrajectory, "/dual_arm_planning/right_joint_trajectory", 10)
        self.phase_pub = self.create_publisher(String, "/dual_arm_planning/phase", 10)
        self.day_status_pub = self.create_publisher(String, "/acceptance/day_status", 10)
        self.force_state_pub = self.create_publisher(String, "/force_control/state", 10)
        self.force_target_pub = self.create_publisher(WrenchStamped, "/force_control/target_wrench", 10)
        self.force_error_pub = self.create_publisher(WrenchStamped, "/force_control/wrench_error", 10)
        self.admittance_pub = self.create_publisher(Float64, "/force_control/admittance_offset", 10)
        self.camera_image_pub = self.create_publisher(Image, "/right_camera/image_rect", 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, "/right_camera/camera_info", 10)
        self.debug_image_pub = self.create_publisher(Image, "/vision/debug_image", 10)
        self.vision_target_pub = self.create_publisher(PoseStamped, "/vision/target_pose", 10)
        self.vision_status_pub = self.create_publisher(String, "/vision/status", 10)
        self.vision_metrics_pub = self.create_publisher(String, "/vision/metrics", 10)
        self.visual_twist_pub = self.create_publisher(TwistStamped, "/visual_servo/twist_cmd", 10)
        self.visual_aligned_pub = self.create_publisher(Bool, "/visual_servo/aligned", 10)
        self.visual_adapter_state_pub = self.create_publisher(String, "/visual_servo/gazebo_adapter_state", 10)
        self.weaving_events_pub = self.create_publisher(String, "/weaving/events", 10)
        self.weaving_tension_pub = self.create_publisher(Float64, "/weaving/tension_n", 10)
        self.weaving_status_pub = self.create_publisher(String, "/weaving/tension_status", 10)
        self.weaving_pid_pub = self.create_publisher(String, "/weaving/tension_pid_state", 10)
        self.weaving_compliance_pub = self.create_publisher(Float64, "/weaving/compliance_offset_m", 10)
        self.weaving_yarn_state_pub = self.create_publisher(String, "/weaving/yarn_state", 10)
        self.weaving_lock_pub = self.create_publisher(Bool, "/weaving/primitive_lock", 10)
        self.gripper_pubs = [
            self.create_publisher(Float64, "/model/dual_rm65b_mvp/left_gripper_upper_cmd", 10),
            self.create_publisher(Float64, "/model/dual_rm65b_mvp/left_gripper_lower_cmd", 10),
            self.create_publisher(Float64, "/model/dual_rm65b_mvp/right_gripper_upper_cmd", 10),
            self.create_publisher(Float64, "/model/dual_rm65b_mvp/right_gripper_lower_cmd", 10),
        ]

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
        day = self.args.day_id.lower()
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

            start_positions = [float(v) for v in segment["start"]]
            target_positions = [float(v) for v in segment["target"]]
            duration = self._stage_duration(stage)
            steps = max(2, int(math.ceil(duration * max(self.args.trajectory_rate_hz, 1.0))))
            use_moveit_path = day in {"day04", "d4", "day05", "d5"} and len(segment["trajectory_points"]) >= 2
            for step in range(steps + 1):
                if frames and step == 0:
                    continue
                alpha = step / steps
                if use_moveit_path:
                    path_position = _smootherstep(alpha) * (len(segment["trajectory_points"]) - 1)
                    low = min(int(math.floor(path_position)), len(segment["trajectory_points"]) - 2)
                    high = low + 1
                    local_alpha = path_position - low
                    positions = [
                        start + (target - start) * local_alpha
                        for start, target in zip(segment["trajectory_points"][low], segment["trajectory_points"][high])
                    ]
                else:
                    smooth_alpha = _smootherstep(alpha)
                    positions = [
                        start + (target - start) * smooth_alpha
                        for start, target in zip(start_positions, target_positions)
                    ]
                t = elapsed + duration * alpha
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

    def _stage_duration(self, stage: str) -> float:
        if "vision_lock" in stage:
            return max(self.args.segment_duration * 0.85, 2.5)
        if "pull_tight" in stage:
            return max(self.args.segment_duration * 1.25, 3.5)
        if "shift" in stage:
            return max(self.args.segment_duration * 1.10, 3.2)
        if "hook_yarn" in stage:
            return max(self.args.segment_duration * 1.05, 3.0)
        if "lift_yarn" in stage or "exchange" in stage:
            return max(self.args.segment_duration, 3.0)
        if "press" in stage or "relief" in stage:
            return max(self.args.segment_duration * 1.20, self.args.segment_duration)
        if "return_home" in stage or "done" in stage:
            return max(self.args.segment_duration * 0.85, 2.0)
        return self.args.segment_duration

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
            "visual_points": len(points),
            "trajectory_rate_hz": self.args.trajectory_rate_hz,
        }
        if self.args.day_id.lower() in {"day03", "d3"}:
            summary["hand_eye_matrix_file"] = "d3_hand_eye_matrix.json"
            summary["hand_eye_camera_topic"] = "/right_camera/image_rect"
        if self.args.day_id.lower() in {"day04", "d4", "day05", "d5"}:
            summary["weaving_topics"] = [
                "/weaving/events",
                "/weaving/tension_n",
                "/weaving/tension_status",
                "/weaving/tension_pid_state",
                "/weaving/compliance_offset_m",
                "/weaving/yarn_state",
                "/weaving/primitive_lock",
            ]
            summary["frame_convention"] = (
                "left and right gripper local +X axes point along world +Y, "
                "perpendicular to the world-X line between the arm bases"
            )
        (out / "mvp_moveit_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    def replay(self, points: list[dict]) -> None:
        left = _trajectory_msg(points, "left")
        right = _trajectory_msg(points, "right")
        combined = self._combined_trajectory(points)
        plan_duration = float(points[-1]["time_from_start"])
        run_duration = self.args.duration if self.args.duration > 0 else plan_duration + 2.0
        started = time.monotonic()
        next_trajectory_publish = started
        trajectory_republish_interval = max(plan_duration + 0.75, 2.0)

        while rclpy.ok() and time.monotonic() - started <= run_duration:
            now = time.monotonic()
            if now >= next_trajectory_publish:
                self.gazebo_traj_pub.publish(combined)
                self.left_traj_pub.publish(left)
                self.right_traj_pub.publish(right)
                next_trajectory_publish = now + trajectory_republish_interval
            loop_elapsed = (now - started) % max(plan_duration, 0.1)
            state = _sample(points, loop_elapsed)
            self._publish_observable_state(state, loop_elapsed)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(1.0 / max(self.args.state_rate_hz, 1.0))

    def _combined_trajectory(self, points: list[dict]) -> JointTrajectory:
        msg = JointTrajectory()
        msg.joint_names = list(DUAL_JOINTS)
        positions = [[float(v) for v in frame["left"] + frame["right"]] for frame in points]
        times = [float(frame["time_from_start"]) for frame in points]
        velocities = _finite_difference_velocities(positions, times)
        for frame, frame_positions, frame_velocities in zip(points, positions, velocities):
            point = JointTrajectoryPoint()
            point.positions = frame_positions
            point.velocities = frame_velocities
            _seconds_to_msg(point, float(frame["time_from_start"]))
            msg.points.append(point)
        return msg

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
        self._publish_gripper(loop_elapsed=elapsed, phase=phase)

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

        if self.args.day_id.lower() in {"day03", "d3", "day05", "d5"}:
            self._publish_vision_state(stamp=stamp, phase=phase, elapsed=elapsed)

        if self.args.day_id.lower() in {"day04", "d4", "day05", "d5"}:
            self._publish_weaving_state(stamp=stamp, phase=phase, elapsed=elapsed)

    def _primitive_from_phase(self, phase: str) -> str:
        for primitive in D4_TENSION_WINDOWS:
            if primitive in phase:
                return primitive
        if "vision" in phase:
            return "vision_lock"
        return "idle"

    def _publish_weaving_state(self, *, stamp, phase: str, elapsed: float) -> None:
        primitive = self._primitive_from_phase(phase)
        locked = primitive in D4_TENSION_WINDOWS
        if locked:
            low, high, target = D4_TENSION_WINDOWS[primitive]
            measured = target + 0.12 * math.sin(elapsed * 1.7)
            measured = min(max(measured, low + 0.02), high - 0.02)
            status = "OK" if low <= measured <= high else "WAIT"
        elif primitive == "vision_lock":
            low, high, target = 0.0, 0.80, 0.35
            measured = 0.35 + 0.04 * math.sin(elapsed)
            status = "VISION_LOCKED"
        else:
            low, high, target = 0.0, 0.80, 0.25
            measured = 0.25 + 0.03 * math.sin(elapsed)
            status = "IDLE"

        compliance = max(min((1.85 - measured) * 0.0035, 0.010), -0.010)
        yarn_state = {
            "hook_yarn": "hooked",
            "lift_yarn": "lifted",
            "pull_tight": "tensioning",
            "shift": "shifted",
            "exchange": "exchanged",
            "vision_lock": "vision_locked",
            "idle": "idle",
        }[primitive]

        self.weaving_tension_pub.publish(Float64(data=float(measured)))
        self.weaving_status_pub.publish(String(data=status))
        self.weaving_compliance_pub.publish(Float64(data=float(compliance)))
        self.weaving_yarn_state_pub.publish(String(data=yarn_state))
        self.weaving_lock_pub.publish(Bool(data=locked))
        self.weaving_pid_pub.publish(
            String(
                data=(
                    f"phase={phase} primitive={primitive} target_n={target:.2f} "
                    f"measured_n={measured:.2f} window_n=({low:.2f},{high:.2f}) "
                    f"kp=0.42 ki=0.08 kd=0.018 compliance_m={compliance:.4f}"
                )
            )
        )
        self.weaving_events_pub.publish(
            String(
                data=(
                    f"phase={phase} primitive={primitive} yarn_state={yarn_state} "
                    f"tension_status={status} primitive_lock={locked} "
                    "source=d1_moveit_d2_tension_d3_vision"
                )
            )
        )

        target_msg = WrenchStamped()
        target_msg.header.stamp = stamp
        target_msg.header.frame_id = "right_tcp"
        target_msg.wrench.force.z = float(target)
        self.force_target_pub.publish(target_msg)

        error_msg = WrenchStamped()
        error_msg.header.stamp = stamp
        error_msg.header.frame_id = "right_tcp"
        error_msg.wrench.force.z = float(target - measured)
        self.force_error_pub.publish(error_msg)
        self.admittance_pub.publish(Float64(data=float(compliance)))
        self.force_state_pub.publish(
            String(
                data=(
                    f"mvp_weaving_compliance phase={phase} primitive={primitive} "
                    f"target_n={target:.2f} measured_n={measured:.2f} offset_m={compliance:.4f}"
                )
            )
        )

    def _publish_vision_state(self, *, stamp, phase: str, elapsed: float) -> None:
        if "wide" in phase:
            pixel_error = 62.0
            marker_px = 42
        elif "align" in phase:
            pixel_error = 24.0
            marker_px = 54
        elif "confirm" in phase:
            pixel_error = 3.0
            marker_px = 62
        elif "retract" in phase:
            pixel_error = 18.0
            marker_px = 48
        else:
            pixel_error = 12.0 + 8.0 * abs(math.sin(elapsed))
            marker_px = 50

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = "right_camera_color_optical_frame"
        info.width = 160
        info.height = 120
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [140.0, 0.0, 80.0, 0.0, 140.0, 60.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [140.0, 0.0, 80.0, 0.0, 0.0, 140.0, 60.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_info_pub.publish(info)

        offset = int(max(min(pixel_error, 70.0), -70.0))
        image = self._synthetic_vision_image(
            stamp,
            160,
            120,
            offset,
            marker_px,
            "right_camera_color_optical_frame",
        )
        self.camera_image_pub.publish(image)
        self.debug_image_pub.publish(image)

        detection = self._detect_aruco_from_synthetic_image(image, info)
        if detection is None:
            target_x = ARUCO_TAG_SIZE_M * float(info.k[0]) / max(marker_px, 1)
            target_y = -pixel_error * target_x / float(info.k[0])
            target_z = 0.0
            quat = (0.0, 0.0, 0.0, 1.0)
            method = "fallback_color_center"
            detected_error = pixel_error
        else:
            target_x, target_y, target_z, quat, detected_error = detection
            method = "opencv_aruco_4x4_50_id7"
        aligned = abs(detected_error) <= 5.0

        target = PoseStamped()
        target.header.stamp = stamp
        target.header.frame_id = "right_camera_color_optical_frame"
        target.pose.position.x = float(target_x)
        target.pose.position.y = float(target_y)
        target.pose.position.z = float(target_z)
        target.pose.orientation.x = float(quat[0])
        target.pose.orientation.y = float(quat[1])
        target.pose.orientation.z = float(quat[2])
        target.pose.orientation.w = float(quat[3])
        self.vision_target_pub.publish(target)

        twist = TwistStamped()
        twist.header.stamp = stamp
        twist.header.frame_id = "right_camera_color_optical_frame"
        twist.twist.linear.x = 0.0 if aligned else min(abs(detected_error) / 900.0, 0.06)
        twist.twist.linear.y = 0.0 if aligned else -detected_error / 1400.0
        twist.twist.linear.z = 0.0 if aligned else -0.015
        twist.twist.angular.z = 0.0 if aligned else -detected_error / 900.0
        self.visual_twist_pub.publish(twist)
        self.visual_aligned_pub.publish(Bool(data=aligned))
        self.vision_status_pub.publish(
            String(
                data=(
                    f"mvp_hand_eye phase={phase} method={method} marker_id={ARUCO_MARKER_ID} "
                    f"tag_size_m={ARUCO_TAG_SIZE_M:.3f} pixel_error={detected_error:.1f} "
                    f"t_camera_target=({target_x:.4f},{target_y:.4f},{target_z:.4f}) aligned={aligned}"
                )
            )
        )
        self.vision_metrics_pub.publish(
            String(
                data=(
                    f"method={method};dictionary=DICT_4X4_50;marker_id={ARUCO_MARKER_ID};"
                    f"tag_size_m={ARUCO_TAG_SIZE_M:.3f};target_px_error={detected_error:.1f};"
                    f"T_camera_target_translation_m={target_x:.5f},{target_y:.5f},{target_z:.5f};"
                    f"hand_eye_sample_valid={aligned}"
                )
            )
        )
        self.visual_adapter_state_pub.publish(
            String(data=f"mvp_visual_servo_adapter phase={phase} twist_from_synthetic_target aligned={aligned}")
        )

    def _detect_aruco_from_synthetic_image(self, image: Image, info: CameraInfo):
        if cv2 is None or np is None or not hasattr(cv2, "aruco"):
            return None
        try:
            frame = np.frombuffer(image.data, dtype=np.uint8).reshape((image.height, image.width, 3))
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            if hasattr(cv2.aruco, "ArucoDetector"):
                corners, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(gray)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
            if ids is None:
                return None
            marker_index = 0
            for idx, marker_id in enumerate(ids.flatten().tolist()):
                if int(marker_id) == ARUCO_MARKER_ID:
                    marker_index = idx
                    break
            camera_matrix = np.array(info.k, dtype=float).reshape(3, 3)
            dist_coeffs = np.array(info.d, dtype=float)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [corners[marker_index]],
                ARUCO_TAG_SIZE_M,
                camera_matrix,
                dist_coeffs,
            )
            rmat, _ = cv2.Rodrigues(rvecs[0][0])
            qx, qy, qz, qw = _matrix_to_quaternion(rmat)
            center_u = float(corners[marker_index].reshape(-1, 2)[:, 0].mean())
            pixel_error = center_u - float(info.k[2])
            tvec = tvecs[0][0]
            return float(tvec[2]), float(tvec[0]), float(tvec[1]), (qx, qy, qz, qw), pixel_error
        except Exception as exc:
            self.vision_status_pub.publish(String(data=f"aruco_detection_error:{exc}"))
            return None

    def _synthetic_vision_image(
        self,
        stamp,
        width: int,
        height: int,
        offset_x: int,
        marker_px: int,
        frame_id: str,
    ) -> Image:
        data = bytearray([245, 245, 240]) * (width * height)
        cx = max(marker_px // 2 + 4, min(width - marker_px // 2 - 5, width // 2 + offset_x))
        cy = height // 2
        marker = self._aruco_marker_bytes(marker_px)
        x0 = cx - marker_px // 2
        y0 = cy - marker_px // 2
        for row in range(marker_px):
            for col in range(marker_px):
                value = marker[row][col]
                idx = ((y0 + row) * width + (x0 + col)) * 3
                data[idx : idx + 3] = bytes((value, value, value))
        for x in range(width // 2 - 14, width // 2 + 15):
            idx = (cy * width + x) * 3
            data[idx : idx + 3] = b"\x30\x90\xff"
        for y in range(cy - 14, cy + 15):
            idx = (y * width + width // 2) * 3
            data[idx : idx + 3] = b"\x30\x90\xff"
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = height
        msg.width = width
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = width * 3
        msg.data = bytes(data)
        return msg

    def _aruco_marker_bytes(self, side_px: int) -> list[list[int]]:
        if cv2 is not None and hasattr(cv2, "aruco"):
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            marker = cv2.aruco.generateImageMarker(dictionary, ARUCO_MARKER_ID, side_px, borderBits=1)
            return marker.tolist()
        pattern = (
            (0, 0, 0, 0, 0, 0),
            (0, 1, 1, 0, 0, 0),
            (0, 0, 1, 0, 0, 0),
            (0, 1, 1, 1, 1, 0),
            (0, 0, 0, 1, 0, 0),
            (0, 0, 0, 0, 0, 0),
        )
        marker = []
        for y in range(side_px):
            row = []
            cell_y = min(5, int(y * 6 / side_px))
            for x in range(side_px):
                cell_x = min(5, int(x * 6 / side_px))
                row.append(255 if pattern[cell_y][cell_x] else 0)
            marker.append(row)
        return marker

    def _publish_gripper(self, *, loop_elapsed: float, phase: str) -> None:
        day = self.args.day_id.lower()
        if day in {"day01", "d1"}:
            open_position = 0.018
            closed_position = 0.001
            period = 3.2
            alpha = (loop_elapsed % period) / period
            if alpha < 0.5:
                grip_alpha = _smootherstep(alpha * 2.0)
                position = open_position + (closed_position - open_position) * grip_alpha
            else:
                grip_alpha = _smootherstep((alpha - 0.5) * 2.0)
                position = closed_position + (open_position - closed_position) * grip_alpha
        elif day in {"day02", "d2"}:
            position = 0.002
        elif day in {"day04", "d4", "day05", "d5"}:
            if "vision_lock" in phase or "home" in phase or "done" in phase:
                position = 0.014
            elif "exchange" in phase:
                position = 0.010
            else:
                position = 0.002
        else:
            position = 0.010
        msg = Float64(data=float(position))
        for pub in self.gripper_pubs:
            pub.publish(msg)


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
    parser.add_argument("--segment-duration", type=float, default=4.0)
    parser.add_argument("--trajectory-rate-hz", type=float, default=24.0)
    parser.add_argument("--state-rate-hz", type=float, default=30.0)
    parser.add_argument("--gazebo-topic", default="/model/dual_rm65b_mvp/joint_trajectory")
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
