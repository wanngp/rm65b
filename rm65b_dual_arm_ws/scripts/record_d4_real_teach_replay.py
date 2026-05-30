#!/usr/bin/env python3
"""Record D4 teach/replay primitives from live ROS joint states.

This is an on-site recorder, not an offline MoveIt-plan converter.  It subscribes
to a real or namespaced `/joint_states` stream while an operator moves the arms
with the teach pendant, then writes a primitive YAML file that can be replayed by
`rm65b_weaving_primitives/weaving_coordinator`.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

try:
    import yaml  # type: ignore
except Exception:  # pragma: no cover - optional fallback for field laptops
    yaml = None


DEFAULT_PRIMITIVES = ["hook_yarn", "lift_yarn", "pull_tight", "shift", "exchange"]
DEFAULT_LEFT_JOINTS = [f"left_joint{i}" for i in range(1, 7)]
DEFAULT_RIGHT_JOINTS = [f"right_joint{i}" for i in range(1, 7)]
DEFAULT_CONTROLLERS = {
    "left": "/left_arm_controller/follow_joint_trajectory",
    "right": "/right_arm_controller/follow_joint_trajectory",
}


def yaml_scalar(value: Any) -> str:
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return f"{value:.9g}"
    text = str(value)
    if not text or any(ch in text for ch in ":#[]{}&,*!|>'\"%@`"):
        return '"' + text.replace("\\", "\\\\").replace('"', '\\"') + '"'
    return text


def fallback_dump_yaml(value: Any, indent: int = 0) -> str:
    pad = " " * indent
    if isinstance(value, dict):
        lines: list[str] = []
        for key, item in value.items():
            if isinstance(item, (dict, list)):
                lines.append(f"{pad}{key}:")
                lines.append(fallback_dump_yaml(item, indent + 2))
            else:
                lines.append(f"{pad}{key}: {yaml_scalar(item)}")
        return "\n".join(lines)
    if isinstance(value, list):
        lines = []
        for item in value:
            if isinstance(item, dict):
                lines.append(f"{pad}-")
                lines.append(fallback_dump_yaml(item, indent + 2))
            elif isinstance(item, list):
                lines.append(f"{pad}-")
                lines.append(fallback_dump_yaml(item, indent + 2))
            else:
                lines.append(f"{pad}- {yaml_scalar(item)}")
        return "\n".join(lines)
    return f"{pad}{yaml_scalar(value)}"


def write_yaml(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if yaml is not None:
        text = yaml.safe_dump(value, sort_keys=False, allow_unicode=False)
    else:
        text = fallback_dump_yaml(value) + "\n"
    path.write_text(text, encoding="utf-8")


def parse_csv(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def now_utc() -> str:
    return datetime.now(timezone.utc).isoformat()


class D4TeachReplayRecorder:
    def __init__(self, args: argparse.Namespace) -> None:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_srvs.srv import Trigger

        class RecorderNode(Node):
            pass

        self.rclpy = rclpy
        self.Trigger = Trigger
        self.JointState = JointState
        self.args = args
        self.node = RecorderNode("d4_teach_replay_recorder")
        self.node.declare_parameter("primitive_name", args.primitives[0])
        self.node.declare_parameter("output_file", str(args.output_file))
        self.node.create_subscription(JointState, args.joint_topic, self._joint_callback, 20)
        self.node.create_service(
            Trigger, "/d4_teach_replay_recorder/start_recording", self._srv_start
        )
        self.node.create_service(
            Trigger, "/d4_teach_replay_recorder/stop_recording", self._srv_stop
        )

        self.lock = threading.Lock()
        self.active_primitive: str | None = None
        self.start_time_ns: int | None = None
        self.last_sample_time = -1.0
        self.latest_msg_seen = False
        self.points: dict[str, list[dict[str, Any]]] = {"left": [], "right": []}
        self.recorded: dict[str, dict[str, Any]] = {}
        self.sessions: list[dict[str, Any]] = []

    def _srv_start(self, _request: Any, response: Any) -> Any:
        primitive = str(self.node.get_parameter("primitive_name").value)
        self.start_primitive(primitive)
        response.success = True
        response.message = f"recording {primitive}"
        return response

    def _srv_stop(self, _request: Any, response: Any) -> Any:
        primitive = self.active_primitive or str(self.node.get_parameter("primitive_name").value)
        counts = self.stop_primitive()
        self.write_output(Path(str(self.node.get_parameter("output_file").value)))
        response.success = True
        response.message = f"stopped {primitive}, wrote {counts}"
        return response

    def _joint_callback(self, msg: Any) -> None:
        with self.lock:
            self.latest_msg_seen = True
            if self.active_primitive is None or self.start_time_ns is None:
                return
            stamp_ns = time.monotonic_ns()
            elapsed = (stamp_ns - self.start_time_ns) / 1e9
            if elapsed - self.last_sample_time < 1.0 / float(self.args.sample_hz):
                return
            by_name = {name: pos for name, pos in zip(msg.name, msg.position)}
            side_positions = {
                "left": self._positions_for_side(by_name, self.args.left_joints),
                "right": self._positions_for_side(by_name, self.args.right_joints),
            }
            if self.args.require_both and (
                side_positions["left"] is None or side_positions["right"] is None
            ):
                return
            captured = False
            for side, positions in side_positions.items():
                if positions is None:
                    continue
                self.points[side].append(
                    {
                        "time_from_start": round(elapsed, 4),
                        "positions": [round(float(value), 6) for value in positions],
                    }
                )
                captured = True
            if captured:
                self.last_sample_time = elapsed

    @staticmethod
    def _positions_for_side(by_name: dict[str, float], joint_names: list[str]) -> list[float] | None:
        values = []
        for name in joint_names:
            if name not in by_name:
                return None
            values.append(float(by_name[name]))
        return values

    def wait_for_joint_states(self, timeout_sec: float) -> bool:
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            with self.lock:
                if self.latest_msg_seen:
                    return True
            time.sleep(0.05)
        return False

    def start_primitive(self, primitive: str) -> None:
        with self.lock:
            self.active_primitive = primitive
            self.start_time_ns = time.monotonic_ns()
            self.last_sample_time = -1.0
            self.points = {"left": [], "right": []}
        self.node.get_logger().info(f"D4 recording started: {primitive}")

    def stop_primitive(self) -> dict[str, int]:
        with self.lock:
            primitive = self.active_primitive
            points = {side: list(items) for side, items in self.points.items()}
            self.active_primitive = None
            self.start_time_ns = None
        if primitive is None:
            return {"left": 0, "right": 0}

        primitive_data: dict[str, Any] = {
            "description": f"Real teach-pendant recording for {primitive}.",
            "source": "real_joint_states_teach_pendant",
        }
        counts: dict[str, int] = {}
        for side, items in points.items():
            if not items:
                counts[side] = 0
                continue
            primitive_data[side] = self._zero_first_time(items)
            counts[side] = len(items)
        self.recorded[primitive] = primitive_data
        self.sessions.append(
            {
                "primitive": primitive,
                "stopped_at": now_utc(),
                "sample_count": counts,
                "sample_hz_limit": float(self.args.sample_hz),
            }
        )
        self.node.get_logger().info(f"D4 recording stopped: {primitive} {counts}")
        return counts

    @staticmethod
    def _zero_first_time(items: list[dict[str, Any]]) -> list[dict[str, Any]]:
        if not items:
            return []
        first = float(items[0]["time_from_start"])
        normalized = []
        for item in items:
            normalized.append(
                {
                    "time_from_start": round(max(0.0, float(item["time_from_start"]) - first), 4),
                    "positions": item["positions"],
                }
            )
        return normalized

    def build_output(self) -> dict[str, Any]:
        return {
            "metadata": {
                "robot_model": "RM65-B dual arm",
                "evidence_type": "real_teach_replay_primitive_yaml",
                "source": "live_ros_joint_states",
                "joint_topic": self.args.joint_topic,
                "generated_at": now_utc(),
                "produced_by": "rm65b_dual_arm_ws/scripts/record_d4_real_teach_replay.py",
                "not_offline_moveit_yaml": True,
                "replay_node": "rm65b_weaving_primitives/weaving_coordinator",
                "units": {"position": "rad", "duration": "s"},
            },
            "controllers": dict(DEFAULT_CONTROLLERS),
            "joint_names": {"left": self.args.left_joints, "right": self.args.right_joints},
            "primitive_order": list(self.recorded.keys()),
            "primitives": self.recorded,
            "recording_sessions": self.sessions,
        }

    def write_output(self, path: Path) -> None:
        write_yaml(path, self.build_output())
        self.node.get_logger().info(f"wrote D4 primitive YAML: {path}")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-file",
        type=Path,
        required=True,
        help="Primitive YAML to write, for example outputs/on_site/day04/logs/day04_real_primitives.yaml",
    )
    parser.add_argument(
        "--primitive",
        dest="primitives",
        action="append",
        help="Primitive name to record. Repeat for multiple primitives.",
    )
    parser.add_argument("--joint-topic", default="/joint_states")
    parser.add_argument("--left-joints", type=parse_csv, default=DEFAULT_LEFT_JOINTS)
    parser.add_argument("--right-joints", type=parse_csv, default=DEFAULT_RIGHT_JOINTS)
    parser.add_argument("--sample-hz", type=float, default=20.0)
    parser.add_argument("--duration", type=float, help="Timed mode duration per primitive in seconds.")
    parser.add_argument("--pre-roll", type=float, default=0.5)
    parser.add_argument("--wait-joint-state", type=float, default=10.0)
    parser.add_argument(
        "--allow-partial",
        action="store_true",
        help="Record whichever side is present instead of requiring both arms in each sample.",
    )
    parser.add_argument(
        "--service-mode",
        action="store_true",
        help="Expose start/stop Trigger services and wait until Ctrl-C.",
    )
    return parser


def spin_in_background(recorder: D4TeachReplayRecorder) -> threading.Thread:
    thread = threading.Thread(target=recorder.rclpy.spin, args=(recorder.node,), daemon=True)
    thread.start()
    return thread


def interactive_record(recorder: D4TeachReplayRecorder, primitives: list[str]) -> None:
    for primitive in primitives:
        input(f"Place the robot at the start of {primitive}, then press Enter to record.")
        recorder.start_primitive(primitive)
        input(f"Teach {primitive} now. Press Enter to stop recording.")
        counts = recorder.stop_primitive()
        print(f"{primitive}: {counts}")


def timed_record(recorder: D4TeachReplayRecorder, primitives: list[str], duration: float) -> None:
    for primitive in primitives:
        print(f"recording {primitive} for {duration:.1f}s")
        time.sleep(float(recorder.args.pre_roll))
        recorder.start_primitive(primitive)
        time.sleep(duration)
        counts = recorder.stop_primitive()
        print(f"{primitive}: {counts}")


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    args.primitives = args.primitives or list(DEFAULT_PRIMITIVES)
    args.require_both = not args.allow_partial

    import rclpy

    rclpy.init()
    recorder = D4TeachReplayRecorder(args)
    spin_thread = spin_in_background(recorder)
    try:
        if not recorder.wait_for_joint_states(float(args.wait_joint_state)):
            print(
                f"ERROR: no JointState received on {args.joint_topic} within "
                f"{args.wait_joint_state}s",
                file=sys.stderr,
            )
            return 2
        if args.service_mode:
            print("service mode ready:")
            print("  ros2 param set /d4_teach_replay_recorder primitive_name hook_yarn")
            print("  ros2 service call /d4_teach_replay_recorder/start_recording std_srvs/srv/Trigger {}")
            print("  ros2 service call /d4_teach_replay_recorder/stop_recording std_srvs/srv/Trigger {}")
            while rclpy.ok():
                time.sleep(0.25)
        elif args.duration:
            timed_record(recorder, args.primitives, float(args.duration))
            recorder.write_output(args.output_file)
        else:
            interactive_record(recorder, args.primitives)
            recorder.write_output(args.output_file)
    except KeyboardInterrupt:
        if recorder.active_primitive:
            recorder.stop_primitive()
        recorder.write_output(args.output_file)
    finally:
        recorder.node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
    print(f"yaml={args.output_file.resolve()}")
    print(
        "replay=ros2 run rm65b_weaving_primitives weaving_coordinator --ros-args "
        f"-p trajectory_file:={args.output_file.resolve()} -p dry_run:=false "
        "-p playback_stage:=D4_real_teach_replay"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
