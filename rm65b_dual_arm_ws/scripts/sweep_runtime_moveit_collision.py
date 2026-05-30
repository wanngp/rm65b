#!/usr/bin/env python3
"""Runtime MoveIt PlanningScene/FCL collision sweep via /check_state_validity.

The URDF/SRDF files are used only to discover joints, limits, and seed states.
Each generated robot state is sent to MoveIt's GetStateValidity service, so the
validity/collision result comes from the running PlanningScene collision checker.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import sys
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional
from xml.etree import ElementTree as ET


try:
    import yaml  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    yaml = None


LEFT_JOINTS = ["left_joint%d" % idx for idx in range(1, 7)]
RIGHT_JOINTS = ["right_joint%d" % idx for idx in range(1, 7)]
DUAL_JOINTS = LEFT_JOINTS + RIGHT_JOINTS


def repo_root_from_script() -> Path:
    return Path(__file__).resolve().parents[2]


def parse_urdf(path: Path) -> dict[str, Any]:
    root = ET.parse(path).getroot()
    limits: dict[str, dict[str, float]] = {}
    movable_joints = []
    for joint in root.findall("joint"):
        joint_type = joint.attrib.get("type", "")
        name = joint.attrib.get("name", "")
        limit = joint.find("limit")
        if joint_type in {"fixed", ""} or limit is None:
            continue
        if "lower" not in limit.attrib or "upper" not in limit.attrib:
            continue
        lower = float(limit.attrib["lower"])
        upper = float(limit.attrib["upper"])
        if not math.isfinite(lower) or not math.isfinite(upper):
            continue
        limits[name] = {
            "lower": lower,
            "upper": upper,
            "mid": (lower + upper) / 2.0,
        }
        movable_joints.append(name)
    return {
        "robot_name": root.attrib.get("name", ""),
        "joint_limits": limits,
        "movable_joints": movable_joints,
    }


def parse_srdf(path: Path) -> dict[str, Any]:
    root = ET.parse(path).getroot()
    groups: dict[str, Any] = {}
    for group in root.findall("group"):
        groups[group.attrib["name"]] = {
            "chains": [
                {
                    "base_link": chain.attrib.get("base_link", ""),
                    "tip_link": chain.attrib.get("tip_link", ""),
                }
                for chain in group.findall("chain")
            ],
            "subgroups": [sub.attrib.get("name", "") for sub in group.findall("group")],
        }
    group_states = []
    for state in root.findall("group_state"):
        group_states.append(
            {
                "name": state.attrib.get("name", ""),
                "group": state.attrib.get("group", ""),
                "joints": {
                    joint.attrib["name"]: float(joint.attrib.get("value", "nan"))
                    for joint in state.findall("joint")
                },
            }
        )
    return {
        "robot_name": root.attrib.get("name", ""),
        "groups": groups,
        "group_states": group_states,
    }


def load_yaml(path: Path) -> dict[str, Any]:
    if yaml is not None:
        with path.open("r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
        return data if isinstance(data, dict) else {}

    data: dict[str, Any] = {"points": []}
    current: Optional[dict[str, Any]] = None
    side: Optional[str] = None
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        stripped = raw.strip()
        if stripped.startswith("- time_from_start:"):
            if current:
                data["points"].append(current)
            current = {"time_from_start": stripped.split(":", 1)[1].strip()}
            side = None
        elif current is not None and stripped.startswith("phase:"):
            current["phase"] = stripped.split(":", 1)[1].strip()
        elif current is not None and stripped in {"left:", "right:"}:
            side = stripped[:-1]
            current[side] = []
        elif current is not None and side and stripped.startswith("- "):
            try:
                current[side].append(float(stripped[2:]))
            except ValueError:
                pass
    if current:
        data["points"].append(current)
    return data


def linspace(lower: float, upper: float, count: int) -> list[float]:
    if count <= 1:
        return [(lower + upper) / 2.0]
    return [lower + (upper - lower) * idx / float(count - 1) for idx in range(count)]


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def ordered_joint_names(limits: dict[str, Any]) -> list[str]:
    preferred = [name for name in DUAL_JOINTS if name in limits]
    rest = sorted(name for name in limits if name not in preferred)
    return preferred + rest


def seed_state(srdf: dict[str, Any], limits: dict[str, Any], group_name: str) -> dict[str, float]:
    seed = {name: info["mid"] for name, info in limits.items()}
    preferred_names = ["dual_ready", "ready", "home"]
    states = srdf.get("group_states", [])
    for preferred in preferred_names:
        for state in states:
            if state.get("group") != group_name and group_name != "dual_arms":
                continue
            if state.get("name") != preferred:
                continue
            for joint, value in state.get("joints", {}).items():
                if joint in limits and math.isfinite(value):
                    seed[joint] = clamp(value, limits[joint]["lower"], limits[joint]["upper"])
            return seed
    for state in states:
        for joint, value in state.get("joints", {}).items():
            if joint in limits and math.isfinite(value):
                seed[joint] = clamp(value, limits[joint]["lower"], limits[joint]["upper"])
    return seed


def make_sample(
    sample_id: str,
    scenario: str,
    positions: dict[str, float],
    varied_joints: list[str],
    detail: str = "",
) -> dict[str, Any]:
    return {
        "sample_id": sample_id,
        "scenario": scenario,
        "positions": dict(positions),
        "varied_joints": list(varied_joints),
        "detail": detail,
    }


def generate_sweep_samples(
    joint_names: list[str],
    limits: dict[str, Any],
    seed: dict[str, float],
    samples_per_joint: int,
    samples_per_pair: int,
    random_samples: int,
    random_seed: int,
) -> list[dict[str, Any]]:
    samples: list[dict[str, Any]] = []
    samples.append(make_sample("seed_0000", "seed_state", seed, [], "SRDF seed or URDF midpoint"))

    for joint in joint_names:
        for idx, value in enumerate(linspace(limits[joint]["lower"], limits[joint]["upper"], samples_per_joint)):
            positions = dict(seed)
            positions[joint] = value
            samples.append(
                make_sample(
                    "joint_%s_%04d" % (joint, idx),
                    "single_joint_full_range",
                    positions,
                    [joint],
                    "%s=%0.6f" % (joint, value),
                )
            )

    for idx in range(1, 7):
        left = "left_joint%d" % idx
        right = "right_joint%d" % idx
        if left not in limits or right not in limits:
            continue
        left_values = linspace(limits[left]["lower"], limits[left]["upper"], samples_per_pair)
        right_values = linspace(limits[right]["lower"], limits[right]["upper"], samples_per_pair)
        for li, left_value in enumerate(left_values):
            for ri, right_value in enumerate(right_values):
                positions = dict(seed)
                positions[left] = left_value
                positions[right] = right_value
                samples.append(
                    make_sample(
                        "cross_pair_j%d_%02d_%02d" % (idx, li, ri),
                        "left_right_same_index_pair_grid",
                        positions,
                        [left, right],
                        "%s=%0.6f %s=%0.6f" % (left, left_value, right, right_value),
                    )
                )

    rng = random.Random(random_seed)
    for idx in range(random_samples):
        positions = {
            joint: rng.uniform(limits[joint]["lower"], limits[joint]["upper"])
            for joint in joint_names
        }
        samples.append(
            make_sample(
                "random_global_%04d" % idx,
                "random_global_joint_space",
                positions,
                list(joint_names),
                "uniform random within URDF limits",
            )
        )
    return samples


def append_plan_samples(
    samples: list[dict[str, Any]],
    plan_yaml: Path,
    seed: dict[str, float],
    limits: dict[str, Any],
    stride: int,
) -> None:
    plan = load_yaml(plan_yaml)
    points = plan.get("points", []) or []
    stride = max(1, stride)
    for idx, point in enumerate(points):
        if idx % stride != 0:
            continue
        positions = dict(seed)
        for side, names in [("left", LEFT_JOINTS), ("right", RIGHT_JOINTS)]:
            values = point.get(side) or []
            for joint, value in zip(names, values):
                if joint in limits:
                    positions[joint] = clamp(float(value), limits[joint]["lower"], limits[joint]["upper"])
        samples.append(
            make_sample(
                "plan_%04d" % idx,
                "planned_trajectory_sample",
                positions,
                [joint for joint in DUAL_JOINTS if joint in positions],
                "phase=%s source=%s" % (point.get("phase", ""), plan_yaml),
            )
        )


def contact_to_dict(contact: Any) -> dict[str, Any]:
    body1 = getattr(contact, "contact_body_1", getattr(contact, "body_name_1", ""))
    body2 = getattr(contact, "contact_body_2", getattr(contact, "body_name_2", ""))
    position = getattr(contact, "position", None)
    normal = getattr(contact, "normal", None)
    return {
        "body1": body1,
        "body2": body2,
        "pair": "::".join(sorted([str(body1), str(body2)])),
        "depth": float(getattr(contact, "depth", 0.0)),
        "position": {
            "x": float(getattr(position, "x", 0.0)),
            "y": float(getattr(position, "y", 0.0)),
            "z": float(getattr(position, "z", 0.0)),
        },
        "normal": {
            "x": float(getattr(normal, "x", 0.0)),
            "y": float(getattr(normal, "y", 0.0)),
            "z": float(getattr(normal, "z", 0.0)),
        },
    }


def create_state_validity_client(args: argparse.Namespace, joint_names: list[str]):
    try:
        import rclpy
        from moveit_msgs.srv import GetStateValidity
        from rclpy.node import Node
    except Exception as exc:
        raise RuntimeError(
            "ROS 2/MoveIt Python modules are not importable. Source /opt/ros/<distro>/setup.bash "
            "and the workspace install/setup.bash before running this script."
        ) from exc

    class StateValidityClient(Node):
        def __init__(self) -> None:
            super().__init__("rm65b_runtime_collision_sweep")
            self.client = self.create_client(GetStateValidity, args.service_name)

        def wait(self) -> None:
            if not self.client.wait_for_service(timeout_sec=args.wait_timeout):
                raise RuntimeError(
                    "%s is not available after %.1fs; start move_group first"
                    % (args.service_name, args.wait_timeout)
                )

        def check(self, sample: dict[str, Any]) -> dict[str, Any]:
            request = GetStateValidity.Request()
            request.group_name = args.group_name
            request.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
            request.robot_state.joint_state.name = list(joint_names)
            request.robot_state.joint_state.position = [
                float(sample["positions"][joint]) for joint in joint_names
            ]
            request.robot_state.is_diff = False
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=args.call_timeout)
            if not future.done():
                return {
                    "valid": False,
                    "service_error": "timeout after %.1fs" % args.call_timeout,
                    "contacts": [],
                }
            response = future.result()
            if response is None:
                return {
                    "valid": False,
                    "service_error": "service returned no response",
                    "contacts": [],
                }
            contacts = [contact_to_dict(contact) for contact in getattr(response, "contacts", [])]
            return {
                "valid": bool(response.valid),
                "service_error": "",
                "contacts": contacts,
            }

    return rclpy, StateValidityClient


def run_sweep(args: argparse.Namespace) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    urdf = parse_urdf(args.urdf)
    srdf = parse_srdf(args.srdf)
    limits = urdf["joint_limits"]
    joint_names = args.joints or ordered_joint_names(limits)
    missing = [joint for joint in joint_names if joint not in limits]
    if missing:
        raise ValueError("requested joints missing URDF limits: %s" % ", ".join(missing))

    seed = seed_state(srdf, limits, args.group_name)
    samples = generate_sweep_samples(
        joint_names,
        limits,
        seed,
        args.samples_per_joint,
        args.samples_per_pair,
        args.random_samples,
        args.random_seed,
    )
    if args.plan_yaml:
        append_plan_samples(samples, args.plan_yaml, seed, limits, args.plan_stride)
    if args.max_samples and args.max_samples > 0:
        samples = samples[: args.max_samples]

    rclpy, client_class = create_state_validity_client(args, joint_names)
    started = datetime.now(timezone.utc)
    rows: list[dict[str, Any]] = []
    pair_counter: Counter[str] = Counter()
    scenario_counter: Counter[str] = Counter()
    invalid_counter: Counter[str] = Counter()

    rclpy.init()
    client = client_class()
    try:
        client.wait()
        for idx, sample in enumerate(samples):
            result = client.check(sample)
            contacts = result["contacts"]
            contact_pairs = sorted({contact["pair"] for contact in contacts if contact.get("pair")})
            pair_counter.update(contact_pairs)
            scenario_counter[sample["scenario"]] += 1
            if not result["valid"]:
                invalid_counter[sample["scenario"]] += 1
            row = {
                "index": idx,
                "sample_id": sample["sample_id"],
                "scenario": sample["scenario"],
                "group_name": args.group_name,
                "valid": result["valid"],
                "collision": bool(contacts),
                "contact_count": len(contacts),
                "contact_pairs": ";".join(contact_pairs),
                "service_error": result["service_error"],
                "varied_joints": ";".join(sample["varied_joints"]),
                "detail": sample["detail"],
                "positions_json": json.dumps(
                    {joint: round(float(sample["positions"][joint]), 8) for joint in joint_names},
                    sort_keys=True,
                ),
                "contacts_json": json.dumps(contacts, sort_keys=True),
            }
            rows.append(row)
            if args.progress_every and (idx + 1) % args.progress_every == 0:
                print(
                    "checked=%d/%d invalid=%d collisions=%d"
                    % (
                        idx + 1,
                        len(samples),
                        sum(1 for item in rows if not item["valid"]),
                        sum(1 for item in rows if item["collision"]),
                    ),
                    flush=True,
                )
    finally:
        client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    ended = datetime.now(timezone.utc)
    invalid_rows = [row for row in rows if not row["valid"]]
    collision_rows = [row for row in rows if row["collision"]]
    report = {
        "generated_at": ended.isoformat(),
        "mode": "runtime_moveit_check_state_validity",
        "runtime_checker": "MoveIt PlanningScene/FCL via %s" % args.service_name,
        "urdf_path": str(args.urdf),
        "srdf_path": str(args.srdf),
        "plan_yaml": str(args.plan_yaml) if args.plan_yaml else "",
        "group_name": args.group_name,
        "joint_names": joint_names,
        "seed_state": {joint: seed[joint] for joint in joint_names},
        "joint_limits": {joint: limits[joint] for joint in joint_names},
        "started_at": started.isoformat(),
        "duration_s": (ended - started).total_seconds(),
        "summary": {
            "sample_count": len(rows),
            "valid_count": sum(1 for row in rows if row["valid"]),
            "invalid_count": len(invalid_rows),
            "collision_count": len(collision_rows),
            "service_error_count": sum(1 for row in rows if row["service_error"]),
            "scenario_counts": dict(sorted(scenario_counter.items())),
            "scenario_invalid_counts": dict(sorted(invalid_counter.items())),
            "contact_pair_counts": dict(sorted(pair_counter.items())),
            "first_invalid_samples": invalid_rows[:20],
            "first_collision_samples": collision_rows[:20],
        },
        "limitations": [
            "This script validates sampled states, not a mathematically exhaustive 12-DOF continuum.",
            "Contact pairs are reported only when MoveIt's GetStateValidity response includes contacts.",
            "URDF/SRDF parsing is used for sampling metadata; collision truth comes from the running MoveIt service.",
            "The running move_group parameters, Allowed Collision Matrix, attached objects, and planning scene diffs affect results.",
        ],
    }
    return report, rows


def write_outputs(output_dir: Path, report: dict[str, Any], rows: list[dict[str, Any]]) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / "runtime_collision_sweep_report.json"
    csv_path = output_dir / "runtime_collision_sweep_samples.csv"
    json_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    fieldnames = [
        "index",
        "sample_id",
        "scenario",
        "group_name",
        "valid",
        "collision",
        "contact_count",
        "contact_pairs",
        "service_error",
        "varied_joints",
        "detail",
        "positions_json",
        "contacts_json",
    ]
    with csv_path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return json_path, csv_path


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    root = repo_root_from_script()
    config_dir = root / "rm65b_dual_arm_ws" / "src" / "rm65b_dual_arm_moveit_config" / "config"
    default_out = root / "outputs" / "runtime_collision_sweep"

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=config_dir / "rm65b_dual_arm.urdf")
    parser.add_argument("--srdf", type=Path, default=config_dir / "rm65b_dual_arm.srdf")
    parser.add_argument("--output-dir", type=Path, default=default_out)
    parser.add_argument("--service-name", default="/check_state_validity")
    parser.add_argument("--group-name", default="dual_arms")
    parser.add_argument("--wait-timeout", type=float, default=30.0)
    parser.add_argument("--call-timeout", type=float, default=5.0)
    parser.add_argument("--samples-per-joint", type=int, default=9)
    parser.add_argument("--samples-per-pair", type=int, default=5)
    parser.add_argument("--random-samples", type=int, default=0)
    parser.add_argument("--random-seed", type=int, default=65)
    parser.add_argument("--max-samples", type=int, default=0)
    parser.add_argument("--progress-every", type=int, default=25)
    parser.add_argument("--plan-yaml", type=Path)
    parser.add_argument("--plan-stride", type=int, default=5)
    parser.add_argument("--joints", nargs="+")
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    report, rows = run_sweep(args)
    json_path, csv_path = write_outputs(args.output_dir, report, rows)
    summary = report["summary"]
    print("output_dir=%s" % args.output_dir.resolve())
    print("json=%s" % json_path.resolve())
    print("csv=%s" % csv_path.resolve())
    print("samples=%d" % summary["sample_count"])
    print("invalid=%d" % summary["invalid_count"])
    print("collisions=%d" % summary["collision_count"])
    print("service_errors=%d" % summary["service_error_count"])
    return 1 if summary["service_error_count"] else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
