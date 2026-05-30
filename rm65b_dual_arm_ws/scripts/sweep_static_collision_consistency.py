#!/usr/bin/env python3
"""Static MoveIt/SRDF collision and sampled plan consistency sweep.

This is an offline report. It does not run FCL or MoveIt; it audits the static
URDF/SRDF collision matrix and checks latest planned joint samples against URDF
limits and SRDF group definitions.
"""

from __future__ import annotations

import argparse
import csv
import json
import itertools
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from xml.etree import ElementTree as ET


try:
    import yaml  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    yaml = None


def repo_root_from_script() -> Path:
    return Path(__file__).resolve().parents[2]


def latest_output_root(outputs_dir: Path) -> Path:
    candidates = []
    for child in outputs_dir.iterdir() if outputs_dir.exists() else []:
        if not child.is_dir():
            continue
        if all((child / f"day0{i}").is_dir() for i in range(1, 6)):
            candidates.append(child)
    if not candidates:
        raise FileNotFoundError(f"no output root with day01-day05 under {outputs_dir}")
    return max(candidates, key=lambda p: p.stat().st_mtime)


def parse_urdf(path: Path) -> dict[str, Any]:
    root = ET.parse(path).getroot()
    links = {link.attrib["name"]: link for link in root.findall("link")}
    collision_links = {
        name for name, elem in links.items() if elem.findall("collision")
    }
    joints: dict[str, dict[str, Any]] = {}
    joint_limits: dict[str, dict[str, float]] = {}
    for joint in root.findall("joint"):
        name = joint.attrib["name"]
        parent = joint.find("parent")
        child = joint.find("child")
        joints[name] = {
            "type": joint.attrib.get("type"),
            "parent": parent.attrib.get("link") if parent is not None else None,
            "child": child.attrib.get("link") if child is not None else None,
        }
        limit = joint.find("limit")
        if limit is not None and "lower" in limit.attrib and "upper" in limit.attrib:
            joint_limits[name] = {
                "lower": float(limit.attrib["lower"]),
                "upper": float(limit.attrib["upper"]),
                "velocity": float(limit.attrib.get("velocity", "nan")),
                "effort": float(limit.attrib.get("effort", "nan")),
            }
    return {
        "robot_name": root.attrib.get("name"),
        "links": sorted(links.keys()),
        "collision_links": sorted(collision_links),
        "joints": joints,
        "joint_limits": joint_limits,
    }


def parse_srdf(path: Path) -> dict[str, Any]:
    root = ET.parse(path).getroot()
    groups: dict[str, dict[str, Any]] = {}
    for group in root.findall("group"):
        name = group.attrib["name"]
        groups[name] = {
            "chains": [
                {"base_link": chain.attrib.get("base_link"), "tip_link": chain.attrib.get("tip_link")}
                for chain in group.findall("chain")
            ],
            "subgroups": [sub.attrib.get("name") for sub in group.findall("group")],
        }
    group_states = []
    for state in root.findall("group_state"):
        group_states.append(
            {
                "name": state.attrib.get("name"),
                "group": state.attrib.get("group"),
                "joints": {
                    joint.attrib["name"]: float(joint.attrib.get("value", "nan"))
                    for joint in state.findall("joint")
                },
            }
        )
    disabled_pairs = []
    for item in root.findall("disable_collisions"):
        pair = tuple(sorted([item.attrib.get("link1", ""), item.attrib.get("link2", "")]))
        disabled_pairs.append(
            {
                "link1": pair[0],
                "link2": pair[1],
                "reason": item.attrib.get("reason"),
            }
        )
    return {
        "robot_name": root.attrib.get("name"),
        "groups": groups,
        "group_states": group_states,
        "disabled_pairs": disabled_pairs,
    }


def load_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    if yaml is not None:
        with path.open("r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
        return data if isinstance(data, dict) else {}
    # Minimal fallback for this project's dual_moveit_plans.yaml.
    data: dict[str, Any] = {"points": [], "segments": []}
    current_point: dict[str, Any] | None = None
    current_side: str | None = None
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw.rstrip()
        stripped = line.strip()
        if stripped.startswith("- time_from_start:"):
            if current_point:
                data["points"].append(current_point)
            current_point = {"time_from_start": float(stripped.split(":", 1)[1])}
            current_side = None
        elif current_point is not None and stripped.startswith("phase:"):
            current_point["phase"] = stripped.split(":", 1)[1].strip()
        elif current_point is not None and stripped in {"left:", "right:"}:
            current_side = stripped[:-1]
            current_point[current_side] = []
        elif current_point is not None and current_side and stripped.startswith("- "):
            try:
                current_point[current_side].append(float(stripped[2:]))
            except ValueError:
                pass
    if current_point:
        data["points"].append(current_point)
    return data


def side_of_link(link: str) -> str:
    if link.startswith("left_") or link.startswith("left"):
        return "left"
    if link.startswith("right_") or link.startswith("right"):
        return "right"
    return "shared"


def disabled_pair_audit(srdf: dict[str, Any], urdf: dict[str, Any]) -> dict[str, Any]:
    valid_links = set(urdf["links"])
    seen = set()
    duplicates = []
    invalid = []
    reasons: dict[str, int] = {}
    for pair in srdf["disabled_pairs"]:
        key = (pair["link1"], pair["link2"])
        if key in seen:
            duplicates.append(pair)
        seen.add(key)
        if pair["link1"] not in valid_links or pair["link2"] not in valid_links:
            invalid.append(pair)
        reasons[str(pair["reason"])] = reasons.get(str(pair["reason"]), 0) + 1
    collision_links = urdf["collision_links"]
    all_pairs = {tuple(sorted(pair)) for pair in itertools.combinations(collision_links, 2)}
    disabled = {tuple(sorted((pair["link1"], pair["link2"]))) for pair in srdf["disabled_pairs"]}
    active = sorted(all_pairs - disabled)
    cross_arm_active = [
        pair for pair in active
        if {side_of_link(pair[0]), side_of_link(pair[1])} == {"left", "right"}
    ]
    return {
        "disabled_pair_count": len(srdf["disabled_pairs"]),
        "disabled_pair_unique_count": len(seen),
        "duplicate_disabled_pairs": duplicates,
        "invalid_disabled_pairs": invalid,
        "disabled_reasons": reasons,
        "collision_link_count": len(collision_links),
        "active_collision_pair_count": len(active),
        "active_cross_arm_pair_count": len(cross_arm_active),
        "active_cross_arm_pairs_sample": cross_arm_active[:30],
    }


def state_and_plan_audit(
    srdf: dict[str, Any],
    urdf: dict[str, Any],
    output_root: Path,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    joint_limits = urdf["joint_limits"]
    rows: list[dict[str, Any]] = []
    issues: list[dict[str, Any]] = []

    for state in srdf["group_states"]:
        for joint, value in state["joints"].items():
            limit = joint_limits.get(joint)
            status = "ok"
            detail = ""
            if not limit:
                status = "missing_joint_limit"
                detail = "joint not found in URDF limits"
            elif value < limit["lower"] or value > limit["upper"]:
                status = "out_of_limit"
                detail = f"{value} not in [{limit['lower']}, {limit['upper']}]"
            row = {
                "source": "srdf_group_state",
                "day_id": "",
                "state_or_phase": state["name"],
                "sample_index": "",
                "joint": joint,
                "value": value,
                "lower": limit.get("lower") if limit else "",
                "upper": limit.get("upper") if limit else "",
                "status": status,
                "detail": detail,
            }
            rows.append(row)
            if status != "ok":
                issues.append(row)

    for day_dir in sorted(output_root.glob("day0[1-5]")):
        plan_path = day_dir / "logs" / "dual_moveit_plans.yaml"
        plan = load_yaml(plan_path)
        prev_time = None
        for idx, point in enumerate(plan.get("points", []) or []):
            time_value = point.get("time_from_start")
            if prev_time is not None and time_value is not None and time_value < prev_time:
                issues.append(
                    {
                        "source": "plan_time",
                        "day_id": day_dir.name,
                        "state_or_phase": point.get("phase"),
                        "sample_index": idx,
                        "status": "non_monotonic_time",
                        "detail": f"{time_value} < {prev_time}",
                    }
                )
            if time_value is not None:
                prev_time = time_value
            for side in ["left", "right"]:
                values = point.get(side) or []
                if len(values) != 6:
                    issues.append(
                        {
                            "source": "plan_sample",
                            "day_id": day_dir.name,
                            "state_or_phase": point.get("phase"),
                            "sample_index": idx,
                            "status": "bad_joint_count",
                            "detail": f"{side} has {len(values)} values",
                        }
                    )
                for joint_idx, value in enumerate(values, start=1):
                    joint = f"{side}_joint{joint_idx}"
                    limit = joint_limits.get(joint)
                    status = "ok"
                    detail = ""
                    if not limit:
                        status = "missing_joint_limit"
                        detail = "joint not found in URDF limits"
                    elif value < limit["lower"] or value > limit["upper"]:
                        status = "out_of_limit"
                        detail = f"{value} not in [{limit['lower']}, {limit['upper']}]"
                    row = {
                        "source": "planned_point",
                        "day_id": day_dir.name,
                        "state_or_phase": point.get("phase"),
                        "sample_index": idx,
                        "joint": joint,
                        "value": value,
                        "lower": limit.get("lower") if limit else "",
                        "upper": limit.get("upper") if limit else "",
                        "status": status,
                        "detail": detail,
                    }
                    rows.append(row)
                    if status != "ok":
                        issues.append(row)
    return rows, issues


def build_report(urdf_path: Path, srdf_path: Path, output_root: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    urdf = parse_urdf(urdf_path)
    srdf = parse_srdf(srdf_path)
    rows, issues = state_and_plan_audit(srdf, urdf, output_root)
    disabled_audit = disabled_pair_audit(srdf, urdf)
    report = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "mode": "static_srdf_urdf_and_sampled_plan_consistency",
        "limitation": "No FCL/MoveIt runtime collision checker is invoked; active pairs are static SRDF candidates and plan samples are limit/time consistency checks.",
        "urdf_path": str(urdf_path),
        "srdf_path": str(srdf_path),
        "output_root": str(output_root),
        "robot": {
            "urdf_name": urdf["robot_name"],
            "srdf_name": srdf["robot_name"],
            "link_count": len(urdf["links"]),
            "joint_limit_count": len(urdf["joint_limits"]),
            "groups": srdf["groups"],
            "group_state_count": len(srdf["group_states"]),
        },
        "collision_matrix": disabled_audit,
        "sample_status": {
            "checked_rows": len(rows),
            "issue_count": len(issues),
            "issues": issues[:100],
        },
    }
    return report, rows


def write_outputs(output_root: Path, report: dict[str, Any], rows: list[dict[str, Any]]) -> tuple[Path, Path]:
    out_dir = output_root / "offline_analysis"
    out_dir.mkdir(parents=True, exist_ok=True)
    json_path = out_dir / "static_collision_sweep_report.json"
    csv_path = out_dir / "static_collision_sweep_samples.csv"
    json_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    fieldnames = [
        "source",
        "day_id",
        "state_or_phase",
        "sample_index",
        "joint",
        "value",
        "lower",
        "upper",
        "status",
        "detail",
    ]
    with csv_path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return json_path, csv_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    root = repo_root_from_script()
    default_config = root / "rm65b_dual_arm_ws" / "src" / "rm65b_dual_arm_moveit_config" / "config"
    parser.add_argument("--urdf", type=Path, default=default_config / "rm65b_dual_arm.urdf")
    parser.add_argument("--srdf", type=Path, default=default_config / "rm65b_dual_arm.srdf")
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--outputs-dir", type=Path, default=root / "outputs")
    args = parser.parse_args()

    output_root = (args.output_root or latest_output_root(args.outputs_dir)).resolve()
    report, rows = build_report(args.urdf.resolve(), args.srdf.resolve(), output_root)
    json_path, csv_path = write_outputs(output_root, report, rows)
    print(f"output_root={output_root}")
    print(f"json={json_path}")
    print(f"csv={csv_path}")
    print(f"disabled_pairs={report['collision_matrix']['disabled_pair_count']}")
    print(f"active_cross_arm_pairs={report['collision_matrix']['active_cross_arm_pair_count']}")
    print(f"sample_issues={report['sample_status']['issue_count']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
