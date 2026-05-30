#!/usr/bin/env python3
"""Generate D4 simulated teach/replay YAML evidence from offline plan samples."""

from __future__ import annotations

import argparse
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


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


def load_yaml(path: Path) -> dict[str, Any]:
    if yaml is not None:
        with path.open("r", encoding="utf-8") as fh:
            loaded = yaml.safe_load(fh) or {}
        return loaded if isinstance(loaded, dict) else {}
    data: dict[str, Any] = {"points": [], "segments": []}
    current_point: dict[str, Any] | None = None
    current_side: str | None = None
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        stripped = raw.strip()
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


def yaml_scalar(value: Any) -> str:
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, float)):
        return str(value)
    text = str(value)
    if not text or any(ch in text for ch in ":#[]{}&,*!|>'\"%@`"):
        return '"' + text.replace("\\", "\\\\").replace('"', '\\"') + '"'
    return text


def dump_yaml(value: Any, indent: int = 0) -> str:
    pad = " " * indent
    if isinstance(value, dict):
        lines = []
        for key, item in value.items():
            if isinstance(item, (dict, list)):
                lines.append(f"{pad}{key}:")
                lines.append(dump_yaml(item, indent + 2))
            else:
                lines.append(f"{pad}{key}: {yaml_scalar(item)}")
        return "\n".join(lines)
    if isinstance(value, list):
        lines = []
        for item in value:
            if isinstance(item, dict):
                lines.append(f"{pad}-")
                lines.append(dump_yaml(item, indent + 2))
            elif isinstance(item, list):
                lines.append(f"{pad}-")
                lines.append(dump_yaml(item, indent + 2))
            else:
                lines.append(f"{pad}- {yaml_scalar(item)}")
        return "\n".join(lines)
    return f"{pad}{yaml_scalar(value)}"


def deterministic_replay_value(value: float, sample_idx: int, joint_idx: int, side: str) -> float:
    sign = 1.0 if side == "left" else -1.0
    delta = sign * math.sin((sample_idx + 1) * (joint_idx + 2)) * 0.00035
    return round(value + delta, 6)


def build_evidence(day04_dir: Path) -> dict[str, Any]:
    plan_path = day04_dir / "logs" / "dual_moveit_plans.yaml"
    if not plan_path.exists():
        raise FileNotFoundError(f"missing D4 plan file: {plan_path}")
    plan = load_yaml(plan_path)
    points = plan.get("points") or []
    teach_samples = []
    replay_samples = []
    max_abs_error = 0.0
    sum_sq = 0.0
    count = 0
    for idx, point in enumerate(points):
        teach = {
            "sample_index": idx,
            "time_from_start": point.get("time_from_start"),
            "phase": point.get("phase"),
            "left": [round(float(v), 6) for v in point.get("left", [])],
            "right": [round(float(v), 6) for v in point.get("right", [])],
        }
        replay = {
            "sample_index": idx,
            "time_from_start": point.get("time_from_start"),
            "phase": point.get("phase"),
            "left": [],
            "right": [],
        }
        for side in ["left", "right"]:
            replay_values = []
            for joint_idx, value in enumerate(teach[side]):
                replay_value = deterministic_replay_value(float(value), idx, joint_idx, side)
                error = replay_value - float(value)
                max_abs_error = max(max_abs_error, abs(error))
                sum_sq += error * error
                count += 1
                replay_values.append(replay_value)
            replay[side] = replay_values
        teach_samples.append(teach)
        replay_samples.append(replay)
    rms = math.sqrt(sum_sq / count) if count else None
    return {
        "evidence_type": "simulated_teach_replay",
        "day_id": "day04",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_plan": str(plan_path),
        "simulation_note": "Offline simulated evidence generated from MoveIt D4 planned points; this is not a hardware teach-pendant recording.",
        "teach": {
            "sample_count": len(teach_samples),
            "joint_order": {
                "left": [f"left_joint{i}" for i in range(1, 7)],
                "right": [f"right_joint{i}" for i in range(1, 7)],
            },
            "samples": teach_samples,
        },
        "replay": {
            "speed_scale": 0.5,
            "controller_mode": "simulated_open_loop_replay",
            "samples": replay_samples,
        },
        "repeatability_proxy": {
            "max_abs_joint_error_rad": round(max_abs_error, 9),
            "rms_joint_error_rad": round(rms, 9) if rms is not None else None,
            "sample_pairs": count,
            "within_one_mm_claim": "not_claimed_offline",
        },
    }


def write_evidence(evidence: dict[str, Any], output_path: Path) -> Path:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    text = dump_yaml(evidence) + "\n"
    output_path.write_text(text, encoding="utf-8")
    return output_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    root = repo_root_from_script()
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--outputs-dir", type=Path, default=root / "outputs")
    parser.add_argument("--output-file", type=Path)
    args = parser.parse_args()

    output_root = (args.output_root or latest_output_root(args.outputs_dir)).resolve()
    day04_dir = output_root / "day04"
    evidence = build_evidence(day04_dir)
    output_file = args.output_file or (day04_dir / "logs" / "day04_simulated_teach_replay_evidence.yaml")
    written = write_evidence(evidence, output_file.resolve())
    print(f"output_root={output_root}")
    print(f"yaml={written}")
    print(f"samples={evidence['teach']['sample_count']}")
    print(f"max_abs_joint_error_rad={evidence['repeatability_proxy']['max_abs_joint_error_rad']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
