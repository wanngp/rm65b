#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import yaml


def smoothstep(alpha: float) -> float:
    alpha = max(0.0, min(1.0, alpha))
    return alpha * alpha * (3.0 - 2.0 * alpha)


def interpolate(points: list[dict[str, Any]], sample_hz: float) -> list[dict[str, Any]]:
    if len(points) <= 1:
        return points
    duration = float(points[-1]["time_from_start"])
    if duration <= 0.0:
        return points
    dt = 1.0 / sample_hz
    output: list[dict[str, Any]] = []
    idx = 0
    steps = int(math.ceil(duration / dt)) + 1
    for step in range(steps):
        t = min(duration, step * dt)
        while idx + 1 < len(points) and float(points[idx + 1]["time_from_start"]) < t:
            idx += 1
        p0 = points[idx]
        p1 = points[min(idx + 1, len(points) - 1)]
        t0 = float(p0["time_from_start"])
        t1 = float(p1["time_from_start"])
        span = max(t1 - t0, 1.0e-6)
        alpha = smoothstep((t - t0) / span)
        q0 = [float(v) for v in p0["positions"]]
        q1 = [float(v) for v in p1["positions"]]
        positions = [round(a + (b - a) * alpha, 6) for a, b in zip(q0, q1)]
        output.append({"time_from_start": round(t, 4), "positions": positions})
    return output


def max_velocity(points: list[dict[str, Any]]) -> float:
    max_v = 0.0
    for prev, cur in zip(points, points[1:]):
        dt = max(float(cur["time_from_start"]) - float(prev["time_from_start"]), 1.0e-6)
        for a, b in zip(prev["positions"], cur["positions"]):
            max_v = max(max_v, abs(float(b) - float(a)) / dt)
    return max_v


def max_step(points: list[dict[str, Any]]) -> float:
    result = 0.0
    for prev, cur in zip(points, points[1:]):
        for a, b in zip(prev["positions"], cur["positions"]):
            result = max(result, abs(float(b) - float(a)))
    return result


def stretch_time(points: list[dict[str, Any]], factor: float) -> list[dict[str, Any]]:
    return [
        {
            "time_from_start": round(float(point["time_from_start"]) * factor, 4),
            "positions": point["positions"],
        }
        for point in points
    ]


def process_primitive(
    primitive: dict[str, Any], sample_hz: float, max_velocity_rad_s: float
) -> tuple[dict[str, Any], dict[str, Any]]:
    output = {key: value for key, value in primitive.items() if key not in {"left", "right"}}
    quality: dict[str, Any] = {}
    for side in ("left", "right"):
        if side not in primitive:
            continue
        raw_points = primitive[side]
        smoothed = interpolate(raw_points, sample_hz)
        v = max_velocity(smoothed)
        if v > max_velocity_rad_s:
            factor = v / max_velocity_rad_s
            smoothed = stretch_time(smoothed, factor)
            smoothed = interpolate(smoothed, sample_hz)
            v = max_velocity(smoothed)
        output[side] = smoothed
        quality[side] = {
            "raw_points": len(raw_points),
            "smoothed_points": len(smoothed),
            "duration_s": smoothed[-1]["time_from_start"] if smoothed else 0.0,
            "max_step_rad": max_step(smoothed),
            "max_velocity_rad_s": v,
        }
    return output, quality


def main() -> int:
    parser = argparse.ArgumentParser(description="Smooth D4 teach/replay primitive YAML")
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--quality-json", required=True, type=Path)
    parser.add_argument("--sample-hz", type=float, default=25.0)
    parser.add_argument("--max-velocity-rad-s", type=float, default=0.45)
    args = parser.parse_args()

    data = yaml.safe_load(args.input.read_text(encoding="utf-8")) or {}
    result = dict(data)
    result.setdefault("metadata", {})
    result["metadata"]["smoothing"] = {
        "method": "smoothstep interpolation with velocity limiting",
        "sample_hz": args.sample_hz,
        "max_velocity_rad_s": args.max_velocity_rad_s,
        "source_file": str(args.input),
    }
    quality = {
        "input": str(args.input),
        "output": str(args.output),
        "sample_hz": args.sample_hz,
        "max_velocity_rad_s": args.max_velocity_rad_s,
        "primitives": {},
    }
    result["primitives"] = {}
    for name, primitive in (data.get("primitives") or {}).items():
        result["primitives"][name], quality["primitives"][name] = process_primitive(
            primitive, args.sample_hz, args.max_velocity_rad_s
        )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.quality_json.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(result, sort_keys=False, allow_unicode=False), encoding="utf-8")
    args.quality_json.write_text(json.dumps(quality, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {args.output}")
    print(f"wrote {args.quality_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
