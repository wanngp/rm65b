#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml


def box_inertia(mass: float, size: list[float]) -> list[list[float]]:
    x, y, z = size
    return [
        [mass * (y * y + z * z) / 12.0, 0.0, 0.0],
        [0.0, mass * (x * x + z * z) / 12.0, 0.0],
        [0.0, 0.0, mass * (x * x + y * y) / 12.0],
    ]


def mat_add(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [[a[i][j] + b[i][j] for j in range(3)] for i in range(3)]


def parallel_axis(mass: float, delta: list[float]) -> list[list[float]]:
    dx, dy, dz = delta
    r2 = dx * dx + dy * dy + dz * dz
    return [
        [mass * (r2 - dx * dx), -mass * dx * dy, -mass * dx * dz],
        [-mass * dy * dx, mass * (r2 - dy * dy), -mass * dy * dz],
        [-mass * dz * dx, -mass * dz * dy, mass * (r2 - dz * dz)],
    ]


def identify(components: dict) -> dict:
    total_mass = sum(float(item["mass_kg"]) for item in components.values())
    com = [0.0, 0.0, 0.0]
    for item in components.values():
        mass = float(item["mass_kg"])
        xyz = [float(v) for v in item["xyz_m"]]
        for idx in range(3):
            com[idx] += mass * xyz[idx]
    com = [value / total_mass for value in com]

    inertia = [[0.0, 0.0, 0.0] for _ in range(3)]
    for item in components.values():
        mass = float(item["mass_kg"])
        xyz = [float(v) for v in item["xyz_m"]]
        size = [float(v) for v in item["size_m"]]
        delta = [xyz[idx] - com[idx] for idx in range(3)]
        inertia = mat_add(inertia, box_inertia(mass, size))
        inertia = mat_add(inertia, parallel_axis(mass, delta))

    return {
        "mass_kg": round(total_mass, 6),
        "com_m": [round(v, 6) for v in com],
        "inertia_about_com_kg_m2": {
            "ixx": round(inertia[0][0], 10),
            "ixy": round(inertia[0][1], 10),
            "ixz": round(inertia[0][2], 10),
            "iyy": round(inertia[1][1], 10),
            "iyz": round(inertia[1][2], 10),
            "izz": round(inertia[2][2], 10),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        default="src/rm65b_dual_arm_planning/config/d2_end_effector_dynamics.yaml",
    )
    parser.add_argument(
        "--output",
        default="outputs/d2_dynamics_identification/d2_identified_end_effector_dynamics.json",
    )
    args = parser.parse_args()

    config_path = Path(args.config)
    data = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    model = data["d2_end_effector_dynamics"]
    result = {
        "method": "rigid box component aggregation with parallel-axis theorem",
        "frame": model["frame"],
        "components": model["components"],
        "identified_total": identify(model["components"]),
        "gazebo_link_parameters": model["gazebo_links"],
        "source_config": str(config_path),
    }
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result["identified_total"], indent=2))
    print(f"wrote {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
