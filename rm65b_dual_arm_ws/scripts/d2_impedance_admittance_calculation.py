#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import yaml


def simulate(
    *,
    mass: float,
    damping: float,
    stiffness: float,
    joint_gain: float,
    force_step_n: float,
    duration_s: float,
    dt: float,
) -> list[dict]:
    x = 0.0
    xdot = 0.0
    rows = []
    steps = int(duration_s / dt)
    for idx in range(steps + 1):
        t = idx * dt
        force_error = force_step_n if t >= 0.50 else 0.0
        xdd = (force_error - damping * xdot - stiffness * x) / mass
        xdot += xdd * dt
        x += xdot * dt
        impedance_force = mass * xdd + damping * xdot + stiffness * x
        rows.append(
            {
                "time_s": round(t, 4),
                "force_error_n": round(force_error, 6),
                "admittance_x_m": round(x, 8),
                "admittance_xdot_mps": round(xdot, 8),
                "admittance_xdd_mps2": round(xdd, 8),
                "joint6_offset_rad": round(joint_gain * x, 8),
                "impedance_force_n": round(impedance_force, 8),
            }
        )
    return rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        default="src/rm65b_dual_arm_planning/config/d2_end_effector_dynamics.yaml",
    )
    parser.add_argument(
        "--output",
        default="outputs/d2_force_control/d2_impedance_admittance_step_response.csv",
    )
    parser.add_argument("--force-step-n", type=float, default=3.0)
    parser.add_argument("--duration-s", type=float, default=4.0)
    parser.add_argument("--dt", type=float, default=0.01)
    args = parser.parse_args()

    config = yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))
    admittance = config["d2_force_control"]["admittance"]
    rows = simulate(
        mass=float(admittance["virtual_mass_kg"]),
        damping=float(admittance["virtual_damping_ns_per_m"]),
        stiffness=float(admittance["virtual_stiffness_n_per_m"]),
        joint_gain=float(admittance["joint6_rad_per_m"]),
        force_step_n=float(args.force_step_n),
        duration_s=float(args.duration_s),
        dt=float(args.dt),
    )

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {output}")
    print(rows[-1])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
