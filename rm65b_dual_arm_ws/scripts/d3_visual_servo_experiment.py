#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def main() -> int:
    parser = argparse.ArgumentParser(description="D3 IBVS calculation trace")
    parser.add_argument("--output", type=Path, default=Path("outputs/d3_visual_servo/ibvs_trace.csv"))
    parser.add_argument("--initial-pixel-error-u", type=float, default=120.0)
    parser.add_argument("--initial-pixel-error-v", type=float, default=-70.0)
    parser.add_argument("--fx", type=float, default=600.0)
    parser.add_argument("--fy", type=float, default=600.0)
    parser.add_argument("--target-depth-m", type=float, default=0.25)
    parser.add_argument("--gain-y", type=float, default=0.8)
    parser.add_argument("--gain-z", type=float, default=0.5)
    parser.add_argument("--max-linear-mps", type=float, default=0.03)
    parser.add_argument("--target-tolerance-m", type=float, default=0.001)
    parser.add_argument("--joint1-gain-per-mps", type=float, default=0.55)
    parser.add_argument("--joint2-gain-per-mps", type=float, default=0.45)
    parser.add_argument("--max-joint-step-rad", type=float, default=0.030)
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument("--duration", type=float, default=12.0)
    args = parser.parse_args()

    error_y = -args.initial_pixel_error_u * args.target_depth_m / args.fx
    error_z = -args.initial_pixel_error_v * args.target_depth_m / args.fy
    joint1 = 0.046
    joint2 = -0.840

    rows = []
    steps = int(args.duration / args.dt) + 1
    for step in range(steps):
        time_s = step * args.dt
        norm = math.sqrt(error_y * error_y + error_z * error_z)
        aligned = norm <= args.target_tolerance_m
        twist_y = 0.0 if aligned else clamp(-args.gain_y * error_y, args.max_linear_mps)
        twist_z = 0.0 if aligned else clamp(-args.gain_z * error_z, args.max_linear_mps)
        pixel_error_u = -error_y * args.fx / args.target_depth_m
        pixel_error_v = -error_z * args.fy / args.target_depth_m

        rows.append(
            {
                "time_s": f"{time_s:.3f}",
                "pixel_error_u_px": f"{pixel_error_u:.3f}",
                "pixel_error_v_px": f"{pixel_error_v:.3f}",
                "target_y_m": f"{error_y:.6f}",
                "target_z_m": f"{error_z:.6f}",
                "twist_y_mps": f"{twist_y:.6f}",
                "twist_z_mps": f"{twist_z:.6f}",
                "joint1_cmd_rad": f"{joint1:.6f}",
                "joint2_cmd_rad": f"{joint2:.6f}",
                "aligned": str(aligned).lower(),
            }
        )

        joint1 += clamp(twist_y * args.joint1_gain_per_mps, args.max_joint_step_rad)
        joint2 += clamp(twist_z * args.joint2_gain_per_mps, args.max_joint_step_rad)
        error_y += twist_y * args.dt
        error_z += twist_z * args.dt

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    final = rows[-1]
    print(f"wrote {args.output}")
    print(
        "final "
        f"pixel_error=({final['pixel_error_u_px']},{final['pixel_error_v_px']}) px "
        f"target=({final['target_y_m']},{final['target_z_m']}) m "
        f"aligned={final['aligned']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
