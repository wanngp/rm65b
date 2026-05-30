#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import yaml


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def main() -> int:
    parser = argparse.ArgumentParser(description="Offline D4 yarn tension PID tuning simulation")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("src/rm65b_weaving_primitives/config/d4_tension_pid_compliance.yaml"),
    )
    parser.add_argument("--csv", type=Path, default=Path("outputs/d4_tension_pid/step_response.csv"))
    parser.add_argument("--json", type=Path, default=Path("outputs/d4_tension_pid/tuning_record.json"))
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--dt", type=float, default=0.01)
    args = parser.parse_args()

    cfg = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    model = cfg["tension_model"]
    pid = cfg["pid"]
    compliance = cfg["compliance"]
    acceptance = cfg["acceptance"]

    baseline = float(model["baseline_n"])
    target = float(model["target_tension_n"])
    min_ok = float(model["min_ok_n"])
    max_ok = float(model["max_ok_n"])
    mass = float(model["yarn_mass_kg"])
    spring_k = float(model["spring_k_n_per_m"])
    damping = float(model["damping_n_s_per_m"])
    max_elongation = float(model["max_elongation_m"])
    kp = float(pid["kp"])
    ki = float(pid["ki"])
    kd = float(pid["kd"])
    integral_limit = float(pid["integral_limit_n_s"])
    derivative_alpha = float(pid["derivative_filter_alpha"])
    compliance_gain = float(compliance["compliance_gain_m_per_n"])
    max_offset = float(compliance["max_offset_m"])

    elongation = 0.0
    velocity = 0.0
    tension = baseline
    integral = 0.0
    derivative = 0.0
    previous_error = target - tension
    commanded_elongation = 0.018
    rows = []
    settled_at = None
    peak_tension = tension

    for idx in range(int(args.duration / args.dt) + 1):
        time_s = idx * args.dt
        error = target - tension
        integral = clamp(integral + error * args.dt, -integral_limit, integral_limit)
        raw_derivative = (error - previous_error) / args.dt
        derivative = derivative_alpha * raw_derivative + (1.0 - derivative_alpha) * derivative
        previous_error = error
        pid_output = kp * error + ki * integral + kd * derivative
        compliance_offset = clamp(pid_output * compliance_gain, -max_offset, max_offset)
        equilibrium = clamp(commanded_elongation + compliance_offset, 0.0, max_elongation)
        acceleration = (spring_k * (equilibrium - elongation) - damping * velocity) / mass
        velocity += acceleration * args.dt
        elongation = clamp(elongation + velocity * args.dt, 0.0, max_elongation)
        tension = max(0.0, baseline + spring_k * elongation)
        peak_tension = max(peak_tension, tension)
        status = "LOW" if tension < min_ok else "HIGH" if tension > max_ok else "OK"
        if settled_at is None and abs(tension - target) <= 0.05 and status == "OK":
            settled_at = time_s
        rows.append(
            {
                "time_s": f"{time_s:.3f}",
                "tension_n": f"{tension:.6f}",
                "status": status,
                "elongation_m": f"{elongation:.6f}",
                "velocity_mps": f"{velocity:.6f}",
                "pid_output": f"{pid_output:.6f}",
                "compliance_offset_m": f"{compliance_offset:.6f}",
            }
        )

    args.csv.parent.mkdir(parents=True, exist_ok=True)
    args.json.parent.mkdir(parents=True, exist_ok=True)
    with args.csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    overshoot = max(0.0, peak_tension - target)
    record = {
        "config": str(args.config),
        "selected_pid": {"kp": kp, "ki": ki, "kd": kd},
        "compliance": {
            "gain_m_per_n": compliance_gain,
            "max_offset_m": max_offset,
        },
        "metrics": {
            "target_tension_n": target,
            "peak_tension_n": peak_tension,
            "overshoot_n": overshoot,
            "settled_at_s": settled_at,
            "final_tension_n": tension,
            "final_status": rows[-1]["status"],
            "acceptance_pass": (
                rows[-1]["status"] == "OK"
                and overshoot <= float(acceptance["overshoot_limit_n"])
                and (settled_at is not None and settled_at <= float(acceptance["tension_settle_time_s"]))
            ),
        },
        "tuning_record": cfg["tuning_record"],
    }
    args.json.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {args.csv}")
    print(f"wrote {args.json}")
    print(json.dumps(record["metrics"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
