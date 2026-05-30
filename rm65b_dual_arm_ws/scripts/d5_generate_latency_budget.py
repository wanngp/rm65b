#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


LATENCY_BUDGET = [
    {
        "chain": "camera_to_vision_pose_ms",
        "source": "/left_camera/image_rect",
        "sink": "/vision/target_pose",
        "budget_ms": 80.0,
        "recorded_by": "rm65b_safety/d5_latency_probe",
    },
    {
        "chain": "vision_pose_to_twist_ms",
        "source": "/vision/target_pose",
        "sink": "/visual_servo/twist_cmd",
        "budget_ms": 30.0,
        "recorded_by": "rm65b_safety/d5_latency_probe",
    },
    {
        "chain": "twist_to_visual_adapter_ms",
        "source": "/visual_servo/twist_cmd",
        "sink": "/visual_servo/gazebo_adapter_state",
        "budget_ms": 40.0,
        "recorded_by": "rm65b_safety/d5_latency_probe",
    },
    {
        "chain": "force_input_to_error_ms",
        "source": "/force_control/target_wrench",
        "sink": "/force_control/wrench_error",
        "budget_ms": 30.0,
        "recorded_by": "rm65b_safety/d5_latency_probe",
    },
    {
        "chain": "force_error_to_admittance_ms",
        "source": "/force_control/wrench_error",
        "sink": "/force_control/admittance_offset",
        "budget_ms": 40.0,
        "recorded_by": "rm65b_safety/d5_latency_probe",
    },
    {
        "chain": "weaving_event_to_tension_ms",
        "source": "/weaving/events",
        "sink": "/weaving/tension_n",
        "budget_ms": 80.0,
        "recorded_by": "rm65b_safety/d5_latency_probe",
    },
    {
        "chain": "safety_fault_to_estop_ms",
        "source": "/safety/fault",
        "sink": "/safety/estop",
        "budget_ms": 50.0,
        "recorded_by": "rm65b_safety/d5_latency_probe",
    },
]


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate the D5 communication latency budget table.")
    parser.add_argument("--output-json", type=Path, default=Path("outputs/d5_system/day05_latency_budget.json"))
    parser.add_argument("--output-csv", type=Path, default=Path("outputs/d5_system/day05_latency_budget.csv"))
    args = parser.parse_args()

    payload = {
        "acceptance_limit_ms": 200.0,
        "record_type": "budget_template_for_live_measurement",
        "live_measurement_file": "outputs/d5_system/day05_latency_record.json",
        "chains": LATENCY_BUDGET,
        "budget_pass": all(item["budget_ms"] < 200.0 for item in LATENCY_BUDGET),
    }
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    with args.output_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["chain", "source", "sink", "budget_ms", "recorded_by"],
        )
        writer.writeheader()
        writer.writerows(LATENCY_BUDGET)
    print(f"wrote {args.output_json}")
    print(f"wrote {args.output_csv}")


if __name__ == "__main__":
    main()
