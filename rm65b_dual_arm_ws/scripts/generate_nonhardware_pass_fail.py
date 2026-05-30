#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]


def count(day: dict[str, Any], topic: str) -> int:
    return int(day.get("topic_counts", {}).get(topic, 0) or 0)


def check(name: str, passed: bool, evidence: dict[str, Any], limitation: str | None = None) -> dict[str, Any]:
    item: dict[str, Any] = {"name": name, "status": "PASS" if passed else "FAIL", "evidence": evidence}
    if limitation:
        item["limitation"] = limitation
    return item


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def relative_evidence_path(path: Path, root: Path) -> str:
    try:
        return path.relative_to(root).as_posix()
    except ValueError:
        return path.as_posix()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", required=True)
    args = parser.parse_args()

    output_root = Path(args.output_root).resolve()
    analysis_dir = output_root / "offline_analysis"
    daily = load_json(analysis_dir / "daily_evidence_report.json")
    sweep = load_json(analysis_dir / "static_collision_sweep_report.json")
    runtime_sweep_path = analysis_dir / "runtime_collision_sweep" / "runtime_collision_sweep_report.json"
    runtime_sweep = load_json(runtime_sweep_path) if runtime_sweep_path.exists() else {}
    days = {day["day_id"]: day for day in daily["days"]}

    checks: list[dict[str, Any]] = []
    for day_id, day in days.items():
        expected_frames = {"day01": 140, "day02": 140, "day03": 140, "day04": 190, "day05": 260}.get(day_id, 1)
        left_frame_count = (
            day.get("camera", {}).get("frames", {}).get("left_camera_frames", {}).get("count", 0)
        )
        checks.append(
            check(
                f"{day_id} media and Gazebo camera",
                bool(day.get("camera", {}).get("media_ok")) and int(left_frame_count or 0) >= expected_frames,
                {
                    "media_ok": day.get("camera", {}).get("media_ok"),
                    "left_camera_hz": day.get("camera", {})
                    .get("camera_topic_rates", {})
                    .get("/left_camera/image_rect", {})
                    .get("rate_hz"),
                    "left_camera_frames": left_frame_count,
                    "expected_left_camera_frames": expected_frames,
                },
            )
        )
        checks.append(
            check(
                f"{day_id} movable Gazebo gripper commands",
                count(day, "/rm65b_gripper/upper_finger_cmd") > 0
                and count(day, "/rm65b_gripper/lower_finger_cmd") > 0,
                {
                    "upper_cmd_count": count(day, "/rm65b_gripper/upper_finger_cmd"),
                    "lower_cmd_count": count(day, "/rm65b_gripper/lower_finger_cmd"),
                },
            )
        )
        if day_id in {"day04", "day05"}:
            checks.append(
                check(
                    f"{day_id} BehaviorTree/weaving runtime events",
                    count(day, "/weaving/events") > 0,
                    {"weaving_event_count": count(day, "/weaving/events")},
                )
            )

    for day_id in ("day03", "day05"):
        day = days[day_id]
        checks.append(
            check(
                f"{day_id} Gazebo-camera vision drives IBVS and robot command",
                count(day, "/vision/target_pose") > 0
                and count(day, "/visual_servo/twist_cmd") > 0
                and count(day, "/visual_servo/gazebo_adapter_state") > 0
                and count(day, "/model/left_rm65b/joint_trajectory") > 0,
                {
                    "vision_target_pose": count(day, "/vision/target_pose"),
                    "twist_cmd": count(day, "/visual_servo/twist_cmd"),
                    "adapter_state": count(day, "/visual_servo/gazebo_adapter_state"),
                    "left_gazebo_trajectory": count(day, "/model/left_rm65b/joint_trajectory"),
                },
            )
        )

    for day_id in ("day02", "day04", "day05"):
        day = days[day_id]
        checks.append(
            check(
                f"{day_id} force loop writes corrected trajectory to Gazebo",
                count(day, "/force_control/corrected_right_joint_trajectory") > 0
                and count(day, "/force_control/gazebo_relay_state") > 0
                and count(day, "/model/right_rm65b/joint_trajectory") > 0,
                {
                    "corrected_right_trajectory": count(day, "/force_control/corrected_right_joint_trajectory"),
                    "relay_state": count(day, "/force_control/gazebo_relay_state"),
                    "right_gazebo_trajectory": count(day, "/model/right_rm65b/joint_trajectory"),
                },
            )
        )
        checks.append(
            check(
                f"{day_id} Gazebo contact force feeds force controller",
                count(day, "/force_control/gazebo_contact_force_state") > 0
                and count(day, "/right_rm_driver/rm_driver/udp_six_force") > 0,
                {
                    "contact_force_state": count(day, "/force_control/gazebo_contact_force_state"),
                    "right_sixforce": count(day, "/right_rm_driver/rm_driver/udp_six_force"),
                },
            )
        )
        force = day.get("force_loop", {})
        checks.append(
            check(
                f"{day_id} force-loop settling proxy",
                force.get("settling_time_s_proxy") is not None,
                {
                    "sample_count": force.get("sample_count"),
                    "first_norm": force.get("wrench_error_norm_first"),
                    "final_norm": force.get("wrench_error_norm_final"),
                    "threshold_norm": force.get("settling_threshold_norm"),
                    "settling_time_s_proxy": force.get("settling_time_s_proxy"),
                },
                "Offline proxy from /force_control/wrench_error; this is a simulated loop metric, not hardware force calibration.",
            )
        )

    contact_topics = {
        "day02": [
            "/world/rm65b_world/model/d2_force_target_panel/link/link/sensor/d2_force_target_panel_contact/contact"
        ],
        "day04": [
            "/world/rm65b_world/model/d4_shuttle_lane/link/link/sensor/d4_shuttle_lane_contact/contact",
            "/world/rm65b_world/model/d4_tension_scale/link/link/sensor/d4_tension_scale_contact/contact",
        ],
        "day05": [
            "/world/rm65b_world/model/d2_force_target_panel/link/link/sensor/d2_force_target_panel_contact/contact",
            "/world/rm65b_world/model/d4_shuttle_lane/link/link/sensor/d4_shuttle_lane_contact/contact",
            "/world/rm65b_world/model/d4_tension_scale/link/link/sensor/d4_tension_scale_contact/contact",
        ],
    }
    for day_id, topics in contact_topics.items():
        day = days[day_id]
        checks.append(
            check(
                f"{day_id} Gazebo contact physics messages",
                any(count(day, topic) > 0 for topic in topics),
                {topic: count(day, topic) for topic in topics},
            )
        )

    teach_file = output_root / "day04" / "logs" / "day04_recorded_primitives_from_live_joint_states.yaml"
    fallback_teach_file = output_root / "day04" / "logs" / "day04_simulated_teach_replay_evidence.yaml"
    selected_teach_file = teach_file if teach_file.exists() else fallback_teach_file
    checks.append(
        check(
            "day04 live joint-state teach/replay primitive YAML",
            teach_file.exists() and teach_file.stat().st_size > 0,
            {
                "relative_path": relative_evidence_path(selected_teach_file, output_root),
                "bytes": selected_teach_file.stat().st_size if selected_teach_file.exists() else 0,
                "source": "live /joint_states recorder" if teach_file.exists() else "offline fallback",
            },
            None
            if teach_file.exists()
            else "Fallback is generated from D4 MoveIt simulated plan samples, not a live recorder.",
        )
    )
    if runtime_sweep:
        runtime_summary = runtime_sweep.get("summary", {})
        scenario_invalid = runtime_summary.get("scenario_invalid_counts", {})
        checks.append(
            check(
                "MoveIt runtime PlanningScene/FCL collision sweep",
                int(runtime_summary.get("service_error_count", 1)) == 0
                and int(scenario_invalid.get("planned_trajectory_sample", 0) or 0) == 0,
                {
                    "report": relative_evidence_path(runtime_sweep_path, output_root),
                    "sample_count": runtime_summary.get("sample_count"),
                    "valid_count": runtime_summary.get("valid_count"),
                    "invalid_count": runtime_summary.get("invalid_count"),
                    "collision_count": runtime_summary.get("collision_count"),
                    "service_error_count": runtime_summary.get("service_error_count"),
                    "scenario_invalid_counts": scenario_invalid,
                    "contact_pair_counts": runtime_summary.get("contact_pair_counts"),
                },
                "Runtime FCL found self-collision hazards at extreme joint3/joint5 tool-folded poses; D1-D5 planned trajectory samples are valid. This is a sampled sweep, not a mathematical continuum proof.",
            )
        )
    else:
        checks.append(
            check(
                "MoveIt runtime PlanningScene/FCL collision sweep",
                False,
                {"report": relative_evidence_path(runtime_sweep_path, output_root), "exists": False},
                "Runtime /check_state_validity sweep has not been run.",
            )
        )
    checks.append(
        check(
            "MoveIt static collision sweep/sample consistency",
            int(sweep.get("sample_status", {}).get("issue_count", 1)) == 0
            and int(sweep.get("collision_matrix", {}).get("active_cross_arm_pair_count", 0)) > 0,
            {
                "sample_issues": sweep.get("sample_status", {}).get("issue_count"),
                "active_cross_arm_pairs": sweep.get("collision_matrix", {}).get("active_cross_arm_pair_count"),
                "checked_rows": sweep.get("sample_status", {}).get("checked_rows"),
            },
            "Static URDF/SRDF consistency cross-check retained as a secondary check; runtime collision truth is the MoveIt PlanningScene/FCL report above.",
        )
    )

    status = "PASS" if all(item["status"] == "PASS" for item in checks) else "FAIL"
    result = {
        "output_root": str(output_root),
        "status": status,
        "checks": checks,
        "failed_checks": [item for item in checks if item["status"] != "PASS"],
    }
    out_json = analysis_dir / "nonhardware_pass_fail_metrics.json"
    out_md = analysis_dir / "nonhardware_pass_fail_metrics.md"
    out_json.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    lines = [f"# Non-Hardware Pass/Fail Metrics", "", f"Overall: **{status}**", ""]
    for item in checks:
        lines.append(f"- {item['status']} {item['name']}: `{json.dumps(item['evidence'], ensure_ascii=False)}`")
        if item.get("limitation"):
            lines.append(f"  Limitation: {item['limitation']}")
    out_md.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"json={out_json}")
    print(f"md={out_md}")
    print(f"status={status}")
    print(f"failed={len(result['failed_checks'])}")
    return 0 if status == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
