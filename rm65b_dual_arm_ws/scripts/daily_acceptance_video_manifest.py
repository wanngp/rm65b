#!/usr/bin/env python3
"""Emit the D1-D5 acceptance video shot list and evidence manifest."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


VIDEO_MANIFEST: dict[str, dict[str, Any]] = {
    "D1": {
        "name": "gripper_tf_dualarm",
        "video_goal": "Show both RM65-B arms, grippers, calibrated TF frames, and synchronized safe dual-arm motion.",
        "must_show": [
            "Left and right physical arms in one wide shot with grippers mounted.",
            "RViz TF tree or frame overlay for base, Link6, and gripper TCP frames.",
            "Open/close gripper command and visible completion on both arms.",
            "Dual-arm cooperative motion with no inter-arm collision.",
        ],
        "ros_evidence": [
            "/tf",
            "/tf_static",
            "/joint_states",
            "/left_gripper_controller/*",
            "/right_gripper_controller/*",
            "/move_action/_action/status",
        ],
        "acceptance_signals": [
            "Measured TCP/TF values are displayed or included in the log.",
            "Gripper command latency is reported.",
            "Motion is replayed from the real dual-arm stack, not only an offline animation.",
        ],
    },
    "D2": {
        "name": "force_compliance_contact",
        "video_goal": "Show force-controlled contact and compliant response against a real fixture.",
        "must_show": [
            "Force sensor zeroing or gravity compensation state before contact.",
            "End effector touching the fixture or yarn/contact target.",
            "Force trace, wrench error, and admittance offset changing during contact.",
            "Robot backing off or settling inside the configured force window.",
        ],
        "ros_evidence": [
            "/right_rm_driver/rm_driver/udp_six_force",
            "/force_control/wrench_error",
            "/force_control/admittance_offset",
            "/force_control/state",
            "/safety/state",
        ],
        "acceptance_signals": [
            "Target force window is visible.",
            "Contact is physical, not a plotted-only signal.",
            "Any overshoot, oscillation, or safety stop is kept in the recording.",
        ],
    },
    "D3": {
        "name": "vision_ibvs_alignment",
        "video_goal": "Show camera detection and IBVS closing the visual error on the robot.",
        "must_show": [
            "Camera view with marker, yarn edge, or target overlay.",
            "Detection fps and pixel error trend.",
            "Hand-eye or camera frame relationship visible in RViz.",
            "Robot motion driven by visual servo output until the target is aligned.",
        ],
        "ros_evidence": [
            "/left_camera/image_rect",
            "/left_camera/camera_info",
            "/vision/target_pose",
            "/vision/metrics",
            "/visual_servo/twist_cmd",
        ],
        "acceptance_signals": [
            "Initial error and final error are both shown.",
            "The target remains visible during motion.",
            "The final alignment result is measured in pixels and robot/world units.",
        ],
    },
    "D4": {
        "name": "weaving_teach_replay",
        "video_goal": "Show real teach-pendant primitive recording, YAML creation, and replay through the coordinator.",
        "must_show": [
            "Operator teaches hook_yarn, lift_yarn, pull_tight, shift, and exchange on the real arms.",
            "record_d4_real_teach_replay.py receiving /joint_states and writing primitive YAML.",
            "Primitive YAML metadata contains evidence_type=real_teach_replay_primitive_yaml.",
            "d4_smooth_teach_trajectory.py writes smoothed replay YAML and quality JSON.",
            "BehaviorTree.CPP replays the recorded YAML with weaving events visible.",
            "Yarn tension PID/compliance response is visible during pull_tight.",
        ],
        "ros_evidence": [
            "/joint_states",
            "/weaving/events",
            "/weaving/tension_n",
            "/weaving/tension_status",
            "/weaving/tension_pid_state",
            "/weaving/compliance_offset_m",
            "/safety/state",
            "/left_arm_controller/follow_joint_trajectory/_action/status",
            "/right_arm_controller/follow_joint_trajectory/_action/status",
        ],
        "acceptance_signals": [
            "The D4 YAML source is live /joint_states, not dual_moveit_plans.yaml.",
            "Replay uses the recorded primitive YAML path.",
            "Repeatability checks compare teach and replay pose/joint traces.",
        ],
        "record_command": (
            "python3 rm65b_dual_arm_ws/scripts/record_d4_real_teach_replay.py "
            "--output-file outputs/on_site/day04/logs/day04_real_recorded_primitives.yaml"
        ),
        "replay_command": (
            "ros2 run rm65b_weaving_primitives weaving_coordinator --ros-args "
            "-p trajectory_file:=outputs/on_site/day04/logs/day04_real_recorded_primitives.yaml "
            "-p dry_run:=false -p playback_stage:=D4_real_teach_replay"
        ),
    },
    "D5": {
        "name": "integrated_system_demo",
        "video_goal": "Show one-launch startup, node graph, latency record, safety fault handling, and the first weaving demo loop.",
        "must_show": [
            "integrated_simulation.launch.py or full_system.launch.py starts from a clean terminal.",
            "/system/launch_audit and day05_node_graph.json show the node/topic graph.",
            "day05_latency_record.json reports no missing chains and all chains below 200 ms.",
            "A force, joint, collision, or hardware-estop fault triggers /safety/fault and /safety/estop.",
            "Reset follows recover_home, ack_reset, then reset.",
            "Vision target, dual-arm motion, gripper/yarn action, weaving events, and tension telemetry occur in one demo.",
        ],
        "ros_evidence": [
            "/acceptance/day_status",
            "/system/launch_audit",
            "/weaving/events",
            "/weaving/tension_n",
            "/weaving/tension_pid_state",
            "/vision/target_pose",
            "/visual_servo/twist_cmd",
            "/force_control/state",
            "/force_control/wrench_error",
            "/safety/state",
            "/safety/fault",
            "/safety/estop",
            "/safety/reset_required",
            "/joint_states",
        ],
        "acceptance_signals": [
            "The launch audit reports no missing required nodes/topics.",
            "Measured latency has no missing chains and max_ms is below 200 ms.",
            "Fault injection creates a recorded stop and a successful reset sequence.",
            "The final video shows the integrated closed loop rather than isolated arm motion.",
        ],
    },
}


def to_markdown(manifest: dict[str, dict[str, Any]]) -> str:
    lines = ["# D1-D5 acceptance video manifest", ""]
    for day, spec in manifest.items():
        lines.extend([f"## {day} {spec['name']}", "", spec["video_goal"], ""])
        lines.append("Must show:")
        lines.extend(f"- {item}" for item in spec["must_show"])
        lines.extend(["", "ROS evidence:"])
        lines.extend(f"- `{item}`" for item in spec["ros_evidence"])
        lines.extend(["", "Acceptance signals:"])
        lines.extend(f"- {item}" for item in spec["acceptance_signals"])
        if "record_command" in spec:
            lines.extend(["", "Record command:", "", "```bash", spec["record_command"], "```"])
        if "replay_command" in spec:
            lines.extend(["", "Replay command:", "", "```bash", spec["replay_command"], "```"])
        lines.append("")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--format", choices=["markdown", "json"], default="markdown")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    text = (
        json.dumps(VIDEO_MANIFEST, indent=2, ensure_ascii=False)
        if args.format == "json"
        else to_markdown(VIDEO_MANIFEST)
    )
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
