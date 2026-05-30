#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
import time
from pathlib import Path

import yaml


GRIPPER_OPEN = 0.018
GRIPPER_CLOSED = 0.000
GRIPPER_RELAXED = 0.010


def load_duration(plan_file: Path) -> float:
    with plan_file.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    points = data.get("points") or []
    if not points:
        return 0.0
    return float(points[-1].get("time_from_start", 0.0) or 0.0)


class PhysicalInteractionController:
    """Drive only physical controls: gripper commands and Gazebo link joints.

    This controller intentionally never calls /world/*/set_pose. Props move only
    when Gazebo physics carries an attached link or when robot collisions contact
    fixed/dynamic objects.
    """

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.day = args.day_id.lower()
        self.duration = max(load_duration(Path(args.plan_file)), 1.0)
        self.log_file = Path(args.log_file)
        self.gripper_target_file = Path(args.gripper_target_file) if args.gripper_target_file else None
        self.fired: set[str] = set()

    def log(self, text: str) -> None:
        stamp = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        with self.log_file.open("a", encoding="utf-8") as handle:
            handle.write(f"{stamp} {text}\n")

    def set_gripper(self, position: float, reason: str) -> None:
        if self.gripper_target_file:
            self.gripper_target_file.write_text(f"{position:.3f}\n", encoding="utf-8")
        self.log(f"gripper target={position:.3f} reason={reason}")

    def gz_empty(self, topic: str, label: str) -> None:
        pids: list[int] = []
        for _ in range(3):
            proc = subprocess.Popen(
                ["gz", "topic", "-t", topic, "-m", "gz.msgs.Empty", "-p", "", "-d", "1.0"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            pids.append(proc.pid)
            time.sleep(0.08)
        self.log(f"gz_empty topic={topic} label={label} async_pids={','.join(str(pid) for pid in pids)}")

    def once(self, key: str, progress: float, fn) -> None:
        if key in self.fired:
            return
        if progress >= 0.0:
            self.fired.add(key)
            fn()

    def fire_at(self, key: str, progress: float, threshold: float, fn) -> None:
        if key not in self.fired and progress >= threshold:
            self.fired.add(key)
            fn()

    def initial_detach_all(self) -> None:
        day = self.day
        topics = []
        if day in {"day01", "d1"}:
            return
        if day in {"day04", "d4", "day05", "d5"}:
            topics.extend(
                [
                    "/rm65b/physical/d4_right_yarn/detach",
                    "/rm65b/physical/d4_right_probe/detach",
                ]
            )
        for topic in topics:
            self.gz_empty(topic, "initial_detach")

    def update(self, progress: float) -> None:
        day = self.day
        if day in {"day01", "d1"}:
            self.fire_at("d1_close_both", progress, 0.10, lambda: self.set_gripper(GRIPPER_CLOSED, "d1_gripper_close_both"))
            self.fire_at("d1_open_both", progress, 0.24, lambda: self.set_gripper(GRIPPER_OPEN, "d1_gripper_open_both"))
            self.fire_at(
                "d1_close_for_sync_motion",
                progress,
                0.52,
                lambda: self.set_gripper(GRIPPER_CLOSED, "d1_gripper_close_during_dual_motion"),
            )
            self.fire_at(
                "d1_final_open",
                progress,
                0.78,
                lambda: self.set_gripper(GRIPPER_OPEN, "d1_gripper_final_open"),
            )
        elif day in {"day02", "d2"}:
            self.fire_at("d2_close", progress, 0.01, lambda: self.set_gripper(GRIPPER_CLOSED, "d2_contact_stiff_grip"))
        elif day in {"day04", "d4"}:
            self.fire_at("d4_hook_close", progress, 0.18, lambda: self.set_gripper(GRIPPER_CLOSED, "d4_hook_yarn_close"))
            self.fire_at("d4_attach_yarn", progress, 0.24, lambda: self.gz_empty("/rm65b/physical/d4_right_yarn/attach", "d4_yarn_attach"))
            self.fire_at("d4_attach_probe", progress, 0.24, lambda: self.gz_empty("/rm65b/physical/d4_right_probe/attach", "d4_probe_attach"))
            self.fire_at("d4_lift_hold", progress, 0.36, lambda: self.set_gripper(GRIPPER_CLOSED, "d4_lift_yarn_hold"))
            self.fire_at("d4_pull_hold", progress, 0.58, lambda: self.set_gripper(GRIPPER_CLOSED, "d4_pull_tight_hold"))
            self.fire_at("d4_exchange_relax", progress, 0.82, lambda: self.set_gripper(GRIPPER_RELAXED, "d4_exchange_relax"))
            self.fire_at("d4_detach_probe", progress, 0.90, lambda: self.gz_empty("/rm65b/physical/d4_right_probe/detach", "d4_probe_release"))
            self.fire_at("d4_detach_yarn", progress, 0.90, lambda: self.gz_empty("/rm65b/physical/d4_right_yarn/detach", "d4_yarn_release"))
            self.fire_at("d4_open", progress, 0.93, lambda: self.set_gripper(GRIPPER_OPEN, "d4_release_open"))
        elif day in {"day05", "d5"}:
            self.fire_at("d5_force_close", progress, 0.30, lambda: self.set_gripper(GRIPPER_CLOSED, "d5_force_contact_close"))
            self.fire_at("d5_pick_close", progress, 0.45, lambda: self.set_gripper(GRIPPER_CLOSED, "d5_yarn_pick_close"))
            self.fire_at("d5_attach_yarn", progress, 0.49, lambda: self.gz_empty("/rm65b/physical/d4_right_yarn/attach", "d5_yarn_attach"))
            self.fire_at("d5_attach_probe", progress, 0.49, lambda: self.gz_empty("/rm65b/physical/d4_right_probe/attach", "d5_probe_attach"))
            self.fire_at("d5_detach_probe", progress, 0.86, lambda: self.gz_empty("/rm65b/physical/d4_right_probe/detach", "d5_probe_release"))
            self.fire_at("d5_detach_yarn", progress, 0.86, lambda: self.gz_empty("/rm65b/physical/d4_right_yarn/detach", "d5_yarn_release"))
            self.fire_at("d5_open", progress, 0.88, lambda: self.set_gripper(GRIPPER_OPEN, "d5_release_open"))
        else:
            self.fire_at("d3_open", progress, 0.02, lambda: self.set_gripper(GRIPPER_OPEN, "d3_vision_open"))

    def run(self) -> None:
        self.log("physical_interaction_controller started; dynamic set_pose disabled")
        if self.day in {"day02", "d2"}:
            self.set_gripper(GRIPPER_CLOSED, "initial_d2_closed")
        else:
            self.set_gripper(GRIPPER_OPEN, "initial_open")
        self.initial_detach_all()
        time.sleep(max(self.args.start_delay, 0.0))
        start = time.monotonic()
        period = 1.0 / max(self.args.rate_hz, 1.0)
        run_s = max(self.args.duration, self.duration)
        while time.monotonic() - start <= run_s:
            elapsed = time.monotonic() - start
            progress = min(max(elapsed / self.duration, 0.0), 1.0)
            self.update(progress)
            time.sleep(period)
        self.update(1.0)
        self.log("physical_interaction_controller complete")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan-file", required=True)
    parser.add_argument("--day-id", required=True)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--start-delay", type=float, default=0.25)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--gripper-target-file", default="")
    parser.add_argument("--log-file", required=True)
    return parser.parse_args()


def main() -> int:
    PhysicalInteractionController(parse_args()).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
