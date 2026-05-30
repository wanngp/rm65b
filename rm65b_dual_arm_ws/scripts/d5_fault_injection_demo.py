#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class FaultInjectionDemo(Node):
    def __init__(self, output_json: Path) -> None:
        super().__init__("d5_fault_injection_demo")
        self.output_json = output_json
        self.events: list[dict] = []
        self.estop_state: bool | None = None
        self.state_text = ""
        self.fault_pub = self.create_publisher(String, "/safety/inject_fault", 10)
        self.hardware_estop_pub = self.create_publisher(Bool, "/safety/hardware_estop", 10)
        self.create_subscription(Bool, "/safety/estop", self._estop_callback, 10)
        self.create_subscription(String, "/safety/state", self._state_callback, 10)
        self.create_subscription(String, "/safety/fault", self._fault_callback, 10)
        self.ack_client = self.create_client(Trigger, "/safety/ack_reset")
        self.reset_client = self.create_client(Trigger, "/safety/reset")
        self.recover_client = self.create_client(Trigger, "/safety/recover_home")

    def _record(self, event: str, **data) -> None:
        payload = {"event": event, "time_s": time.time(), **data}
        self.events.append(payload)
        self.output_json.parent.mkdir(parents=True, exist_ok=True)
        self.output_json.write_text(json.dumps(self.events, indent=2, sort_keys=True), encoding="utf-8")

    def _estop_callback(self, msg: Bool) -> None:
        self.estop_state = bool(msg.data)
        self._record("estop_topic", estop=self.estop_state)

    def _state_callback(self, msg: String) -> None:
        self.state_text = msg.data

    def _fault_callback(self, msg: String) -> None:
        self._record("fault_topic", payload=msg.data)

    def trigger_fault(self, fault: str) -> bool:
        if fault == "hardware_estop":
            self.hardware_estop_pub.publish(Bool(data=True))
        else:
            self.fault_pub.publish(String(data=fault))
        self._record("fault_command", fault=fault)
        deadline = time.time() + 5.0
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.estop_state:
                self._record("fault_triggered", fault=fault, state=self.state_text)
                return True
        self._record("fault_timeout", fault=fault, state=self.state_text)
        return False

    def call_trigger(self, client, name: str, timeout_s: float = 5.0) -> bool:
        if not client.wait_for_service(timeout_sec=timeout_s):
            self._record("service_unavailable", service=name)
            return False
        future = client.call_async(Trigger.Request())
        deadline = time.time() + timeout_s
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                response = future.result()
                self._record(
                    "service_response",
                    service=name,
                    success=bool(response.success),
                    message=response.message,
                )
                return bool(response.success)
        self._record("service_timeout", service=name)
        return False


def main() -> None:
    parser = argparse.ArgumentParser(description="Record D5 safety fault trigger and reset evidence.")
    parser.add_argument(
        "--fault",
        choices=["force_limit", "joint_limit", "collision", "hardware_estop", "manual_fault"],
        default="force_limit",
    )
    parser.add_argument("--output-json", type=Path, default=Path("outputs/d5_system/day05_fault_demo.json"))
    parser.add_argument("--no-reset", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = FaultInjectionDemo(args.output_json)
    try:
        triggered = node.trigger_fault(args.fault)
        if triggered and not args.no_reset:
            node.call_trigger(node.recover_client, "/safety/recover_home")
            node.call_trigger(node.ack_client, "/safety/ack_reset")
            node.call_trigger(node.reset_client, "/safety/reset")
        print(f"fault_triggered={triggered}; record={args.output_json}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
