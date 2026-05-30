#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageSaver(Node):
    def __init__(self, topic: str, output_dir: Path, count: int) -> None:
        super().__init__("rm65b_image_saver")
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.count = count
        self.index = 0
        self.done = False
        self.create_subscription(Image, topic, self.callback, 10)
        self.get_logger().info(f"saving {count} frames from {topic} to {output_dir}")

    def callback(self, msg: Image) -> None:
        if self.index >= self.count:
            return
        rgb = self.to_rgb(msg)
        if rgb is None:
            return
        path = self.output_dir / f"camera_{self.index:04d}.ppm"
        with path.open("wb") as handle:
            handle.write(f"P6\n{msg.width} {msg.height}\n255\n".encode("ascii"))
            handle.write(rgb)
        self.index += 1
        if self.index >= self.count:
            self.get_logger().info("image capture complete")
            self.done = True

    def to_rgb(self, msg: Image) -> bytes | None:
        data = bytes(msg.data)
        pixels = msg.width * msg.height
        encoding = msg.encoding.lower()
        if encoding in {"rgb8", "r8g8b8"}:
            return data[: pixels * 3]
        if encoding in {"bgr8", "b8g8r8"}:
            out = bytearray(pixels * 3)
            for i in range(pixels):
                b, g, r = data[i * 3 : i * 3 + 3]
                out[i * 3 : i * 3 + 3] = bytes((r, g, b))
            return bytes(out)
        if encoding in {"rgba8", "r8g8b8a8"}:
            out = bytearray(pixels * 3)
            for i in range(pixels):
                r, g, b, _ = data[i * 4 : i * 4 + 4]
                out[i * 3 : i * 3 + 3] = bytes((r, g, b))
            return bytes(out)
        self.get_logger().warn(f"unsupported image encoding: {msg.encoding}")
        return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/rm65b/evidence_camera/image")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--count", type=int, default=120)
    args = parser.parse_args()

    rclpy.init()
    node = ImageSaver(args.topic, Path(args.output_dir), args.count)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
