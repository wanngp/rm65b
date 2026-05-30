from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class GazeboCameraInfoPublisher(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_camera_info_publisher")
        self.declare_parameter("frame_id", "left_camera_color_optical_frame")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fx", 554.256)
        self.declare_parameter("fy", 554.256)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fx = float(self.get_parameter("fx").value)
        self.fy = float(self.get_parameter("fy").value)
        self.cx = float(self.get_parameter("cx").value)
        self.cy = float(self.get_parameter("cy").value)
        self.pub = self.create_publisher(CameraInfo, "/left_camera/camera_info", 10)
        self.create_timer(1.0 / 30.0, self._publish)

    def _publish(self) -> None:
        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.width = self.width
        msg.height = self.height
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [
            self.fx,
            0.0,
            self.cx,
            0.0,
            0.0,
            self.fy,
            self.cy,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboCameraInfoPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
