#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

try:
    import cv2
except Exception:  # pragma: no cover - optional runtime dependency
    cv2 = None

try:
    import numpy as np
except Exception:  # pragma: no cover - runtime dependency from ROS/python3-numpy
    np = None


@dataclass(frozen=True)
class Detection:
    u: float
    v: float
    method: str
    area_px: int
    score: float
    threshold: float


class ArucoTargetNode(Node):
    def __init__(self) -> None:
        super().__init__("aruco_target_node")
        self.declare_parameter("image_topic", "/left_camera/image_rect")
        self.declare_parameter("camera_info_topic", "/left_camera/camera_info")
        self.declare_parameter("target_frame", "left_camera_color_optical_frame")
        self.declare_parameter("target_pose_topic", "/vision/target_pose")
        self.declare_parameter("status_topic", "/vision/status")
        self.declare_parameter("metrics_topic", "/vision/metrics")
        self.declare_parameter("debug_image_topic", "/vision/debug_image")
        self.declare_parameter("tag_size_m", 0.035)
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("target_depth_m", 0.25)
        self.declare_parameter("min_target_area_px", 80)
        self.declare_parameter("max_target_area_fraction", 0.35)
        self.declare_parameter("min_target_contrast", 18.0)
        self.declare_parameter("fallback_publish_center", False)

        self.camera_info: CameraInfo | None = None
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.target_depth_m = float(self.get_parameter("target_depth_m").value)
        self.min_area_px = int(self.get_parameter("min_target_area_px").value)
        self.max_area_fraction = float(self.get_parameter("max_target_area_fraction").value)
        self.min_contrast = float(self.get_parameter("min_target_contrast").value)
        self._last_frame_time: float | None = None
        self._fps_ema = 0.0

        self.pose_pub = self.create_publisher(
            PoseStamped, self.get_parameter("target_pose_topic").value, 10
        )
        self.status_pub = self.create_publisher(
            String, self.get_parameter("status_topic").value, 10
        )
        self.metrics_pub = self.create_publisher(
            String, self.get_parameter("metrics_topic").value, 10
        )
        self.debug_image_pub = self.create_publisher(
            Image, self.get_parameter("debug_image_topic").value, 10
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._camera_info_callback,
            10,
        )
        self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self._image_callback,
            10,
        )

        if np is None:
            self.get_logger().error("numpy unavailable; image detection cannot run")
        elif cv2 is None:
            self.get_logger().warn(
                "OpenCV unavailable; using raw image salient-target detector only"
            )
        else:
            self.get_logger().info("Vision target node ready: ArUco plus salient target detector")

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def _image_callback(self, msg: Image) -> None:
        fps = self._update_fps()
        gray, decode_error = self._image_to_gray(msg)
        if gray is None:
            self._publish_no_target(msg, fps, decode_error)
            return

        detection = self._detect_aruco(gray)
        if detection is None:
            detection = self._detect_salient_target(gray)

        if detection is None:
            self._publish_debug_image(msg, gray, None, fps, "no target")
            self._publish_no_target(msg, fps, "no target")
            return

        pose = self._pixel_to_pose(msg, detection.u, detection.v)
        self.pose_pub.publish(pose)
        self._publish_debug_image(msg, gray, detection, fps, "")
        self._publish_detection(msg, detection, fps)

    def _update_fps(self) -> float:
        now = time.monotonic()
        if self._last_frame_time is None:
            self._last_frame_time = now
            return 0.0

        dt = max(now - self._last_frame_time, 1.0e-6)
        self._last_frame_time = now
        fps = 1.0 / dt
        self._fps_ema = fps if self._fps_ema <= 0.0 else 0.85 * self._fps_ema + 0.15 * fps
        return self._fps_ema

    def _image_to_gray(self, msg: Image):
        if np is None:
            return None, "numpy unavailable"
        if msg.height <= 0 or msg.width <= 0:
            return None, "empty image"

        encoding = (msg.encoding or "").lower()
        height = int(msg.height)
        width = int(msg.width)
        step = int(msg.step) if msg.step else 0
        data = np.frombuffer(msg.data, dtype=np.uint8)

        try:
            if encoding in ("mono8", "8uc1"):
                step = step or width
                rows = self._rows_from_image_buffer(data, height, step)
                return rows[:, :width].copy(), ""

            if encoding in ("rgb8", "bgr8", "8uc3"):
                step = step or width * 3
                rows = self._rows_from_image_buffer(data, height, step)
                image = rows[:, : width * 3].reshape((height, width, 3))
                return self._rgb_like_to_gray(image, encoding), ""

            if encoding in ("rgba8", "bgra8", "8uc4"):
                step = step or width * 4
                rows = self._rows_from_image_buffer(data, height, step)
                image = rows[:, : width * 4].reshape((height, width, 4))
                return self._rgb_like_to_gray(image[:, :, :3], encoding), ""

            return None, f"unsupported encoding:{msg.encoding}"
        except Exception as exc:
            return None, f"decode error:{exc}"

    def _rows_from_image_buffer(self, data, height: int, step: int):
        required = height * step
        if data.size < required:
            raise ValueError(f"buffer too small {data.size} < {required}")
        return data[:required].reshape((height, step))

    def _rgb_like_to_gray(self, image, encoding: str):
        if encoding.startswith("bgr") or encoding == "8uc3" or encoding == "8uc4":
            blue = image[:, :, 0].astype(np.float32)
            green = image[:, :, 1].astype(np.float32)
            red = image[:, :, 2].astype(np.float32)
        else:
            red = image[:, :, 0].astype(np.float32)
            green = image[:, :, 1].astype(np.float32)
            blue = image[:, :, 2].astype(np.float32)
        return np.clip(0.299 * red + 0.587 * green + 0.114 * blue, 0, 255).astype(np.uint8)

    def _detect_aruco(self, gray) -> Detection | None:
        if cv2 is None or not hasattr(cv2, "aruco"):
            return None

        try:
            dictionary_name = str(self.get_parameter("dictionary").value)
            dictionary_id = getattr(cv2.aruco, dictionary_name)
            dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
            if hasattr(cv2.aruco, "ArucoDetector"):
                detector = cv2.aruco.ArucoDetector(dictionary)
                corners, ids, _ = detector.detectMarkers(gray)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
        except Exception as exc:
            self.status_pub.publish(String(data=f"aruco_error:{exc}"))
            return None

        if ids is None or len(corners) == 0:
            return None

        first = corners[0].reshape(-1, 2)
        area = max(1.0, float(cv2.contourArea(first.astype("float32")))) if cv2 is not None else 1.0
        return Detection(
            u=float(first[:, 0].mean()),
            v=float(first[:, 1].mean()),
            method="aruco",
            area_px=int(area),
            score=float(area),
            threshold=0.0,
        )

    def _detect_salient_target(self, gray) -> Detection | None:
        height, width = gray.shape
        total_area = height * width
        if total_area <= 0:
            return None

        gray_f = gray.astype(np.float32)
        p05, p50, p95 = np.percentile(gray_f, [5, 50, 95])
        min_contrast = max(1.0, self.min_contrast)
        max_area = max(self.min_area_px, int(total_area * self.max_area_fraction))

        dark_threshold = min(p50 - min_contrast, p05 + 0.5 * max(0.0, p50 - p05))
        bright_threshold = max(p50 + min_contrast, p95 - 0.5 * max(0.0, p95 - p50))

        dark_candidates: list[Detection] = []
        bright_candidates: list[Detection] = []
        if p50 - dark_threshold >= min_contrast * 0.5:
            dark_candidates.extend(
                self._mask_candidates(
                    gray_f <= dark_threshold,
                    gray_f,
                    "salient_dark",
                    p50,
                    dark_threshold,
                    max_area,
                )
            )
        if bright_threshold - p50 >= min_contrast * 0.5:
            bright_candidates.extend(
                self._mask_candidates(
                    gray_f >= bright_threshold,
                    gray_f,
                    "salient_bright",
                    p50,
                    bright_threshold,
                    max_area,
                )
            )

        # The D3/D5 vision target is a bright center mark on a darker board.
        # Prefer the deliberate bright target over the larger dark backing plate.
        candidates = bright_candidates or dark_candidates
        return max(candidates, key=lambda item: item.score) if candidates else None

    def _mask_candidates(
        self,
        mask,
        gray_f,
        method: str,
        background_level: float,
        threshold: float,
        max_area: int,
    ) -> list[Detection]:
        mask_u8 = mask.astype(np.uint8)
        if cv2 is not None:
            kernel = np.ones((3, 3), dtype=np.uint8)
            mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_OPEN, kernel)
            mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE, kernel)
            return self._cv2_components(mask_u8, gray_f, method, background_level, threshold, max_area)
        return self._numpy_components(mask_u8.astype(bool), gray_f, method, background_level, threshold, max_area)

    def _cv2_components(
        self,
        mask_u8,
        gray_f,
        method: str,
        background_level: float,
        threshold: float,
        max_area: int,
    ) -> list[Detection]:
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_u8, 8)
        detections: list[Detection] = []
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            width = int(stats[label, cv2.CC_STAT_WIDTH])
            height = int(stats[label, cv2.CC_STAT_HEIGHT])
            if not self._valid_component(area, width, height, max_area):
                continue
            pixels = gray_f[labels == label]
            score = self._component_score(pixels, background_level, area, method)
            if score > 0.0:
                detections.append(
                    Detection(
                        u=float(centroids[label][0]),
                        v=float(centroids[label][1]),
                        method=method,
                        area_px=area,
                        score=score,
                        threshold=float(threshold),
                    )
                )
        return detections

    def _numpy_components(
        self,
        mask,
        gray_f,
        method: str,
        background_level: float,
        threshold: float,
        max_area: int,
    ) -> list[Detection]:
        height, width = mask.shape
        visited = np.zeros_like(mask, dtype=bool)
        detections: list[Detection] = []
        ys, xs = np.nonzero(mask)
        for start_y, start_x in zip(ys.tolist(), xs.tolist()):
            if visited[start_y, start_x]:
                continue
            stack = [(start_y, start_x)]
            visited[start_y, start_x] = True
            comp_x: list[int] = []
            comp_y: list[int] = []
            while stack:
                y, x = stack.pop()
                comp_x.append(x)
                comp_y.append(y)
                for ny in (y - 1, y, y + 1):
                    for nx in (x - 1, x, x + 1):
                        if (
                            0 <= ny < height
                            and 0 <= nx < width
                            and not visited[ny, nx]
                            and mask[ny, nx]
                        ):
                            visited[ny, nx] = True
                            stack.append((ny, nx))

            area = len(comp_x)
            comp_width = max(comp_x) - min(comp_x) + 1
            comp_height = max(comp_y) - min(comp_y) + 1
            if not self._valid_component(area, comp_width, comp_height, max_area):
                continue
            pixels = gray_f[comp_y, comp_x]
            score = self._component_score(pixels, background_level, area, method)
            if score > 0.0:
                detections.append(
                    Detection(
                        u=float(sum(comp_x) / area),
                        v=float(sum(comp_y) / area),
                        method=method,
                        area_px=area,
                        score=score,
                        threshold=float(threshold),
                    )
                )
        return detections

    def _valid_component(self, area: int, width: int, height: int, max_area: int) -> bool:
        if area < self.min_area_px or area > max_area:
            return False
        if width < 4 or height < 4:
            return False
        aspect = max(width / max(height, 1), height / max(width, 1))
        return aspect <= 8.0

    def _component_score(self, pixels, background_level: float, area: int, method: str) -> float:
        mean_value = float(np.mean(pixels)) if len(pixels) else background_level
        if method.endswith("dark"):
            contrast = background_level - mean_value
        else:
            contrast = mean_value - background_level
        if contrast < self.min_contrast * 0.5:
            return 0.0
        return float(contrast * math.sqrt(max(area, 1)))

    def _pixel_to_pose(self, msg: Image, u: float, v: float) -> PoseStamped:
        fx = self.camera_info.k[0] if self.camera_info else 600.0
        fy = self.camera_info.k[4] if self.camera_info else 600.0
        cx = self.camera_info.k[2] if self.camera_info else msg.width / 2.0
        cy = self.camera_info.k[5] if self.camera_info else msg.height / 2.0
        depth = self.target_depth_m

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.target_frame
        pose.pose.position.x = depth
        pose.pose.position.y = -(u - cx) * depth / fx
        pose.pose.position.z = -(v - cy) * depth / fy
        pose.pose.orientation.w = 1.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        if not all(
            math.isfinite(value)
            for value in (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        ):
            pose.pose.position.x = depth
            pose.pose.position.y = 0.0
            pose.pose.position.z = 0.0
        return pose

    def _publish_debug_image(
        self, msg: Image, gray, detection: Detection | None, fps: float, reason: str
    ) -> None:
        if np is None:
            return
        height, width = gray.shape
        if cv2 is not None:
            debug = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            center = (width // 2, height // 2)
            cv2.drawMarker(
                debug,
                center,
                (255, 180, 0),
                markerType=cv2.MARKER_CROSS,
                markerSize=24,
                thickness=2,
            )
            cv2.line(debug, (center[0] - 32, center[1]), (center[0] + 32, center[1]), (255, 180, 0), 1)
            cv2.line(debug, (center[0], center[1] - 32), (center[0], center[1] + 32), (255, 180, 0), 1)
            if detection is not None:
                target = (int(round(detection.u)), int(round(detection.v)))
                cv2.circle(debug, target, 16, (0, 255, 0), 2)
                cv2.drawMarker(
                    debug,
                    target,
                    (0, 255, 0),
                    markerType=cv2.MARKER_TILTED_CROSS,
                    markerSize=20,
                    thickness=2,
                )
                cv2.line(debug, center, target, (0, 220, 255), 2)
                error_x = detection.u - center[0]
                error_y = detection.v - center[1]
                lines = [
                    f"TARGET {detection.method}",
                    f"u={detection.u:.1f} v={detection.v:.1f}",
                    f"du={error_x:.1f}px dv={error_y:.1f}px fps={fps:.1f}",
                    f"area={detection.area_px} score={detection.score:.1f}",
                ]
                color = (0, 255, 0)
            else:
                lines = [f"NO TARGET {reason}", f"fps={fps:.1f}"]
                color = (0, 0, 255)
            for idx, line in enumerate(lines):
                cv2.putText(
                    debug,
                    line,
                    (12, 24 + idx * 22),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    color,
                    2,
                    cv2.LINE_AA,
                )
        else:
            debug = np.repeat(gray[:, :, None], 3, axis=2)
            self._draw_numpy_cross(debug, width // 2, height // 2, (255, 180, 0), 12)
            if detection is not None:
                self._draw_numpy_cross(
                    debug,
                    int(round(detection.u)),
                    int(round(detection.v)),
                    (0, 255, 0),
                    10,
                )

        out = Image()
        out.header = msg.header
        out.height = int(height)
        out.width = int(width)
        out.encoding = "bgr8"
        out.is_bigendian = 0
        out.step = int(width * 3)
        out.data = debug.astype(np.uint8).tobytes()
        self.debug_image_pub.publish(out)

    def _draw_numpy_cross(self, image, x: int, y: int, color: tuple[int, int, int], radius: int) -> None:
        height, width, _ = image.shape
        x0 = max(0, x - radius)
        x1 = min(width, x + radius + 1)
        y0 = max(0, y - radius)
        y1 = min(height, y + radius + 1)
        if 0 <= y < height:
            image[y, x0:x1, :] = color
        if 0 <= x < width:
            image[y0:y1, x, :] = color

    def _publish_detection(self, msg: Image, detection: Detection, fps: float) -> None:
        cx = self.camera_info.k[2] if self.camera_info else msg.width / 2.0
        cy = self.camera_info.k[5] if self.camera_info else msg.height / 2.0
        error_x = detection.u - cx
        error_y = detection.v - cy
        self.status_pub.publish(
            String(
                data=(
                    f"target method={detection.method} "
                    f"u={detection.u:.1f} v={detection.v:.1f} "
                    f"pixel_error_x={error_x:.1f} pixel_error_y={error_y:.1f} "
                    f"fps={fps:.2f}"
                )
            )
        )
        self.metrics_pub.publish(
            String(
                data=json.dumps(
                    {
                        "target_found": True,
                        "method": detection.method,
                        "pixel_error_x": error_x,
                        "pixel_error_y": error_y,
                        "fps": fps,
                        "area_px": detection.area_px,
                        "score": detection.score,
                        "threshold": detection.threshold,
                        "image_width": int(msg.width),
                        "image_height": int(msg.height),
                    },
                    separators=(",", ":"),
                )
            )
        )

    def _publish_no_target(self, msg: Image, fps: float, reason: str) -> None:
        self.status_pub.publish(String(data=f"no_target reason={reason} fps={fps:.2f}"))
        self.metrics_pub.publish(
            String(
                data=json.dumps(
                    {
                        "target_found": False,
                        "reason": reason,
                        "fps": fps,
                        "image_width": int(msg.width),
                        "image_height": int(msg.height),
                    },
                    separators=(",", ":"),
                )
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoTargetNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
