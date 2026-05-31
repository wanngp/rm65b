#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Iterable

try:
    import cv2
    import numpy as np
except Exception:  # pragma: no cover - optional calibration dependency
    cv2 = None
    np = None


Matrix4 = list[list[float]]


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> list[list[float]]:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def pose_to_matrix(values: Iterable[float]) -> Matrix4:
    x, y, z, roll, pitch, yaw = [float(item) for item in values]
    r = rpy_to_matrix(roll, pitch, yaw)
    return [
        [r[0][0], r[0][1], r[0][2], x],
        [r[1][0], r[1][1], r[1][2], y],
        [r[2][0], r[2][1], r[2][2], z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def matmul(a: Matrix4, b: Matrix4) -> Matrix4:
    return [
        [sum(a[row][k] * b[k][col] for k in range(4)) for col in range(4)]
        for row in range(4)
    ]


def rigid_inverse(t: Matrix4) -> Matrix4:
    r_t = [[t[col][row] for col in range(3)] for row in range(3)]
    p = [t[row][3] for row in range(3)]
    inv_p = [-sum(r_t[row][col] * p[col] for col in range(3)) for row in range(3)]
    return [
        [r_t[0][0], r_t[0][1], r_t[0][2], inv_p[0]],
        [r_t[1][0], r_t[1][1], r_t[1][2], inv_p[1]],
        [r_t[2][0], r_t[2][1], r_t[2][2], inv_p[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def rounded_matrix(matrix: Matrix4, digits: int = 9) -> Matrix4:
    rounded: Matrix4 = []
    for row in matrix:
        rounded_row = []
        for value in row:
            item = round(float(value), digits)
            rounded_row.append(0.0 if abs(item) < 10 ** (-digits) else item)
        rounded.append(rounded_row)
    return rounded


def solve_with_opencv(samples_path: Path) -> Matrix4:
    if cv2 is None or np is None:
        raise RuntimeError("OpenCV and numpy are required for sample-based hand-eye solving")
    data = json.loads(samples_path.read_text(encoding="utf-8"))
    samples = data.get("samples") or []
    if len(samples) < 3:
        raise RuntimeError("At least three hand-eye samples are required")

    r_gripper2base = []
    t_gripper2base = []
    r_target2cam = []
    t_target2cam = []
    for sample in samples:
        t_base_gripper = np.array(sample["T_base_gripper"], dtype=float)
        t_camera_target = np.array(sample["T_camera_target"], dtype=float)
        r_gripper2base.append(t_base_gripper[:3, :3])
        t_gripper2base.append(t_base_gripper[:3, 3])
        r_target2cam.append(t_camera_target[:3, :3])
        t_target2cam.append(t_camera_target[:3, 3])

    r_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        r_gripper2base,
        t_gripper2base,
        r_target2cam,
        t_target2cam,
        method=cv2.CALIB_HAND_EYE_TSAI,
    )
    result = [
        [float(r_cam2gripper[0, 0]), float(r_cam2gripper[0, 1]), float(r_cam2gripper[0, 2]), float(t_cam2gripper[0])],
        [float(r_cam2gripper[1, 0]), float(r_cam2gripper[1, 1]), float(r_cam2gripper[1, 2]), float(t_cam2gripper[1])],
        [float(r_cam2gripper[2, 0]), float(r_cam2gripper[2, 1]), float(r_cam2gripper[2, 2]), float(t_cam2gripper[2])],
        [0.0, 0.0, 0.0, 1.0],
    ]
    return result


def parse_pose(text: str) -> list[float]:
    values = [float(item) for item in text.replace(",", " ").split()]
    if len(values) != 6:
        raise argparse.ArgumentTypeError("pose must contain x y z roll pitch yaw")
    return values


def main() -> int:
    parser = argparse.ArgumentParser(description="D3 hand-eye calibration matrix utility")
    parser.add_argument(
        "--gripper-to-camera-pose",
        type=parse_pose,
        default=parse_pose("0.064 0 0.045 0 0.35 0"),
        help="Configured Gazebo eye-in-hand sensor pose in gripper frame",
    )
    parser.add_argument(
        "--link6-to-gripper-pose",
        type=parse_pose,
        default=parse_pose("0.018 0 0 0 0 0"),
        help="Gripper palm pose in mechanical Link6 frame",
    )
    parser.add_argument(
        "--samples-json",
        type=Path,
        help="Optional real calibration samples with T_base_gripper and T_camera_target matrices",
    )
    parser.add_argument(
        "--arm-side",
        choices=("left", "right"),
        default="right",
        help="Arm/camera side for labeling the generated calibration result",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("outputs/d3_hand_eye/hand_eye_matrix.json"),
    )
    args = parser.parse_args()

    if args.samples_json:
        t_gripper_camera = solve_with_opencv(args.samples_json)
        source = f"opencv_calibrateHandEye:{args.samples_json}"
    else:
        t_gripper_camera = pose_to_matrix(args.gripper_to_camera_pose)
        source = "gazebo_configured_eye_in_hand_pose"

    t_link6_gripper = pose_to_matrix(args.link6_to_gripper_pose)
    t_link6_camera = matmul(t_link6_gripper, t_gripper_camera)
    t_camera_gripper = rigid_inverse(t_gripper_camera)

    result = {
        "source": source,
        "arm_side": args.arm_side,
        "camera_topic": f"/{args.arm_side}_camera/image_rect",
        "camera_info_topic": f"/{args.arm_side}_camera/camera_info",
        "marker": {
            "dictionary": "DICT_4X4_50",
            "id": 7,
            "size_m": 0.080,
        },
        "frames": {
            "T_gripper_camera": "camera pose expressed in gripper palm frame",
            "T_camera_gripper": "gripper palm pose expressed in camera frame",
            "T_link6_camera": "camera pose expressed in mechanical Link6 frame",
        },
        "T_gripper_camera": rounded_matrix(t_gripper_camera),
        "T_camera_gripper": rounded_matrix(t_camera_gripper),
        "T_link6_camera": rounded_matrix(t_link6_camera),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
