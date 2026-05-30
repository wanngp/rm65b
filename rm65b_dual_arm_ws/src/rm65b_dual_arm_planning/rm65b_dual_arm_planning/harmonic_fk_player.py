from __future__ import annotations

import argparse
import csv
import math
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .rm65b_kinematics import (
    Rm65Kinematics,
    build_keyframes,
    load_weaving_yaml,
    matmul,
    matrix_to_pose,
    pose_matrix,
    sample_program,
    translation,
)

try:
    from rm_ros_interfaces.msg import Sixforce
except Exception:  # pragma: no cover - verified in ROS runtime
    Sixforce = None


LEFT_JOINTS = [f"left_joint{i}" for i in range(1, 7)]
RIGHT_JOINTS = [f"right_joint{i}" for i in range(1, 7)]
LINKS = ("base_link", "Link1", "Link2", "Link3", "Link4", "Link5", "Link6")


def _write_link_sdf(path: Path, mesh_uri: str) -> None:
    path.write_text(
        f"""<?xml version="1.0" ?>
<sdf version="1.10">
  <model name="rm65b_visual_link">
    <static>true</static>
    <link name="visual_link">
      <visual name="mesh_visual">
        <geometry>
          <mesh>
            <uri>file://{mesh_uri}</uri>
          </mesh>
        </geometry>
        <material>
          <ambient>0.92 0.94 0.95 1</ambient>
          <diffuse>0.92 0.94 0.95 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""",
        encoding="utf-8",
    )


def _write_gripper_sdf(path: Path) -> None:
    path.write_text(
        """<?xml version="1.0" ?>
<sdf version="1.10">
  <model name="rm65b_scaled_two_finger_gripper">
    <static>true</static>
    <link name="gripper_link">
      <visual name="flange_adapter">
        <pose>0 0 0 0 0 0</pose>
        <geometry><box><size>0.038 0.038 0.012</size></box></geometry>
        <material><ambient>0.16 0.22 0.28 1</ambient><diffuse>0.16 0.22 0.28 1</diffuse></material>
      </visual>
      <visual name="palm">
        <pose>0.030 0 0 0 0 0</pose>
        <geometry><box><size>0.040 0.026 0.018</size></box></geometry>
        <material><ambient>0.16 0.22 0.28 1</ambient><diffuse>0.16 0.22 0.28 1</diffuse></material>
      </visual>
      <visual name="upper_finger">
        <pose>0.070 0.018 0 0 0 0</pose>
        <geometry><box><size>0.052 0.006 0.010</size></box></geometry>
        <material><ambient>0.05 0.06 0.07 1</ambient><diffuse>0.05 0.06 0.07 1</diffuse></material>
      </visual>
      <visual name="lower_finger">
        <pose>0.070 -0.018 0 0 0 0</pose>
        <geometry><box><size>0.052 0.006 0.010</size></box></geometry>
        <material><ambient>0.05 0.06 0.07 1</ambient><diffuse>0.05 0.06 0.07 1</diffuse></material>
      </visual>
      <visual name="upper_yarn_hook">
        <pose>0.098 0.018 0 0 0 0</pose>
        <geometry><box><size>0.014 0.014 0.010</size></box></geometry>
        <material><ambient>0.05 0.06 0.07 1</ambient><diffuse>0.05 0.06 0.07 1</diffuse></material>
      </visual>
      <visual name="lower_yarn_hook">
        <pose>0.098 -0.018 0 0 0 0</pose>
        <geometry><box><size>0.014 0.014 0.010</size></box></geometry>
        <material><ambient>0.05 0.06 0.07 1</ambient><diffuse>0.05 0.06 0.07 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
""",
        encoding="utf-8",
    )


class HarmonicFkPlayer(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("rm65b_harmonic_fk_player")
        self.args = args
        self.models_dir = Path(args.models_dir)
        self.models_dir.mkdir(parents=True, exist_ok=True)
        self.pose_log = Path(args.pose_log)
        self.pose_log.parent.mkdir(parents=True, exist_ok=True)

        self.kin = Rm65Kinematics(args.urdf, args.mesh_root)
        self.keyframes = build_keyframes(load_weaving_yaml(args.trajectory_file or None))
        self.left_base = pose_matrix(args.left_x, args.left_y, args.left_z, args.left_yaw)
        self.right_base = pose_matrix(args.right_x, args.right_y, args.right_z, args.right_yaw)

        self.phase_pub = self.create_publisher(String, "/dual_arm_planning/phase", 10)
        self.event_pub = self.create_publisher(String, "/weaving/events", 10)
        self.status_pub = self.create_publisher(String, "/acceptance/day_status", 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.left_gripper_pub = self.create_publisher(
            JointState, "/left_gripper_controller/joint_states", 10
        )
        self.right_gripper_pub = self.create_publisher(
            JointState, "/right_gripper_controller/joint_states", 10
        )
        self.left_traj_pub = self.create_publisher(
            JointTrajectory, "/dual_arm_planning/left_joint_trajectory", 10
        )
        self.right_traj_pub = self.create_publisher(
            JointTrajectory, "/dual_arm_planning/right_joint_trajectory", 10
        )
        self.image_pub = self.create_publisher(Image, "/left_camera/image_rect", 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, "/left_camera/camera_info", 10)
        self.target_pub = self.create_publisher(PoseStamped, "/vision/target_pose", 10)
        self.twist_pub = self.create_publisher(TwistStamped, "/visual_servo/twist_cmd", 10)
        self.aligned_pub = self.create_publisher(Bool, "/visual_servo/aligned", 10)
        self.left_force_pub = None
        self.right_force_pub = None
        if Sixforce is not None:
            self.left_force_pub = self.create_publisher(
                Sixforce, "/left_rm_driver/rm_driver/udp_six_force", 10
            )
            self.right_force_pub = self.create_publisher(
                Sixforce, "/right_rm_driver/rm_driver/udp_six_force", 10
            )

    def prepare_models(self) -> None:
        for link in LINKS:
            visual = self.kin.visuals[link]
            _write_link_sdf(self.models_dir / f"{link}.sdf", visual.mesh)
        _write_gripper_sdf(self.models_dir / "scaled_gripper.sdf")

    def spawn_models(self) -> None:
        self.prepare_models()
        for side in ("left", "right"):
            for link in LINKS:
                self._spawn(f"{side}_rm65b_{link}", self.models_dir / f"{link}.sdf")
            self._spawn(f"{side}_rm65b_gripper", self.models_dir / "scaled_gripper.sdf")

    def run(self) -> None:
        self.spawn_models()
        self._publish_full_trajectories()
        start = time.monotonic()
        next_frame = start
        with self.pose_log.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["elapsed_s", "phase", "model", "x", "y", "z", "qx", "qy", "qz", "qw"])
            while rclpy.ok():
                now = time.monotonic()
                elapsed = now - start
                if elapsed > self.args.duration:
                    break
                if now < next_frame:
                    rclpy.spin_once(self, timeout_sec=min(next_frame - now, 0.05))
                    continue
                sample = sample_program(self.keyframes, elapsed, loop=False)
                self._publish_ros(sample, elapsed)
                self._set_all_poses(sample, writer)
                next_frame += 1.0 / max(self.args.rate, 0.5)
        self.get_logger().info("Harmonic FK dual-arm playback complete")

    def _spawn(self, name: str, sdf_file: Path) -> None:
        cmd = [
            "ros2",
            "run",
            "ros_gz_sim",
            "create",
            "-world",
            self.args.world,
            "-file",
            str(sdf_file),
            "-name",
            name,
        ]
        subprocess.run(cmd, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _set_all_poses(self, sample, writer: csv.writer) -> None:
        for side, base, joints in (
            ("left", self.left_base, sample.left),
            ("right", self.right_base, sample.right),
        ):
            fk = self.kin.fk(joints)
            for link in LINKS:
                if link == "base_link":
                    world_t = base
                else:
                    world_t = matmul(base, fk[link])
                self._set_pose(f"{side}_rm65b_{link}", world_t, sample.phase, writer)
            gripper_t = matmul(base, matmul(fk["Link6"], translation((0.018, 0.0, 0.0))))
            self._set_pose(f"{side}_rm65b_gripper", gripper_t, sample.phase, writer)
        self._set_named_xyz("weft_yarn", sample.yarn_xyz)
        self._set_named_xyz("vision_target", sample.target_xyz)

    def _set_named_xyz(self, name: str, xyz) -> None:
        req = (
            f"name: '{name}' position {{x: {xyz[0]:.6f} y: {xyz[1]:.6f} z: {xyz[2]:.6f}}} "
            "orientation {x: 0.000000 y: 0.000000 z: 0.000000 w: 1.000000}"
        )
        self._gz_service(req)

    def _set_pose(self, model_name: str, matrix, phase: str, writer: csv.writer) -> None:
        x, y, z, qx, qy, qz, qw = matrix_to_pose(matrix)
        req = (
            f"name: '{model_name}' position {{x: {x:.6f} y: {y:.6f} z: {z:.6f}}} "
            f"orientation {{x: {qx:.6f} y: {qy:.6f} z: {qz:.6f} w: {qw:.6f}}}"
        )
        self._gz_service(req)
        writer.writerow([f"{self.get_clock().now().nanoseconds / 1e9:.3f}", phase, model_name, x, y, z, qx, qy, qz, qw])

    def _gz_service(self, req: str) -> None:
        cmd = [
            "gz",
            "service",
            "-s",
            f"/world/{self.args.world}/set_pose",
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            str(self.args.gz_timeout_ms),
            "--req",
            req,
        ]
        subprocess.run(cmd, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _publish_full_trajectories(self) -> None:
        self.left_traj_pub.publish(self._trajectory("left"))
        self.right_traj_pub.publish(self._trajectory("right"))

    def _trajectory(self, side: str) -> JointTrajectory:
        msg = JointTrajectory()
        msg.joint_names = LEFT_JOINTS if side == "left" else RIGHT_JOINTS
        for frame in self.keyframes:
            point = JointTrajectoryPoint()
            point.positions = list(frame.left if side == "left" else frame.right)
            point.time_from_start.sec = int(frame.time_s)
            point.time_from_start.nanosec = int((frame.time_s - int(frame.time_s)) * 1e9)
            msg.points.append(point)
        return msg

    def _publish_ros(self, sample, elapsed: float) -> None:
        stamp = self.get_clock().now().to_msg()
        joint = JointState()
        joint.header.stamp = stamp
        joint.name = LEFT_JOINTS + RIGHT_JOINTS
        joint.position = list(sample.left) + list(sample.right)
        self.joint_pub.publish(joint)

        for pub, name, position, effort in (
            (self.left_gripper_pub, "left_finger_joint", sample.left_gripper, 18.0),
            (self.right_gripper_pub, "right_finger_joint", sample.right_gripper, 20.0),
        ):
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = [name]
            msg.position = [float(position)]
            msg.effort = [effort]
            pub.publish(msg)

        self.phase_pub.publish(String(data=sample.phase))
        self.event_pub.publish(
            String(data=f"planned_phase:{sample.phase}:bases_fixed:true:scaled_gripper:true")
        )
        self.status_pub.publish(String(data="day05 integrated_demo active: harmonic_fk_planning"))
        self._publish_camera(stamp, elapsed)
        self._publish_vision(stamp, sample)
        self._publish_force(elapsed)

    def _publish_camera(self, stamp, elapsed: float) -> None:
        width = 320
        height = 240
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = "left_camera_color_optical_frame"
        info.width = width
        info.height = height
        info.k = [250.0, 0.0, width / 2, 0.0, 250.0, height / 2, 0.0, 0.0, 1.0]
        info.p = [250.0, 0.0, width / 2, 0.0, 0.0, 250.0, height / 2, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_info_pub.publish(info)

        u = int(width / 2 + 34 * math.sin(elapsed * 0.45))
        v = int(height / 2 + 20 * math.cos(elapsed * 0.45))
        pixels = bytearray(width * height)
        for y in range(height):
            row = y * width
            for x in range(width):
                value = 44
                if abs(x - u) < 22 and abs(y - v) < 22:
                    value = 225
                elif (x // 18 + y // 18) % 2 == 0:
                    value = 68
                pixels[row + x] = value
        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = info.header.frame_id
        img.height = height
        img.width = width
        img.encoding = "mono8"
        img.is_bigendian = 0
        img.step = width
        img.data = bytes(pixels)
        self.image_pub.publish(img)

    def _publish_vision(self, stamp, sample) -> None:
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "left_camera_color_optical_frame"
        pose.pose.position.x = 0.25
        pose.pose.position.y = sample.target_xyz[1] - 0.20
        pose.pose.position.z = sample.target_xyz[2] - 0.36
        pose.pose.orientation.w = 1.0
        self.target_pub.publish(pose)

        twist = TwistStamped()
        twist.header.stamp = stamp
        twist.header.frame_id = pose.header.frame_id
        twist.twist.linear.y = max(-0.03, min(0.03, -0.8 * pose.pose.position.y))
        twist.twist.linear.z = max(-0.03, min(0.03, -0.6 * pose.pose.position.z))
        self.twist_pub.publish(twist)
        self.aligned_pub.publish(
            Bool(data=abs(pose.pose.position.y) < 0.003 and abs(pose.pose.position.z) < 0.003)
        )

    def _publish_force(self, elapsed: float) -> None:
        if Sixforce is None or self.left_force_pub is None or self.right_force_pub is None:
            return
        tension = 7.0 + 1.8 * math.sin(elapsed * 0.55)
        left = Sixforce()
        left.force_fx = 0.4 * math.sin(elapsed)
        left.force_fy = 1.1
        left.force_fz = float(tension)
        left.force_mx = 0.07
        left.force_my = 0.04
        left.force_mz = 0.02
        right = Sixforce()
        right.force_fx = -0.3 * math.sin(elapsed)
        right.force_fy = -1.0
        right.force_fz = float(tension + 0.6)
        right.force_mx = -0.06
        right.force_my = 0.04
        right.force_mz = -0.02
        self.left_force_pub.publish(left)
        self.right_force_pub.publish(right)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--mesh-root", required=True)
    parser.add_argument("--trajectory-file", default="")
    parser.add_argument("--models-dir", required=True)
    parser.add_argument("--pose-log", required=True)
    parser.add_argument("--world", default="rm65b_world")
    parser.add_argument("--duration", type=float, default=24.0)
    parser.add_argument("--rate", type=float, default=1.5)
    parser.add_argument("--gz-timeout-ms", type=int, default=2000)
    parser.add_argument("--left-x", type=float, default=-0.45)
    parser.add_argument("--left-y", type=float, default=0.0)
    parser.add_argument("--left-z", type=float, default=0.02)
    parser.add_argument("--left-yaw", type=float, default=1.5708)
    parser.add_argument("--right-x", type=float, default=0.45)
    parser.add_argument("--right-y", type=float, default=0.0)
    parser.add_argument("--right-z", type=float, default=0.02)
    parser.add_argument("--right-yaw", type=float, default=-1.5708)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = HarmonicFkPlayer(args)
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
