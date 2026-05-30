from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

try:
    import yaml
except Exception:  # pragma: no cover - exercised in ROS runtime
    yaml = None


Matrix = list[list[float]]


@dataclass(frozen=True)
class JointSpec:
    name: str
    parent: str
    child: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]
    axis: tuple[float, float, float]


@dataclass(frozen=True)
class LinkVisualSpec:
    link: str
    mesh: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]


@dataclass(frozen=True)
class Keyframe:
    name: str
    time_s: float
    left: tuple[float, ...]
    right: tuple[float, ...]
    left_gripper: float
    right_gripper: float
    yarn_xyz: tuple[float, float, float]
    target_xyz: tuple[float, float, float]


@dataclass(frozen=True)
class ProgramSample:
    phase: str
    elapsed_s: float
    left: tuple[float, ...]
    right: tuple[float, ...]
    left_gripper: float
    right_gripper: float
    yarn_xyz: tuple[float, float, float]
    target_xyz: tuple[float, float, float]


def _floats(text: str, count: int) -> tuple[float, ...]:
    values = tuple(float(x) for x in text.split())
    if len(values) != count:
        raise ValueError(f"expected {count} floats, got {text!r}")
    return values


def identity() -> Matrix:
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def matmul(a: Matrix, b: Matrix) -> Matrix:
    out = [[0.0 for _ in range(4)] for _ in range(4)]
    for r in range(4):
        for c in range(4):
            out[r][c] = sum(a[r][k] * b[k][c] for k in range(4))
    return out


def translation(xyz: Iterable[float]) -> Matrix:
    x, y, z = xyz
    m = identity()
    m[0][3] = float(x)
    m[1][3] = float(y)
    m[2][3] = float(z)
    return m


def rpy_matrix(rpy: Iterable[float]) -> Matrix:
    roll, pitch, yaw = (float(v) for v in rpy)
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, 0.0],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, 0.0],
        [-sp, cp * sr, cp * cr, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def transform_from_xyz_rpy(xyz: Iterable[float], rpy: Iterable[float]) -> Matrix:
    return matmul(translation(xyz), rpy_matrix(rpy))


def axis_angle(axis: Iterable[float], angle: float) -> Matrix:
    x, y, z = (float(v) for v in axis)
    norm = math.sqrt(x * x + y * y + z * z)
    if norm <= 1e-12:
        return identity()
    x, y, z = x / norm, y / norm, z / norm
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1.0 - c
    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0.0],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0.0],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def pose_matrix(x: float, y: float, z: float, yaw: float) -> Matrix:
    return transform_from_xyz_rpy((x, y, z), (0.0, 0.0, yaw))


def matrix_to_pose(m: Matrix) -> tuple[float, float, float, float, float, float, float]:
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m[2][1] - m[1][2]) / s
        qy = (m[0][2] - m[2][0]) / s
        qz = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        qw = (m[2][1] - m[1][2]) / s
        qx = 0.25 * s
        qy = (m[0][1] + m[1][0]) / s
        qz = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        qw = (m[0][2] - m[2][0]) / s
        qx = (m[0][1] + m[1][0]) / s
        qy = 0.25 * s
        qz = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        qw = (m[1][0] - m[0][1]) / s
        qx = (m[0][2] + m[2][0]) / s
        qy = (m[1][2] + m[2][1]) / s
        qz = 0.25 * s
    return (m[0][3], m[1][3], m[2][3], qx, qy, qz, qw)


def lerp(a: Iterable[float], b: Iterable[float], alpha: float) -> tuple[float, ...]:
    return tuple(float(x) + (float(y) - float(x)) * alpha for x, y in zip(a, b))


class Rm65Kinematics:
    link_order = ("base_link", "Link1", "Link2", "Link3", "Link4", "Link5", "Link6")

    def __init__(self, urdf_file: str | Path, mesh_root: str | Path | None = None) -> None:
        self.urdf_file = Path(urdf_file)
        self.mesh_root = Path(mesh_root) if mesh_root else None
        root = ET.parse(self.urdf_file).getroot()
        self.visuals = self._parse_visuals(root)
        self.joints = self._parse_chain(root)

    def _parse_visuals(self, root: ET.Element) -> dict[str, LinkVisualSpec]:
        visuals: dict[str, LinkVisualSpec] = {}
        for link in root.findall("link"):
            name = link.get("name", "")
            visual = link.find("visual")
            if visual is None:
                continue
            origin = visual.find("origin")
            xyz = _floats(origin.get("xyz", "0 0 0"), 3) if origin is not None else (0.0, 0.0, 0.0)
            rpy = _floats(origin.get("rpy", "0 0 0"), 3) if origin is not None else (0.0, 0.0, 0.0)
            mesh = visual.find("geometry/mesh")
            filename = mesh.get("filename", "") if mesh is not None else ""
            if filename.startswith("package://rm_description/meshes") and self.mesh_root:
                filename = filename.replace(
                    "package://rm_description/meshes",
                    self.mesh_root.as_posix(),
                    1,
                )
            visuals[name] = LinkVisualSpec(name, filename, xyz, rpy)
        return visuals

    def _parse_chain(self, root: ET.Element) -> list[JointSpec]:
        joints_by_parent: dict[str, list[JointSpec]] = {}
        for joint in root.findall("joint"):
            if joint.get("type") not in {"revolute", "continuous"}:
                continue
            origin = joint.find("origin")
            parent = joint.find("parent")
            child = joint.find("child")
            axis = joint.find("axis")
            spec = JointSpec(
                name=joint.get("name", ""),
                parent=parent.get("link", "") if parent is not None else "",
                child=child.get("link", "") if child is not None else "",
                xyz=_floats(origin.get("xyz", "0 0 0"), 3) if origin is not None else (0.0, 0.0, 0.0),
                rpy=_floats(origin.get("rpy", "0 0 0"), 3) if origin is not None else (0.0, 0.0, 0.0),
                axis=_floats(axis.get("xyz", "0 0 1"), 3) if axis is not None else (0.0, 0.0, 1.0),
            )
            joints_by_parent.setdefault(spec.parent, []).append(spec)

        chain: list[JointSpec] = []
        parent = "base_link"
        while parent != "Link6":
            candidates = joints_by_parent.get(parent, [])
            if not candidates:
                raise ValueError(f"cannot continue RM65-B chain from {parent}")
            spec = candidates[0]
            chain.append(spec)
            parent = spec.child
        if len(chain) != 6:
            raise ValueError(f"expected 6 RM65-B joints, got {len(chain)}")
        return chain

    def fk(self, positions: Iterable[float]) -> dict[str, Matrix]:
        values = tuple(float(v) for v in positions)
        if len(values) != 6:
            raise ValueError("RM65-B FK requires 6 joint positions")
        transforms: dict[str, Matrix] = {"base_link": identity()}
        for spec, position in zip(self.joints, values):
            parent = transforms[spec.parent]
            fixed = transform_from_xyz_rpy(spec.xyz, spec.rpy)
            motion = axis_angle(spec.axis, position)
            transforms[spec.child] = matmul(parent, matmul(fixed, motion))
        return transforms


def default_trajectory_file() -> Path | None:
    try:
        from ament_index_python.packages import get_package_share_directory

        return (
            Path(get_package_share_directory("rm65b_weaving_primitives"))
            / "trajectories"
            / "weaving_primitives.yaml"
        )
    except Exception:
        return None


def load_weaving_yaml(path: str | Path | None = None) -> dict:
    if yaml is None:
        return {}
    candidate = Path(path) if path else default_trajectory_file()
    if candidate and candidate.exists():
        with candidate.open("r", encoding="utf-8") as handle:
            return yaml.safe_load(handle) or {}
    return {}


def _positions(config: dict, primitive: str, side: str, index: int) -> tuple[float, ...]:
    try:
        steps = config["primitives"][primitive][side]
        return tuple(float(v) for v in steps[index]["positions"])
    except Exception:
        fallback = {
            "left": (0.04, -0.50, 0.60, 0.06, 0.78, 0.10),
            "right": (-0.06, -0.40, 0.52, -0.05, 0.88, -0.12),
        }
        return fallback[side]


def build_keyframes(config: dict | None = None) -> list[Keyframe]:
    cfg = config or {}
    home_left = _positions(cfg, "hook_yarn", "left", 0)
    home_right = _positions(cfg, "hook_yarn", "right", 0)
    hook_left = _positions(cfg, "hook_yarn", "left", -1)
    hook_right = _positions(cfg, "hook_yarn", "right", -1)
    lift_left = _positions(cfg, "lift_yarn", "left", -1)
    lift_right = _positions(cfg, "lift_yarn", "right", -1)
    pick_left = _positions(cfg, "pick_yarn", "left", -1)
    pull_right = _positions(cfg, "pull_tight", "right", -1)
    shift_left = _positions(cfg, "shift", "left", -1)
    shift_right = _positions(cfg, "shift", "right", -1)
    exchange_left = _positions(cfg, "exchange", "left", -1)
    exchange_right = _positions(cfg, "exchange", "right", -1)
    return [
        Keyframe("home", 0.0, home_left, home_right, 0.032, 0.032, (0.00, 0.43, 0.34), (-0.08, 0.18, 0.34)),
        Keyframe("hook_yarn", 4.0, hook_left, hook_right, 0.022, 0.024, (-0.12, 0.44, 0.34), (-0.05, 0.19, 0.35)),
        Keyframe("lift_yarn", 8.0, lift_left, lift_right, 0.006, 0.020, (-0.04, 0.48, 0.36), (-0.01, 0.20, 0.36)),
        Keyframe("pick_yarn", 8.5, pick_left, lift_right, 0.006, 0.020, (-0.04, 0.48, 0.36), (-0.01, 0.20, 0.36)),
        Keyframe("pull_tight", 13.0, lift_left, pull_right, 0.006, 0.008, (0.08, 0.54, 0.37), (0.04, 0.21, 0.36)),
        Keyframe("shift", 18.0, shift_left, shift_right, 0.012, 0.010, (0.16, 0.56, 0.38), (0.08, 0.20, 0.36)),
        Keyframe("exchange", 22.0, exchange_left, exchange_right, 0.026, 0.026, (0.18, 0.55, 0.38), (0.08, 0.20, 0.36)),
        Keyframe("complete", 24.0, exchange_left, exchange_right, 0.032, 0.032, (0.18, 0.55, 0.38), (0.08, 0.20, 0.36)),
    ]


def sample_program(keyframes: list[Keyframe], elapsed_s: float, loop: bool = False) -> ProgramSample:
    if not keyframes:
        keyframes = build_keyframes({})
    duration = keyframes[-1].time_s
    t = elapsed_s % duration if loop and duration > 0.0 else min(max(elapsed_s, 0.0), duration)
    prev = keyframes[0]
    nxt = keyframes[-1]
    for idx in range(1, len(keyframes)):
        if t <= keyframes[idx].time_s:
            prev = keyframes[idx - 1]
            nxt = keyframes[idx]
            break
    span = max(nxt.time_s - prev.time_s, 1e-6)
    alpha = min(max((t - prev.time_s) / span, 0.0), 1.0)
    return ProgramSample(
        phase=nxt.name,
        elapsed_s=t,
        left=lerp(prev.left, nxt.left, alpha),
        right=lerp(prev.right, nxt.right, alpha),
        left_gripper=prev.left_gripper + (nxt.left_gripper - prev.left_gripper) * alpha,
        right_gripper=prev.right_gripper + (nxt.right_gripper - prev.right_gripper) * alpha,
        yarn_xyz=lerp(prev.yarn_xyz, nxt.yarn_xyz, alpha),  # type: ignore[arg-type]
        target_xyz=lerp(prev.target_xyz, nxt.target_xyz, alpha),  # type: ignore[arg-type]
    )
