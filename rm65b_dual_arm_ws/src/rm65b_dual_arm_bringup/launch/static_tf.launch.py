from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_transforms(config_path):
    with Path(config_path).open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    return data.get("transforms", [])


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("rm65b_dual_arm_bringup"))
        / "config"
        / "dual_arm_frames.yaml"
    )
    config = LaunchConfiguration("frames_config")

    # Launch substitutions are not resolved here, so static defaults are used
    # for the node list. Override by editing the YAML before launching.
    nodes = []
    for item in _load_transforms(default_config):
        x, y, z = item["xyz"]
        qx, qy, qz, qw = item["quat_xyzw"]
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"static_tf_{item['name']}",
                arguments=[
                    str(x),
                    str(y),
                    str(z),
                    str(qx),
                    str(qy),
                    str(qz),
                    str(qw),
                    item["parent_frame_id"],
                    item["child_frame_id"],
                ],
                output="screen",
            )
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("frames_config", default_value=default_config),
            *nodes,
        ]
    )
