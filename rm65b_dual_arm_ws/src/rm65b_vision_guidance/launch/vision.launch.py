from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("rm65b_vision_guidance"))
    return LaunchDescription(
        [
            Node(
                package="rm65b_vision_guidance",
                executable="aruco_target_node",
                namespace="left_vision",
                parameters=[str(share / "config" / "vision_left.yaml")],
                output="screen",
            ),
            Node(
                package="rm65b_vision_guidance",
                executable="ibvs_controller",
                parameters=[str(share / "config" / "ibvs.yaml")],
                output="screen",
            ),
        ]
    )
