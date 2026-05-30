from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("rm65b_safety"))
    return LaunchDescription(
        [
            Node(
                package="rm65b_safety",
                executable="safety_supervisor",
                parameters=[str(share / "config" / "safety_supervisor.yaml")],
                output="screen",
            )
        ]
    )
