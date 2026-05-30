from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    planner_share = Path(get_package_share_directory("rm65b_dual_arm_planning"))
    return LaunchDescription(
        [
            Node(
                package="rm65b_dual_arm_planning",
                executable="dual_arm_planner",
                name="dual_arm_planner",
                parameters=[str(planner_share / "config" / "dual_arm_planner.yaml")],
                output="screen",
            )
        ]
    )
