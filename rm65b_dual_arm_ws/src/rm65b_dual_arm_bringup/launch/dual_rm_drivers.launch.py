from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("rm65b_dual_arm_bringup"))
    use_hardware = LaunchConfiguration("use_hardware")

    left_config = str(share / "config" / "rm65b_left_driver.yaml")
    right_config = str(share / "config" / "rm65b_right_driver.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_hardware", default_value="false"),
            Node(
                condition=IfCondition(use_hardware),
                package="rm_driver",
                executable="rm_driver",
                namespace="left_rm_driver",
                name="rm_driver",
                parameters=[left_config],
                output="screen",
            ),
            Node(
                condition=IfCondition(use_hardware),
                package="rm_driver",
                executable="rm_driver",
                namespace="right_rm_driver",
                name="rm_driver",
                parameters=[right_config],
                output="screen",
            ),
        ]
    )
