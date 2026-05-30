from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("rm65b_gripper_control"))
    return LaunchDescription(
        [
            Node(
                package="rm65b_gripper_control",
                executable="gripper_action_server",
                namespace="left_gripper_controller",
                parameters=[str(share / "config" / "left_gripper.yaml")],
                output="screen",
            ),
            Node(
                package="rm65b_gripper_control",
                executable="gripper_action_server",
                namespace="right_gripper_controller",
                parameters=[str(share / "config" / "right_gripper.yaml")],
                output="screen",
            ),
        ]
    )
