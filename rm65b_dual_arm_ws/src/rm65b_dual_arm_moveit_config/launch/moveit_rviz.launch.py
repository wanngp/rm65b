from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("rm65b_dual_arm", package_name="rm65b_dual_arm_moveit_config")
        .robot_description(file_path="config/rm65b_dual_arm.urdf")
        .robot_description_semantic(file_path="config/rm65b_dual_arm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(moveit_config.package_path / "config/dual_arm_acceptance.rviz"),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[moveit_config.robot_description, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rm65b_dual_arm_acceptance_rviz",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
            ),
        ]
    )
