from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    allow_trajectory_execution = LaunchConfiguration("allow_trajectory_execution")
    publish_monitored_planning_scene = LaunchConfiguration("publish_monitored_planning_scene")

    moveit_config = (
        MoveItConfigsBuilder("rm65b_dual_arm", package_name="rm65b_dual_arm_moveit_config")
        .robot_description(file_path="config/rm65b_dual_arm.urdf")
        .robot_description_semantic(file_path="config/rm65b_dual_arm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    planning_scene_monitor_parameters = {
        "publish_planning_scene": ParameterValue(
            publish_monitored_planning_scene, value_type=bool
        ),
        "publish_geometry_updates": ParameterValue(
            publish_monitored_planning_scene, value_type=bool
        ),
        "publish_state_updates": ParameterValue(
            publish_monitored_planning_scene, value_type=bool
        ),
        "publish_transforms_updates": ParameterValue(
            publish_monitored_planning_scene, value_type=bool
        ),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("allow_trajectory_execution", default_value="false"),
            DeclareLaunchArgument("publish_monitored_planning_scene", default_value="true"),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    planning_scene_monitor_parameters,
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "allow_trajectory_execution": ParameterValue(
                            allow_trajectory_execution, value_type=bool
                        ),
                    },
                ],
            ),
        ]
    )
