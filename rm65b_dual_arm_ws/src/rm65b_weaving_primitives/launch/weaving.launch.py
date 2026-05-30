from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    share_dir = Path(get_package_share_directory("rm65b_weaving_primitives"))
    dry_run = LaunchConfiguration("dry_run")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_tension_simulator = LaunchConfiguration("enable_tension_simulator")
    bt_runtime = LaunchConfiguration("bt_runtime")
    return LaunchDescription(
        [
            DeclareLaunchArgument("dry_run", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("enable_tension_simulator", default_value="true"),
            DeclareLaunchArgument("bt_runtime", default_value="cpp"),
            Node(
                condition=IfCondition(PythonExpression(["'", bt_runtime, "' == 'cpp'"])),
                package="rm65b_weaving_primitives",
                executable="weaving_bt_runner",
                parameters=[
                    {
                        "dry_run": ParameterValue(dry_run, value_type=bool),
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "bt_runtime_name": "cpp_behavior_tree_cpp_v3",
                        "playback_stage": "D4_D5",
                    }
                ],
                output="screen",
            ),
            Node(
                condition=IfCondition(PythonExpression(["'", bt_runtime, "' == 'python'"])),
                package="rm65b_weaving_primitives",
                executable="weaving_coordinator",
                parameters=[
                    {
                        "dry_run": ParameterValue(dry_run, value_type=bool),
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "use_behavior_tree_xml": True,
                        "bt_runtime_name": "python_xml_bt",
                        "playback_stage": "D4_D5",
                    }
                ],
                output="screen",
            ),
            Node(
                condition=IfCondition(enable_tension_simulator),
                package="rm65b_weaving_primitives",
                executable="tension_simulator",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "config_file": str(share_dir / "config" / "d4_tension_pid_compliance.yaml"),
                    }
                ],
                output="screen",
            ),
        ]
    )
