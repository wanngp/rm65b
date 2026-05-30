from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _include(package, launch_file, **kwargs):
    path = Path(get_package_share_directory(package)) / "launch" / launch_file
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)),
        launch_arguments=kwargs.items(),
    )


def generate_launch_description():
    use_hardware = LaunchConfiguration("use_hardware")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_vision = LaunchConfiguration("enable_vision")
    enable_planning = LaunchConfiguration("enable_planning")
    enable_gazebo_camera_info = LaunchConfiguration("enable_gazebo_camera_info")
    enable_force_admittance = LaunchConfiguration("enable_force_admittance")
    enable_contact_force_estimator = LaunchConfiguration("enable_contact_force_estimator")
    enable_gazebo_relays = LaunchConfiguration("enable_gazebo_relays")
    enable_tension_simulator = LaunchConfiguration("enable_tension_simulator")
    enable_weaving_coordinator = LaunchConfiguration("enable_weaving_coordinator")
    enable_launch_audit = LaunchConfiguration("enable_launch_audit")
    enable_latency_probe = LaunchConfiguration("enable_latency_probe")
    d5_launch_audit_json = LaunchConfiguration("d5_launch_audit_json")
    d5_latency_json = LaunchConfiguration("d5_latency_json")
    tension_config = LaunchConfiguration("tension_config")
    dry_run = LaunchConfiguration("dry_run")
    day_id = LaunchConfiguration("day_id")

    gripper_share = Path(get_package_share_directory("rm65b_gripper_control"))
    weaving_share = Path(get_package_share_directory("rm65b_weaving_primitives"))
    safety_config = str(
        Path(get_package_share_directory("rm65b_safety"))
        / "config"
        / "safety_supervisor.yaml"
    )
    left_gripper = str(gripper_share / "config" / "left_gripper.yaml")
    right_gripper = str(gripper_share / "config" / "right_gripper.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_hardware", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("enable_vision", default_value="true"),
            DeclareLaunchArgument("enable_planning", default_value="true"),
            DeclareLaunchArgument("enable_gazebo_camera_info", default_value="true"),
            DeclareLaunchArgument("enable_force_admittance", default_value="true"),
            DeclareLaunchArgument("enable_contact_force_estimator", default_value="true"),
            DeclareLaunchArgument("enable_gazebo_relays", default_value="true"),
            DeclareLaunchArgument("enable_tension_simulator", default_value="true"),
            DeclareLaunchArgument("enable_weaving_coordinator", default_value="true"),
            DeclareLaunchArgument("enable_launch_audit", default_value="true"),
            DeclareLaunchArgument("enable_latency_probe", default_value="true"),
            DeclareLaunchArgument(
                "d5_launch_audit_json",
                default_value="outputs/d5_system/day05_launch_audit.json",
            ),
            DeclareLaunchArgument(
                "d5_latency_json",
                default_value="outputs/d5_system/day05_latency_record.json",
            ),
            DeclareLaunchArgument(
                "tension_config",
                default_value=str(weaving_share / "config" / "d4_tension_pid_compliance.yaml"),
            ),
            DeclareLaunchArgument("dry_run", default_value="true"),
            DeclareLaunchArgument("day_id", default_value="day05"),
            _include("rm65b_dual_arm_bringup", "static_tf.launch.py"),
            _include(
                "rm65b_dual_arm_bringup",
                "dual_rm_drivers.launch.py",
                use_hardware=use_hardware,
            ),
            Node(
                package="rm65b_gripper_control",
                executable="gripper_action_server",
                namespace="left_gripper_controller",
                name="gripper_action_server",
                parameters=[
                    left_gripper,
                    {
                        "hardware_enabled": ParameterValue(use_hardware, value_type=bool),
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    },
                ],
                output="screen",
            ),
            Node(
                package="rm65b_gripper_control",
                executable="gripper_action_server",
                namespace="right_gripper_controller",
                name="gripper_action_server",
                parameters=[
                    right_gripper,
                    {
                        "hardware_enabled": ParameterValue(use_hardware, value_type=bool),
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    },
                ],
                output="screen",
            ),
            Node(
                package="rm65b_safety",
                executable="safety_supervisor",
                name="safety_supervisor",
                parameters=[safety_config, {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
                output="screen",
            ),
            Node(
                condition=IfCondition(enable_launch_audit),
                package="rm65b_safety",
                executable="launch_audit_recorder",
                name="launch_audit_recorder",
                parameters=[
                    {
                        "output_json": ParameterValue(d5_launch_audit_json, value_type=str),
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    }
                ],
                output="screen",
            ),
            Node(
                condition=IfCondition(enable_latency_probe),
                package="rm65b_safety",
                executable="d5_latency_probe",
                name="d5_latency_probe",
                parameters=[
                    {
                        "output_json": ParameterValue(d5_latency_json, value_type=str),
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    }
                ],
                output="screen",
            ),
            Node(
                condition=IfCondition(enable_gazebo_camera_info),
                package="rm65b_vision_guidance",
                executable="gazebo_camera_info_publisher",
                name="gazebo_camera_info_publisher",
                parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
                output="screen",
            ),
            Node(
                condition=IfCondition(enable_vision),
                package="rm65b_vision_guidance",
                executable="aruco_target_node",
                name="left_aruco_target_node",
                namespace="left_vision",
                parameters=[
                    str(
                        Path(get_package_share_directory("rm65b_vision_guidance"))
                        / "config"
                        / "vision_left.yaml"
                    ),
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
                output="screen",
            ),
            Node(
                condition=IfCondition(enable_vision),
                package="rm65b_vision_guidance",
                executable="ibvs_controller",
                name="ibvs_controller",
                parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
                output="screen",
            ),
            Node(
                condition=IfCondition(enable_planning),
                package="rm65b_dual_arm_planning",
                executable="dual_arm_planner",
                name="dual_arm_planner",
                parameters=[
                    {
                        "day_id": ParameterValue(day_id, value_type=str),
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "publish_synthetic_camera": False,
                        "publish_synthetic_vision": False,
                        "publish_simulated_force": False,
                    }
                ],
                output="screen",
            ),
            Node(
                condition=IfCondition(enable_force_admittance),
                package="rm65b_dual_arm_planning",
                executable="force_admittance_controller",
                name="force_admittance_controller",
                parameters=[
                    {
                        "day_id": ParameterValue(day_id, value_type=str),
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    }
                ],
                output="screen",
            ),
            Node(
                condition=IfCondition(enable_contact_force_estimator),
                package="rm65b_dual_arm_planning",
                executable="gazebo_contact_force_estimator",
                name="gazebo_contact_force_estimator",
                parameters=[
                    {
                        "day_id": ParameterValue(day_id, value_type=str),
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    }
                ],
                output="screen",
            ),
            Node(
                condition=IfCondition(enable_gazebo_relays),
                package="rm65b_dual_arm_planning",
                executable="force_gazebo_trajectory_relay",
                name="force_gazebo_trajectory_relay",
                parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
                output="screen",
            ),
            Node(
                condition=IfCondition(enable_gazebo_relays),
                package="rm65b_dual_arm_planning",
                executable="visual_servo_gazebo_adapter",
                name="visual_servo_gazebo_adapter",
                parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
                output="screen",
            ),
            Node(
                condition=IfCondition(enable_weaving_coordinator),
                package="rm65b_weaving_primitives",
                executable="weaving_bt_runner",
                name="weaving_bt_runner",
                parameters=[
                    {
                        "dry_run": ParameterValue(dry_run, value_type=bool),
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "behavior_tree_file": str(weaving_share / "config" / "weaving_tree.xml"),
                        "bt_runtime_name": "cpp_behavior_tree_cpp_v3",
                        "playback_stage": "D4_D5",
                    }
                ],
                output="screen",
            ),
            Node(
                package="rm65b_weaving_primitives",
                executable="tension_simulator",
                name="tension_simulator",
                condition=IfCondition(enable_tension_simulator),
                parameters=[
                    {
                        "config_file": ParameterValue(tension_config, value_type=str),
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    }
                ],
                output="screen",
            ),
        ]
    )
