from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _default_world() -> str:
    latest_day05 = (
        Path.cwd()
        / "outputs"
        / "rm65b_gazebo_camera_force_loop_days_harmonic_20260527_2245"
        / "day05"
        / "logs"
        / "rm65b_planned_world_runtime.sdf"
    )
    return str(latest_day05) if latest_day05.exists() else ""


def _default_robot_sdf() -> str:
    latest_robot = (
        Path.cwd()
        / "outputs"
        / "rm65b_gazebo_camera_force_loop_days_harmonic_20260527_2245"
        / "day05"
        / "logs"
        / "rm_65_harmonic_fixed_base_controller.sdf"
    )
    return str(latest_robot) if latest_robot.exists() else ""


def _launch_gazebo(context, *args, **kwargs):
    if LaunchConfiguration("enable_gazebo").perform(context).lower() not in {"1", "true", "yes", "on"}:
        return []
    world = LaunchConfiguration("world").perform(context)
    if not world:
        return [
            LogInfo(
                msg=(
                    "enable_gazebo=true but no world was provided. "
                    "Pass world:=/path/to/rm65b_planned_world_runtime.sdf."
                )
            )
        ]
    if not Path(world).exists():
        return [LogInfo(msg=f"Gazebo world does not exist: {world}")]
    return [
        ExecuteProcess(
            cmd=["gz", "sim", "-v", "4", "-r", "-s", world],
            name="gz_sim",
            output="screen",
        )
    ]


def _spawn_robots(context, *args, **kwargs):
    if LaunchConfiguration("enable_spawn_robots").perform(context).lower() not in {
        "1",
        "true",
        "yes",
        "on",
    }:
        return []
    robot_sdf = LaunchConfiguration("robot_sdf").perform(context)
    if not robot_sdf:
        return [
            LogInfo(
                msg=(
                    "enable_spawn_robots=true but no robot_sdf was provided. "
                    "Pass robot_sdf:=/path/to/rm_65_harmonic_fixed_base_controller.sdf."
                )
            )
        ]
    if not Path(robot_sdf).exists():
        return [LogInfo(msg=f"Robot SDF does not exist: {robot_sdf}")]
    world_name = LaunchConfiguration("world_name").perform(context)
    return [
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "run",
                        "ros_gz_sim",
                        "create",
                        "-world",
                        world_name,
                        "-file",
                        robot_sdf,
                        "-name",
                        "left_rm65b",
                        "-x",
                        "-0.45",
                        "-y",
                        "0",
                        "-z",
                        "0.02",
                        "-Y",
                        "1.5708",
                    ],
                    name="spawn_left_rm65b",
                    output="screen",
                ),
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "run",
                        "ros_gz_sim",
                        "create",
                        "-world",
                        world_name,
                        "-file",
                        robot_sdf,
                        "-name",
                        "right_rm65b",
                        "-x",
                        "0.45",
                        "-y",
                        "0",
                        "-z",
                        "0.02",
                        "-Y",
                        "-1.5708",
                    ],
                    name="spawn_right_rm65b",
                    output="screen",
                ),
            ],
        )
    ]


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("rm65b_dual_arm_bringup"))
    full_system = bringup_share / "launch" / "full_system.launch.py"

    day_id = LaunchConfiguration("day_id")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_bridge = LaunchConfiguration("enable_bridge")
    enable_gazebo_relays = LaunchConfiguration("enable_gazebo_relays")

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=_default_world()),
            DeclareLaunchArgument("world_name", default_value="rm65b_world"),
            DeclareLaunchArgument("robot_sdf", default_value=_default_robot_sdf()),
            DeclareLaunchArgument("day_id", default_value="day05"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("enable_gazebo", default_value="true"),
            DeclareLaunchArgument("enable_bridge", default_value="true"),
            DeclareLaunchArgument("enable_spawn_robots", default_value="true"),
            DeclareLaunchArgument("enable_gazebo_relays", default_value="true"),
            DeclareLaunchArgument("enable_move_group", default_value="true"),
            DeclareLaunchArgument("moveit_config_package", default_value="rm65b_dual_arm_moveit_config"),
            OpaqueFunction(function=_launch_gazebo),
            OpaqueFunction(function=_spawn_robots),
            ExecuteProcess(
                condition=IfCondition(enable_bridge),
                cmd=[
                    "ros2",
                    "run",
                    "ros_gz_bridge",
                    "parameter_bridge",
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    "/left_camera/image_rect@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/right_camera/image_rect@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/rm65b/evidence_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/model/left_rm65b/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory",
                    "/model/right_rm65b/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory",
                    "/rm65b_gripper/upper_finger_cmd@std_msgs/msg/Float64]gz.msgs.Double",
                    "/rm65b_gripper/lower_finger_cmd@std_msgs/msg/Float64]gz.msgs.Double",
                    "/contacts/d1_handoff_block@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                    "/contacts/d2_force_target_panel@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                    "/contacts/d2_contact_pad@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                    "/contacts/vision_target@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                    "/contacts/d2_left_compliance_bar@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                    "/contacts/d2_right_compliance_bar@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                    "/contacts/d4_upper_loom_rail@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                    "/contacts/d4_lower_loom_rail@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                    "/contacts/d4_shuttle_lane@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                    "/contacts/d4_tension_scale@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                    "/world/rm65b_world/model/d1_handoff_block/link/link/sensor/d1_handoff_block_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                    "/world/rm65b_world/model/d2_force_target_panel/link/link/sensor/d2_force_target_panel_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                    "/world/rm65b_world/model/d2_contact_pad/link/link/sensor/d2_contact_pad_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                    "/world/rm65b_world/model/vision_target/link/target_link/sensor/vision_target_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                    "/world/rm65b_world/model/d2_left_compliance_bar/link/link/sensor/d2_left_compliance_bar_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                    "/world/rm65b_world/model/d2_right_compliance_bar/link/link/sensor/d2_right_compliance_bar_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                    "/world/rm65b_world/model/d4_upper_loom_rail/link/link/sensor/d4_upper_loom_rail_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                    "/world/rm65b_world/model/d4_lower_loom_rail/link/link/sensor/d4_lower_loom_rail_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                    "/world/rm65b_world/model/d4_shuttle_lane/link/link/sensor/d4_shuttle_lane_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                    "/world/rm65b_world/model/d4_tension_scale/link/link/sensor/d4_tension_scale_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                ],
                name="ros_gz_bridge",
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(full_system)),
                launch_arguments={
                    "use_hardware": "false",
                    "use_sim_time": use_sim_time,
                    "enable_move_group": LaunchConfiguration("enable_move_group"),
                    "moveit_config_package": LaunchConfiguration("moveit_config_package"),
                    "moveit_allow_trajectory_execution": "false",
                    "enable_vision": "true",
                    "enable_planning": "true",
                    "enable_gazebo_camera_info": "true",
                    "enable_force_admittance": "true",
                    "enable_contact_force_estimator": "true",
                    "enable_gazebo_relays": enable_gazebo_relays,
                    "enable_tension_simulator": "true",
                    "enable_weaving_coordinator": "true",
                    "dry_run": "true",
                    "day_id": day_id,
                }.items(),
            ),
            LogInfo(
                msg=[
                    "Integrated simulation launch started for ",
                    day_id,
                    ". Topics expected: /left_camera/camera_info, /force_control/state, ",
                    "/weaving/tension_n, /weaving/events.",
                ]
            ),
        ]
    )
