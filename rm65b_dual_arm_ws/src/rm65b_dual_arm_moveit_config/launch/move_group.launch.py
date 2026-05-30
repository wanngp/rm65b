from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
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
    return generate_move_group_launch(moveit_config)
