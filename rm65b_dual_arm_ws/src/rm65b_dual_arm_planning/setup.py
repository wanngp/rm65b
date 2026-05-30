from glob import glob
from setuptools import setup

package_name = "rm65b_dual_arm_planning"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RM65-B Lab",
    maintainer_email="lab@example.com",
    description="Dual-arm planning and Harmonic FK playback for RM65-B.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "dual_arm_planner = rm65b_dual_arm_planning.dual_arm_planner:main",
            "harmonic_fk_player = rm65b_dual_arm_planning.harmonic_fk_player:main",
            "moveit_plan_client = rm65b_dual_arm_planning.moveit_plan_client:main",
            "dual_moveit_plan_client = rm65b_dual_arm_planning.dual_moveit_plan_client:main",
            "force_admittance_controller = rm65b_dual_arm_planning.force_admittance_controller:main",
            "gazebo_contact_force_estimator = rm65b_dual_arm_planning.gazebo_contact_force_estimator:main",
            "force_gazebo_trajectory_relay = rm65b_dual_arm_planning.gazebo_trajectory_relays:force_relay_main",
            "visual_servo_gazebo_adapter = rm65b_dual_arm_planning.gazebo_trajectory_relays:visual_servo_adapter_main",
        ],
    },
)
