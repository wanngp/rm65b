from glob import glob
from setuptools import setup

package_name = "rm65b_vision_guidance"

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
    description="Vision target detection and IBVS scaffold for RM65-B experiments.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "aruco_target_node = rm65b_vision_guidance.aruco_target_node:main",
            "ibvs_controller = rm65b_vision_guidance.ibvs_controller:main",
            "gazebo_camera_info_publisher = rm65b_vision_guidance.gazebo_camera_info_publisher:main",
        ],
    },
)
