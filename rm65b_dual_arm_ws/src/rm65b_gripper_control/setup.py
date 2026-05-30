from glob import glob
from setuptools import setup

package_name = "rm65b_gripper_control"

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
    description="Gripper action server for RM65-B weaving experiments.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "gripper_action_server = rm65b_gripper_control.gripper_action_server:main",
        ],
    },
)
