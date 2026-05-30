from glob import glob
from setuptools import setup

package_name = "rm65b_weaving_primitives"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*")),
        (f"share/{package_name}/trajectories", glob("trajectories/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RM65-B Lab",
    maintainer_email="lab@example.com",
    description="Weaving primitive coordinator and trajectory tools for RM65-B.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "weaving_coordinator = rm65b_weaving_primitives.weaving_coordinator:main",
            "trajectory_recorder = rm65b_weaving_primitives.trajectory_recorder:main",
            "tension_simulator = rm65b_weaving_primitives.tension_simulator:main",
        ],
    },
)
