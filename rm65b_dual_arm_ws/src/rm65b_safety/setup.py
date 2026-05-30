from glob import glob
from setuptools import setup

package_name = "rm65b_safety"

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
    description="Safety supervisor for RM65-B dual-arm experiments.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "safety_supervisor = rm65b_safety.safety_supervisor:main",
            "d5_latency_probe = rm65b_safety.d5_latency_probe:main",
            "launch_audit_recorder = rm65b_safety.launch_audit_recorder:main",
        ],
    },
)
