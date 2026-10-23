from glob import glob

from setuptools import setup


package_name = "g1_task_manager"


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            "share/" + package_name + "/launch",
            glob("launch/*.launch.py"),
        ),
        (
            "share/" + package_name + "/config",
            glob("config/*.yaml"),
        ),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="g1test",
    maintainer_email="g1test@example.com",
    description=(
        "Fixed waypoint and multi-point mission manager "
        "for Unitree G1"
    ),
    license="BSD-3-Clause",
    tests_require=[
        "pytest",
    ],
    entry_points={
        "console_scripts": [
            "g1_mission = g1_task_manager.mission_action_cli:main",
            (
                "g1_task_manager_node = "
                "g1_task_manager.g1_task_manager_node:main"
            ),
            (
                "g1_waypoint = "
                "g1_task_manager.waypoint_cli:main"
            ),
            (
                "g1_taskctl = "
                "g1_task_manager.task_cli:main"
            ),
        ],
    },
)
