from setuptools import find_packages, setup
import os
from glob import glob

package_name = "dog_robot_kinematic_viz"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "rviz"),
            glob("rviz/*.rviz")),
        (os.path.join("share", package_name, "config"),
            glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy", "pyyaml"],
    zip_safe=True,
    maintainer="nguyenvd",
    maintainer_email="nguyenvd11@fpt.com",
    description="Kinematic-only RViz rig for the 12-DOF dog robot.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "kinematic_node = dog_robot_kinematic_viz.kinematic_node:main",
            "teleop_keyboard = dog_robot_kinematic_viz.teleop_keyboard:main",
            "gui_teleop = dog_robot_kinematic_viz.gui_teleop:main",
        ],
    },
)
