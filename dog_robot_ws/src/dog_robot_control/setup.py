from setuptools import find_packages, setup
import os
from glob import glob

package_name = "dog_robot_control"

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
        (os.path.join("share", package_name, "config"),
            glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="nguyenvd",
    maintainer_email="nguyenvd11@fpt.com",
    description="ROS2 controller node for dog robot",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "teleop_keyboard = dog_robot_control.teleop_keyboard:main",
            "stand_controller = dog_robot_control.stand_controller:main",
            "walker_controller = dog_robot_control.walker_controller:main",
        ],
    },
)
