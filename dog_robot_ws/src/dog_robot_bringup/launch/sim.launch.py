"""Full simulation: Gazebo Classic + ros2_control + gait controller node."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_pkg = FindPackageShare("dog_robot_description")
    ctrl_pkg = FindPackageShare("dog_robot_control")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([desc_pkg, "/launch/gazebo.launch.py"])
    )
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ctrl_pkg, "/launch/controller.launch.py"])
    )

    return LaunchDescription([
        gazebo,
        # Wait for Gazebo + ros2_control to be ready before starting the gait controller.
        TimerAction(period=8.0, actions=[controller]),
    ])
