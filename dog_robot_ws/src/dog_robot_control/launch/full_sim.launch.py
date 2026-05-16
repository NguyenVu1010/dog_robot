from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    desc_pkg = FindPackageShare("dog_robot_description")
    ctrl_pkg = FindPackageShare("dog_robot_control")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            desc_pkg, "/launch/gazebo.launch.py"
        ])
    )
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            ctrl_pkg, "/launch/controller.launch.py"
        ])
    )

    return LaunchDescription([
        gazebo,
        # Wait 8s for Gazebo + ros2_control to be ready
        TimerAction(period=8.0, actions=[controller]),
    ])
