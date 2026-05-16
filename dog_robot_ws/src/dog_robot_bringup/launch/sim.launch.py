"""Full simulation: Gazebo Classic + ros2_control + gait controller + keyboard teleop.

Teleop is launched in a new gnome-terminal window so its raw-stdin input
does not collide with the main launch console. Set teleop:=false to skip it.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_pkg = FindPackageShare("dog_robot_description")
    ctrl_pkg = FindPackageShare("dog_robot_control")

    teleop_arg = DeclareLaunchArgument(
        "teleop",
        default_value="true",
        description="Launch keyboard teleop in a separate terminal.",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([desc_pkg, "/launch/gazebo.launch.py"])
    )
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ctrl_pkg, "/launch/controller.launch.py"])
    )

    teleop = Node(
        package="dog_robot_control",
        executable="teleop_keyboard",
        name="teleop_keyboard",
        output="screen",
        prefix="gnome-terminal --",
        condition=IfCondition(LaunchConfiguration("teleop")),
    )

    return LaunchDescription([
        teleop_arg,
        gazebo,
        # Wait for Gazebo + ros2_control before starting the gait controller and teleop.
        TimerAction(period=8.0, actions=[controller, teleop]),
    ])
