"""Keyboard teleop publishing /cmd_vel for the dog robot."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="dog_robot_control",
            executable="teleop_keyboard",
            name="teleop_keyboard",
            output="screen",
            prefix="xterm -e",  # run in its own terminal so stdin is interactive
        ),
    ])
