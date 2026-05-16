import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg = FindPackageShare("dog_robot_control")
    params = PathJoinSubstitution([pkg, "config", "controller_params.yaml"])
    return LaunchDescription([
        Node(
            package="dog_robot_control",
            executable="controller_node",
            name="controller_node",
            parameters=[params],
            output="screen",
        ),
    ])
