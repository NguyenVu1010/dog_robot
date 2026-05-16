"""RViz + joint_state_publisher_gui to visualize the robot URDF."""
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("dog_robot_description")
    urdf_xacro = PathJoinSubstitution([pkg, "urdf", "dog_robot.urdf.xacro"])
    rviz_config = PathJoinSubstitution([pkg, "rviz", "dog_robot.rviz"])

    robot_description = {
        "robot_description": Command([FindExecutable(name="xacro"), " ", urdf_xacro])
    }

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
