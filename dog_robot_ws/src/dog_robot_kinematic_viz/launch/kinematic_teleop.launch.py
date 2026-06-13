"""Kinematic viz + keyboard teleop in one launch.

walker(kinematic_mode) -> /joint_states -> RSP -> RViz; teleop_keyboard
publishes /cmd_vel from WASD/JL keys. Run this in a real terminal so
the teleop has a TTY.
"""
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    descr = FindPackageShare("dog_robot_description")
    ctrl = FindPackageShare("dog_robot_control")
    viz = FindPackageShare("dog_robot_kinematic_viz")

    urdf_xacro = PathJoinSubstitution([descr, "urdf", "dog_robot.urdf.xacro"])
    controllers_yaml = PathJoinSubstitution([descr, "config", "ros2_controllers.yaml"])
    walker_params = PathJoinSubstitution([ctrl, "config", "walker_params.yaml"])
    rviz_cfg = PathJoinSubstitution([viz, "rviz", "kinematic.rviz"])

    robot_description = {
        "robot_description": Command([
            FindExecutable(name="xacro"), " ", urdf_xacro,
            " controllers_yaml_path:=", controllers_yaml,
        ])
    }

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    walker = Node(
        package="dog_robot_control",
        executable="walker_controller",
        name="walker_controller",
        parameters=[walker_params, {"kinematic_mode": True}],
        output="screen",
    )

    # teleop needs its own TTY. gnome-terminal is what's installed here;
    # if you have a different emulator, edit the prefix.
    teleop = Node(
        package="dog_robot_control",
        executable="teleop_keyboard",
        name="teleop_keyboard",
        prefix="gnome-terminal --",
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_cfg],
        output="screen",
    )

    return LaunchDescription([rsp, walker, teleop, rviz])
