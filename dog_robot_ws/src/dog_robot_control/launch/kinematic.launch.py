"""Kinematic-only viz: walker (kinematic_mode) -> /joint_states -> RSP -> RViz.

No Gazebo. cmd_vel drives the full gait + IK pipeline, joint angles are
streamed straight to RViz. Pure geometric sanity check.
"""
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    descr = FindPackageShare("dog_robot_description")
    ctrl = FindPackageShare("dog_robot_control")

    urdf_xacro = PathJoinSubstitution([descr, "urdf", "dog_robot.urdf.xacro"])
    controllers_yaml = PathJoinSubstitution([descr, "config", "ros2_controllers.yaml"])
    walker_params = PathJoinSubstitution([ctrl, "config", "walker_params.yaml"])
    rviz_cfg = PathJoinSubstitution([ctrl, "rviz", "kinematic.rviz"])

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

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_cfg],
        output="screen",
    )

    return LaunchDescription([rsp, walker, rviz])
