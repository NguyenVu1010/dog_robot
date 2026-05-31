"""Single-leg kinematic rig: full URDF, only one leg actively driven.

Inactive legs publish idle_joints (default [0,0,0]) so they render at
the URDF neutral pose. Pass `leg:=FR` (or BL/BR) to drive a different
leg. The full URDF still loads — only the IK pipeline is restricted.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    descr = FindPackageShare("dog_robot_description")
    viz = FindPackageShare("dog_robot_kinematic_viz")

    urdf_xacro = PathJoinSubstitution([descr, "urdf", "dog_robot.urdf.xacro"])
    link_params_yaml = PathJoinSubstitution([descr, "config", "link_params.yaml"])
    urdf_joints_yaml = PathJoinSubstitution([descr, "config", "urdf_joints.yaml"])
    kine_params = PathJoinSubstitution([viz, "config", "kinematic_params.yaml"])
    rviz_cfg = PathJoinSubstitution([viz, "rviz", "kinematic.rviz"])

    leg = LaunchConfiguration("leg")
    base_height = LaunchConfiguration("base_height")

    robot_description = {
        "robot_description": Command([
            FindExecutable(name="xacro"), " ", urdf_xacro,
        ])
    }

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base",
        arguments=["0", "0", base_height, "0", "0", "0", "world", "base_link"],
        output="screen",
    )

    kinematic_node = Node(
        package="dog_robot_kinematic_viz",
        executable="kinematic_node",
        name="kinematic_node",
        parameters=[
            kine_params,
            {"link_params_yaml": link_params_yaml},
            {"urdf_joints_yaml": urdf_joints_yaml},
            {"active_legs": [leg]},
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_cfg],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("leg", default_value="FL",
                              description="Which leg to drive: FL, FR, BL, BR"),
        DeclareLaunchArgument("base_height", default_value="0.20"),
        rsp, static_tf, kinematic_node, rviz,
    ])
