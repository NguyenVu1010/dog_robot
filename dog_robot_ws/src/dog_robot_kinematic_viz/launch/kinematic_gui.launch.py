"""Full 4-leg kinematic rig + Tk GUI teleop + RViz.

Same as kinematic.launch.py plus a gui_teleop node (Tk window) that
publishes /cmd_vel and calls the /sit + /release services on
kinematic_node.
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
        ],
        output="screen",
    )

    gui = Node(
        package="dog_robot_kinematic_viz",
        executable="gui_teleop",
        name="gui_teleop",
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_cfg],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("base_height", default_value="0.20"),
        rsp, static_tf, kinematic_node, gui, rviz,
    ])
