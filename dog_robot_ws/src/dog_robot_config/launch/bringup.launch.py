"""dog_robot CHAMP bringup (no Gazebo). Adapted from champ_config."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_pkg = get_package_share_directory("dog_robot_config")
    descr_pkg = get_package_share_directory("dog_robot_description")

    joints_yaml = os.path.join(config_pkg, "config", "joints", "joints.yaml")
    links_yaml = os.path.join(config_pkg, "config", "links", "links.yaml")
    gait_yaml = os.path.join(config_pkg, "config", "gait", "gait.yaml")
    default_xacro = os.path.join(descr_pkg, "urdf", "dog_robot.urdf.xacro")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("robot_name", default_value="dog_robot"),
        DeclareLaunchArgument("description_path", default_value=default_xacro),
        DeclareLaunchArgument("joints_map_path", default_value=joints_yaml),
        DeclareLaunchArgument("links_map_path", default_value=links_yaml),
        DeclareLaunchArgument("gait_config_path", default_value=gait_yaml),
        DeclareLaunchArgument("gazebo", default_value="false"),
        DeclareLaunchArgument("lite", default_value="false"),
        DeclareLaunchArgument("hardware_connected", default_value="false"),
        DeclareLaunchArgument("publish_foot_contacts", default_value="false"),
        DeclareLaunchArgument("close_loop_odom", default_value="false"),
        DeclareLaunchArgument(
            "joint_controller_topic",
            default_value="joint_group_effort_controller/joint_trajectory",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("champ_bringup"),
                    "launch",
                    "bringup.launch.py",
                )
            ),
            launch_arguments={
                "description_path": LaunchConfiguration("description_path"),
                "joints_map_path": LaunchConfiguration("joints_map_path"),
                "links_map_path": LaunchConfiguration("links_map_path"),
                "gait_config_path": LaunchConfiguration("gait_config_path"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "robot_name": LaunchConfiguration("robot_name"),
                "gazebo": LaunchConfiguration("gazebo"),
                "lite": LaunchConfiguration("lite"),
                "rviz": LaunchConfiguration("rviz"),
                "joint_controller_topic": LaunchConfiguration("joint_controller_topic"),
                "hardware_connected": LaunchConfiguration("hardware_connected"),
                "publish_foot_contacts": LaunchConfiguration("publish_foot_contacts"),
                "close_loop_odom": LaunchConfiguration("close_loop_odom"),
            }.items(),
        ),
    ])
