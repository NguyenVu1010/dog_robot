"""dog_robot CHAMP gazebo bringup. Adapted from champ_config."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_pkg = get_package_share_directory("dog_robot_config")
    descr_pkg = get_package_share_directory("dog_robot_description")

    # Resolve once so gzserver / gzclient inherit the env. SetEnvironmentVariable
    # on the launch description does not propagate into IncludeLaunchDescription.
    install_share = os.path.dirname(descr_pkg)
    existing_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    os.environ["GAZEBO_MODEL_PATH"] = (
        install_share + ((":" + existing_model_path) if existing_model_path else "")
    )
    # Disable online model database fetch (each timeout adds ~30 s of startup).
    os.environ["GAZEBO_MODEL_DATABASE_URI"] = ""

    joints_yaml = os.path.join(config_pkg, "config", "joints", "joints.yaml")
    links_yaml = os.path.join(config_pkg, "config", "links", "links.yaml")
    gait_yaml = os.path.join(config_pkg, "config", "gait", "gait.yaml")
    ros_control_yaml = os.path.join(
        config_pkg, "config", "ros_control", "ros_control.yaml"
    )
    default_xacro = os.path.join(descr_pkg, "urdf", "dog_robot.urdf.xacro")
    default_world = os.path.join(config_pkg, "worlds", "simple.world")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("robot_name", default_value="dog_robot"),
        DeclareLaunchArgument("description_path", default_value=default_xacro),
        DeclareLaunchArgument("joints_map_path", default_value=joints_yaml),
        DeclareLaunchArgument("links_map_path", default_value=links_yaml),
        DeclareLaunchArgument("gait_config_path", default_value=gait_yaml),
        DeclareLaunchArgument("ros_control_file", default_value=ros_control_yaml),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("lite", default_value="false"),
        DeclareLaunchArgument("world_init_x", default_value="0.0"),
        DeclareLaunchArgument("world_init_y", default_value="0.0"),
        DeclareLaunchArgument("world_init_heading", default_value="0.0"),

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
                "gazebo": "true",
                "lite": LaunchConfiguration("lite"),
                "rviz": LaunchConfiguration("rviz"),
                "joint_controller_topic":
                    "joint_group_effort_controller/joint_trajectory",
                "hardware_connected": "false",
                "publish_foot_contacts": "false",
                "close_loop_odom": "true",
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("champ_gazebo"),
                    "launch",
                    "gazebo.launch.py",
                )
            ),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "robot_name": LaunchConfiguration("robot_name"),
                "world": LaunchConfiguration("world"),
                "lite": LaunchConfiguration("lite"),
                "world_init_x": LaunchConfiguration("world_init_x"),
                "world_init_y": LaunchConfiguration("world_init_y"),
                "world_init_heading": LaunchConfiguration("world_init_heading"),
                "gui": LaunchConfiguration("gui"),
                "close_loop_odom": "true",
            }.items(),
        ),
    ])
