"""Stand-only launch: Gazebo + spawn dog_robot + JTC + stand_controller."""
import os

from launch import LaunchDescription
from launch.actions import (ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Disable Gazebo's online model database fetch BEFORE spawning gzclient/gzserver.
    # Default URI points to gazebosim.org/models; an unreachable host stalls
    # gzclient at "preparing world" for the full TCP SYN timeout (~75s+).
    os.environ["GAZEBO_MODEL_DATABASE_URI"] = ""

    descr = FindPackageShare("dog_robot_description")
    ctrl = FindPackageShare("dog_robot_control")

    urdf_xacro = PathJoinSubstitution([descr, "urdf", "dog_robot.urdf.xacro"])
    controllers_yaml = PathJoinSubstitution([descr, "config", "ros2_controllers.yaml"])
    stand_params = PathJoinSubstitution([ctrl, "config", "dh_params.yaml"])

    robot_description = {
        "robot_description": Command([
            FindExecutable(name="xacro"), " ", urdf_xacro,
            " controllers_yaml_path:=", controllers_yaml,
        ])
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo_ros"), "/launch/gazebo.launch.py"]),
        launch_arguments={"verbose": "false"}.items(),
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "dog_robot",
                   "-z", "0.30", "-timeout", "120"],
        output="screen",
    )

    load_jsb = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "joint_state_broadcaster"],
        output="screen",
    )
    load_jtc = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "joint_trajectory_controller"],
        output="screen",
    )
    stand = Node(
        package="dog_robot_control",
        executable="stand_controller",
        name="stand_controller",
        parameters=[stand_params],
        output="screen",
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[load_jsb])),
        RegisterEventHandler(OnProcessExit(target_action=load_jsb, on_exit=[load_jtc])),
        RegisterEventHandler(OnProcessExit(target_action=load_jtc, on_exit=[stand])),
    ])
