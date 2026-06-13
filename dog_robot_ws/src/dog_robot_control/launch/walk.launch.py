"""Walking launch: Gazebo + spawn dog_robot + JTC + walker_controller."""
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
    walker_params = PathJoinSubstitution([ctrl, "config", "walker_params.yaml"])

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

    # Initial joint pose in ros2_control.xacro is the BENT stand pose
    # (thigh=-0.4146, knee=1.1498), so spawn at body z=0.16 puts feet
    # ~10 mm above ground => tiny settle, no contact impulse.
    # (Spawning at z=0.18 with theta=0 legs put foot center at z=-0.0003
    # i.e. 18 mm of sphere penetration => 360 N upward => robot exploded.)
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "dog_robot",
                   "-z", "0.16", "-timeout", "120"],
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
    walker = Node(
        package="dog_robot_control",
        executable="walker_controller",
        name="walker_controller",
        parameters=[walker_params],
        output="screen",
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[load_jsb])),
        RegisterEventHandler(OnProcessExit(target_action=load_jsb, on_exit=[load_jtc])),
        RegisterEventHandler(OnProcessExit(target_action=load_jtc, on_exit=[walker])),
    ])
