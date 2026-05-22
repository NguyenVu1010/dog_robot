#!/usr/bin/env bash
# Kill orphan dog_robot sim processes. Use full-cmdline match (-f) since several
# names exceed pkill's 15-char limit, and run from a script file to avoid
# pkill matching its own shell command line.
set +e
pkill -f gzserver
pkill -f gzclient
pkill -f ros_gz_bridge
pkill -f spawn_entity
pkill -f robot_state_publisher
pkill -f joint_state_broadcaster
pkill -f joint_trajectory_controller
pkill -f controller_manager
pkill -f stand_controller
pkill -f rviz2
pkill -f champ_base
pkill -f champ_gazebo
sleep 0.5
echo "[dog_kill_all] done"
