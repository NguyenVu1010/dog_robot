#!/usr/bin/env bash
# Kill orphan dog_robot sim processes. Use full-cmdline match (-f) since several
# names exceed pkill's 15-char limit, and run from a script file to avoid
# pkill matching its own shell command line.
# Two-pass: SIGTERM first to let processes shut down cleanly, then SIGKILL
# anything still alive after 1s. gzserver in particular can wedge and ignore
# SIGTERM, leaving a zombie that conflicts with the next launch.
set +e

PATTERNS=(
  gzserver gzclient ros_gz_bridge spawn_entity
  robot_state_publisher joint_state_broadcaster
  joint_trajectory_controller controller_manager
  ros2_control_node ros2_control gazebo_ros2_control
  stand_controller rviz2 champ_base champ_gazebo
)

for p in "${PATTERNS[@]}"; do pkill    -f "$p"; done
sleep 1
for p in "${PATTERNS[@]}"; do pkill -9 -f "$p"; done
sleep 0.3

echo "[dog_kill_all] done"
