#!/usr/bin/env bash
# Kill stale processes, build, launch the kinematic-only RViz rig.
#
# Plain `colcon build` (no --symlink-install): setuptools >=81 broke colcon's
# symlink/develop path, but copy-install works fine for ament_python and
# installs console_scripts + share/ files. See memory note
# feedback_setuptools_81_colcon.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$WS_DIR"

"$SCRIPT_DIR/dog_kill_all.sh"

source /opt/ros/humble/setup.bash

colcon build --packages-select \
  dog_robot_description dog_robot_kinematics dog_robot_kinematic_viz

source install/setup.bash

exec ros2 launch dog_robot_kinematic_viz kinematic_teleop.launch.py
