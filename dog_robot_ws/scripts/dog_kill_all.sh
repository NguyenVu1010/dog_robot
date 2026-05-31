#!/usr/bin/env bash
# Kill orphan dog_robot kinematic-viz processes. Run from a script file to
# avoid pkill matching its own shell command line, and use -f because some
# node names exceed pkill's 15-char limit (see feedback_pkill_dog_robot_orphans).
# Two-pass: SIGTERM, then SIGKILL anything still alive after 1s.
set +e

PATTERNS=(
  robot_state_publisher
  static_transform_publisher
  kinematic_node
  teleop_keyboard
  rviz2
  "ros2 launch"
)

for p in "${PATTERNS[@]}"; do pkill    -f "$p"; done
sleep 1
for p in "${PATTERNS[@]}"; do pkill -9 -f "$p"; done
sleep 0.3

echo "[dog_kill_all] done"
