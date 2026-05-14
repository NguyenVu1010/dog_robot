# dog_robot_description

URDF + meshes + launch for 12-DOF quadruped dog robot.

## Build

```bash
cd dog_robot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select dog_robot_description
source install/setup.bash
```

## Launch

```bash
ros2 launch dog_robot_description display.launch.py     # RViz + joint slider
ros2 launch dog_robot_description gazebo.launch.py      # Gazebo Classic sim
```

## Test

```bash
colcon test --packages-select dog_robot_description
colcon test-result --verbose
```
