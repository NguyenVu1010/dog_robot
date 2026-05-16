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

Entry points live in `dog_robot_bringup`:

```bash
ros2 launch dog_robot_bringup display.launch.py   # RViz + joint slider
ros2 launch dog_robot_bringup sim.launch.py       # Gazebo + control stack
```

`gazebo.launch.py` here is a low-level building block used by bringup; you
usually do not invoke it directly.

## Test

```bash
colcon test --packages-select dog_robot_description
colcon test-result --verbose
```
