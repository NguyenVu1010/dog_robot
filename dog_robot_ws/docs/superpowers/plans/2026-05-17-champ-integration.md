# CHAMP Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the broken in-house controller (`dog_kinematics` + `dog_gait` + `dog_robot_control/controller_node`) with the working CHAMP framework while keeping the custom URDF (`dog_robot_description`) and the existing `teleop_keyboard`. End state: robot walks in Gazebo on `cmd_vel` input.

**Architecture:** Copy CHAMP packages (already built and validated in `champ_ws`) into `dog_robot_ws/src/`. Create a new `dog_robot_config` package mirroring `chvmp/robots/configs/spotmicro_config` that points CHAMP at `dog_robot_description`. Modify the URDF to switch the `<ros2_control>` interface from `position` to `effort` and to reference the new `ros_control.yaml`.

**Tech Stack:** ROS 2 Humble · Gazebo Classic 11 · `gazebo_ros2_control` (effort interface) · `ros2_controllers/joint_trajectory_controller` · `robot_localization` · CHAMP (`chvmp/champ` ros2 branch + `chvmp/libchamp` submodule)

**Spec:** `docs/superpowers/specs/2026-05-17-champ-integration-design.md`

---

## Task 1: Create branch and copy CHAMP packages

**Files:**
- Create branch: `feature/champ-integration` cut from current `feature/control-pkg` (or `main`)
- Create directory: `dog_robot_ws/src/champ/` (copy of `champ_ws/src/champ/` including the `libchamp` submodule at `champ/champ/include/champ/`)

- [ ] **Step 1: Create new branch from current**

```bash
cd /home/nguyenvd/workspace/dog_robot/dog_robot_ws
git status -sb                                  # confirm clean (or stash) before branching
git checkout -b feature/champ-integration
```

Expected: `Switched to a new branch 'feature/champ-integration'`.

- [ ] **Step 2: Copy CHAMP source tree (preserve libchamp submodule)**

```bash
cp -r /home/nguyenvd/workspace/dog_robot/champ_ws/src/champ \
      /home/nguyenvd/workspace/dog_robot/dog_robot_ws/src/champ
ls /home/nguyenvd/workspace/dog_robot/dog_robot_ws/src/champ/champ/include/champ/
```

Expected output (libchamp header dirs visible):
```
body_controller/  geometry/   kinematics/   leg_controller/   macros/   motion/   odometry/   quadruped_base/   utils/
```

If the directory is empty, re-run inside the source tree:
```bash
cd /home/nguyenvd/workspace/dog_robot/dog_robot_ws/src/champ && \
  git submodule update --init --recursive 2>&1 | tail -3
```

- [ ] **Step 3: Build the copied CHAMP packages in the new workspace**

```bash
cd /home/nguyenvd/workspace/dog_robot/dog_robot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select champ champ_msgs \
  champ_base champ_gazebo champ_description \
  champ_navigation champ_bringup champ_config 2>&1 | tail -10
```

Expected: All 8 packages report `Finished`. No `Failed`.

- [ ] **Step 4: Commit the CHAMP import**

```bash
cd /home/nguyenvd/workspace/dog_robot/dog_robot_ws
git add src/champ
git commit -m "$(cat <<'EOF'
chore(deps): vendor chvmp/champ (ros2 branch + libchamp submodule)

Imported from champ_ws/src/champ where Gazebo walking has been verified.
champ_config and champ_description are kept as reference; dog_robot_config
(added in a later commit) is what the launch files use.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

Expected: 1 commit with hundreds of files added.

---

## Task 2: Delete obsolete in-house packages

**Files to delete:**
- `dog_robot_ws/src/dog_kinematics/` (entire package)
- `dog_robot_ws/src/dog_gait/` (entire package)
- `dog_robot_ws/src/dog_robot_bringup/` (entire package)

- [ ] **Step 1: Confirm nothing else depends on them**

```bash
cd /home/nguyenvd/workspace/dog_robot/dog_robot_ws
grep -rln "dog_kinematics\|dog_gait\|dog_robot_bringup" src/ \
  --include='package.xml' --include='*.cmake' --include='CMakeLists.txt'
```

Expected: only references are inside the three packages themselves and inside `dog_robot_control/dog_robot_control/controller_node.py` (which will also be removed in Task 3). If any other package still imports them, stop and reassess.

- [ ] **Step 2: Remove the three packages**

```bash
rm -rf src/dog_kinematics src/dog_gait src/dog_robot_bringup
```

Expected: directories gone. `ls src/` shows `champ/  dog_robot_control/  dog_robot_description/`.

- [ ] **Step 3: Clean any prior install/build artifacts of those packages**

```bash
rm -rf build/dog_kinematics build/dog_gait build/dog_robot_bringup
rm -rf install/dog_kinematics install/dog_gait install/dog_robot_bringup
```

- [ ] **Step 4: Commit the removal**

```bash
git add -A src/
git commit -m "$(cat <<'EOF'
refactor: remove dog_kinematics, dog_gait, dog_robot_bringup

These are superseded by CHAMP's quadruped_controller + gait planner +
champ_bringup. The IK formula in dog_kinematics was demonstrated to be
inconsistent with the URDF axes (see 2026-05-17 champ-integration spec).

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: Trim dog_robot_control to teleop-only

**Files:**
- Delete: `dog_robot_ws/src/dog_robot_control/dog_robot_control/controller_node.py`
- Delete: `dog_robot_ws/src/dog_robot_control/test/test_node_integration.py` (it imports the deleted node)
- Delete: `dog_robot_ws/src/dog_robot_control/test/test_sim_smoke.py` (it talks to the deleted `/enable` service)
- Modify: `dog_robot_ws/src/dog_robot_control/setup.py` — remove `controller_node` console_script entry
- Modify: `dog_robot_ws/src/dog_robot_control/package.xml` — remove dependencies on `dog_gait` / `dog_kinematics`

- [ ] **Step 1: Inspect current setup.py to find the entry-point lines**

```bash
cat /home/nguyenvd/workspace/dog_robot/dog_robot_ws/src/dog_robot_control/setup.py
```

Note the `entry_points={'console_scripts': [...]}` block. There should be two entries: `controller_node` and `teleop_keyboard` (and possibly others).

- [ ] **Step 2: Edit setup.py to keep only teleop**

Open `src/dog_robot_control/setup.py` and replace the `entry_points` block with exactly:

```python
    entry_points={
        'console_scripts': [
            'teleop_keyboard = dog_robot_control.teleop_keyboard:main',
        ],
    },
```

- [ ] **Step 3: Inspect package.xml**

```bash
cat /home/nguyenvd/workspace/dog_robot/dog_robot_ws/src/dog_robot_control/package.xml
```

If any `<depend>` or `<exec_depend>` lines reference `dog_gait` or `dog_kinematics`, remove just those lines. Keep `<exec_depend>rclpy</exec_depend>`, `<exec_depend>geometry_msgs</exec_depend>` etc.

- [ ] **Step 4: Delete the obsolete python module and tests**

```bash
cd /home/nguyenvd/workspace/dog_robot/dog_robot_ws
rm -f src/dog_robot_control/dog_robot_control/controller_node.py
rm -f src/dog_robot_control/test/test_node_integration.py
rm -f src/dog_robot_control/test/test_sim_smoke.py
```

- [ ] **Step 5: Clean rebuild dog_robot_control and verify only teleop installed**

```bash
rm -rf build/dog_robot_control install/dog_robot_control
source /opt/ros/humble/setup.bash
colcon build --packages-select dog_robot_control 2>&1 | tail -5
source install/setup.bash
ros2 pkg executables dog_robot_control
```

Expected last line: `dog_robot_control teleop_keyboard` (and only that).

- [ ] **Step 6: Commit**

```bash
git add src/dog_robot_control
git commit -m "$(cat <<'EOF'
refactor(control): trim dog_robot_control to teleop_keyboard only

The in-house controller_node + its integration tests depended on
dog_kinematics/dog_gait, which are being replaced by CHAMP. teleop_keyboard
already publishes /cmd_vel which is exactly what CHAMP subscribes to.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: Scaffold `dog_robot_config` package

**Files:**
- Create: `dog_robot_ws/src/dog_robot_config/package.xml`
- Create: `dog_robot_ws/src/dog_robot_config/CMakeLists.txt`
- Create directory tree: `config/{gait,joints,links,ros_control}/`, `launch/`, `worlds/`

- [ ] **Step 1: Create directory tree**

```bash
cd /home/nguyenvd/workspace/dog_robot/dog_robot_ws
mkdir -p src/dog_robot_config/{config/gait,config/joints,config/links,config/ros_control,launch,worlds}
```

- [ ] **Step 2: Create package.xml**

Write file `src/dog_robot_config/package.xml` with exactly:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>dog_robot_config</name>
  <version>0.1.0</version>
  <description>CHAMP config for the dog_robot custom quadruped</description>
  <maintainer email="nguyenvd11@fpt.com">nguyenvd11</maintainer>
  <license>BSD</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>champ_base</exec_depend>
  <exec_depend>champ_bringup</exec_depend>
  <exec_depend>champ_gazebo</exec_depend>
  <exec_depend>dog_robot_description</exec_depend>
  <exec_depend>gazebo_ros</exec_depend>
  <exec_depend>gazebo_ros2_control</exec_depend>
  <exec_depend>joint_state_broadcaster</exec_depend>
  <exec_depend>joint_trajectory_controller</exec_depend>
  <exec_depend>robot_localization</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>xacro</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 3: Create CMakeLists.txt**

Write file `src/dog_robot_config/CMakeLists.txt` with exactly:

```cmake
cmake_minimum_required(VERSION 3.8)
project(dog_robot_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

- [ ] **Step 4: Build the empty package to confirm scaffolding compiles**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select dog_robot_config 2>&1 | tail -5
```

Expected: `Finished <<< dog_robot_config`.

- [ ] **Step 5: Commit scaffold**

```bash
git add src/dog_robot_config
git commit -m "$(cat <<'EOF'
feat(dog_robot_config): scaffold ament_cmake package skeleton

Empty package with the config / launch / worlds install hooks. Yaml and
launch files will be added in subsequent commits.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: Author joints.yaml and links.yaml

**Files:**
- Create: `src/dog_robot_config/config/joints/joints.yaml`
- Create: `src/dog_robot_config/config/links/links.yaml`

These map CHAMP's slot names (left_front / right_front / left_hind / right_hind) onto the URDF's actual joint and link names. The fourth entry of each leg is a synthetic "foot" joint/link CHAMP uses for end-effector — for the URDF that uses a fixed `*_foot_link`, the foot entry is the fixed foot joint and the foot link.

- [ ] **Step 1: Inspect URDF joint and link names to populate the yaml**

```bash
cd /home/nguyenvd/workspace/dog_robot/dog_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -c "
import os, xacro
from ament_index_python.packages import get_package_share_directory
share = get_package_share_directory('dog_robot_description')
xml = xacro.process_file(os.path.join(share, 'urdf', 'dog_robot.urdf.xacro')).toxml()
import re
print('JOINTS:'); [print(' ', m.group(1)) for m in re.finditer(r'<joint name=\"([A-Z]{2}_[^\"]+)\" type', xml)]
print('LINKS:');  [print(' ', m.group(1)) for m in re.finditer(r'<link name=\"([A-Z]{2}_[^\"]+)\">', xml)]"
```

Expected output (16 joints, 16 links):
```
JOINTS:
  FL_hip_yaw
  FL_thigh_pitch
  FL_knee_pitch
  FL_foot_fixed
  FR_hip_yaw  ...
  ...
LINKS:
  FL_hip_link
  FL_thigh_link
  FL_shank_link
  FL_foot_link
  ...
```

Confirm the prefixes are `FL/FR/BL/BR` — front-left, front-right, back-left, back-right.

- [ ] **Step 2: Write joints.yaml**

Write file `src/dog_robot_config/config/joints/joints.yaml`:

```yaml
/**:
  ros__parameters:
    joints_map:
      left_front:
        - FL_hip_yaw
        - FL_thigh_pitch
        - FL_knee_pitch
        - FL_foot_fixed
      right_front:
        - FR_hip_yaw
        - FR_thigh_pitch
        - FR_knee_pitch
        - FR_foot_fixed
      left_hind:
        - BL_hip_yaw
        - BL_thigh_pitch
        - BL_knee_pitch
        - BL_foot_fixed
      right_hind:
        - BR_hip_yaw
        - BR_thigh_pitch
        - BR_knee_pitch
        - BR_foot_fixed
```

- [ ] **Step 3: Write links.yaml**

Write file `src/dog_robot_config/config/links/links.yaml`:

```yaml
/**:
  ros__parameters:
    links_map:
      base: base_link
      left_front:
        - FL_hip_link
        - FL_thigh_link
        - FL_shank_link
        - FL_foot_link
      right_front:
        - FR_hip_link
        - FR_thigh_link
        - FR_shank_link
        - FR_foot_link
      left_hind:
        - BL_hip_link
        - BL_thigh_link
        - BL_shank_link
        - BL_foot_link
      right_hind:
        - BR_hip_link
        - BR_thigh_link
        - BR_shank_link
        - BR_foot_link
```

- [ ] **Step 4: Build dog_robot_config and confirm yamls install**

```bash
colcon build --packages-select dog_robot_config 2>&1 | tail -3
ls install/dog_robot_config/share/dog_robot_config/config/joints/
ls install/dog_robot_config/share/dog_robot_config/config/links/
```

Expected: `joints.yaml` and `links.yaml` visible in install.

- [ ] **Step 5: Commit**

```bash
git add src/dog_robot_config/config/joints src/dog_robot_config/config/links
git commit -m "$(cat <<'EOF'
feat(dog_robot_config): joints/links maps for CHAMP

joints_map and links_map place the URDF's FL/FR/BL/BR joints into CHAMP's
left_front/right_front/left_hind/right_hind slots so the controller can
build the leg objects.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: Author gait.yaml and ros_control.yaml

**Files:**
- Create: `src/dog_robot_config/config/gait/gait.yaml`
- Create: `src/dog_robot_config/config/ros_control/ros_control.yaml`

- [ ] **Step 1: Write gait.yaml**

Write file `src/dog_robot_config/config/gait/gait.yaml`:

```yaml
/**:
  ros__parameters:
    gait:
      knee_orientation : ">>"
      pantograph_leg : false
      odom_scaler: 1.0
      max_linear_velocity_x : 0.3
      max_linear_velocity_y : 0.15
      max_angular_velocity_z : 1.0
      com_x_translation: 0.0
      swing_height : 0.04
      stance_depth : 0.0
      stance_duration : 0.25
      nominal_height : 0.18
```

Rationale (recorded in spec): the dog robot is roughly Spot-Micro-class size. `nominal_height: 0.18` leaves the legs bent ~0.5 rad below the fully-extended 0.224 m so the support polygon is stable. Velocity limits are conservative for a ~2 kg platform.

- [ ] **Step 2: Write ros_control.yaml**

Write file `src/dog_robot_config/config/ros_control/ros_control.yaml`:

```yaml
controller_manager:
  ros__parameters:
    use_sim_time: True
    update_rate: 250  # Hz

    joint_states_controller:
      type: joint_state_broadcaster/JointStateBroadcaster

    joint_group_effort_controller:
      type: joint_trajectory_controller/JointTrajectoryController

joint_group_effort_controller:
  ros__parameters:
    use_sim_time: True
    joints:
      - FL_hip_yaw
      - FL_thigh_pitch
      - FL_knee_pitch
      - FR_hip_yaw
      - FR_thigh_pitch
      - FR_knee_pitch
      - BL_hip_yaw
      - BL_thigh_pitch
      - BL_knee_pitch
      - BR_hip_yaw
      - BR_thigh_pitch
      - BR_knee_pitch
    command_interfaces:
      - effort
    state_interfaces:
      - position
      - velocity

    gains:
      FL_hip_yaw     : {p: 20.0, i: 0.05, d: 0.2, i_clamp: 0.5}
      FL_thigh_pitch : {p: 20.0, i: 0.05, d: 0.2, i_clamp: 0.5}
      FL_knee_pitch  : {p: 20.0, i: 0.05, d: 0.2, i_clamp: 0.5}
      FR_hip_yaw     : {p: 20.0, i: 0.05, d: 0.2, i_clamp: 0.5}
      FR_thigh_pitch : {p: 20.0, i: 0.05, d: 0.2, i_clamp: 0.5}
      FR_knee_pitch  : {p: 20.0, i: 0.05, d: 0.2, i_clamp: 0.5}
      BL_hip_yaw     : {p: 20.0, i: 0.05, d: 0.2, i_clamp: 0.5}
      BL_thigh_pitch : {p: 20.0, i: 0.05, d: 0.2, i_clamp: 0.5}
      BL_knee_pitch  : {p: 20.0, i: 0.05, d: 0.2, i_clamp: 0.5}
      BR_hip_yaw     : {p: 20.0, i: 0.05, d: 0.2, i_clamp: 0.5}
      BR_thigh_pitch : {p: 20.0, i: 0.05, d: 0.2, i_clamp: 0.5}
      BR_knee_pitch  : {p: 20.0, i: 0.05, d: 0.2, i_clamp: 0.5}
```

Rationale: CHAMP's default gains are `p:100, d:1.0` tuned for a ~10 kg robot. Scaling by ~×0.2 for a ~2 kg dog. Task 14 covers re-tuning if behavior is wrong.

- [ ] **Step 3: Build and confirm yamls install**

```bash
colcon build --packages-select dog_robot_config 2>&1 | tail -3
ls install/dog_robot_config/share/dog_robot_config/config/gait/
ls install/dog_robot_config/share/dog_robot_config/config/ros_control/
```

Expected: yaml files in install dirs.

- [ ] **Step 4: Commit**

```bash
git add src/dog_robot_config/config/gait src/dog_robot_config/config/ros_control
git commit -m "$(cat <<'EOF'
feat(dog_robot_config): gait params and ros2_control yaml

Effort interface joint_group_effort_controller + joint_states broadcaster,
gains scaled ×0.2 from CHAMP's 10 kg defaults for the 2 kg dog_robot.
Gait nominal_height 0.18 m gives a stable bent-knee stance.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 7: Author worlds/simple.world

**Files:**
- Create: `src/dog_robot_config/worlds/simple.world`

- [ ] **Step 1: Write simple.world**

Write file `src/dog_robot_config/worlds/simple.world` with exactly:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <gravity>0 0 -9.81</gravity>

    <physics name="default_physics" default="0" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>
  </world>
</sdf>
```

Note: `<gravity>` lives at `<world>` level — putting it inside `<physics>` causes a `bad any_cast` error (already debugged in dog_robot_description/worlds/simple.world).

- [ ] **Step 2: Build and confirm install**

```bash
colcon build --packages-select dog_robot_config 2>&1 | tail -3
ls install/dog_robot_config/share/dog_robot_config/worlds/
```

Expected: `simple.world` listed.

- [ ] **Step 3: Commit**

```bash
git add src/dog_robot_config/worlds
git commit -m "$(cat <<'EOF'
feat(dog_robot_config): simple Gazebo world

ground_plane + sun + ODE physics with gravity at world level (not inside
<physics> — that triggers a bad_any_cast in Gazebo Classic 11).

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 8: Switch URDF to effort interface and point plugin at new yaml

**Files:**
- Modify: `src/dog_robot_description/urdf/ros2_control.xacro`
- Modify: `src/dog_robot_description/urdf/gazebo.xacro`

- [ ] **Step 1: Inspect current ros2_control.xacro**

```bash
cat /home/nguyenvd/workspace/dog_robot/dog_robot_ws/src/dog_robot_description/urdf/ros2_control.xacro
```

It currently declares `<command_interface name="position">` for each joint with `min` / `max` / `initial_value` params.

- [ ] **Step 2: Replace ros2_control.xacro with the effort version**

Replace the entire contents of `src/dog_robot_description/urdf/ros2_control.xacro` with:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="dog_robot_ros2_control">

  <xacro:macro name="joint_iface" params="name lower upper">
    <joint name="${name}">
      <command_interface name="effort">
        <param name="min">-2.0</param>
        <param name="max">2.0</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </xacro:macro>

  <ros2_control name="dog_robot_hw" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>

    <xacro:joint_iface name="FL_hip_yaw"     lower="-0.785" upper="0.785"/>
    <xacro:joint_iface name="FL_thigh_pitch" lower="-1.571" upper="1.571"/>
    <xacro:joint_iface name="FL_knee_pitch"  lower="0.0"    upper="2.617"/>
    <xacro:joint_iface name="FR_hip_yaw"     lower="-0.785" upper="0.785"/>
    <xacro:joint_iface name="FR_thigh_pitch" lower="-1.571" upper="1.571"/>
    <xacro:joint_iface name="FR_knee_pitch"  lower="0.0"    upper="2.617"/>
    <xacro:joint_iface name="BL_hip_yaw"     lower="-0.785" upper="0.785"/>
    <xacro:joint_iface name="BL_thigh_pitch" lower="-1.571" upper="1.571"/>
    <xacro:joint_iface name="BL_knee_pitch"  lower="0.0"    upper="2.617"/>
    <xacro:joint_iface name="BR_hip_yaw"     lower="-0.785" upper="0.785"/>
    <xacro:joint_iface name="BR_thigh_pitch" lower="-1.571" upper="1.571"/>
    <xacro:joint_iface name="BR_knee_pitch"  lower="0.0"    upper="2.617"/>
  </ros2_control>

</robot>
```

Note: `lower` / `upper` are no longer used for effort (kept here for potential reuse, but the macro body ignores them and emits effort min/max ±2 N·m). If you prefer cleaner xacro, you can drop the unused params — keeping them avoids churn elsewhere.

- [ ] **Step 3: Replace gazebo.xacro plugin block**

Open `src/dog_robot_description/urdf/gazebo.xacro` and find the existing `<plugin filename="libgazebo_ros2_control.so">` block (it currently uses a `__PATH_TO_CONTROLLER__` placeholder). Replace just that `<gazebo>...</gazebo>` plugin block with:

```xml
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find dog_robot_config)/config/ros_control/ros_control.yaml</parameters>
    </plugin>
  </gazebo>
```

Keep everything else in `gazebo.xacro` (foot friction macros, self_collide, base material) unchanged.

- [ ] **Step 4: Verify xacro processes cleanly**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
xacro $(ros2 pkg prefix dog_robot_description)/share/dog_robot_description/urdf/dog_robot.urdf.xacro \
  | head -1
xacro $(ros2 pkg prefix dog_robot_description)/share/dog_robot_description/urdf/dog_robot.urdf.xacro \
  | grep -c "<command_interface name=\"effort\""
```

Expected: first command prints an XML declaration; second prints `12` (one effort interface per joint).

- [ ] **Step 5: check_urdf passes**

```bash
xacro $(ros2 pkg prefix dog_robot_description)/share/dog_robot_description/urdf/dog_robot.urdf.xacro \
  > /tmp/dog_robot.urdf
check_urdf /tmp/dog_robot.urdf 2>&1 | tail -3
```

Expected: `Successfully Parsed XML` and a brief tree summary.

- [ ] **Step 6: Rebuild description, then commit**

```bash
colcon build --packages-select dog_robot_description 2>&1 | tail -3
git add src/dog_robot_description/urdf/ros2_control.xacro src/dog_robot_description/urdf/gazebo.xacro
git commit -m "$(cat <<'EOF'
feat(description): switch ros2_control to effort interface for CHAMP

CHAMP's quadruped_controller drives joints via effort. The gazebo plugin
now reads its yaml from dog_robot_config so the controller list / gains
travel with the config package rather than being hardcoded in the URDF.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 9: bringup.launch.py (RViz-only, no Gazebo)

**Files:**
- Create: `src/dog_robot_config/launch/bringup.launch.py`

This launch is the "no Gazebo" path used to verify the joint maps before running physics. It's adapted from `champ/champ_config/launch/bringup.launch.py` with the description path swapped.

- [ ] **Step 1: Inspect the reference**

```bash
head -80 /home/nguyenvd/workspace/dog_robot/dog_robot_ws/src/champ/champ_config/launch/bringup.launch.py
```

Note how it declares `description_path`, `joints_map_path`, `links_map_path`, `gait_config_path` arguments and forwards them into `IncludeLaunchDescription(champ_bringup/bringup.launch.py)`.

- [ ] **Step 2: Write our bringup.launch.py**

Write file `src/dog_robot_config/launch/bringup.launch.py`:

```python
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
```

- [ ] **Step 3: Build and inspect**

```bash
colcon build --packages-select dog_robot_config 2>&1 | tail -3
ros2 launch dog_robot_config bringup.launch.py --print-description 2>&1 | head -15
```

Expected: prints a list of actions (DeclareLaunchArgument × 11, IncludeLaunchDescription). No exceptions.

- [ ] **Step 4: Commit**

```bash
git add src/dog_robot_config/launch/bringup.launch.py
git commit -m "$(cat <<'EOF'
feat(dog_robot_config): bringup.launch.py for no-Gazebo verification

Forwards dog_robot's description + joint/link/gait yamls into
champ_bringup. Lets us validate the joint maps in RViz before paying for a
3+ minute Gazebo init.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 10: gazebo.launch.py

**Files:**
- Create: `src/dog_robot_config/launch/gazebo.launch.py`

- [ ] **Step 1: Write gazebo.launch.py**

Write file `src/dog_robot_config/launch/gazebo.launch.py`:

```python
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
```

- [ ] **Step 2: Build and print description**

```bash
colcon build --packages-select dog_robot_config 2>&1 | tail -3
ros2 launch dog_robot_config gazebo.launch.py --print-description 2>&1 | head -10
```

Expected: prints actions, no exception.

- [ ] **Step 3: Commit**

```bash
git add src/dog_robot_config/launch/gazebo.launch.py
git commit -m "$(cat <<'EOF'
feat(dog_robot_config): gazebo.launch.py

Bringup + champ_gazebo IncludeLaunchDescription pair, with GAZEBO_MODEL_PATH
and GAZEBO_MODEL_DATABASE_URI patched in os.environ (SetEnvironmentVariable
does not propagate into IncludeLaunchDescription, see Gazebo Classic notes
in spec).

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 11: No-Gazebo bringup verification

This is the cheap gate before paying for a Gazebo run. If the joint or link maps disagree with the URDF, CHAMP's `quadruped_controller_node` logs `joint not found` here and dies.

- [ ] **Step 1: Kill any leftover sim processes (avoids stale gzserver collisions)**

```bash
pkill -9 -f gzserver 2>/dev/null
pkill -9 -f gzclient 2>/dev/null
pkill -9 -f "ros2 launch" 2>/dev/null
pkill -9 -f champ_base 2>/dev/null
pkill -9 -f robot_state_publisher 2>/dev/null
pkill -9 -f ekf_node 2>/dev/null
pkill -9 -f rviz2 2>/dev/null
sleep 3
pgrep -af "gz|champ|ekf|robot_state|rviz" | grep -v claude- || echo clean
```

Expected last line: `clean`.

- [ ] **Step 2: Build the full workspace fresh**

```bash
cd /home/nguyenvd/workspace/dog_robot/dog_robot_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build 2>&1 | tail -10
```

Expected: every package reports `Finished`. If anything fails, fix it before continuing.

- [ ] **Step 3: Launch bringup (no Gazebo)**

```bash
source install/setup.bash
rm -f /tmp/dog_robot_bringup.log
ros2 launch dog_robot_config bringup.launch.py rviz:=true > /tmp/dog_robot_bringup.log 2>&1 &
LAUNCH_PID=$!
```

- [ ] **Step 4: Wait for quadruped_controller to log readiness, then sample**

```bash
until grep -qE "quadruped_controller|joint not found|ERROR" /tmp/dog_robot_bringup.log 2>/dev/null; do
  sleep 2
done
echo "==="; head -50 /tmp/dog_robot_bringup.log
echo "=== errors ==="; grep -iE "joint not found|ERROR|Exception" /tmp/dog_robot_bringup.log | head -5
```

Pass criteria:
- No `joint not found` message.
- No `ERROR` lines apart from optional RViz panel warnings.
- A `robot_state_publisher: got segment FL_hip_link` style line for each of the 16 URDF links.

If any joint name mismatch shows up, edit `joints.yaml` / `links.yaml` in Task 5 to fix and rerun.

- [ ] **Step 5: Stop the launch and commit nothing (this is a checkpoint)**

```bash
kill -9 $LAUNCH_PID 2>/dev/null
pkill -9 -f rviz2 2>/dev/null
pkill -9 -f robot_state_publisher 2>/dev/null
pkill -9 -f champ_base 2>/dev/null
pkill -9 -f ekf_node 2>/dev/null
sleep 2
pgrep -af "rviz2|champ|ekf|robot_state" | grep -v claude- || echo clean
```

If any earlier task needed a config tweak to pass this checkpoint, fold that into its commit by amending (the workspace has no other consumer yet).

---

## Task 12: Gazebo spawn + controllers active

The first time this runs, gazebo Classic walks its model database. With `GAZEBO_MODEL_DATABASE_URI=""` in `gazebo.launch.py`, init should be ~30 s instead of ~180 s.

- [ ] **Step 1: Make sure no stale processes**

```bash
pkill -9 -f gzserver 2>/dev/null
pkill -9 -f gzclient 2>/dev/null
pkill -9 -f "ros2 launch" 2>/dev/null
pkill -9 -f champ_base 2>/dev/null
pkill -9 -f robot_state_publisher 2>/dev/null
pkill -9 -f ekf_node 2>/dev/null
sleep 3
pgrep -af "gz|champ|ekf|robot_state" | grep -v claude- || echo clean
```

Expected last line: `clean`.

- [ ] **Step 2: Launch Gazebo bringup**

```bash
cd /home/nguyenvd/workspace/dog_robot/dog_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
rm -f /tmp/dog_robot_gz.log
ros2 launch dog_robot_config gazebo.launch.py > /tmp/dog_robot_gz.log 2>&1 &
```

- [ ] **Step 3: Wait until both controllers report active (or fail)**

```bash
until grep -qE "Successfully loaded controller joint_group_effort_controller|Spawn service failed|ERROR.*controller" /tmp/dog_robot_gz.log; do
  sleep 5
done
grep -nE "Spawn status|Successfully loaded|ERROR|fail" /tmp/dog_robot_gz.log | tail -10
```

Pass criteria:
- A line `spawn_entity: Successfully spawned entity [dog_robot]`.
- A line `Successfully loaded controller joint_states_controller into state active`.
- A line `Successfully loaded controller joint_group_effort_controller into state active`.
- `ros2 control list_controllers` (run in a second shell with the same env) shows both `[active]`.

If the spawn fails with "Entity already exists", re-run Task 12 step 1 then retry — a stale gzserver is holding the model.

- [ ] **Step 4: Sample robot pose and confirm it spawned upright**

```bash
gz model -m dog_robot -i | head -22
```

Expected: `z` between 0.05 m and 0.30 m (robot above ground, sphere feet in contact). Orientation `w` near 1 (no significant tilt). If `z` is negative-infinity or huge, the URDF mesh resolution failed and `GAZEBO_MODEL_PATH` env didn't propagate — re-check Task 10 step 1's `os.environ` patch.

---

## Task 13: cmd_vel walking test

- [ ] **Step 1: Capture pose before commanding velocity**

```bash
gz model -m dog_robot -i | grep -A 4 "^pose" | head -8 > /tmp/dog_robot_pose_before.txt
cat /tmp/dog_robot_pose_before.txt
```

- [ ] **Step 2: Publish cmd_vel forward for 5 s**

```bash
source /opt/ros/humble/setup.bash
source /home/nguyenvd/workspace/dog_robot/dog_robot_ws/install/setup.bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1}}' &
CMD_PID=$!
sleep 5
kill -9 $CMD_PID 2>/dev/null
```

- [ ] **Step 3: Capture pose after, compute delta**

```bash
gz model -m dog_robot -i | grep -A 4 "^pose" | head -8 > /tmp/dog_robot_pose_after.txt
diff /tmp/dog_robot_pose_before.txt /tmp/dog_robot_pose_after.txt
```

Pass criteria:
- `x` change ≥ 0.3 m (so the robot moved forward at ≈ 0.06 m/s minimum sustained).
- `z` change | Δz | ≤ 0.05 m (robot stayed standing).
- `w` (orientation) stays near 1 (robot didn't tip).

- [ ] **Step 4: Repeat for lateral and yaw (sanity)**

```bash
# Lateral strafe (y axis)
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {y: 0.1}}' &
sleep 5 ; kill -9 $! 2>/dev/null
gz model -m dog_robot -i | grep -A 4 "^pose" | head -8

# Yaw
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.5}}' &
sleep 5 ; kill -9 $! 2>/dev/null
gz model -m dog_robot -i | grep -A 4 "^pose" | head -8
```

Pass criteria (relative to start of each command):
- Lateral: `| Δy | ≥ 0.2 m`, robot still upright.
- Yaw: heading change ≥ 1.5 rad, robot still upright.

- [ ] **Step 5: Stop sim and commit a marker**

```bash
pkill -9 -f gzserver 2>/dev/null
pkill -9 -f gzclient 2>/dev/null
pkill -9 -f "ros2 launch" 2>/dev/null
pkill -9 -f champ_base 2>/dev/null
pkill -9 -f robot_state_publisher 2>/dev/null
pkill -9 -f ekf_node 2>/dev/null
sleep 3
git commit --allow-empty -m "$(cat <<'EOF'
test: CHAMP + dog_robot Gazebo walk verified

ros2 launch dog_robot_config gazebo.launch.py spawns + activates both
controllers and the robot tracks /cmd_vel in x, y and yaw with the
default gains. See plan task 13.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

If any axis fails the criteria, do not commit — go to Task 14.

---

## Task 14: Gain tuning (only if Task 13 failed)

**Files:**
- Modify: `src/dog_robot_config/config/ros_control/ros_control.yaml`

If the robot bounces / oscillates: `p` too high.
If the robot collapses / can't lift body: `p` too low or effort limits too tight.
If the robot drifts when stationary: `i_clamp` too high.

- [ ] **Step 1: Pick a new gain set**

A reasonable bisection from the Task 6 default (`p: 20.0`):
- Oscillating → `p: 10.0`, keep `d: 0.2`.
- Collapsing → `p: 40.0`, `d: 0.4`, and effort `min/max ±4.0` in `ros2_control.xacro`.

- [ ] **Step 2: Edit `ros_control.yaml`**

Apply the new gain values uniformly to all 12 joint entries.

- [ ] **Step 3: Rebuild only the config package**

```bash
colcon build --packages-select dog_robot_config 2>&1 | tail -3
```

- [ ] **Step 4: Re-run Task 12 step 1 (kill leftovers), then Task 12 step 2, then Task 13**

Iterate steps 1–4 until Task 13 criteria pass.

- [ ] **Step 5: Commit the working gains**

```bash
git add src/dog_robot_config/config/ros_control/ros_control.yaml
git commit -m "$(cat <<'EOF'
tune(dog_robot_config): adjusted effort gains for stable walking

Tuned via Gazebo iteration — see plan task 14.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 15: Update teleop launch and final smoke test

**Files:**
- Modify: `src/dog_robot_control/launch/teleop.launch.py` (the file that wraps `teleop_keyboard` in a gnome-terminal — already exists, may need a refresh)

The user's teleop_keyboard already publishes `/cmd_vel`. We just confirm the launch is still wired correctly after `dog_robot_bringup` was removed.

- [ ] **Step 1: Inspect teleop.launch.py**

```bash
cat /home/nguyenvd/workspace/dog_robot/dog_robot_ws/src/dog_robot_control/launch/teleop.launch.py
```

It should run `teleop_keyboard` from `dog_robot_control` package and not reference any deleted package. If it does reference `dog_robot_bringup` or `dog_kinematics`, remove that line.

- [ ] **Step 2: Run Gazebo + teleop end-to-end**

In terminal A:
```bash
source /opt/ros/humble/setup.bash
source /home/nguyenvd/workspace/dog_robot/dog_robot_ws/install/setup.bash
ros2 launch dog_robot_config gazebo.launch.py
```

Wait for both controllers active (Task 12 criteria).

In terminal B:
```bash
source /opt/ros/humble/setup.bash
source /home/nguyenvd/workspace/dog_robot/dog_robot_ws/install/setup.bash
ros2 launch dog_robot_control teleop.launch.py
```

The teleop terminal opens. Press the forward key and observe robot walks in Gazebo.

- [ ] **Step 3: Kill everything and merge**

```bash
pkill -9 -f gzserver 2>/dev/null
pkill -9 -f gzclient 2>/dev/null
pkill -9 -f "ros2 launch" 2>/dev/null
pkill -9 -f teleop 2>/dev/null
sleep 2
git checkout main
git merge --no-ff feature/champ-integration -m "$(cat <<'EOF'
Merge feature/champ-integration

Switches dog_robot_ws to use vendored CHAMP for control + state estimation.
dog_kinematics, dog_gait and dog_robot_bringup are removed. Verified by
Gazebo walking tests on simple.world.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

Do NOT push (per user policy — only push on explicit request).

---

## Self-Review

- **Spec coverage:** Architecture (Tasks 1, 2, 3, 4), URDF modification (Task 8), `dog_robot_config` package contents (Tasks 4–7, 9, 10), bringup launches (Tasks 9, 10), error mitigations (Tasks 10, 11, 12 step 4), testing (Tasks 11, 12, 13), rollback (Task 1 step 1 + Task 15). All covered.
- **Placeholder scan:** No TBD / TODO / "add appropriate error handling" / "similar to Task N" markers.
- **Type/name consistency:** `dog_robot_config` package name, the 12 URDF joint names (`FL_hip_yaw` etc.), controller names (`joint_group_effort_controller`, `joint_states_controller`), launch arg names (`joints_map_path`, `links_map_path`, `gait_config_path`, `ros_control_file`) — used identically across tasks.
- **Open risks:** the `$(find dog_robot_config)/config/ros_control/ros_control.yaml` substitution in `gazebo.xacro` — Task 8 step 3 follows the CHAMP pattern that already works in `champ_ws`. If it hangs at Gazebo init, fall back to writing the literal absolute path or the placeholder + str.replace pattern from the previous dog_robot_description launch (this is a recoverable runtime fix, not a structural plan defect).
