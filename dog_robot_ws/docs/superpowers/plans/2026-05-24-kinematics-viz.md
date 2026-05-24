# Kinematics-only visualization — Implementation Plan

> **For agentic workers:** Execute task-by-task. Each task is a checkpoint.

**Goal:** Extract kinematics into `dog_robot_kinematics` package, add `kinematic_mode` to walker, ship a Gazebo-free `kinematic.launch.py` driving RViz from `/cmd_vel`.

**Architecture:** New ament_python package owns kinematics math + viz launch + rviz config. walker_controller gets a flag to publish JointState instead of JointTrajectory. Spec: `docs/superpowers/specs/2026-05-24-kinematics-viz-design.md`.

**Tech Stack:** ROS 2 Humble, ament_python, numpy, RViz2.

---

### Task 1: Scaffold `dog_robot_kinematics` package

**Files:**
- Create: `src/dog_robot_kinematics/package.xml`
- Create: `src/dog_robot_kinematics/setup.py`
- Create: `src/dog_robot_kinematics/setup.cfg`
- Create: `src/dog_robot_kinematics/resource/dog_robot_kinematics` (empty marker)
- Create: `src/dog_robot_kinematics/dog_robot_kinematics/__init__.py`

**Acceptance:** `colcon build --packages-select dog_robot_kinematics` succeeds. `ros2 pkg list | grep dog_robot_kinematics` returns the package.

### Task 2: Move kinematics math + config + tests

**Files:**
- Move: `src/dog_robot_control/dog_robot_control/kinematics_dh.py` → `src/dog_robot_kinematics/dog_robot_kinematics/kinematics_dh.py`
- Move: `src/dog_robot_control/dog_robot_control/leg_config.py` → `src/dog_robot_kinematics/dog_robot_kinematics/leg_config.py`
- Move: `src/dog_robot_control/config/dh_params.yaml` → `src/dog_robot_kinematics/config/dh_params.yaml`
- Move all `dog_robot_control/test/test_kinematics_dh*.py`, `test_leg_config*.py`, `test_urdf*kinematics*.py` → `src/dog_robot_kinematics/test/`
- Modify: `src/dog_robot_kinematics/dog_robot_kinematics/__init__.py` — re-export public API
- Modify: `src/dog_robot_kinematics/setup.py` — install config/, test glob
- Modify any test import paths from `dog_robot_control.kinematics_dh` → `dog_robot_kinematics.kinematics_dh`

**Acceptance:** `colcon test --packages-select dog_robot_kinematics` runs all moved tests green.

### Task 3: Update `dog_robot_control` to import from new package

**Files:**
- Modify: `src/dog_robot_control/dog_robot_control/walker_controller.py` — change `from dog_robot_control.kinematics_dh import ...` to `from dog_robot_kinematics import ...` (and same for leg_config).
- Modify: `src/dog_robot_control/dog_robot_control/stand_controller.py` — same import update.
- Modify: `src/dog_robot_control/dog_robot_control/gait/leg_controller.py` and any other gait file referencing kinematics — same import update.
- Modify: `src/dog_robot_control/package.xml` — add `<depend>dog_robot_kinematics</depend>`.

**Acceptance:** `colcon test --packages-select dog_robot_control` still passes (any walker integration tests).

### Task 4: Add `kinematic_mode` to walker_controller

**Files:**
- Modify: `src/dog_robot_control/dog_robot_control/walker_controller.py`

Changes:
1. Declare param `kinematic_mode: bool = false`.
2. Add `JointState` import (`from sensor_msgs.msg import JointState`).
3. In `__init__`, branch on `kinematic_mode`:
   - true → create `/joint_states` publisher (`self.pub_js`), call `_compute_stand_target()` directly to populate `ramp_target`, set `start_angles = ramp_target.copy()`, `ramp_done = True`, do NOT subscribe to `/joint_states`.
   - false → existing path.
4. In `_tick`, after computing `q`, branch:
   - kinematic_mode → publish `JointState(header.stamp=now, name=joint_order, position=q.tolist())`.
   - else → existing JointTrajectory publish path.

**Acceptance:** Walker runs with `kinematic_mode:=true` and publishes `/joint_states` at ~50 Hz without `/joint_states` ever being subscribed externally; runs with `kinematic_mode:=false` and behaves identically to today.

### Task 5: Create RViz config + launch file

**Files:**
- Create: `src/dog_robot_kinematics/rviz/kinematic.rviz` — copy of `dog_robot_description/rviz/dog_robot.rviz`.
- Create: `src/dog_robot_kinematics/launch/kinematic.launch.py` — RSP + walker (kinematic_mode=true) + rviz2 with the new config. URDF includes the existing xacro + controllers_yaml arg (xacro requires it).
- Modify: `src/dog_robot_kinematics/setup.py` — install rviz/ and launch/.

**Acceptance:** `ros2 launch dog_robot_kinematics kinematic.launch.py` opens RViz with robot visible. `ros2 topic hz /joint_states` ≈ 50 Hz. `ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}}'` makes legs cycle visibly.

### Task 6: Smoke + regression

- Run unit tests across all packages: `colcon test --packages-select dog_robot_kinematics dog_robot_control`.
- Launch `kinematic.launch.py`, publish `cmd_vel{linear:{x:0.1}}` for 5 s, confirm IK does not error.
- Launch `walk.launch.py` (Gazebo) — must still behave like before the refactor (joint angles settle to stand pose, no explosion).

**Acceptance:** All tests pass; both launches work.
