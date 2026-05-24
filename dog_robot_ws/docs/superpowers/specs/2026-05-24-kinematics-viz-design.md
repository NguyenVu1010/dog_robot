# Kinematics-only visualization — Design

**Date:** 2026-05-24
**Status:** Approved (user authorized auto-implementation)
**Scope:** Extract kinematics math into a standalone ROS 2 package and add a Gazebo-free launch that drives the robot model in RViz from `/cmd_vel` (walker → IK → joint_states → TF → RViz).

---

## Goals

1. Test the full `cmd_vel → gait pipeline → IK → joint angles` chain visually, without Gazebo or physics.
2. Promote kinematics math (`kinematics_dh.py`, `leg_config.py`, `dh_params.yaml`, their tests) into its own package `dog_robot_kinematics` so it has a clear boundary and can be reused.
3. Same code path as Gazebo run: same `walker_controller`, same gait engine, same IK. Only the output sink differs (`JointState` instead of `JointTrajectory`).

## Non-goals

- No Gazebo, no ros2_control, no JTC, no contact physics.
- No closed-loop response (no sensor feedback, no IMU).
- No new visualization markers (foot trails, hip axes) — minimal RViz only.
- No changes to gait/ subpackage location (stays in dog_robot_control).
- No changes to URDF or to `dog_robot_description`.

---

## Architecture

```
              /cmd_vel (Twist)
                   │
                   ▼
       ┌──────────────────────────────────────┐
       │  walker_controller (kinematic_mode)  │
       │  ─ gait pipeline                     │
       │  ─ ik_leg per leg                    │
       │  ─ publish JointState (50 Hz)        │
       └─────────────┬────────────────────────┘
                     ▼
                /joint_states
                     │
                     ▼
            robot_state_publisher  ──→  /tf
                                          │
                                          ▼
                                       rviz2
```

No Gazebo, no `ros2_control_node`, no `joint_state_broadcaster`, no JTC.

## Package layout

```
src/
  dog_robot_kinematics/                       ← NEW ament_python package
    package.xml
    setup.py
    setup.cfg
    resource/dog_robot_kinematics
    dog_robot_kinematics/
      __init__.py                             ← re-exports public API
      kinematics_dh.py                        ← MOVED from dog_robot_control
      leg_config.py                           ← MOVED from dog_robot_control
    config/
      dh_params.yaml                          ← MOVED from dog_robot_control/config
    launch/
      kinematic.launch.py                     ← NEW
    rviz/
      kinematic.rviz                          ← NEW (minimal)
    test/
      test_kinematics_dh.py                   ← MOVED
      test_leg_config.py                      ← MOVED
      test_urdf_kinematics_consistency.py     ← MOVED (the URDF↔IK roundtrip test)
  dog_robot_control/
    dog_robot_control/
      walker_controller.py                    ← MODIFIED (kinematic_mode param)
      stand_controller.py                     ← MODIFIED (import path only)
      gait/                                   ← unchanged (stays here)
    package.xml                               ← add <depend>dog_robot_kinematics</depend>
```

## Components

### `dog_robot_kinematics` (new package)

Pure-Python library, zero `rclpy` imports in the math files.

**Public API** (`__init__.py`):

```python
from dog_robot_kinematics.kinematics_dh import DHParams, fk_leg, ik_leg, mdh_transform
from dog_robot_kinematics.leg_config import LegConfig, LEGS
```

Consumers (`dog_robot_control`, plus any future package) import from
`dog_robot_kinematics` only.

`dh_params.yaml` ships under `share/dog_robot_kinematics/config/` so launches
can reference it via `FindPackageShare`.

### `walker_controller.py` changes

Add one parameter:

```python
self.declare_parameter("kinematic_mode", False)
self.kinematic_mode = bool(self.get_parameter("kinematic_mode").value)
```

In `__init__`, when `kinematic_mode=true`:

- Replace the `JointTrajectory` publisher on `/joint_trajectory_controller/joint_trajectory` with a `JointState` publisher on `/joint_states`.
- Skip the "wait for `/joint_states` before ramping" path. Set `start_angles = compute_stand_target()` immediately and mark `ramp_done = true`, so the first tick already publishes the stand pose and subsequent ticks add gait deltas as cmd_vel arrives.
- Do NOT subscribe to `/joint_states` in kinematic mode (we are the producer, not the consumer).

In `_tick`, after computing the 12-vector `q`:

```python
if self.kinematic_mode:
    msg = JointState()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.name = self.joint_order
    msg.position = q.tolist()
    self.pub_js.publish(msg)
else:
    # existing JointTrajectory path, unchanged
```

### `kinematic.launch.py`

Inside the new package. No launch args.

```python
def generate_launch_description():
    descr = FindPackageShare("dog_robot_description")
    ctrl  = FindPackageShare("dog_robot_control")
    kin   = FindPackageShare("dog_robot_kinematics")

    urdf_xacro       = PathJoinSubstitution([descr, "urdf", "dog_robot.urdf.xacro"])
    controllers_yaml = PathJoinSubstitution([descr, "config", "ros2_controllers.yaml"])
    walker_params    = PathJoinSubstitution([ctrl,  "config", "walker_params.yaml"])
    rviz_cfg         = PathJoinSubstitution([kin,   "rviz",   "kinematic.rviz"])

    robot_description = {"robot_description": Command([
        FindExecutable(name="xacro"), " ", urdf_xacro,
        " controllers_yaml_path:=", controllers_yaml,
    ])}

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               parameters=[robot_description], output="screen")
    walker = Node(package="dog_robot_control", executable="walker_controller",
                  parameters=[walker_params, {"kinematic_mode": True}],
                  output="screen")
    rviz = Node(package="rviz2", executable="rviz2",
                arguments=["-d", rviz_cfg], output="screen")

    return LaunchDescription([rsp, walker, rviz])
```

Notes:
- The xacro still requires `controllers_yaml_path` to expand because of the gazebo_ros2_control `<plugin>` tag. That tag is harmless at RViz time (it lives inside a `<gazebo>` block that RViz ignores) — we just need the URDF to parse.
- `walker_params.yaml` ships nominal_height, gait timings, joint_order. Same file the Gazebo walk uses. No duplication.

### `kinematic.rviz`

Copy of `dog_robot_description/rviz/dog_robot.rviz`. No Gazebo-specific displays exist there now, so it's essentially identical content under a new path. Lives under `dog_robot_kinematics/rviz/` so the verification rig is self-contained.

## Data flow per tick

1. cmd_vel callback updates `req_vel` (Twist).
2. 50 Hz timer fires `_tick`.
3. `body_controller.pose_command(req_pose)` → 4 foot positions in body frame.
4. `leg_controller.velocity_command(feet, req_vel, t_now)` → gait deltas applied.
5. Per leg: rotate to hip frame, call `ik_leg` → 3 joints.
6. Build `JointState` (kinematic_mode) or `JointTrajectoryPoint` (normal mode).
7. Publish.

In kinematic mode steps 3–7 are exactly the gait + IK code path that runs against Gazebo. The only diverging line is step 6.

## Error handling

- IK out-of-reach: log warn, skip publish for this tick (joint_states sticks at last value). Same behavior as today's walker.
- cmd_vel timeout: req_vel zeros, gait decays to stand pose (phase_generator zeros at zero velocity). Same as today.
- No `/joint_states` subscription in kinematic mode → no "missing joint_states" warning path.
- xacro/URDF parse failure: launch fails fast (same as today's launches).

## Testing strategy

### Unit (no ROS)

Move existing tests verbatim to `dog_robot_kinematics/test/`:
- `test_kinematics_dh.py` — FK + IK roundtrip + sign conventions.
- `test_leg_config.py` — LEGS tuple integrity.
- `test_urdf_kinematics_consistency.py` — URDF↔IK consistency check.

Run all under `colcon test --packages-select dog_robot_kinematics`.

### Smoke test (with ROS)

After implementation:

```bash
ros2 launch dog_robot_kinematics kinematic.launch.py &
sleep 5
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}}'
# Verify in RViz: legs cycle visually.
ros2 topic hz /joint_states  # expect ~50 Hz
ros2 topic echo /joint_states --once  # 12 names + 12 positions
```

Acceptance:
- `/joint_states` publishes at 50 Hz ± 1 Hz.
- `/tf` shows base_link → each link with sane values (no NaN, foot z < 0 below body).
- Visual: with `vx=0.1`, legs alternate in trot pattern; with `wz=0.3`, robot turns in place (kinematically — body doesn't move in RViz because no odometry, but feet sweep in a yaw pattern).
- IK never errors for `vx ∈ [-0.15, 0.15]`, `vy ∈ [-0.08, 0.08]`, `wz ∈ [-0.5, 0.5]`.

### Regression on existing flows

- `ros2 launch dog_robot_control walk.launch.py` (Gazebo) still works — walker default `kinematic_mode=false`, behavior unchanged.
- `ros2 launch dog_robot_control stand.launch.py` still works — stand_controller import path updated, no behavior change.
- Existing `colcon test` packages all pass.

## Migration / cleanup

- Delete `src/dog_robot_control/dog_robot_control/{kinematics_dh,leg_config}.py` after move; the new package becomes the single source.
- Delete `src/dog_robot_control/config/dh_params.yaml`; new location is canonical.
- Delete the moved test files from `dog_robot_control/test/`.
- `walker_params.yaml` keeps the `dh:` block — walker reads DH params from its own params file (not the kinematics package's yaml). Both files exist; kinematics yaml is for any future tool that wants to read DH standalone.
- `dog_robot_kill_all.sh` already includes `walker_controller`; no script change needed.
- README: short section under Kinematics pointing at the new package + `kinematic.launch.py`.

## Open questions (resolved during brainstorming)

- ✅ Scope: full cmd_vel → joint pipeline, visualized; not standalone FK or IK sliders.
- ✅ JT→JS path: walker publishes JointState directly via `kinematic_mode` flag.
- ✅ Module split: new package `dog_robot_kinematics` (kinematics_dh + leg_config + dh_params.yaml + tests).
- ✅ RViz scope: RobotModel + TF + Grid only — no markers.
- ✅ Walker code structure: flag inside `walker_controller`, not a separate node.
