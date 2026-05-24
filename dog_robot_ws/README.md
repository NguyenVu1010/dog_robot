# dog_robot_ws

ROS 2 workspace for a 12-DOF quadruped robot.

## Packages

| Package | Build | Role |
|---|---|---|
| `dog_robot_description` | ament_python | URDF/xacro, meshes, ros2_controllers.yaml |
| `dog_robot_kinematics` | ament_python | Pure-Python DH FK/IK + leg config (no ROS deps) |
| `dog_robot_control` | ament_python | walker / stand controllers, gait engine, teleop |
| `dog_robot_kinematic_viz` | ament_cmake | Gazebo-free RViz rig (launch + rviz config only) |
| `dog_robot_config` | ament_cmake | CHAMP-era config (legacy) |

## Kinematics

The dog_robot uses Modified Denavit-Hartenberg (Craig) convention. Each leg is a
3-DOF chain (hip yaw, thigh pitch, knee pitch). One symmetric DH table covers
all four legs; per-leg variation lives in the base→hip fixed transform.

### Frames

- **Body B** — URDF root `base_link` (X forward, Y left, Z up).
- **Hip H_<leg>** — fixed transform per leg; Z_H along the hip yaw axis (= body
  X), X_H downward (= -body Z).
- **DH frames 1-3** — at each joint, Z along that joint's axis.

### DH Table

| i | α_{i-1} | a_{i-1}            | d_i | θ_i      |
|---|---------|---------------------|-----|----------|
| 1 | 0       | 0                  | 0   | θ_hip    |
| 2 | -π/2    | L_hh = 0.02553 m   | 0   | θ_thigh  |
| 3 | 0       | L_th = 0.11725 m   | 0   | θ_knee   |
| F | 0       | L_sh = 0.07043 m   | 0   | 0        |

Lengths come from `src/dog_robot_description/scripts/compute_dh_lengths.py`,
which averages the four legs' CAD measurements.

### Forward kinematics

```python
from dog_robot_kinematics import DHParams, fk_leg
dh = DHParams(L_hh=0.02553, L_th=0.11725, L_sh=0.07043)
foot_xyz = fk_leg(dh, (theta_hip, theta_thigh, theta_knee))
```

### Inverse kinematics

```python
from dog_robot_kinematics import ik_leg
theta_hip, theta_thigh, theta_knee = ik_leg(dh, foot_xyz_in_hip_frame,
                                            knee_direction=+1)
```

Closed-form 2R planar + hip yaw decomposition. Raises `ValueError` for foot
targets on the hip yaw axis or beyond reach.

### Per-leg base→hip transforms

| Leg | base→hip xyz (m)            | base→hip rpy (rad)    | Mirror |
|-----|------------------------------|------------------------|--------|
| FL  | ( 0.0748,  0.0400, 0.0351)  | (0, π/2, 0)           | +1     |
| FR  | ( 0.0748, -0.0400, 0.0351)  | (0, π/2, π)           | -1     |
| BL  | (-0.0748,  0.0400, 0.0351)  | (0, π/2, 0)           | +1     |
| BR  | (-0.0748, -0.0400, 0.0351)  | (0, π/2, π)           | -1     |

The right-side `π` yaw places right legs on the opposite side of body Y while
keeping the same DH table — IK and FK code is identical for all four legs.

### Verification

```bash
python3 -m pytest src/dog_robot_kinematics/test/
```

Tests check FK/IK roundtrip (200 random configs) and URDF chain ↔ kinematics
module agreement on 40 random joint angle sets across all four legs.

## Kinematic-only visualization (no Gazebo)

`dog_robot_kinematic_viz` is the Gazebo-free debugging rig. cmd_vel drives the
full gait + IK pipeline; joint angles render straight in RViz.

```
              /cmd_vel
                 │
                 ▼
       walker_controller (kinematic_mode=True)
                 │
                 ▼  /joint_states (50 Hz)
       robot_state_publisher ──→ /tf
                                    │
                                    ▼
                                  RViz
```

No `gzserver`, no `ros2_control_node`, no JTC. Walker publishes `JointState`
directly when `kinematic_mode=True`.

### Run

```bash
./scripts/dog_relaunch_kinematic.sh
```

The script kills stale processes, rebuilds, and launches
`dog_robot_kinematic_viz/kinematic_teleop.launch.py` (RSP + walker + teleop +
RViz). The teleop opens its own `gnome-terminal` window — WASD/JL keys publish
`/cmd_vel` at 10 Hz continuously (so walker's 0.5 s cmd_vel timeout never
fires mid-key-hold).

Bare RViz without teleop:

```bash
ros2 launch dog_robot_kinematic_viz kinematic.launch.py
# in a separate terminal:
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}}' -r 10
```

### Teleop keymap

| Key | Action |
|---|---|
| `w` / `s` | linear.x ± 0.02 m/s (cap 0.15) |
| `a` / `d` | linear.y ± 0.02 m/s (cap 0.15) |
| `j` / `l` | angular.z ± 0.05 rad/s (cap 0.50) |
| space | zero all velocities |
| `x` or Ctrl-C | quit |

## Walking (Gazebo)

`walker_controller` is the production controller. Subsumes `stand_controller`:
when `/cmd_vel` is zero, walker holds the stand pose; non-zero cmd_vel triggers
a trot gait (Bernstein-Bezier swing + linear stance) computed in Python and
converted to joint commands via DH IK.

### Run

```bash
./scripts/dog_relaunch_walk.sh
```

Same script pattern as the kinematic version — wipes build/log, rebuilds
non-pip packages, then launches `walk.launch.py` (Gazebo + spawn + JTC +
walker). Robot ramps from the URDF-set stand pose; the spawn position is
already at stand height so there is no contact impulse at boot.

Then publish cmd_vel:

```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}}' -r 10
```

Twist field map:
- `linear.x` — forward / backward (m/s), capped at `gait.max_linear_velocity_x`
- `linear.y` — sideways (m/s)
- `angular.z` — yaw (rad/s)

Stop publishing or send zeros → 0.5 s timeout → walker decays back to stand.

### Gait config

`dog_robot_control/config/walker_params.yaml` exposes tunable gait params
(stance duration, swing height, velocity caps, knee direction, etc.). See
inline comments in the YAML.

### Architecture

`/cmd_vel` → `BodyController.pose_command` → `LegController.velocity_command`
(`phase_generator` + `trajectory_planner` per leg) → rotate each foot to its
DH hip frame → `ik_leg` (from `dog_robot_kinematics`) → `JointTrajectory` →
joint_trajectory_controller → Gazebo. Python layout:
`dog_robot_control/dog_robot_control/gait/`.

## Build & run notes

Setuptools 81 removed the `--editable` flag, which breaks colcon's default
build path for ament_python packages. The workaround used here:

- `dog_robot_control` and `dog_robot_kinematics` are installed via
  `pip install -e <absolute path>`; their `install/` trees are preserved
  across `colcon build` cycles.
- The relaunch scripts (`scripts/dog_relaunch_{walk,kinematic}.sh`) wipe
  `build/`, `log/`, and most of `install/` but keep `install/dog_robot_control`
  intact, then run `colcon build --packages-skip dog_robot_control
  dog_robot_kinematics`.
- Launch + config files under `share/` are file copies (not symlinks), so the
  scripts rsync them after the build to pick up edits.

`scripts/dog_kill_all.sh` is the SIGTERM→SIGKILL fallback for orphan
`gzserver`/`controller_node`/`walker_controller` processes; always run it
before relaunching to avoid the next launch hanging on a wedged orphan.

## Stand controller (deprecated)

`stand_controller` + `stand.launch.py` remain in the tree for reference but
are deprecated. Use the walker (with cmd_vel=0 → stand pose) or the kinematic
viz rig instead.
