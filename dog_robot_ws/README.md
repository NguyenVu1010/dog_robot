# dog_robot_ws

ROS 2 workspace for a 12-DOF quadruped robot.

## Packages

| Package | Build | Role |
|---|---|---|
| `dog_robot_description` | ament_python | URDF/xacro, meshes, ros2_controllers.yaml |
| `dog_robot_kinematics` | ament_python | Pure-Python DH FK/IK + leg config (no ROS deps) |
| `dog_robot_control` | ament_python | walker / stand controllers, gait engine, teleop |
| `dog_robot_kinematic_viz` | ament_cmake | Gazebo-free RViz rig (launch + rviz config; reuses walker for IK) |
| `dog_robot_config` | ament_cmake | CHAMP-era config (legacy) |

## Kinematics

Each leg is a 3-DOF chain (hip roll, thigh pitch, knee pitch) modelled as a
sequence of fixed parent→child joint transforms (translation + rpy rotation)
each followed by a revolute Z rotation at that joint. Front and back legs use
distinct transforms (different CAD geometry), stored verbatim per leg —
nothing is averaged.

### Frames

- **Body B** — URDF root `base_link` (X forward, Y left, Z up).
- **Hip H_<leg>** — fixed `base_link → <leg>_hip_yaw` joint transform.
- **Thigh / Shank / Foot** — each at the parent joint's location; Z along that
  joint's revolute axis.

### Source of truth

`src/dog_robot_description/config/link_params.yaml` is auto-generated from
CAD by `src/dog_robot_description/scripts/derive_joint_frames.py`. It holds:

- Scalar gait lengths `L_hh`, `L_th`, `L_sh` (full joint-to-joint distances).
- For each leg (FL/FR/BL/BR), three `(xyz, rpy)` blocks:
  `hip_to_thigh`, `thigh_to_knee`, `knee_to_foot`. Each is the rigid transform
  in the parent joint's frame, with Z aligned to the parent joint's axis.

The URDF macro and the kinematics module both read this YAML, so FK/IK and
the rendered model stay in lock-step.

### Forward kinematics

```python
from dog_robot_kinematics import load_link_params, fk_leg
lp = load_link_params("install/dog_robot_description/share/.../link_params.yaml", "FL")
foot_xyz_in_hip = fk_leg(lp, (theta_hip, theta_thigh, theta_knee))
```

`fk_leg` composes 6 transforms (3 fixed × 3 revolute-Z) in the hip frame.

### Inverse kinematics

```python
from dog_robot_kinematics import ik_leg
theta_hip, theta_thigh, theta_knee = ik_leg(lp, foot_xyz_in_hip,
                                            knee_branch=+1)
```

Closed-form: hip yaw from foot projection onto the hip plane, then 2R planar
solve in the thigh/shank plane. Raises `ValueError` for foot targets on the
hip axis or beyond reach.

### Verification

```bash
python3 -m pytest src/dog_robot_kinematics/test/
```

Tests cover: FK/IK roundtrip (random configs), URDF chain ↔ `fk_leg`
agreement across all four legs, and the `knee_branch` sign convention.

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

### Spawn pose

`ros2_control.xacro` sets each joint's `initial_value` to the bent stand pose
(`thigh=-0.4146`, `knee=1.1498`) so the plugin reports that pose at Load()
before the first physics tick. Both launch files spawn the body at `z=0.16`,
which puts the feet ~10 mm above the ground for a gentle settle. Spawning
straight legs at `z=0.18` instead pushed the foot 18 mm into the ground at
boot, triggering a 360 N contact impulse that exploded the robot.

The `dog_robot_kinematic_viz` RViz config lives in
`src/dog_robot_kinematic_viz/rviz/kinematic.rviz`. Both `kinematic.launch.py`
and `kinematic_teleop.launch.py` load it from that package (previously they
referenced an old copy under `dog_robot_control/rviz/`).

## Stand controller (deprecated)

`stand_controller` + `stand.launch.py` remain in the tree for reference but
are deprecated. Use the walker (with cmd_vel=0 → stand pose) or the kinematic
viz rig instead.
