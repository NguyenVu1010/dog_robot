# dog_robot_ws

ROS 2 workspace for a 12-DOF quadruped robot — kinematic-only branch. No
Gazebo / no `ros2_control`: `/cmd_vel` drives a Python gait + closed-form IK
that publishes `/joint_states` straight to `robot_state_publisher` for RViz.

## Packages

| Package | Build | Role |
|---|---|---|
| `dog_robot_description` | ament_cmake | URDF/xacro, meshes, `link_params.yaml`, `urdf_joints.yaml` |
| `dog_robot_kinematics` | ament_python | Pure-Python link-frame FK/IK (no ROS deps) |
| `dog_robot_kinematic_viz` | ament_python | Gait engine, ROS node, teleop, GUI, launch files, RViz config |

Gazebo / `ros2_control` / `dog_robot_control` / CHAMP config were stripped
in commit `c91f043`. This branch is the kinematic rig only.

## Kinematics

Each leg is a 3-DOF chain (hip roll, thigh pitch, knee pitch) modelled as a
sequence of fixed parent→child joint transforms (translation + rpy rotation)
each followed by a revolute Z rotation at that joint. Front and back legs use
distinct transforms (different CAD geometry), stored verbatim per leg —
nothing is averaged.

### Frames

- **Body B** — URDF root `base_link` (X forward, Y left, Z up).
- **Hip H_<leg>** — fixed `base_link → <leg>_hip_roll` joint transform.
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

### Forward / inverse kinematics

```python
from dog_robot_kinematics import load_link_params, fk_leg, ik_leg
lp = load_link_params("install/dog_robot_description/share/.../link_params.yaml", "FL")
foot_xyz_in_hip = fk_leg(lp, (theta_hip, theta_thigh, theta_knee))
theta_hip, theta_thigh, theta_knee = ik_leg(lp, foot_xyz_in_hip, knee_branch=+1)
```

`fk_leg` composes 6 transforms (3 fixed × 3 revolute-Z) in the hip frame.
`ik_leg` is closed-form: hip yaw from foot projection onto the hip plane,
then 2R planar solve in the thigh/shank plane. Raises `ValueError` for foot
targets on the hip axis or beyond reach.

### Verification

```bash
python3 -m pytest src/dog_robot_kinematics/test/
```

Covers FK/IK roundtrip, URDF chain ↔ `fk_leg` agreement across all four legs,
and the `knee_branch` sign convention.

## Kinematic rig

```
                  /cmd_vel  (5 axes — see Twist map below)
                     │
                     ▼
       BodyCommander   ──→  body_z / pitch_amount state
                     │              │
                     ▼              ▼
       FootTarget   per leg     extra_z input
                     │
                     ▼
       LegDriver    ──→  ik_leg  ──→  JointState
                     │
                     ▼  /joint_states (50 Hz)            /foot_trails
       robot_state_publisher ──→ /tf  ─────────────────→  RViz
```

`KinematicNode` (`src/dog_robot_kinematic_viz/.../kinematic_node.py`) owns
the timer, services, and publishers. Inactive legs (those not in
`active_legs`) publish `idle_joints` every tick so RViz always sees 12 joints.

### Launch modes

| Launch file | Adds on top of base (RSP + static_tf + KinematicNode + RViz) |
|---|---|
| `kinematic.launch.py` | nothing — publish `/cmd_vel` yourself |
| `kinematic_teleop.launch.py` | keyboard teleop in a new `gnome-terminal` |
| `kinematic_gui.launch.py` | Tk window with 5 sliders + Sit/Release buttons |
| `kinematic_single_leg.launch.py` | single-leg debug (`active_legs:=[FL]` etc.) |

The base is anchored at a static `world → base_link` transform — no physics.
`base_height` launch arg sets the anchor Z (default 0.20 m).

### One-shot run

```bash
./scripts/dog_relaunch_kinematic.sh
```

Kills stale processes, `colcon build --packages-select` the three packages,
sources `install/`, then `ros2 launch dog_robot_kinematic_viz
kinematic_teleop.launch.py`. Use `dog_kill_all.sh` standalone if a previous
run leaves `kinematic_node`/`rviz2`/`gnome-terminal` orphaned.

### `/cmd_vel` Twist map

The node consumes five axes:

| Axis | Effect | Default range |
|---|---|---|
| `linear.x` | trot forward / back (m/s) | uncapped at node; teleop caps ±0.20 |
| `linear.y` | trot strafe left / right (m/s) | same |
| `linear.z` | body-height velocity (m/s) — integrated into `body_z` | clamped to `[body_z_min, body_z_max]` (default ±0.03 m) |
| `angular.y` | pitch / rear-fold velocity (rad/s) — integrated into `pitch_amount` | clamped to `[pitch_min, pitch_max]` (default ±0.05) |
| `angular.z` | yaw rate (rad/s) — adds per-leg tangential foot velocity | uncapped at node; teleop caps ±0.80 |

`pitch_amount` is the continuous rear-fold input that pairs with the
`/sit` pose: each leg's foot-Z offset is sign-flipped per leg so the rear
folds while the front holds, giving a smooth sit / unsit transition.

Stop publishing → 0.5 s timeout → all five inputs decay to zero (gait halts
in place; `body_z` / `pitch_amount` integrated state is held).

### Named-pose API: `/sit` and `/release`

Two `std_srvs/Trigger` services lock the joints to a fixed snapshot:

```bash
ros2 service call /sit     std_srvs/srv/Trigger {}
ros2 service call /release std_srvs/srv/Trigger {}
```

While sit is engaged, gait + `/cmd_vel` are bypassed and `KinematicNode`
publishes the 12 joints in `sit_pose_joints` directly. `/release` returns
control to the gait engine.

The sit pose is hot-tunable via parameter:

```bash
ros2 param set /kinematic_node sit_pose_joints \
  "[0.0,-0.20,0.50, 0.0,-0.20,0.50, 0.0,0.30,-0.70, 0.0,0.30,-0.70]"
ros2 service call /sit std_srvs/srv/Trigger {}   # handler re-reads the param
```

Order is FL/FR/BL/BR × `hip_roll, thigh_pitch, knee_pitch`. Default values
live in `src/dog_robot_kinematic_viz/config/kinematic_params.yaml`. The
header comment there documents the safe range and the
`scripts/sweep_sit_pose.py` candidate-comparison output that was used to
pick them.

### Keyboard teleop keymap

`kinematic_teleop.launch.py` spawns `teleop_keyboard` inside its own
`gnome-terminal` (so it has a real TTY for raw-mode reads). All keys
publish a fresh `/cmd_vel` immediately on press:

| Key | Action |
|---|---|
| `w` / `s` | `linear.x` ± 0.02 m/s (cap ±0.20) |
| `a` / `d` | `linear.y` ± 0.02 m/s (cap ±0.20) |
| `r` / `f` | `linear.z` ± 0.02 m/s (cap ±0.20) — body up / down |
| `i` / `k` | `angular.y` ± 0.02 rad/s (cap ±0.20) — sit / unsit |
| `j` / `l` | `angular.z` ± 0.10 rad/s (cap ±0.80) — yaw |
| space | zero all five axes |
| `q` / Ctrl-C | quit |

### Tk GUI teleop

`kinematic_gui.launch.py` swaps the terminal teleop for a small Tk window
(`gui_teleop.py`): five sliders (one per axis) plus **Sit** / **Release**
buttons that call the services above. The window publishes `/cmd_vel` at 50 Hz
so slider state always matches the topic. Useful when you don't have a
spare terminal or want to drag axes continuously instead of stepping.

### Foot-tip trail

`KinematicNode` also publishes `/foot_trails` (`visualization_msgs/MarkerArray`)
— one LINE_STRIP per leg of recent foot-tip positions. The default RViz
config (`src/dog_robot_kinematic_viz/rviz/kinematic.rviz`) subscribes to it.
Max points per leg: `foot_trail_max_points` (default 300).

### Gait config

`src/dog_robot_kinematic_viz/config/kinematic_params.yaml`:

- `publish_rate` (Hz)
- `active_legs` — list of `FL`/`FR`/`BL`/`BR` (subset for single-leg debug)
- `idle_joints` — joints reported for legs not in `active_legs`
- `step_freq`, `stride_per_mps`, `swing_height`, `stance_phase_ratio` —
  trot parameters
- `body_z_min` / `body_z_max`, `pitch_min` / `pitch_max` — clamp ranges
  for the integrated state
- `sit_pose_joints` — 12-float sit snapshot (see above)

## Build & run notes

Setuptools 81 removed the `--editable` flag; colcon's symlink/develop path
for ament_python packages is broken on that version. The relaunch script
uses **plain copy-install** (`colcon build` with no `--symlink-install`),
which works fine for ament_python — `console_scripts` and `share/` files
both install correctly. After editing Python sources, re-run
`scripts/dog_relaunch_kinematic.sh` (or `colcon build --packages-select
dog_robot_kinematic_viz` + re-source) to pick up the changes.

`scripts/dog_kill_all.sh` is the SIGTERM→SIGKILL fallback for orphan
`kinematic_node` / `rviz2` / `gnome-terminal` / `teleop_keyboard` processes;
the relaunch script runs it first. Run it standalone if a previous launch
left RViz wedged.
