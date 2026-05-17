# CHAMP Integration into dog_robot_ws — Design

**Date**: 2026-05-17
**Status**: Approved by user, ready for plan
**Branch**: `feature/champ-integration` (to be created)

## Motivation

The custom kinematics module (`dog_kinematics`) ports the SpotMicroAI IK formulas, but its body-frame transform and hip-yaw axis convention do not match REP-103 nor the URDF axes that the rest of the stack uses. The result: `STAND` commands snap joints to physically wrong targets, robot tips, walking never demonstrated.

The CHAMP framework (`chvmp/champ`, `ros2` branch + `libchamp` submodule) implements a Spot Micro-class quadruped controller that has been demonstrated walking in Gazebo Classic with `gazebo_ros2_control` in this same environment (see `champ_ws/` proof of concept — robot moved 0.42 m on `cmd_vel` x=0.2 m/s, z stayed at standing height).

This spec replaces the broken in-house controller/IK with the working CHAMP stack while keeping the custom CAD-derived URDF.

## Goals

- Reuse `dog_robot_description` (custom URDF + CAD meshes) without re-doing CAD work
- Reuse the working `teleop_keyboard` so the input path the user already knows still applies
- Adopt CHAMP's controller + state estimator + gait planner — proven working in `champ_ws`
- Robot stands stably on spawn and walks in all `cmd_vel` directions

## Non-Goals

- Replacing the CAD/URDF authoring pipeline
- Implementing SIT / LIE states (CHAMP supports OFF / STAND / WALK; SIT and LIE are out of scope for this spec)
- Hardware sim2real — Gazebo only

## Architecture

```
dog_robot_ws/src/
├── champ/                            # Copied from champ_ws/src/champ + libchamp submodule
│   ├── champ/                        # Header-only C++ lib (with libchamp/ submodule)
│   ├── champ_base/                   # Quadruped controller + state estimation nodes
│   ├── champ_bringup/                # Base bringup launch
│   ├── champ_gazebo/                 # Gazebo helpers (contact sensors, spawn)
│   ├── champ_msgs/
│   ├── champ_navigation/
│   ├── champ_config/                 # NOT used (kept for reference; champ_description not used)
│   └── champ_description/            # NOT used
├── dog_robot_description/            # KEEP — modify URDF to add ros2_control block (effort interface)
├── dog_robot_config/                 # NEW — replaces both spotmicro_config and our old bringup
│   ├── config/
│   │   ├── joints/joints.yaml        # Map FL/FR/BL/BR slots to FL_hip_yaw / FL_thigh_pitch / FL_knee_pitch
│   │   ├── links/links.yaml          # Map slots to FL_hip_link / FL_thigh_link / FL_shank_link / FL_foot_link
│   │   ├── gait/gait.yaml            # knee_orientation, stance_duration, swing_height, step_height
│   │   └── ros_control/ros_control.yaml  # joint_group_effort_controller + joint_states_controller
│   ├── launch/
│   │   ├── gazebo.launch.py          # Adapt champ_config/gazebo.launch.py: point to dog_robot_description
│   │   └── bringup.launch.py         # Adapt champ_config/bringup.launch.py
│   └── worlds/
│       └── simple.world              # Ground plane + sun
└── dog_robot_control/                # KEEP teleop only
    └── dog_robot_control/teleop_keyboard.py  # Publishes /cmd_vel
```

**Deleted packages**: `dog_kinematics`, `dog_gait`, `dog_robot_bringup`, and `dog_robot_control/controller_node.py` (the file, not the package).

## Components

### 1. `dog_robot_description` — modify

- Add `<ros2_control>` block referencing all 12 joints with `effort` `command_interface` (was `position`)
- Add gazebo plugin block: `<plugin filename="libgazebo_ros2_control.so">` pointing `<parameters>` to literal absolute path of `dog_robot_config/config/ros_control/ros_control.yaml` (use placeholder + string-replace pattern already proven in `dog_robot_description/launch/gazebo.launch.py`)
- Keep joint NAMES as-is (`FL_hip_yaw`, `FL_thigh_pitch`, `FL_knee_pitch`, `FL_foot_link`) — CHAMP discovers them through `joints.yaml` mapping, no rename required
- Keep joint axes as-is — already match `spotmicro_description` (`hip` axis `(1,0,0)`, `thigh`/`knee` axis `(0,1,0)`)
- Keep CAD meshes

### 2. `dog_robot_config` — new

Mimics `chvmp/robots/configs/spotmicro_config` layout. Key files:

- `joints.yaml`: 12 entries mapping CHAMP slots (e.g. `lf_hip`) to the URDF joint names. Verified against URDF via `check_urdf`.
- `links.yaml`: mapping leg link names so state_estimation can compute foot FK.
- `gait.yaml`: trotting parameters tuned for ~2 kg robot:
  - `pantograph_leg: false`
  - `stance_duration: 0.25`
  - `nominal_height: 0.18` (L3 + L4 = 0.224 m fully extended; bend knees ~0.5 rad → effective leg length ≈ 0.18 m for stable stance)
  - `swing_height: 0.04`
  - `max_linear_velocity_x: 0.3`, `max_angular_velocity_z: 1.0` (small robot, conservative limits)
- `ros_control.yaml`: declares two controllers
  - `joint_group_effort_controller` — type `joint_trajectory_controller/JointTrajectoryController`, interface `effort`
  - `joint_states_controller` — type `joint_state_broadcaster/JointStateBroadcaster`
- `launch/gazebo.launch.py`: clone of `champ_config/launch/gazebo.launch.py` with `description_path` swapped to `dog_robot_description/urdf/dog_robot.urdf.xacro`, and `GAZEBO_MODEL_PATH` / `GAZEBO_MODEL_DATABASE_URI` set in `os.environ` at top of `generate_launch_description()` (pattern already used in the existing `dog_robot_description/launch/gazebo.launch.py`).
- `launch/bringup.launch.py`: clone of `champ_config/launch/bringup.launch.py`, swap description path to dog_robot.
- `worlds/simple.world`: ground_plane + sun + `<gravity>0 0 -9.81</gravity>` at world level (not inside `<physics>` — that bug already documented).

### 3. `dog_robot_control` — trim

Keep only `teleop_keyboard.py` and its launch entry. Delete `controller_node.py` and its `console_scripts` entry. The teleop publishes to `/cmd_vel`, which CHAMP's quadruped_controller subscribes to.

### 4. Deletions

- `dog_kinematics/` — broken IK
- `dog_gait/` — superseded by CHAMP gait planner
- `dog_robot_bringup/` — superseded by `dog_robot_config/launch`

## Data Flow

```
teleop_keyboard ─/cmd_vel─► champ_base/quadruped_controller
                              │ (gait planner + foot trajectory + IK)
                              ▼
              /joint_group_effort_controller/joint_trajectory
                              │
                              ▼
                      joint_group_effort_controller (JTC, effort)
                              │
                              ▼
                       gazebo_ros2_control plugin
                              │
                              ▼
                            Gazebo
                              │
                              │ (joint feedback)
                              ▼
                      /joint_states, /tf
                              │
                              ▼
              champ_base/state_estimation + robot_localization/ekf
                              │
                              │ /odom, odom→base TF
                              ▼
                         feedback to quadruped_controller
```

## Error Handling & Failure Modes

| Failure | Detection | Mitigation |
|---|---|---|
| Joint name mismatch in `joints.yaml` | CHAMP node logs "joint not found" on startup | Verify before Gazebo by running `bringup.launch.py rviz:=true gazebo:=false`; sliders must accept all 12 joints |
| `<ros2_control>` block missing or wrong interface | `gazebo_ros2_control` plugin does not load any controller | Inline literal path in `<parameters>`; check `ros2 control list_controllers` after spawn |
| `models.gazebosim.org` timeout (~3 min per missing model) | Sim "hang" 3+ min at startup | Set `os.environ["GAZEBO_MODEL_DATABASE_URI"] = ""` at top of launch script |
| `model://dog_robot_description/...` mesh unresolved → robot falls through ground | `gz model -m dog_robot -i` shows `z` going to ‑inf | Set `os.environ["GAZEBO_MODEL_PATH"]` to include `install/dog_robot_description/share` |
| Robot tips over because CHAMP effort gains tuned for ~10 kg | Robot tumbles right after STAND in Gazebo | Scale `ros_control.yaml` PID + effort limits ~×0.2 for 2 kg robot; iterate by sim test |
| Foot self-collision | Erratic motion | `<self_collide>false</self_collide>` on all leg links in `gazebo.xacro` (already in place) |
| `/cmd_vel` remap mismatch | Robot ignores cmd_vel | CHAMP node already remaps `/cmd_vel/smooth:=/cmd_vel`. Teleop publishes to `/cmd_vel` — no extra remap needed |

### Rollback

Implementation on new git branch `feature/champ-integration` cut from current `feature/control-pkg`. If integration fails, `git checkout feature/control-pkg` restores the broken-but-known state. The `feature/champ-integration` branch can be abandoned without loss.

## Testing

### Build / static
- `xacro dog_robot.urdf.xacro` parses, `check_urdf` passes
- `colcon build --packages-up-to dog_robot_config` succeeds clean
- `ros2 launch dog_robot_config bringup.launch.py rviz:=true gazebo:=false` shows full robot in RViz with joint slider GUI accepting all 12 joints

### Gazebo functional
- `ros2 launch dog_robot_config gazebo.launch.py` — sim starts, robot spawns
- After plugin init (~30 s with `GAZEBO_MODEL_DATABASE_URI=""`), `ros2 control list_controllers` shows `joint_group_effort_controller [active]` and `joint_states_controller [active]`
- **Stand stability**: `spawn_stability_test.py` (adapted for `dog_robot` entity) reports < 5 mm / 2° drift in 5 s after JTC active
- **Forward walk**: `ros2 topic pub --rate 10 /cmd_vel ... linear.x=0.1` for 5 s → `gz model -m dog_robot -i` shows `Δx ≥ 0.3 m`, robot upright
- **Lateral walk**: `linear.y=0.1` 5 s → `Δy ≥ 0.2 m`
- **Yaw**: `angular.z=0.5` 5 s → `Δyaw ≥ 1.5 rad`
- **Stop**: cmd_vel=0 → robot returns to stand, velocity decays to ≈ 0

### Regression vs `champ_ws`
- Same `cmd_vel` produces comparable speed in both sims (within ~30 %)

## Open Questions / Future Work

- Gain auto-tuning for different mass scales (out of scope here, but flag in repo)
- SIT / LIE states (would require extending CHAMP state machine OR a separate hand-coded trajectory action)
- Real-hardware bridge (out of scope)

## References

- `champ_ws/src/champ` — proven working baseline (see Gazebo walking test result)
- `chvmp/spotmicro_description` — joint axes match our URDF, used as convention reference
- `chvmp/robots/configs/spotmicro_config` — template for `dog_robot_config` layout
