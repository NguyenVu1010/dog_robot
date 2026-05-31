# Kinematics-only Worktree — Design

**Date:** 2026-06-01
**Branch:** `kinematics-only`
**Worktree:** `/home/nguyenvd/workspace/dog_robot_kine`
**Goal:** Strip the workspace down to a pure kinematic verification rig — URDF + RViz + per-leg IK driver — with no Gazebo, no controller_manager, no CHAMP. `cmd_vel` drives a body commander that hands a per-leg phase + body pose to each `LegDriver`; `LegDriver` computes a foot target, calls closed-form `ik_leg`, and the node publishes `/joint_states`. The base is anchored at a fixed world transform so the legs swing in space without falling.

## Motivation

The joint-frame export work (`2026-05-26-joint-frame-export-design.md`) gave the project a clean URDF whose joint origins match the FreeCAD CAD exactly, plus a `dog_robot_kinematics.kinematics_link` library with closed-form `fk_leg`/`ik_leg`. Before re-introducing dynamics (Gazebo, ros2_control, gait tuning), we need to **prove the kinematics are right** in isolation: a foot command must produce joint angles that, when fed through RSP + URDF, place the foot mesh exactly where it was commanded. The Gazebo path currently mixes URDF/visual correctness, controller dynamics, contact physics, and gait — too many things to debug at once. This rig removes everything except the geometry pipeline.

## Scope

**In:** URDF (geometry only), `dog_robot_kinematics`, a new `dog_robot_kinematic_viz` ament_python package containing the leg driver / body commander / kinematic node / mini teleop, two launch modes (full 4-leg, single-leg), RViz config, unit + smoke tests.

**Out:** Gazebo, ros2_control, controller_manager, joint_trajectory_controller, CHAMP, ground contact, inertials, walker_controller, stand_controller, gait/* (replaced by the much simpler body commander + foot target modules). Foot trajectories are simple parametric stance/swing, not the previous trajectory_planner.

## Architecture

```
                 /cmd_vel (Twist)
                       │
                       ▼
              ┌──────────────────┐
              │ BodyCommander    │  phase per leg (trot offsets), body pose
              └────────┬─────────┘
                       │ body_pose, leg_phase[name], dt
        ┌──────────────┼──────────────┬──────────────┐
        ▼              ▼              ▼              ▼
   LegDriver(FL)  LegDriver(FR)  LegDriver(BL)  LegDriver(BR)
        │              │              │              │
        ▼              ▼              ▼              ▼
   (3 joints)      (3 joints)     (3 joints)     (3 joints)
        └──────────────┴──────┬───────┴──────────────┘
                              ▼
                     ┌──────────────────┐
                     │ KinematicNode    │  aggregate 12 → /joint_states
                     └────────┬─────────┘
                              ▼
            robot_state_publisher → /tf
                              ▼
                            RViz
   (static TF world→base_link @ (0,0,base_height) holds body in space)
```

**Process discipline:**
- `BodyCommander` and `LegDriver` are plain Python (no ROS). Only `KinematicNode` and `TeleopKeyboard` know about ROS. Unit tests exercise the math without spinning a node.
- `LegDriver` does **not** know about the other legs. The "1 chân → kế thừa các chân" requirement is satisfied by writing `LegDriver` once and instantiating it 4× from the 4 `LegConfig` entries.
- All IK goes through `dog_robot_kinematics.kinematics_link.ik_leg` (joint-frame closed-form). The legacy `kinematics_dh.py` is not imported.

## Worktree & branch

- `git worktree add /home/nguyenvd/workspace/dog_robot_kine -b kinematics-only` (done — branched from `981e7d3`).
- All work in this design lives in the new worktree. The parent repo at `/home/nguyenvd/workspace/dog_robot` is untouched.
- When done: PR `kinematics-only` → `main`, or `git worktree remove /home/nguyenvd/workspace/dog_robot_kine` to discard.

## Cleanup (delete in worktree)

**Whole packages:**
- `src/champ/` — CHAMP fork, deprecated.
- `src/dog_robot_config/` — CHAMP bringup + gazebo config.
- `src/dog_robot_control/` — walker / stand / teleop / gait; replaced by new code in `dog_robot_kinematic_viz`.

**Files in `dog_robot_description/`:**
- `urdf/gazebo.xacro` — friction, self_collide, ros2_control plugin wiring.
- `urdf/ros2_control.xacro` — `<ros2_control>` block, command/state interfaces.
- `urdf/inertial.xacro` — inertials not needed without dynamics.
- `launch/gazebo.launch.py`
- `config/ros2_controllers.yaml`
- `config/dh_link_placements.yaml` — legacy DH artefact; `joint_frames.yaml` + `link_params.yaml` + `urdf_joints.yaml` are the source of truth.
- `scripts/compute_dh_lengths.py`, `derive_dh_frames.py`, `compute_visual_compensation.py` — DH-era scripts.
- **Keep** `scripts/derive_joint_frames.py`, `bake_meshes_to_link_frame.py`, `export_dh_links_from_freecad.py` — still needed to regenerate frames/meshes if CAD changes.

**Workspace scripts:**
- `scripts/dog_relaunch_walk.sh` — delete.
- `scripts/setup_env.zip` — delete (rác).
- `scripts/dog_kill_all.sh` — keep; reused for kill-then-relaunch.
- `scripts/dog_relaunch_kinematic.sh` — keep; update its launch target to the new kinematic launch.

**Edit `dog_robot_description/urdf/dog_robot.urdf.xacro`:**
- Remove `<xacro:include>` of `gazebo.xacro`, `ros2_control.xacro`, `inertial.xacro`.

**Edit `dog_robot_description/urdf/leg.xacro`:**
- Remove the `<xacro:inertial_box>` / `<xacro:inertial_sphere>` calls in each link. RViz does not require `<inertial>`.

**Edit `dog_robot_description/package.xml`:**
- Drop exec_depend: `gazebo_ros`, `gazebo_ros2_control`, `ros2_controllers`, `controller_manager`, `joint_trajectory_controller`, `joint_state_broadcaster`.
- Keep: `xacro`, `urdf`, `robot_state_publisher`, `rviz2`, `joint_state_publisher`.

**Edit `dog_robot_description/CMakeLists.txt`:**
- Keep `install(DIRECTORY urdf meshes config launch rviz ...)`.
- Update or remove `test_urdf.py` if it references the deleted Gazebo plumbing.

## New package: `dog_robot_kinematic_viz` (ament_python)

The existing `dog_robot_kinematic_viz` is converted from `ament_cmake` (launch/rviz only) to `ament_python` and gains the entire kinematic driver.

```
src/dog_robot_kinematic_viz/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/dog_robot_kinematic_viz
├── dog_robot_kinematic_viz/
│   ├── __init__.py
│   ├── leg_geometry.py       # LegGeom + load_leg_geoms (per-leg base→hip)
│   ├── foot_target.py        # stance/swing trajectory in hip frame
│   ├── body_commander.py     # /cmd_vel → body velocity + per-leg phase
│   ├── leg_driver.py         # LegDriver: foot target → ik_leg → 3 joints
│   ├── kinematic_node.py     # ROS node: tick → publish /joint_states
│   └── teleop_keyboard.py    # mini WASD teleop, self-contained
├── launch/
│   ├── kinematic.launch.py              # 4 legs
│   ├── kinematic_single_leg.launch.py   # leg:=FL, other 3 frozen at neutral
│   └── kinematic_teleop.launch.py       # full + teleop in gnome-terminal
├── rviz/
│   └── kinematic.rviz
├── config/
│   └── kinematic_params.yaml
└── test/
    ├── test_foot_target_geometry.py
    ├── test_leg_driver_fk_ik_roundtrip.py
    ├── test_leg_driver_mirror_symmetry.py
    └── test_kinematic_node_smoke.py
```

`package.xml` exec_depend: `rclpy`, `geometry_msgs`, `sensor_msgs`, `tf2_ros`, `robot_state_publisher`, `rviz2`, `xacro`, `dog_robot_description`, `dog_robot_kinematics`.

`setup.py` entry_points: `kinematic_node = dog_robot_kinematic_viz.kinematic_node:main`, `teleop_keyboard = dog_robot_kinematic_viz.teleop_keyboard:main`.

## Module: `foot_target.py`

Pure functions producing a foot target `(x, y, z)` in the leg's **hip frame** for a given gait phase `φ ∈ [0, 1)` and body command.

```
stance_phase_ratio = 0.5             # half stance, half swing
nominal_height = 0.15                # nominal foot drop below hip (m)
stride_length(v_body, freq) → metres
swing_height = 0.03                  # peak Z above stance plane

foot_target_in_hip(phase, v_xy_body_in_hip, leg_cfg) → (x, y, z)
    if phase < stance_phase_ratio:
        # stance: foot drags backwards under hip at constant z
        s = (phase / stance_phase_ratio) * 2.0 - 1.0   # +1 → -1
        return (-0.5 * stride * s_x, -0.5 * stride * s_y, -nominal_height)
    else:
        # swing: parametric arc forward, height bezier-up-bezier-down
        u = (phase - stance_phase_ratio) / (1.0 - stance_phase_ratio)  # 0..1
        s = u * 2.0 - 1.0                                              # -1 → +1
        z_offset = swing_height * sin(pi * u)                          # peaks at u=0.5
        return (+0.5 * stride * s_x, +0.5 * stride * s_y, -nominal_height + z_offset)
```

`v_xy_body_in_hip` = the body XY velocity rotated into the leg's hip frame (because hip yaw offsets each leg differently). Computed by `BodyCommander` from `leg_cfg.base_to_hip_rpy`.

## Module: `leg_geometry.py` (new — per-leg base→hip data)

`dog_robot_kinematics.leg_config` is **not used** in this rig: its simplified `base_to_hip_rpy = (0, π/2, 0)` predates the joint-frame URDF and does not include the yaw splay. We load the real per-leg transforms from `dog_robot_description/config/urdf_joints.yaml`.

```
@dataclass(frozen=True)
class LegGeom:
    name: str                                # "FL" | "FR" | "BL" | "BR"
    base_to_hip_xyz: np.ndarray              # (3,)
    base_to_hip_rpy: tuple[float, float, float]
    R_base_to_hip: np.ndarray                # (3,3), Rz·Ry·Rx

def load_leg_geoms(urdf_joints_yaml) → dict[str, LegGeom]:
    cfg = yaml.safe_load(...)
    out = {}
    for name, blk in cfg["per_leg"].items():
        rpy = tuple(blk["base_to_hip_rpy"])
        out[name] = LegGeom(name, np.asarray(blk["base_to_hip_xyz"]),
                            rpy, _rpy_to_matrix(rpy))
    return out

LEG_NAMES = ("FL", "FR", "BL", "BR")
```

## Module: `body_commander.py`

```
class BodyCommander:
    # Trot pattern. FL & BR move together; FR & BL move together π out of phase.
    PHASE_OFFSETS = {"FL": 0.0, "BR": 0.0, "FR": 0.5, "BL": 0.5}

    def __init__(step_freq=1.5):
        self.step_freq = step_freq
        self._t = 0.0
        self._vx = self._vy = self._wz = 0.0

    def on_cmd_vel(twist):
        self._vx, self._vy, self._wz = twist.linear.x, twist.linear.y, twist.angular.z

    def tick(dt):
        self._t += dt

    def phase(leg_name) → float:
        return ((self._t * self.step_freq) + PHASE_OFFSETS[leg_name]) % 1.0

    def body_vel_xy() → (vx, vy):
        return (self._vx, self._vy)
```

Body pose itself is constant (anchored by static TF); only the phase + body velocity matter.

## Module: `leg_driver.py`

```
class LegDriver:
    def __init__(geom: LegGeom, link_params: LinkParams):
        self.geom = geom                 # per-leg base→hip
        self.link = link_params          # per-leg LinkParams (FL/FR/BL/BR each distinct)
        self._last_joints = (0.0, 0.0, 0.0)

    def step(body_v_xy_in_body: tuple[float, float], phase: float) → (q1, q2, q3):
        # 1. rotate the body-frame XY velocity into the hip frame.
        #    R_base_to_hip maps a hip-frame vector to the body frame; we need
        #    the inverse to take body → hip.
        v3 = np.array([body_v_xy_in_body[0], body_v_xy_in_body[1], 0.0])
        v_hip = self.geom.R_base_to_hip.T @ v3
        # 2. ask foot_target for the hip-frame target at this phase
        p_hip = foot_target_in_hip(phase, (v_hip[0], v_hip[1]))
        # 3. closed-form IK (knee_branch = +1, natural forward-thigh / bent-knee)
        try:
            q = ik_leg(self.link, p_hip, knee_branch=+1)
        except ValueError:
            q = self._last_joints       # unreachable → hold last (don't crash node)
        self._last_joints = q
        return q
```

The single class with no per-leg conditionals is the "1 chân → kế thừa các chân" guarantee. The 4 instances differ only in `geom` and `link_params` injected at ctor time.

## Module: `kinematic_node.py`

```
class KinematicNode(Node):
    PARAMS:
        base_height: float = 0.20          # world→base_link Z
        active_legs: list[str] = ["FL","FR","BL","BR"]
        idle_joints: list[float] = [0,0,0] # for legs not in active_legs
        publish_rate: float = 50.0
        link_params_yaml: str = "$(find dog_robot_description)/config/link_params.yaml"

    on_init:
        self.commander = BodyCommander(step_freq=...)
        geoms = load_leg_geoms(urdf_joints_yaml)            # per-leg base→hip
        self.drivers = {
            name: LegDriver(geoms[name], load_link_params(link_params_yaml, name))
            for name in self.active_legs
        }
        self.create_subscription(Twist, "/cmd_vel", self.commander.on_cmd_vel, 10)
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(1/rate, self.tick)
        self._t_last = now()

    tick():
        dt = now() - self._t_last; self._t_last = now()
        self.commander.tick(dt)
        msg = JointState()
        msg.header.stamp = now()
        for leg in ALL_LEG_NAMES:
            if leg in self.drivers:
                q = self.drivers[leg].step(self.commander.body_vel_xy(),
                                           self.commander.phase(leg))
            else:
                q = self.idle_joints
            msg.name += [f"{leg}_hip_yaw", f"{leg}_thigh_pitch", f"{leg}_knee_pitch"]
            msg.position += list(q)
        self.pub.publish(msg)
```

The static `world→base_link` TF is published by `tf2_ros.static_transform_publisher` from the launch file, not by this node.

## Module: `teleop_keyboard.py`

Minimal WASD + JL teleop, self-contained (`termios` raw mode + `Twist` publisher). Approx. 80 lines. Not a port of the old `dog_robot_control/teleop_keyboard.py` — fresh, simpler.

```
key → twist change:
  w/s: linear.x +/-
  a/d: linear.y +/-
  j/l: angular.z +/-
  space: zero
  q/ctrl-c: quit
```

## Launch files

### `kinematic.launch.py` — full 4-leg

```
robot_state_publisher (URDF via xacro)
static_transform_publisher  world → base_link  (0 0 0.20)
kinematic_node              active_legs=["FL","FR","BL","BR"]
rviz2                       -d kinematic.rviz
```

### `kinematic_single_leg.launch.py` — single leg

```
launch arg: leg (default "FL")
robot_state_publisher
static_transform_publisher  world → base_link
kinematic_node              active_legs=[<leg>], idle_joints=[0,0,0]
rviz2
```

The full URDF still loads (per the design decision), so the other 3 legs render at their neutral pose (joints = 0).

### `kinematic_teleop.launch.py` — full + teleop

```
... (same as kinematic.launch.py) ...
teleop_keyboard  (prefix="gnome-terminal --")
```

## RViz config

- Fixed Frame: `world`
- Grid: enabled (cell 0.1 m, 10×10).
- RobotModel: source `/robot_description`.
- TF: enabled.
- (Optional later) MarkerArray for foot targets / phase indicator.

## Configuration: `kinematic_params.yaml`

```
kinematic_node:
  ros__parameters:
    base_height: 0.20
    publish_rate: 50.0
    active_legs: ["FL","FR","BL","BR"]
    idle_joints: [0.0, 0.0, 0.0]
    link_params_yaml: ""    # absolute path, injected by launch via PathJoinSubstitution
    urdf_joints_yaml: ""    # absolute path, injected by launch via PathJoinSubstitution
    step_freq: 1.5
    stride_length_per_mps: 0.20    # metres per (m/s of body vel)
    nominal_height: 0.15
    swing_height: 0.03
    stance_phase_ratio: 0.5
```

## Testing — kinematic correctness

These run as `colcon test` and as plain `pytest test/`.

### `test_leg_driver_fk_ik_roundtrip.py`
For each of the 4 legs:
1. Sample N=500 random foot targets in the leg's reachable workspace (computed analytically from `L_hh + L_th + L_sh`).
2. Run `ik_leg` → joints `q`.
3. Run `fk_leg(q)` → recovered foot.
4. Assert `‖p_recovered − p_target‖ < 1e-6 m`.

### `test_leg_driver_mirror_symmetry.py`
For each `(p_FL, p_FR=mirror(p_FL))` pair:
1. Drive FL with `p_FL`, FR with `p_FR`.
2. Assert `q_FL.hip_yaw == −q_FR.hip_yaw` and `q_FL.thigh == q_FR.thigh` and `q_FL.knee == q_FR.knee` (within 1e-9).

### `test_foot_target_geometry.py`
1. At `φ = stance_phase_ratio − ε`: foot at stance end (forward end).
2. At `φ = stance_phase_ratio + ε`: foot at swing start, same XY (continuous).
3. At `φ = (1 + stance_phase_ratio) / 2`: peak swing height (z = `−nominal_height + swing_height`).
4. At `φ = 1 − ε` and `φ = 0`: stance start, same XY (continuous across the cycle wrap).

### `test_kinematic_node_smoke.py`
1. Spawn rclpy executor with `KinematicNode` (no GUI), publish a `/cmd_vel` of `(0.1, 0, 0)`.
2. Wait 1 second.
3. Subscribe `/joint_states`, assert: 12 joints, names match expected, positions change (some Δ > 1e-3 from start).

Smoke launch (manual, not automated): `ros2 launch dog_robot_kinematic_viz kinematic.launch.py` boots RViz with the robot visible at the world origin (raised by `base_height`), grid showing, no errors in the log.

## Verification of "code động học" (extra rigor)

Per the user's request "kiểm tra kĩ code động học", the test suite must:

- Use the **same** `link_params.yaml` the runtime uses (no test-specific overrides).
- Cover both sides (left vs right `LinkParams`).
- Cover all 4 legs (not just FL).
- Roundtrip tolerance ≤ 1e-6 m (matches `kinematics_link.ik_leg` precision).
- Assert `q ∈ [lower, upper]` per joint for all sampled targets (or raise `WorkspaceError` outside reach — counted, not silenced).
- Pin the foot-target geometry tests so a refactor that breaks continuity at the stance↔swing transition fails immediately.

## Implementation order

1. Worktree (done).
2. Spec (this file) committed.
3. Cleanup: delete packages, edit URDF, edit `package.xml`/`CMakeLists.txt`. Commit.
4. Convert `dog_robot_kinematic_viz` to ament_python skeleton (`package.xml`, `setup.py`, `setup.cfg`, `resource/`). Commit.
5. Implement `leg_geometry.py` + its test (load all 4 LegGeom from `urdf_joints.yaml`). Commit.
6. Implement `foot_target.py` + geometry test. Commit.
7. Implement `body_commander.py` + phase/trot-pattern test. Commit.
8. Implement `leg_driver.py` + roundtrip + mirror-symmetry tests. Commit.
9. Implement `kinematic_node.py` + smoke test. Commit.
10. Implement `teleop_keyboard.py`. Commit.
11. Three launch files + RViz config + `kinematic_params.yaml`. Commit.
12. Update `scripts/dog_relaunch_kinematic.sh`. Commit.
13. Full colcon build + colcon test + manual launch smoke. Commit any fixes.
14. Final review pass; merge or open PR.

## Out-of-scope (deferred)

- Re-introducing Gazebo dynamics.
- Inverse dynamics, joint torque feedback.
- Realistic stance for "idle" legs in single-leg mode (currently joint=0).
- Body pose visualisation in RViz (foot target markers, gait phase indicator).
- Pose-only commands (height changes, body tilt) — currently `BodyCommander` ignores `linear.z` / `angular.x` / `angular.y`.
