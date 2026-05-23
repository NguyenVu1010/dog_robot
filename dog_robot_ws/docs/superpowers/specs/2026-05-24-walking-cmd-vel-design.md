# Walking controller (omni cmd_vel) — Design

**Date:** 2026-05-24
**Status:** Approved (sections walked through 1–4)
**Scope:** Port CHAMP gait engine (BodyController + LegController + PhaseGenerator + TrajectoryPlanner) from C++ to Python under `dog_robot_control`. Plug DH-IK as the IK back-end. New `walker_controller` node subsumes `stand_controller`. CHAMP packages fully unused after this lands.

---

## Goals

1. Robot follows `/cmd_vel` (Twist: linear.x, linear.y, angular.z) — full omni gait, not just forward/back.
2. Walker also handles stand: cmd_vel=0 → static stand pose (same behaviour as today's `stand_controller`).
3. All gait math in Python under `dog_robot_control/dog_robot_control/gait/`, reusing the existing `kinematics_dh` module.
4. Single Python launch (`walk.launch.py`); existing `stand.launch.py` + `stand_controller.py` kept for backward compat but deprecated.

## Non-goals

- Closed-loop balance / IMU feedback. The controller is open-loop (no body roll/pitch correction from sensor).
- Foot contact sensing or stance compliance. CHAMP's contact publish path is dropped.
- Replacing CHAMP files in `src/champ/` — they stay in-tree (deprecated, not launched).
- STM32/servo hardware bridge.

---

## Architecture

```
              /cmd_vel (Twist)                  /stand_cmd (Pose, optional)
                   │                                  │
                   ▼                                  ▼
       ┌──────────────────────────────────────────────────┐
       │           walker_controller (ROS 2 node)         │
       │  body_controller.pose_command(req_pose)          │
       │     ── produces foot_positions[4] in body-at-hip │
       │  leg_controller.velocity_command(req_vel, t)     │
       │     ── adds gait deltas (Raibert + Bezier)       │
       │  for each leg:                                   │
       │     foot_h = R_bh.T · foot_position              │
       │     theta_3dof = ik_leg(dh, foot_h, knee_dir)    │
       │  publish JointTrajectory (12 joints)             │
       └─────────────┬────────────────────────────────────┘
                     ▼
       /joint_trajectory_controller/joint_trajectory
```

Walker ticks at 50 Hz, same as current stand_controller. JTC keeps the position+open_loop_control profile that proved stable in stand (see [[feedback-jtc-interface-choice]]).

## File layout

```
src/dog_robot_control/
  dog_robot_control/
    walker_controller.py            (new) ROS 2 node
    gait/
      __init__.py
      gait_config.py                (new) dataclass
      phase_generator.py            (new) trot phase
      trajectory_planner.py         (new) Bezier swing + stance
      body_controller.py            (new) zero_stance + pose
      leg_controller.py             (new) Raibert + iterate
    stand_controller.py             (unchanged, deprecated)
    kinematics_dh.py                (unchanged)
    leg_config.py                   (unchanged)
  config/
    walker_params.yaml              (new)
    dh_params.yaml                  (unchanged)
  launch/
    walk.launch.py                  (new)
    stand.launch.py                 (unchanged, deprecated)
  test/
    test_phase_generator.py         (new)
    test_trajectory_planner.py      (new)
    test_body_controller.py         (new)
    test_leg_controller.py          (new)
    test_walker_integration.py      (new)
```

## Components

### gait_config.py

```python
@dataclass(frozen=True)
class GaitConfig:
    nominal_height: float            # m, body z above ground at stand
    stance_duration: float           # s
    swing_height: float              # m, foot lift during swing
    stance_depth: float              # m, foot dip during stance (small)
    max_linear_velocity_x: float
    max_linear_velocity_y: float
    max_angular_velocity_z: float
```

`knee_orientation` (CHAMP's ">>"/"<<" string per leg) maps to per-leg `knee_direction` (+1/-1) and lives in `leg_config.LEGS` extension or in `walker_params.yaml`.

### phase_generator.py

Port of CHAMP's `PhaseGenerator` (state machine):
- Inputs: `target_velocity` (m/s), `step_length` (m), `time` (monotonic float seconds — we use `node.get_clock().now()` in walker, deterministic float in tests).
- Outputs: `stance_phase_signal[4]`, `swing_phase_signal[4]` ∈ [0,1].
- Trot offsets: leg 0/3 in sync at offset 0; leg 1/2 in sync at offset 0.5·stride.
- Warmup: first half-stride forces front-left + back-right to stance (avoids cold-start tipping).

### trajectory_planner.py

Per-leg foot trajectory:
- 12-control-point Bernstein Bezier curve for swing phase (control points copied from CHAMP `ref_control_points_x_/y_`, scaled by step_length / 0.4 m and swing_height / 0.15 m).
- Stance phase: foot x sweeps linearly from +step/2 → -step/2; foot z dips by `stance_depth · cos(πx/step_length)`.
- Adds delta in (x, y, z) onto the foot_position passed in; rotates delta by `rotation` (atan2 of step_y/step_x) so the gait orientation follows commanded velocity direction.

### body_controller.py

```python
def pose_command(req_pose) -> foot_positions[4]:
    # For each leg: foot = zero_stance(leg, nominal_height)
    # foot.translate(-req_pose.x, -req_pose.y, -(zero_stance.z + req_pose.z))
    # foot.rotate Z(-yaw), Y(-pitch), X(-roll)
    # foot -= leg.hip_origin_in_body  → "transform to hip"
```

`zero_stance(leg, nominal_height)` returns the resting foot position in body frame: `(leg.base_to_hip_xyz[0], leg.base_to_hip_xyz[1], 0)` (foot directly below hip, on ground). `leg.hip_origin_in_body = leg.base_to_hip_xyz`.

### leg_controller.py

Port of CHAMP `LegController.velocityCommand`:
1. Cap velocities to gait_config limits.
2. Raibert heuristic: `step_xyθ = (stance_duration/2) · (vx, vy, wz·r)` where `r = center_to_nominal` (distance from body center to nominal foot position).
3. Per leg: `transformLeg(step_x, step_y, theta)` → `(step_length, rotation)`.
4. Average step_length → `phase_generator.run(velocity, mean_step)`.
5. Per leg: `trajectory_planner.generate(foot_positions[i], step_length[i], rotation[i], swing_phase[i], stance_phase[i])`.

### walker_controller.py

```python
class WalkerController(Node):
    # On startup: subscribe /joint_states, /cmd_vel, /stand_cmd.
    # Build LegController, BodyController, PhaseGenerator using GaitConfig from params.
    # Initial ramp (2 s linear interpolation) from spawn joint angles
    #   to stand pose at nominal_height, same as today's stand_controller.
    # 50 Hz timer:
    #   1. Compose foot_positions = body_controller.pose_command(req_pose)
    #   2. leg_controller.velocity_command(foot_positions, req_vel, t_now)
    #   3. For each leg: foot_h = R_bh.T · foot_positions[i]; q = ik_leg(...)
    #   4. Publish JointTrajectory.
    # Cmd_vel timeout: 0.5 s no message → req_vel = zero (smooth stop).
```

## Configuration defaults

`walker_params.yaml`:

```yaml
walker_controller:
  ros__parameters:
    dh: { L_hh: 0.02553, L_th: 0.11725, L_sh: 0.07043 }
    gait:
      nominal_height: 0.15
      stance_duration: 0.30
      swing_height: 0.03
      stance_depth: 0.001
      max_linear_velocity_x: 0.15
      max_linear_velocity_y: 0.08
      max_angular_velocity_z: 0.50
    stand:
      ramp_time: 2.0
      cmd_vel_timeout: 0.5
    knee_direction_per_leg:        # +1 forward bend, -1 backward bend
      FL: 1
      FR: 1
      BL: 1
      BR: 1
    joint_order:
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
```

If walking proves unstable with `knee_direction = 1` for all legs (as it did during stand for a moment), per-leg sign can be flipped without code changes.

## Data flow per tick

1. Read latest `req_vel` and `req_pose` (defaults: vel=0, pose=(0,0,nominal_height,0,0,0)).
2. `body_controller.pose_command(req_pose)` → `foot_positions[4]` in body-at-hip frame.
3. `leg_controller.velocity_command(foot_positions, req_vel, t_now)` → mutates foot_positions with gait deltas.
4. Per leg: `foot_h = R_bh.T · foot_positions[i]` where `R_bh` is rotation from `base_to_hip_rpy`.
5. `theta = ik_leg(dh, foot_h, knee_direction_per_leg[leg.name])`.
6. Build JointTrajectoryPoint with 12 angles in `joint_order`, time_from_start = 100 ms.
7. Publish.

## Error handling

- IK out of reach (`ValueError`): log warn once per leg per 1 s, skip the publish cycle. JTC holds last command.
- cmd_vel topic silent for > 0.5 s: req_vel ← 0 (smooth stop, gait decays into stand because phase_generator zeros signals when target_velocity = 0).
- joint_states never received: walker stays in init state (no publishing), log warn every 2 s.
- Param load failure (`walker_params.yaml` missing keys): fail-fast at startup with explicit message.

## Testing strategy

### Unit (no ROS)

`test_phase_generator.py`:
- target_velocity = 0 → all signals zero.
- After warmup at vx > 0 → leg 0 stance ≈ 0.5 when leg 1 stance ≈ 0 (trot anti-phase).
- One full stride later → signals recycle.

`test_trajectory_planner.py`:
- step_length = 0 → no delta added.
- Stance phase 0→1 → foot x sweeps +step/2 → -step/2 linearly.
- Swing curve endpoints continuous with stance endpoints.
- Swing peak z ≈ -swing_height/2 within ±20% (Bezier asymmetry).

`test_body_controller.py`:
- req_pose at nominal → foot at zero_stance.
- req_pose.z + 0.05 → foot z lifts 0.05.
- req_pose.yaw = 0.2 rad → all 4 feet rotate by -0.2 around body Z.

`test_leg_controller.py`:
- velocity_command(vx=0.1) over 10 strides → foot mean velocity ≈ 0.1 m/s (Raibert closes the loop).

### Integration (pytest with mocked clock)

`test_walker_integration.py`:
- Run full pipeline at simulated 50 Hz for 3 strides at vx=0.1.
- Every tick: ik_leg succeeds for all 4 legs (no ValueError).
- All joint angles within URDF limits.
- Trot timing assertion: leg 0 and leg 3 phase signals match within 1 tick.

### Gazebo regression (manual)

```bash
ros2 launch dog_robot_control walk.launch.py
# wait 3 s for ramp-to-stand
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}}' -r 10
```

Acceptance bar for each single-axis test (vx=±0.1, vy=±0.05, wz=±0.3):
- Body roll/pitch absolute < 0.25 rad for ≥10 s.
- Body z stays within ±0.05 m of `nominal_height`.
- Robot moves in the commanded direction (qualitative).

Combined motions (e.g. vx + wz arc) are nice-to-have, not gating.

## Migration / cleanup

- `stand_controller.py` and `stand.launch.py` stay in tree, marked DEPRECATED in their docstrings. Walker subsumes their functionality.
- README gets a Walking section. The Kinematics section updates to mention walker as the live controller.
- `dog_kill_all.sh` adds `walker_controller` pattern.

## Open questions (resolved during brainstorming)

- ✅ Scope: full omni (vx + vy + wz), not just forward/back.
- ✅ Integration: port CHAMP gait fully to Python, bypass CHAMP entirely.
- ✅ Walker subsumes stand (single node, cmd_vel=0 → stand pose).
- ✅ Control interface: JTC with position command + open_loop_control (kept from stand).
- ✅ knee_orientation: per-leg knee_direction in yaml (default +1 all legs; tune per-leg if needed).
