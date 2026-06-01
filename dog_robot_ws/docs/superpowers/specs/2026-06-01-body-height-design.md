# Body-height control — design spec

**Branch:** `feature/body-height` (branched from `kinematics-only`)
**Worktree:** `/home/nguyenvd/workspace/dog_robot_height`
**Date:** 2026-06-01

## Goal

Let the operator raise/lower the robot body in real time via the existing
`/cmd_vel` channel while the kinematics-only stack is running, so the four
legs lift/lower the base_link relative to the (fixed) world frame. Body
height changes must compose with the existing trot gait — the operator can
walk forward and adjust height simultaneously.

This branch must not regress any kinematics-only behavior; the new feature
is additive and defaults to the current rest pose when `linear.z = 0`.

## Non-goals

- No tuning of swing height / stride for body-height extremes (existing
  defaults are kept).
- No body roll / pitch / yaw control. This spec is body-Z translation only.
- No gazebo / dynamics integration. Pure kinematics + RViz.
- No teleop UX redesign. Only two new keys (`r`, `f`) join the existing
  `w/s/a/d/j/l/space/q` set.

## User-facing contract

- `geometry_msgs/Twist.linear.z` on `/cmd_vel` is interpreted as **body
  height velocity** (m/s). Positive = body rises (feet press downward
  relative to base_link).
- `BodyCommander` integrates `vz` into a body-height state `body_z`,
  clamped to `[body_z_min, body_z_max]` (defaults `-0.04 .. +0.04` m,
  measured from the CAD rest pose).
- Each tick, every leg's foot target is shifted by `-body_z` along
  body +Z (the static-TF base_link Z). In each leg's hip frame the shift
  is `R_base_to_hip.T @ [0, 0, -body_z]`.
- When `linear.z = 0` and `body_z = 0`, the behavior is byte-identical to
  the current `kinematics-only` branch.

Teleop keys (`teleop_keyboard.py`):

| Key | Effect |
|----|--------|
| `r` | `linear.z += LIN_STEP`, clamp ±LIN_MAX |
| `f` | `linear.z -= LIN_STEP`, clamp ±LIN_MAX |
| `space` | zero **all four** axes (vx, vy, vz, wz) |

`LIN_STEP = 0.02 m/s`, `LIN_MAX = 0.20 m/s` (reuse existing constants).

## Architecture

Three units change, one stays untouched.

```
+---------------------+         +-------------------+
| teleop_keyboard     |  Twist  |  kinematic_node   |
| r/f -> vz; publish  |-------->|  parse linear.z   |
+---------------------+ /cmd_vel|  push to commander|
                                +---------+---------+
                                          | on_cmd_vel(vx,vy,vz,wz)
                                          v
                                +---------+---------+
                                | BodyCommander     |
                                |  - integrate vz   |
                                |  - clamp body_z   |
                                |  - phase clock    |
                                +---------+---------+
                                          | body_z(), body_vel_xy(), phase()
                                          v
                                +---------+---------+
                                | LegDriver.step()  |
                                |  rest_shifted =   |
                                |  rest_in_hip      |
                                |   + R^T@[0,0,-bz] |
                                |  foot_target_in_hip|
                                |  ik_leg           |
                                +-------------------+
```

`foot_target_in_hip` keeps its current signature — it stays a pure
"trajectory around a given rest point" function; the shift happens in
`LegDriver` where `R_base_to_hip` is already in scope.

## Component changes

### `dog_robot_kinematic_viz/body_commander.py`

Add `_vz` and `_z` fields; extend `on_cmd_vel`, `tick`, and expose
`body_z()`:

```python
class BodyCommander:
    PHASE_OFFSETS = {"FL": 0.0, "BR": 0.0, "FR": 0.5, "BL": 0.5}

    def __init__(self, step_freq: float = 1.5,
                 body_z_min: float = -0.04,
                 body_z_max: float = +0.04):
        self.step_freq = float(step_freq)
        self.body_z_min = float(body_z_min)
        self.body_z_max = float(body_z_max)
        self._t = 0.0
        self._vx = self._vy = self._vz = self._wz = 0.0
        self._z = 0.0

    def on_cmd_vel(self, lx, ly, lz, wz):
        self._vx, self._vy, self._vz, self._wz = (
            float(lx), float(ly), float(lz), float(wz))

    def tick(self, dt):
        self._t += float(dt)
        self._z = max(self.body_z_min,
                      min(self.body_z_max, self._z + self._vz * float(dt)))

    def body_z(self) -> float:
        return self._z
```

The existing `body_vel_xy()`, `phase()`, `body_yaw_rate()`, `time()` stay
unchanged. The existing 3-arg `on_cmd_vel(linear_x, linear_y, angular_z)`
signature is **replaced** by the new 4-arg
`on_cmd_vel(linear_x, linear_y, linear_z, angular_z)`. Both callers
(`kinematic_node._on_cmd_vel` and the existing
`test_body_commander.test_on_cmd_vel_updates_state`) are updated to pass
the new `linear_z` parameter — no backwards-compat shim.

### `dog_robot_kinematic_viz/leg_driver.py`

`step` gains a `body_z` keyword (default 0.0 for backward compat in tests
that don't care):

```python
def step(self, body_v_xy, phase, body_z=0.0):
    v3 = np.array([float(body_v_xy[0]), float(body_v_xy[1]), 0.0])
    v_hip = self.geom.R_base_to_hip.T @ v3
    bz_hip = self.geom.R_base_to_hip.T @ np.array([0.0, 0.0, -float(body_z)])
    rest = self.rest_in_hip + bz_hip
    target = foot_target_in_hip(rest, phase, (v_hip[0], v_hip[1]), self.ft)
    try:
        q = ik_leg(self.link, target, knee_branch=+1)
    except ValueError:
        return self._last_joints
    self._last_joints = q
    return q
```

`self.rest_in_hip` is **not** mutated — the shift is a per-call recompute.

### `dog_robot_kinematic_viz/kinematic_node.py`

- Declare two new ROS params: `body_z_min` (default `-0.04`),
  `body_z_max` (default `+0.04`); forward to `BodyCommander(__init__)`.
- `_on_cmd_vel`: pass `msg.linear.z` through:

  ```python
  self.commander.on_cmd_vel(
      msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
  ```

- `_tick`: pull `body_z` and pass to each driver:

  ```python
  bz = self.commander.body_z()
  ...
  q = self.drivers[leg].step(v_xy, phase, bz)
  ```

### `dog_robot_kinematic_viz/teleop_keyboard.py`

- Add `self._vz = 0.0`; reset in `space`.
- Add key handling:

  ```python
  elif key == "r":
      self._vz = _clamp(self._vz + LIN_STEP, -LIN_MAX, LIN_MAX)
  elif key == "f":
      self._vz = _clamp(self._vz - LIN_STEP, -LIN_MAX, LIN_MAX)
  ```

- `publish()`: set `msg.linear.z = self._vz`.
- Update `HELP` text + log line to show `vz`.

### `foot_target.py` — unchanged.

The shift is applied to `rest_in_hip` *before* `foot_target_in_hip` is
called; the function stays a pure trajectory generator.

## Data flow (one tick)

1. teleop reads key → updates `vx/vy/vz/wz` → publishes `Twist`.
2. `kinematic_node._on_cmd_vel(msg)` → `commander.on_cmd_vel(...)`.
3. `kinematic_node._tick`: `commander.tick(dt)` integrates `body_z`
   and advances the phase clock.
4. `kinematic_node._tick` pulls `v_xy`, `body_z`, `phase(leg)` and calls
   `driver.step(v_xy, phase, body_z)` for each active leg.
5. `LegDriver.step` shifts `rest`, generates foot target, solves IK,
   returns joint triple.
6. `kinematic_node` packs all 12 positions, publishes `JointState`.
7. `robot_state_publisher` + URDF turn that into TF; RViz renders.

## Error handling

- **Clamp at the source.** `BodyCommander.tick` is the **only** place
  that clamps `body_z`. Downstream code assumes the value it receives is
  already in `[body_z_min, body_z_max]`.
- **IK fallback unchanged.** When the shifted target is unreachable,
  `LegDriver.step` returns `self._last_joints`, same as before. The
  clamp bounds were picked (see "Range rationale" below) so this fallback
  fires only at simultaneously extreme inputs.
- **No new exception types.** No silent default values that mask user
  intent — if the operator drives `vz` outside the clamp, the value
  saturates; if they drive into an unreachable corner, the leg holds.

## Range rationale

The current rest foot is well inside the workspace (q=0 across all 3
joints; L_th + L_sh ≈ 0.244 m total reach, the CAD rest is
roughly mid-stroke). Allowing `body_z ∈ [-0.04, +0.04] m` (= ±4 cm
relative to rest):

- At `body_z = +0.04` the body sits higher → feet drive 4 cm further
  *down* in body Z → thigh+knee extend toward straight. IK roundtrip
  tests (`test_ik_roundtrip_random_targets_all_legs`) currently sample
  `q_thigh ∈ (0.2, 1.0)` and `q_knee ∈ (-1.4, -0.2)`; a 4 cm extension
  keeps every sample inside that envelope by inspection.
- At `body_z = -0.04` the body sits lower → feet retract 4 cm — again
  inside the test envelope.

Range is a parameter so it can be widened later without code changes.

## Testing strategy

The existing 76 tests stay green (no behavioral change at `body_z = 0`).

### Updated tests

- `test_body_commander.py::test_on_cmd_vel_updates_state` — updated to
  pass the new `linear_z` argument and assert it propagates.
- `test_leg_driver.py`: all `d.step((vx, vy), phi)` calls keep working
  because `body_z` defaults to `0.0`. No edits needed for the existing
  cases.
- `test_kinematic_node_smoke.py`: existing assertions hold.

### New tests

`test_body_commander.py` (extend — file exists):
- `test_vz_integrates_into_body_z`: `commander.on_cmd_vel(0,0,0.02,0)`,
  `tick(0.1)` → `body_z() ≈ 0.002`.
- `test_body_z_clamps_at_max`: push `vz=0.1` for 1 s → `body_z() == +0.04`.
- `test_body_z_clamps_at_min`: push `vz=-0.1` for 1 s → `body_z() == -0.04`.
- `test_space_zeros_vz`: after non-zero vz, calling
  `on_cmd_vel(0,0,0,0)` halts integration on subsequent ticks.

`test_leg_driver.py` (extend):
- `test_zero_velocity_stance_with_body_z_shifts_foot_in_body_z`: for each
  leg, `d.step((0,0), 0.0, body_z=+0.04)` → `fk_leg(d.link, q)` in body
  frame equals `rest_body + [0,0,-0.04]` (within IK tolerance).
- `test_body_z_extreme_keeps_joints_in_limits`: for each leg, the full
  forward-velocity cycle (30 phase samples, `v=0.10 m/s`) at
  `body_z = ±0.04` stays within `JOINT_LIMITS`.

`test_kinematic_node_smoke.py` (extend):
- `test_linear_z_drives_body_height_state`: spin up the node, publish a
  Twist with `linear.z = 0.02`, run a few ticks, assert the published
  joint angles diverge from the zero-velocity baseline in the right
  direction (foot down in body Z when body_z > 0).

`test_teleop_keyboard.py` (extend if it exists; otherwise small new file):
- `r` → `_vz > 0`; `f` → `_vz < 0`; `space` → `_vz == 0`.

### Acceptance

- All `pytest` runs in the three packages green (76 existing + ~6 new).
- Manual: `ros2 launch dog_robot_kinematic_viz kinematic_teleop.launch.py`,
  press `r/f` and verify base_link visually rises/falls in RViz with
  feet planted, both standing still and while walking forward (`w`).

## Out-of-scope / follow-ups

- Body roll/pitch (rotation, not Z translation).
- Static-TF `base_height` launch arg auto-updating with `body_z` (current
  behavior: world→base_link is fixed at launch time; the body still
  appears to rise because the leg joints extend downward, not because
  the static TF moves. This is the correct kinematics-only visualisation
  and matches the chosen convention).
- Tuning `swing_height` as a function of `body_z` to keep ground
  clearance constant when crouched.

## Files touched

```
dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/body_commander.py   (edit)
dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/leg_driver.py       (edit)
dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/kinematic_node.py   (edit)
dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/teleop_keyboard.py  (edit)
dog_robot_ws/src/dog_robot_kinematic_viz/test/test_body_commander.py                 (extend)
dog_robot_ws/src/dog_robot_kinematic_viz/test/test_leg_driver.py                     (extend)
dog_robot_ws/src/dog_robot_kinematic_viz/test/test_kinematic_node_smoke.py           (extend)
dog_robot_ws/src/dog_robot_kinematic_viz/test/test_teleop_keyboard.py                (extend or new)
dog_robot_ws/docs/superpowers/specs/2026-06-01-body-height-design.md                 (this file)
```
