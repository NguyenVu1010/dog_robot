# Rear-Z Posture Control (Sit Pose) — Design

Date: 2026-06-10
Branch: `feature/body-height` (worktree `/home/nguyenvd/workspace/dog_robot_height`)
Package: `dog_robot_kinematic_viz`
Related: [2026-06-01-body-height-design.md](2026-06-01-body-height-design.md), [2026-05-24-kinematics-viz-design.md](2026-05-24-kinematics-viz-design.md)

## Goal

Add a continuous rear-only body-Z control to the kinematic viz rig, analogous to the existing whole-body height control (`linear.z` → `body_z`). When the rear control is driven to its positive bound while the front stays at nominal stand, the robot reaches a dog-sit pose (rear feet folded toward body, front feet at stand extension). Sit is an emergent end-state of the slider, not a discrete preset.

This is a kinematic verification feature: base_link is fixed by static TF; success criterion is RViz visually showing BL/BR feet pulled toward the body while FL/FR stay put, without IK crashes and within joint limits.

Lying down and explicit pose presets are out of scope; the design leaves room to extend later by adding a symmetric `front_z` knob.

## Architecture

The existing pipeline is unchanged in shape:

```
/cmd_vel  →  BodyCommander  →  LegDriver × 4  →  /joint_states
```

`BodyCommander` gains one new state, `rear_z` (m), integrated from a new velocity input on the existing Twist (`angular.y`). `foot_target_in_hip` accepts one additional scalar `extra_z` that is summed into the body-frame Z displacement of the foot. `LegDriver` decides whether to forward `rear_z` as `extra_z` based on a per-instance `is_rear` flag set at construction (`BL`, `BR` → True, others → False).

Front legs are bit-for-bit unchanged at runtime: `extra_z` is `0.0` for FL/FR, so their `foot_target` math reduces to the current code path.

### Sign convention

Two scalars describe body-Z posture; their signs map to different physical quantities and must not be conflated.

- **`body_z`** describes the *body's* height above its rest pose. `body_z > 0` ⇒ body raised; in body frame the feet appear to drop. Subtracted in the formula (matches the existing body-height code).
- **`rear_z`** (and the generic `extra_z` it travels under) describes the *rear feet's* lift in body frame. `rear_z > 0` ⇒ rear feet pulled up toward the body, which visually is the sit direction. Added in the formula.

User-facing: pressing the rear-up teleop key (`i`) drives `rear_z` positive, which in RViz visibly lifts BL/BR feet toward the body — the sit pose visual.

## Components

### `dog_robot_kinematic_viz/body_commander.py`

`BodyCommander` is the single source of truth for the rear_z clamp, just as it already is for body_z.

- New ctor params: `rear_z_min: float = -0.05`, `rear_z_max: float = +0.05` (wider than body_z because sit needs more travel than uniform height).
- New private state: `self._rear_z = 0.0`, `self._wy = 0.0` (rear velocity input).
- `on_cmd_vel` signature changes to 5 args:
  `on_cmd_vel(linear_x, linear_y, linear_z, angular_y, angular_z)`.
  Only `kinematic_node` calls it, so no backward-compat shim is needed.
- `tick(dt)` adds `new_rz = self._rear_z + self._wy * dt`, clamped to `[rear_z_min, rear_z_max]`.
- New accessor: `rear_z() -> float`.
- Existing `body_z()`, `body_vel_xy()`, `phase()` unchanged.

### `dog_robot_kinematic_viz/foot_target.py`

`foot_target_in_hip` gains one scalar arg, inserted after `body_z`:

```
foot_target_in_hip(rest_in_hip, phase, v_body_xy, body_z, extra_z, R_base_to_hip, params)
```

Body changes one line: `disp_body.z = z_lift_body - body_z + extra_z`.

`extra_z` is the leg-frame-agnostic foot-lift offset (in body +Z). Callers decide which value to pass (0 for legs that should follow only `body_z`, `rear_z` for legs that should also lift toward the body). The sign is opposite to `body_z` because the two scalars describe different physical things — see the Sign Convention subsection above.

This keeps `foot_target_in_hip` pure math — no knowledge of FL/BL/FR/BR membership.

### `dog_robot_kinematic_viz/leg_driver.py`

`LegDriver.__init__` gains `is_rear: bool = False`. Stored as `self.is_rear`.

`step` signature gains `rear_z: float = 0.0`:

```
step(body_v_xy, phase, body_z=0.0, rear_z=0.0)
```

Body chooses `extra_z = self.rear_z_member` where:

```
extra_z = rear_z if self.is_rear else 0.0
```

and passes to `foot_target_in_hip`. The IK-freeze guard (catch `ValueError`, return `_last_joints`) is unchanged.

`LegDriver` also gains de-bounced WARN-once: `self._saturated = False`. On `ValueError` and `not _saturated`: `logger.warning(...)`, set `_saturated = True`. On `ik_leg` success while `_saturated`: clear the flag. The logger is injected from `kinematic_node` (`logger: Optional[Logger] = None` ctor arg; if None, fall back to `print`).

### `dog_robot_kinematic_viz/kinematic_node.py`

- Declare 2 new params: `rear_z_min: -0.05`, `rear_z_max: +0.05`.
- Pass to `BodyCommander(step_freq=..., body_z_min=..., body_z_max=..., rear_z_min=..., rear_z_max=...)`.
- `_on_cmd_vel(msg)`: forward `msg.angular.y` as `angular_y`:
  `self.commander.on_cmd_vel(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.y, msg.angular.z)`.
- LegDriver construction: `is_rear=(name in ("BL", "BR"))`. Pass `logger=self.get_logger()`.
- `_tick()`: read `rear_z = self.commander.rear_z()`; pass to each `driver.step(v_xy, phase, body_z, rear_z)`.

### `dog_robot_kinematic_viz/teleop_keyboard.py`

- New state: `self._wy = 0.0`.
- New keys:
  - `i` → `_wy = clamp(_wy + LIN_STEP, -LIN_MAX, LIN_MAX)` (rear up)
  - `k` → `_wy = clamp(_wy - LIN_STEP, -LIN_MAX, LIN_MAX)` (rear down)
- `space` now zeros 5 axes: `vx, vy, vz, wy, wz`.
- `publish()` sets `msg.angular.y = self._wy`.
- HELP text updated:
  ```
  i/k  rear up / down  (angular.y — rear-height velocity)
  ```
- Log line in `on_key` includes `wy`.

### `dog_robot_kinematic_viz/config/kinematic_params.yaml`

Append:
```yaml
rear_z_min: -0.05
rear_z_max: +0.05
```

### Launch files

No changes. `kinematic_params.yaml` is already loaded; new params surface automatically.

## Conflict policy (body_z × rear_z)

Per the brainstorming conclusion: orthogonal clamps + IK-freeze fallback + WARN-once.

Recall the per-leg foot-Z displacement: front legs use `−body_z`, rear legs use `−body_z + rear_z`. Positive means foot rises in body frame (toward body, i.e. leg folds).

| body_z | rear_z | Front net | Rear net | Visual | Handle |
|---|---|---|---|---|---|
| 0 | 0 | 0 | 0 | nominal stand | ✓ |
| +max | 0 | extend down | extend down | uniform body up | ✓ existing |
| −max | 0 | fold up | fold up | uniform body down | ✓ existing |
| 0 | +max | 0 | fold up | **sit target** — front nominal, rear folded | ✓ |
| +max | +max | extend down | mild fold | tall stance with slight rear fold | rear may saturate if total > reach |
| −max | +max | fold up | strong fold | low stance with heavy rear fold | rear likely saturates |
| +max | −max | extend down | strong extend | tall front, rear over-extended | rear likely saturates |
| −max | −max | fold up | mild extend | crouched front, rear near nominal | ✓ |

Front legs always follow only `body_z` and are unaffected by `rear_z`, so they never enter a state introduced by this feature. They retain their existing IK-freeze guard for any body_z-driven failures (no behavioral change).

## Data flow (one 20 ms tick at 50 Hz)

```
[teleop key 'i']
  teleop._wy += 0.02 (clamp ±0.20)
  publish Twist(linear=(vx,vy,vz), angular=(0, _wy, wz)) on /cmd_vel

[kinematic_node._on_cmd_vel]
  BodyCommander.on_cmd_vel(vx, vy, vz, msg.angular.y, msg.angular.z)
    store _vz, _wy as velocity inputs

[kinematic_node._tick, dt ≈ 0.02]
  BodyCommander.tick(dt)
    _z      += _vz * dt; clamp [body_z_min, body_z_max]
    _rear_z += _wy * dt; clamp [rear_z_min, rear_z_max]
  body_z = commander.body_z()
  rear_z = commander.rear_z()
  v_xy   = commander.body_vel_xy()
  for leg in (FL, FR, BL, BR):
    driver.step(v_xy, phase, body_z=body_z, rear_z=rear_z)
      extra_z = rear_z if is_rear else 0.0
      foot_target_in_hip(rest, phase, v_xy, body_z, extra_z, R, ft)
        disp_body.z = z_lift - body_z + extra_z
        target = rest_in_hip + R.T @ disp_body
      ik_leg(p, target, knee_branch=+1)
        success → return joints; clear _saturated
        ValueError → if not _saturated: warn + set; return _last_joints
  publish JointState (12 positions) on /joint_states
```

## Error handling

- **IK fail on rear leg:** `LegDriver` returns last good joints; WARN logged once per saturation event (cleared on next success).
- **IK fail on front leg:** existing behavior unchanged.
- **Clamp at BodyCommander:** prevents `rear_z` from exceeding declared bounds even with sustained max velocity.
- **No special handling for combined (body_z + rear_z) exceeding leg reach:** IK-freeze guard absorbs it. Documented in the conflict table.

## Testing

### `test_body_commander.py` (extend)

- `on_cmd_vel(0,0,0, 0, 0); tick(1.0); rear_z() == 0` — no input → no integration.
- `on_cmd_vel(0,0,0, +1.0, 0); tick(0.1); rear_z() == +0.1 ` (raw, pre-clamp under bound).
- `on_cmd_vel(0,0,0, +1.0, 0); tick(1.0); rear_z() == +0.05` — clamp at `rear_z_max`.
- Symmetric for `-1.0` and `rear_z_min`.
- Crosstalk guard: `on_cmd_vel(0,0,+1.0, 0, 0); tick(0.5)` → `body_z() == +0.03`, `rear_z() == 0`.
- And: `on_cmd_vel(0,0,0, +1.0, 0); tick(0.5)` → `body_z() == 0`, `rear_z() == +0.05`.

### `test_foot_target.py` (extend)

- Regression: existing tests pass with new arg defaulted to `extra_z=0.0`.
- `extra_z=+0.05` while `body_z=0, v_xy=0`, phase in stance: output target's body-Z component is `+0.05` higher than the `extra_z=0` case (i.e. foot lifted toward body by 0.05 m in body +Z). Verify by computing `R_base_to_hip @ (target − rest_in_hip)` and reading its Z component.

### `test_leg_driver.py` (extend)

- `is_rear=False, rear_z=±0.05`: joints unchanged from `rear_z=0` baseline at same phase.
- `is_rear=True, rear_z=+0.05`: joints differ from baseline; the rear foot lifts in body frame (knee + thigh fold toward the body).
- WARN-once: feed an unreachable target (e.g., gigantic rear_z = `+10.0` to force IK fail). First step → WARN logged. Second step → no WARN. Then feed reachable target → success, `_saturated` cleared. Force unreachable again → WARN logged a second time.
- Inject `logger` mock to count `warning` calls.

### `test_kinematic_node.py` (extend)

- Publish `cmd_vel.angular.y = +1.0` once, spin N ticks: `BodyCommander.rear_z()` reaches clamp; `JointState` for BL/BR differs from baseline; FL/FR positions unchanged.

### `test_teleop_keyboard.py` (extend)

- `'i'` → `_wy == +0.02`, published `msg.angular.y == +0.02`.
- `'k'` → `_wy == -0.02`.
- 5×`'i'` then `' '` → all five axes back to 0.

### Manual smoke

```
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws
pkill -f rviz2; pkill -f kinematic_node; sleep 1
colcon build --packages-select dog_robot_kinematic_viz
source install/setup.bash
ros2 launch dog_robot_kinematic_viz kinematic_teleop.launch.py
# in the teleop terminal:
#   press 'i' five times → BL/BR feet visibly lift toward body in RViz
#   FL/FR unchanged
#   press 'k' five times → BL/BR return to baseline
#   press 'r' / 'f' → uniform whole-body raise/lower (existing behavior unaffected)
```

## Out of scope

- "Lie" pose (would add a symmetric `front_z` on a second Twist axis).
- Discrete pose presets (e.g., `/set_pose` service with named targets).
- Pose interpolation slider (`sit_progress ∈ [0,1]`).
- Hardware bringup or Gazebo integration.

## Risks

- **angular.y semantic drift:** repurposing `Twist.angular.y` as rear_vz is non-standard. Mitigated by documenting it in the teleop HELP text and the node's `__init__` log line; only `kinematic_node` consumes this Twist, so no external publisher will be confused.
- **WARN-once flag stuck on rapid oscillation:** if rear_z oscillates around the IK boundary, the flag clears then re-trips. This is intended — the user sees a clean WARN each time the limit is newly crossed.
- **Clamp range too tight:** if `±0.05` does not reach a recognizable sit visual, retune via `rear_z_min/max` params at launch time without code changes.
