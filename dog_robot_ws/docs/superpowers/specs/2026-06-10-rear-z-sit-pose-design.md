# Pitch Posture Control (Sit Pose) — Design

Date: 2026-06-10 (revised 2026-06-11)
Branch: `feature/body-height` (worktree `/home/nguyenvd/workspace/dog_robot_height`)
Package: `dog_robot_kinematic_viz`
Related: [2026-06-01-body-height-design.md](2026-06-01-body-height-design.md), [2026-05-24-kinematics-viz-design.md](2026-05-24-kinematics-viz-design.md)

## Goal

Add a continuous body-pitch control to the kinematic viz rig, analogous to the existing whole-body height control (`linear.z` → `body_z`). When the pitch control is driven to its positive bound, the robot reaches a dog-sit pose: rear feet fold toward the body AND front feet extend away from the body in the same gesture. The negative bound is the inverse — front fold, rear extend (play-bow-like). Sit and play-bow are emergent end-states of the slider, not discrete presets.

This is a kinematic verification feature: base_link is fixed by static TF; success criterion is RViz visually showing, on positive pitch, BL/BR feet pulled toward the body while FL/FR are pushed away (legs straighten), without IK crashes and within joint limits.

A future "lie" pose can be reached today via `linear.z` at its negative bound (uniform compression of all four legs) with `pitch_amount = 0`. Explicit pose presets remain out of scope.

### Revision history

- **2026-06-10 v1**: original design used a single `rear_z` knob that lifted only BL/BR; front legs ignored it. Rejected by user during smoke check because real sit visual requires front extension too.
- **2026-06-11 v2 (this doc)**: knob renamed `pitch_amount`; same input affects all four legs, with front legs getting the opposite sign so the body appears pitched up at the front. The implementation rename is a small per-leg sign flip in `LegDriver`.

## Architecture

The existing pipeline is unchanged in shape:

```
/cmd_vel  →  BodyCommander  →  LegDriver × 4  →  /joint_states
```

`BodyCommander` gains one new state, `pitch_amount` (m), integrated from a new velocity input on the existing Twist (`angular.y`). `foot_target_in_hip` accepts one additional scalar `extra_z` that is summed into the body-frame Z displacement of the foot. `LegDriver` decides what to forward as `extra_z` based on a per-instance `is_rear` flag set at construction:

```
extra_z = +pitch_amount  if leg is rear (BL, BR)
extra_z = -pitch_amount  if leg is front (FL, FR)
```

Both halves move on the same input, opposite signs. The net effect is a virtual body pitch: positive `pitch_amount` raises the rear of the body in the world (rear feet pulled up toward body in body frame) AND lowers the front of the body (front feet pushed away from body in body frame, leg extends). This matches the visual of a dog sitting back on its haunches with front legs straight.

### Sign convention

Two scalars describe body-Z posture; their signs map to different physical quantities and must not be conflated.

- **`body_z`** describes the *body's* uniform height above its rest pose. `body_z > 0` ⇒ body raised; in body frame all four feet appear to drop. Subtracted in the formula (matches the existing body-height code).
- **`pitch_amount`** describes the body pitch about its centre. `pitch_amount > 0` ⇒ body pitched up at the front (rear sinks in world). In body frame this manifests as REAR feet lifted toward the body (`+pitch_amount`) and FRONT feet pushed down away from the body (`-pitch_amount`). Added in the formula via `extra_z`, with the sign decided per leg by `is_rear`.

User-facing: pressing the rear-up teleop key (`i`) drives `pitch_amount` positive, which in RViz visibly lifts BL/BR feet toward the body AND drops FL/FR feet farther from the body — the dog-sit pose visual.

## Components

### `dog_robot_kinematic_viz/body_commander.py`

`BodyCommander` is the single source of truth for the pitch clamp, just as it already is for body_z.

- New ctor params: `pitch_min: float = -0.05`, `pitch_max: float = +0.05` (wider than body_z because pitch travel adds to body_z on rear and subtracts on front; each direction needs more headroom than a uniform height change).
- New private state: `self._pitch = 0.0`, `self._wy = 0.0` (pitch velocity input).
- `on_cmd_vel` signature changes to 5 args:
  `on_cmd_vel(linear_x, linear_y, linear_z, angular_y, angular_z)`.
  Only `kinematic_node` calls it, so no backward-compat shim is needed.
- `tick(dt)` adds `new_pitch = self._pitch + self._wy * dt`, clamped to `[pitch_min, pitch_max]`.
- New accessor: `pitch_amount() -> float`.
- Existing `body_z()`, `body_vel_xy()`, `phase()` unchanged.

### `dog_robot_kinematic_viz/foot_target.py`

`foot_target_in_hip` gains one scalar arg, inserted after `body_z`:

```
foot_target_in_hip(rest_in_hip, phase, v_body_xy, body_z, extra_z, R_base_to_hip, params)
```

Body changes one line: `disp_body.z = z_lift_body - body_z + extra_z`.

`extra_z` is the leg-frame-agnostic body-Z foot-lift offset (in body +Z). Callers decide what value to pass per leg (`+pitch_amount` for rear, `-pitch_amount` for front). The sign is opposite to `body_z` because the two scalars describe different physical things — see the Sign Convention subsection above.

This keeps `foot_target_in_hip` pure math — no knowledge of FL/BL/FR/BR membership and no awareness of the pitch concept. The function is the same in v1 and v2; only the caller policy in `LegDriver` changed.

### `dog_robot_kinematic_viz/leg_driver.py`

`LegDriver.__init__` gains `is_rear: bool = False`. Stored as `self.is_rear`.

`step` signature gains `pitch_amount: float = 0.0`:

```
step(body_v_xy, phase, body_z=0.0, pitch_amount=0.0)
```

Body picks the per-leg sign of `extra_z` from `is_rear`:

```
extra_z = pitch_amount if self.is_rear else -pitch_amount
```

and passes to `foot_target_in_hip`. The IK-freeze guard (catch `ValueError`, return `_last_joints`) is unchanged.

`LegDriver` also gains de-bounced WARN-once: `self._saturated = False`. On `ValueError` and `not _saturated`: `logger.warning(...)`, set `_saturated = True`. On `ik_leg` success while `_saturated`: clear the flag. The logger is injected from `kinematic_node`; if None, fall back to `print`.

### `dog_robot_kinematic_viz/kinematic_node.py`

- Declare 2 new params: `pitch_min: -0.05`, `pitch_max: +0.05`.
- Pass to `BodyCommander(step_freq=..., body_z_min=..., body_z_max=..., pitch_min=..., pitch_max=...)`.
- `_on_cmd_vel(msg)`: forward `msg.angular.y` as `angular_y`:
  `self.commander.on_cmd_vel(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.y, msg.angular.z)`.
- LegDriver construction: `is_rear=(name in ("BL", "BR"))`. Pass `logger=self.get_logger()`.
- `_tick()`: read `pitch = self.commander.pitch_amount()`; pass to each `driver.step(v_xy, phase, body_z, pitch)`.

### `dog_robot_kinematic_viz/teleop_keyboard.py`

- New state: `self._wy = 0.0`.
- New keys:
  - `i` → `_wy = clamp(_wy + LIN_STEP, -LIN_MAX, LIN_MAX)` (sit pitch up)
  - `k` → `_wy = clamp(_wy - LIN_STEP, -LIN_MAX, LIN_MAX)` (sit pitch down)
- `space` now zeros 5 axes: `vx, vy, vz, wy, wz`.
- `publish()` sets `msg.angular.y = self._wy`.
- HELP text updated:
  ```
  i/k  sit / unsit       (angular.y — body pitch velocity)
  ```
- Log line in `on_key` includes `wy`.

### `dog_robot_kinematic_viz/config/kinematic_params.yaml`

Append:
```yaml
pitch_min: -0.05
pitch_max: +0.05
```

### Launch files

No changes. `kinematic_params.yaml` is already loaded; new params surface automatically.

## Conflict policy (body_z × pitch_amount)

Per the brainstorming conclusion: orthogonal clamps + IK-freeze fallback + WARN-once.

Recall the per-leg foot-Z displacement:
- front legs: `disp.z = z_lift − body_z − pitch_amount`
- rear  legs: `disp.z = z_lift − body_z + pitch_amount`

Positive `disp.z` means foot rises in body frame (toward body, i.e. leg folds).

| body_z | pitch | Front net | Rear net | Visual | Handle |
|---|---|---|---|---|---|
| 0 | 0 | 0 | 0 | nominal stand | ✓ |
| +max | 0 | extend down | extend down | uniform body up | ✓ existing |
| −max | 0 | fold up | fold up | uniform body down (≈ low crouch / lie) | ✓ existing |
| 0 | +max | extend down | fold up | **sit target** — front extended, rear folded | ✓ |
| 0 | −max | fold up | extend down | play-bow — front folded, rear extended | ✓ |
| +max | +max | strong extend | mild fold | tall stance with rear fold; front may saturate (extension limit) | front WARN |
| −max | +max | mild extend | strong fold | low body with deep sit; rear may saturate (over-fold) | rear WARN |
| +max | −max | mild fold | strong extend | tall stance with rear extension; rear may saturate | rear WARN |
| −max | −max | strong fold | mild extend | deep play-bow; front may saturate | front WARN |

Both halves are now active under `pitch_amount`. Saturation can hit either side; the existing IK-freeze guard and WARN-once apply per-leg independently.

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
    _z     += _vz * dt; clamp [body_z_min, body_z_max]
    _pitch += _wy * dt; clamp [pitch_min,  pitch_max]
  body_z = commander.body_z()
  pitch  = commander.pitch_amount()
  v_xy   = commander.body_vel_xy()
  for leg in (FL, FR, BL, BR):
    driver.step(v_xy, phase, body_z=body_z, pitch_amount=pitch)
      extra_z = pitch if is_rear else -pitch
      foot_target_in_hip(rest, phase, v_xy, body_z, extra_z, R, ft)
        disp_body.z = z_lift - body_z + extra_z
        target = rest_in_hip + R.T @ disp_body
      ik_leg(p, target, knee_branch=+1)
        success → return joints; clear _saturated
        ValueError → if not _saturated: warn + set; return _last_joints
  publish JointState (12 positions) on /joint_states
```

## Error handling

- **IK fail on any leg:** `LegDriver` returns last good joints; WARN logged once per saturation event (cleared on next success). Both front and rear are now reachable failure modes under `pitch_amount` extremes (front saturates on extension; rear saturates on fold).
- **Clamp at BodyCommander:** prevents `pitch_amount` from exceeding declared bounds even with sustained max velocity.
- **No special handling for combined (body_z + pitch_amount) exceeding leg reach:** IK-freeze guard absorbs it. Documented in the conflict table.

## Testing

### `test_body_commander.py` (extend)

- `on_cmd_vel(0,0,0, 0, 0); tick(1.0); pitch_amount() == 0` — no input → no integration.
- `on_cmd_vel(0,0,0, +1.0, 0); tick(1.0); pitch_amount() == +0.05` — clamp at `pitch_max`.
- Symmetric for `-1.0` and `pitch_min`.
- Crosstalk: `vz` does not affect `pitch_amount`; `wy` does not affect `body_z`.
- `space` (zero on_cmd_vel) halts further pitch integration.

### `test_foot_target.py` (unchanged)

`foot_target_in_hip` is unaware of the pitch concept. Its tests stay as written in v1 (`extra_z=+0.05` lifts foot, `extra_z=0` regression, composition with `body_z` + swing).

### `test_leg_driver.py` (extend)

- `is_rear=False, pitch_amount=+0.05`: front foot DROPS by 0.05 m in body Z (extends). Verify by FK roundtrip.
- `is_rear=True,  pitch_amount=+0.05`: rear foot LIFTS by 0.05 m in body Z (folds).
- Symmetry test: at the same `|pitch_amount|`, front foot displacement equals the negative of rear foot displacement (sign-flip relationship).
- `step(... pitch_amount=0.0)` matches the no-arg default at every phase.
- WARN-once on front-extension saturation: feed an unreachable `pitch_amount` (e.g. `-10.0` to extend the FL foot far past reach via `extra_z = +10`) and verify WARN fires exactly once. Recovery + re-saturation fires a second WARN.

### `test_kinematic_node.py` (extend)

- Publish `cmd_vel.angular.y = +1.0` once, spin N ticks: `BodyCommander.pitch_amount()` reaches clamp.
- At `step_freq=0` (gait frozen), positive `angular.y` MOVES all four legs: FL/FR joint snapshots differ from baseline (extension) AND BL/BR joint snapshots differ from baseline (fold).
- Front delta and rear delta are both > 1e-3 at the snapshot; their signs are opposite when projected through FK→body-frame Z.
- Param plumbing: overriding `pitch_min`/`pitch_max` reaches `BodyCommander.pitch_min`/`pitch_max`.

### `test_teleop_keyboard.py` (no semantic change)

`i`/`k` still drive `_wy` ↔ `Twist.angular.y`. The only update needed is the HELP/docstring text describing the renamed meaning. Existing keypress + publish tests stand.

### Manual smoke

```
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws
pkill -f rviz2; pkill -f kinematic_node; sleep 1
colcon build --packages-select dog_robot_kinematic_viz
source install/setup.bash
ros2 launch dog_robot_kinematic_viz kinematic_teleop.launch.py
# in the teleop terminal:
#   press 'i' five times → BL/BR feet visibly lift toward body AND
#                          FL/FR feet visibly drop away from body in RViz
#                          (sit visual: dog leaning back, front legs straight)
#   press 'k' five times → all four legs return to nominal stand
#   press 'k' five more times past nominal → play-bow visual
#                          (front folds, rear extends)
#   press 'r' / 'f' → uniform whole-body raise/lower (existing behavior unaffected)
```

## Out of scope

- A dedicated "lie" preset (achievable today as `body_z = body_z_min`, `pitch_amount = 0`).
- Discrete pose presets (e.g., `/set_pose` service with named targets).
- Pose interpolation slider with stored end-state joint targets.
- Hardware bringup or Gazebo integration.

## Risks

- **angular.y semantic drift:** repurposing `Twist.angular.y` as a pitch velocity is non-standard. Mitigated by documenting it in the teleop HELP text and the node's `__init__` log line; only `kinematic_node` consumes this Twist, so no external publisher will be confused.
- **WARN-once flag stuck on rapid oscillation:** if `pitch_amount` oscillates around the IK boundary, the flag clears then re-trips. This is intended — the user sees a clean WARN each time the limit is newly crossed.
- **Front-extension limit asymmetric to rear-fold limit:** the leg can fold deeper than it can extend (`knee_pitch ∈ [-2.617, +0.5]`). At full positive `pitch_amount`, the front legs reach their extension limit first. The symmetric ±0.05 clamp is a starting point; tune via `pitch_min/pitch_max` in `kinematic_params.yaml` if the asymmetry shows up in the smoke test.
- **Naming churn from v1 to v2:** the rename from `rear_z` → `pitch_amount` (with the per-leg sign flip in `LegDriver`) is the v2 patch. Spec, plan, code, and tests must all carry the new name; the implementation task in v2's plan is the single rename + sign-flip + test update.
