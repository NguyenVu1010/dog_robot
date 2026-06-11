# Named-Pose API (`/sit`, `/release`) — Design

Date: 2026-06-12
Branch: `feature/body-height`
Package: `dog_robot_kinematic_viz`
Related: [2026-06-10-rear-z-sit-pose-design.md](2026-06-10-rear-z-sit-pose-design.md)

## Goal

Add a discrete, locked-pose API on top of the existing continuous-control rig. When a UI (future) or operator (today) calls `/sit`, the robot snaps to a hardcoded sit pose and **ignores `/cmd_vel` updates** until `/release` is called. While locked, the visual is fixed regardless of teleop input.

This complements the continuous `pitch_amount` knob — it does NOT replace it. The two interfaces serve different intents:

| Use case | Interface |
|---|---|
| Interactive exploration of pose space (continuous tuning) | teleop keys `i`/`k` → `Twist.angular.y` → `pitch_amount` |
| "Get to the sit pose now, stay there, and don't let cmd_vel push it around" | `/sit` service |

## Scope

In scope:
- 2 ROS 2 services: `/sit` and `/release`, both `std_srvs/srv/Trigger`.
- Lock state in `KinematicNode` — while locked, `_tick()` publishes hardcoded joint angles instead of computing from BodyCommander.
- Hardcoded sit-pose joint angles in `kinematic_params.yaml` under `sit_pose.joints` (12 floats in canonical joint order).
- `cmd_vel` callbacks still arrive but their updates do not surface in `/joint_states` while locked.
- `BodyCommander.tick(dt)` is NOT called while locked, so its `body_z` and `pitch_amount` integrators freeze at their lock-time values. On `/release` the commander resumes from where it was — the immediate post-release tick reflects whatever cmd_vel had stored.

Out of scope:
- Smooth interpolation between current pose and sit pose (instant snap).
- `/lie`, `/stand`, or other named poses (the architecture leaves room to add them by copying the pattern — each new pose = one service + one yaml block).
- A unified custom `SetPose.srv` (separate Trigger services keep the surface dead-simple for the v1 UI).
- Persisting the lock state across node restart.

## Architecture

`KinematicNode` gains two pieces of state:
- `self._locked_joints: Optional[Tuple[float, ...]] = None` — when not None, `_tick` publishes this exact 12-tuple verbatim. When None, the existing dynamic path runs.
- Service servers `/sit` and `/release` bound to handlers `_on_sit` and `_on_release`.

```
                     ┌─────────────────┐
   /sit ────────────▶│  _on_sit        │──▶ self._locked_joints = SIT_JOINTS
                     └─────────────────┘                ▲
                                                        │ checked at top of
   /release ───────▶ _on_release ──▶ self._locked = None ─── _tick()
                                                        │
   /cmd_vel ─▶ _on_cmd_vel ─▶ BodyCommander (stored)    │
                                                        ▼
   timer ────────────────────────────▶ _tick() ──▶ /joint_states
                                          │
                                          ├─ if locked: publish _locked_joints
                                          └─ else:      existing dynamic path
```

`BodyCommander` is unchanged. `LegDriver` is unchanged. Only `KinematicNode` and the yaml change.

## Components

### `dog_robot_kinematic_viz/kinematic_node.py`

(a) **New parameter**: `sit_pose_joints` — list of 12 floats in canonical joint order (`FL_hip_roll, FL_thigh_pitch, FL_knee_pitch, FR_*, BL_*, BR_*`). Default: a hand-tuned dog-sit configuration (see yaml below).

(b) **New state**: `self._locked_joints: Optional[Tuple[float, ...]] = None`.

(c) **Validation in `__init__`**: read `sit_pose_joints`, check length == 12, store as `self._sit_pose_joints: Tuple[float, ...]`. Raise `ValueError` on length mismatch.

(d) **Service servers**:

```python
from std_srvs.srv import Trigger

self._sit_srv = self.create_service(Trigger, "/sit", self._on_sit)
self._release_srv = self.create_service(Trigger, "/release", self._on_release)
```

(e) **Handlers**:

```python
def _on_sit(self, request, response):
    self._locked_joints = self._sit_pose_joints
    self.get_logger().info("pose lock: /sit engaged")
    response.success = True
    response.message = "sit pose locked"
    return response

def _on_release(self, request, response):
    was_locked = self._locked_joints is not None
    self._locked_joints = None
    self.get_logger().info("pose lock: /release engaged")
    response.success = True
    response.message = "lock released" if was_locked else "was not locked (no-op)"
    return response
```

(f) **`_tick()` short-circuit**:

```python
def _tick(self):
    now = time.monotonic()
    dt = now - self._t_last
    self._t_last = now

    if self._locked_joints is not None:
        # Pose lock: publish the hardcoded snapshot verbatim.
        # BodyCommander.tick is skipped so its integrators freeze.
        stamp = self.get_clock().now().to_msg()
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = self._joint_names
        msg.position = list(self._locked_joints)
        self._pub.publish(msg)
        # Foot trails: do NOT append while locked (would inject a flat segment)
        # but DO re-publish the existing markers so RViz keeps them on screen.
        # ...existing trail publish loop without the append step...
        return

    # ...existing dynamic _tick body unchanged...
```

### `dog_robot_kinematic_viz/config/kinematic_params.yaml`

Append a `sit_pose_joints` block. Starting values are hand estimates; the user will tune in RViz:

```yaml
    # Sit-pose joint snapshot (12 floats in FL/FR/BL/BR × hip_roll/thigh_pitch/knee_pitch order).
    # Engaged via service `/sit`. Released via `/release`. Front legs stay near
    # straight (light knee bend); rear legs fold deeply toward the body.
    sit_pose_joints:
      # FL: front-left
      - 0.0
      - -0.30
      - +0.30
      # FR: front-right
      - 0.0
      - -0.30
      - +0.30
      # BL: back-left (folded)
      - 0.0
      - +1.00
      - -2.20
      # BR: back-right (folded)
      - 0.0
      - +1.00
      - -2.20
```

These defaults are within the URDF joint limits (hip_roll ±0.785, thigh_pitch ±1.571, knee_pitch [-2.617, +0.5]). The user can re-tune at launch time with `--ros-args -p sit_pose_joints:='[...]'` or by editing the yaml.

### No changes to `body_commander.py`, `foot_target.py`, `leg_driver.py`, `teleop_keyboard.py`.

## Data flow (locked state)

```
[client]    ros2 service call /sit std_srvs/srv/Trigger {}
              → kinematic_node._on_sit
              → self._locked_joints = self._sit_pose_joints
              → response {success: true, message: "sit pose locked"}

[timer @50Hz] kinematic_node._tick
              → self._locked_joints is not None
              → publish JointState with self._sit_pose_joints
              → return (skip BodyCommander.tick, LegDriver.step, foot trail append)

[teleop]      user presses 'r' / 'i' / 'w'
              → /cmd_vel published
              → kinematic_node._on_cmd_vel
              → commander.on_cmd_vel stores velocity inputs
              → NO TICK happens during lock, so body_z and pitch_amount stay at lock-time values

[client]    ros2 service call /release std_srvs/srv/Trigger {}
              → kinematic_node._on_release
              → self._locked_joints = None
              → response {success: true, message: "lock released"}

[timer]       next _tick
              → self._locked_joints is None
              → commander.tick(dt) runs (resumes integration from frozen values)
              → driver.step(...) runs as normal
              → publish dynamic JointState
```

## Error handling

- `/sit` on an already-locked node: no-op semantically (just re-assigns the same tuple). Returns `success=true, message="sit pose locked"`. No error.
- `/release` on a non-locked node: no-op, returns `success=true, message="was not locked (no-op)"`.
- Invalid `sit_pose_joints` (wrong length): `__init__` raises `ValueError` before the node is up — fail-fast at launch.
- Service callbacks run on the executor's thread; `_tick` runs on the timer thread. The single assignment `self._locked_joints = ...` is atomic in CPython (GIL holds the reference assignment), and `_tick` reads it once into a local. No lock needed.

## Testing

### Unit / smoke (`test_kinematic_node_smoke.py`)

Add 4 new tests:

- **`test_sit_pose_joints_param_validation`** — passing `sit_pose_joints=[1,2,3]` (length 3) raises `ValueError` at construction.
- **`test_sit_locks_joints_to_yaml_values`** — call `/sit`, spin a few ticks, verify `/joint_states.position` exactly equals the configured 12 floats.
- **`test_release_resumes_dynamic_control`** — call `/sit`, capture locked joints, call `/release`, publish nonzero `cmd_vel`, spin, verify positions diverge from the locked snapshot (back to dynamic).
- **`test_cmd_vel_during_lock_does_not_change_joints`** — call `/sit`, publish `cmd_vel.linear.x = 0.1` for 0.5 s, verify positions stay identical to the locked snapshot.

All four can reuse the existing `rclpy_ctx` fixture and the `_overrides()` helper (add `sit_pose_joints=[...]` support).

## Out of scope and future work

- A `/lie` or `/stand` service follows the same pattern (one extra yaml block + one extra handler). The Trigger-per-pose design keeps each pose self-contained.
- A smooth ramp into the locked pose (instead of snap) would add a `_ramp_start_t`, `_ramp_from`, `_ramp_dur` triple. Not in v1 — the UI may want it; defer until that becomes a real requirement.
- Persisting lock across restart and pose state machines (e.g. "after `/sit`, only `/release` and `/stand` are allowed") are explicit non-goals.

## Risks

- **Frozen `BodyCommander.tick` during lock means commander time `_t` does not advance.** On release, the next `_tick(dt)` sees a `dt` reflecting wall time since last unlock, which advances the gait phase by a lot in one frame. With `step_freq=0` (kinematic verification mode) this is harmless. With normal trot it could produce a one-frame phase jump. Mitigation: clamp `dt` to a sane maximum on the first unlocked tick, or reset `self._t_last = time.monotonic()` inside `_on_release`. Plan picks the latter — simpler and explicit.
- **Trigger services have no payload**, so future pose-name expansion via the same endpoint requires a custom service type. The "one service per pose" choice avoids that for v1 but the count of services grows linearly with poses. Acceptable for the 1-3 named poses we expect.
- **Hardcoded joint values may be out-of-spec** for a future URDF revision (link length changes). Risk is low because the values live in yaml; a URDF change that breaks them just means re-tuning the yaml.
