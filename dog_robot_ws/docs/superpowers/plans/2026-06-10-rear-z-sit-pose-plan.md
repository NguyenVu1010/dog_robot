# Rear-Z Sit-Pose Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a continuous rear-only body-Z control to the kinematic viz rig so the user can press teleop keys `i`/`k` to fold the rear legs toward the body (sit pose) without affecting the front legs.

**Architecture:** Extend `BodyCommander` with a new clamped state `rear_z`, integrated from `Twist.angular.y`. Pass it to `LegDriver` which forwards it as `extra_z` to `foot_target_in_hip` only when the leg is rear (`is_rear=True` for `BL`, `BR`). The math added to the existing displacement formula is `+ extra_z` (foot lifts in body frame). Front legs are byte-for-byte unchanged at runtime.

**Tech Stack:** Python 3.10, ROS 2 Humble, rclpy, NumPy, pytest. Workspace: `/home/nguyenvd/workspace/dog_robot_height/dog_robot_ws`. Branch: `feature/body-height`.

**Spec:** [`docs/superpowers/specs/2026-06-10-rear-z-sit-pose-design.md`](../specs/2026-06-10-rear-z-sit-pose-design.md)

---

## File Map

All paths relative to `dog_robot_ws/src/dog_robot_kinematic_viz/`.

**Modified source files:**
- `dog_robot_kinematic_viz/body_commander.py` — add `rear_z` state + clamp + 5-arg `on_cmd_vel`
- `dog_robot_kinematic_viz/foot_target.py` — add `extra_z` arg; new formula `disp.z = z_lift - body_z + extra_z`
- `dog_robot_kinematic_viz/leg_driver.py` — `is_rear` ctor arg, `rear_z` step kwarg, WARN-once-on-IK-fail
- `dog_robot_kinematic_viz/kinematic_node.py` — declare `rear_z_min/max` params, forward `angular.y`, wire `is_rear`, pass `rear_z` to step
- `dog_robot_kinematic_viz/teleop_keyboard.py` — `i/k` keys → `_wy` → `Twist.angular.y`; space zeros 5 axes
- `config/kinematic_params.yaml` — add `rear_z_min/max` defaults

**Modified test files:**
- `test/test_body_commander.py` — update existing 4-arg calls to 5-arg; add `rear_z` tests
- `test/test_foot_target.py` — update `_ft` helper; add `extra_z` tests
- `test/test_leg_driver.py` — update `_make_drivers` to inject `is_rear`; add `rear_z` + WARN-once tests
- `test/test_kinematic_node_smoke.py` — add `angular.y → BL/BR` smoke + `rear_z_min/max` param test
- `test/test_teleop_keyboard.py` — replace 4-axis space-zero test with 5-axis; add `i/k` tests + `angular.y` publish

---

## Task 1: BodyCommander — rear_z state + 5-arg on_cmd_vel

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/body_commander.py`
- Modify: `dog_robot_ws/src/dog_robot_kinematic_viz/test/test_body_commander.py`
- Modify: `dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/kinematic_node.py` (call-site update only — pass `msg.angular.y` so the signature change does not break the smoke test)

**Why bundled:** Changing `on_cmd_vel` to 5 args breaks `kinematic_node._on_cmd_vel`. Updating the call site in the same task keeps `colcon test` green.

- [ ] **Step 1: Update existing `test_body_commander.py` to 5-arg `on_cmd_vel` calls**

Replace every 4-arg `on_cmd_vel(...)` call. The new arg is `angular_y`, inserted *before* `angular_z`.

In `test/test_body_commander.py`, change these lines:

```python
# test_on_cmd_vel_updates_state
b.on_cmd_vel(0.3, -0.1, 0.03, 0.5)
# ->
b.on_cmd_vel(0.3, -0.1, 0.03, 0.0, 0.5)
```

```python
# test_vz_integrates_into_body_z
b.on_cmd_vel(0.0, 0.0, 0.02, 0.0)
# ->
b.on_cmd_vel(0.0, 0.0, 0.02, 0.0, 0.0)
```

```python
# test_body_z_clamps_at_max
b.on_cmd_vel(0.0, 0.0, 0.10, 0.0)
# ->
b.on_cmd_vel(0.0, 0.0, 0.10, 0.0, 0.0)
```

```python
# test_body_z_clamps_at_min
b.on_cmd_vel(0.0, 0.0, -0.10, 0.0)
# ->
b.on_cmd_vel(0.0, 0.0, -0.10, 0.0, 0.0)
```

```python
# test_space_zeros_vz_halts_integration (two calls)
b.on_cmd_vel(0.0, 0.0, 0.02, 0.0)
b.on_cmd_vel(0.0, 0.0, 0.0, 0.0)
# ->
b.on_cmd_vel(0.0, 0.0, 0.02, 0.0, 0.0)
b.on_cmd_vel(0.0, 0.0, 0.0, 0.0, 0.0)
```

```python
# test_body_z_min_max_params_respected
b.on_cmd_vel(0.0, 0.0, 1.0, 0.0)
# ->
b.on_cmd_vel(0.0, 0.0, 1.0, 0.0, 0.0)
```

- [ ] **Step 2: Add the failing tests for `rear_z`**

Append at the end of `test/test_body_commander.py`:

```python
# --- rear_z tests ---

def test_default_rear_z_is_zero():
    b = BodyCommander()
    assert b.rear_z() == 0.0


def test_wy_integrates_into_rear_z():
    b = BodyCommander()
    b.on_cmd_vel(0.0, 0.0, 0.0, 0.04, 0.0)
    b.tick(0.1)
    assert b.rear_z() == pytest.approx(0.004, abs=1e-9)
    b.tick(0.1)
    assert b.rear_z() == pytest.approx(0.008, abs=1e-9)


def test_rear_z_clamps_at_max():
    b = BodyCommander()  # default rear_z_max = +0.05
    b.on_cmd_vel(0.0, 0.0, 0.0, 0.10, 0.0)
    for _ in range(100):
        b.tick(0.01)
    assert b.rear_z() == pytest.approx(0.05, abs=1e-9)


def test_rear_z_clamps_at_min():
    b = BodyCommander()  # default rear_z_min = -0.05
    b.on_cmd_vel(0.0, 0.0, 0.0, -0.10, 0.0)
    for _ in range(100):
        b.tick(0.01)
    assert b.rear_z() == pytest.approx(-0.05, abs=1e-9)


def test_rear_z_min_max_params_respected():
    b = BodyCommander(rear_z_min=-0.10, rear_z_max=+0.10)
    b.on_cmd_vel(0.0, 0.0, 0.0, 1.0, 0.0)
    for _ in range(50):
        b.tick(0.01)
    assert b.rear_z() == pytest.approx(0.10, abs=1e-9)


def test_wy_does_not_affect_body_z():
    b = BodyCommander()
    b.on_cmd_vel(0.0, 0.0, 0.0, 0.10, 0.0)
    for _ in range(100):
        b.tick(0.01)
    assert b.body_z() == 0.0
    assert b.rear_z() == pytest.approx(0.05, abs=1e-9)


def test_vz_does_not_affect_rear_z():
    b = BodyCommander()
    b.on_cmd_vel(0.0, 0.0, 0.10, 0.0, 0.0)
    for _ in range(100):
        b.tick(0.01)
    assert b.rear_z() == 0.0
    assert b.body_z() == pytest.approx(0.03, abs=1e-9)


def test_space_zeros_wy_halts_rear_z_integration():
    b = BodyCommander()
    b.on_cmd_vel(0.0, 0.0, 0.0, 0.04, 0.0)
    b.tick(0.5)
    rz_after = b.rear_z()
    assert rz_after == pytest.approx(0.02, abs=1e-9)
    b.on_cmd_vel(0.0, 0.0, 0.0, 0.0, 0.0)  # space
    b.tick(1.0)
    assert b.rear_z() == pytest.approx(rz_after, abs=1e-9)
```

- [ ] **Step 3: Run the test file — expect failures**

```bash
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws
pytest src/dog_robot_kinematic_viz/test/test_body_commander.py -v
```

Expected: every existing test fails with `TypeError: on_cmd_vel() takes 5 positional arguments but 6 were given` (or similar, depending on whether Step 1 was applied before the impl); new tests fail with `AttributeError: 'BodyCommander' object has no attribute 'rear_z'`.

- [ ] **Step 4: Update `body_commander.py` to the new signature and state**

Replace the entire body of `dog_robot_kinematic_viz/body_commander.py` with:

```python
"""Body-level command state: cmd_vel + gait phase clock + body-height + rear-Z.

Plain Python (no ROS). The ROS node feeds Twist values to `on_cmd_vel` and
ticks `tick(dt)` on its timer; LegDriver pulls `body_vel_xy()`,
`phase(leg_name)`, `body_z()`, and `rear_z()` each tick. Trot phase pattern:
FL/BR together, FR/BL together 180 deg out of phase.

`body_z` is integrated from `linear.z` (velocity, m/s) and clamped to
[body_z_min, body_z_max]; `rear_z` is integrated from `angular.y` (velocity,
m/s) and clamped to [rear_z_min, rear_z_max]. This class is the single
source of truth for both clamps.
"""
from __future__ import annotations
from typing import Tuple


class BodyCommander:
    # Trot diagonals: FL & BR move together; FR & BL are pi out of phase.
    PHASE_OFFSETS = {"FL": 0.0, "BR": 0.0, "FR": 0.5, "BL": 0.5}

    def __init__(self, step_freq: float = 1.5,
                 body_z_min: float = -0.03,
                 body_z_max: float = +0.03,
                 rear_z_min: float = -0.05,
                 rear_z_max: float = +0.05):
        self.step_freq = float(step_freq)
        self.body_z_min = float(body_z_min)
        self.body_z_max = float(body_z_max)
        self.rear_z_min = float(rear_z_min)
        self.rear_z_max = float(rear_z_max)
        self._t = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._vz = 0.0
        self._wy = 0.0
        self._wz = 0.0
        self._z = 0.0
        self._rear_z = 0.0

    def on_cmd_vel(self, linear_x: float, linear_y: float,
                   linear_z: float, angular_y: float,
                   angular_z: float) -> None:
        self._vx = float(linear_x)
        self._vy = float(linear_y)
        self._vz = float(linear_z)
        self._wy = float(angular_y)
        self._wz = float(angular_z)

    def tick(self, dt: float) -> None:
        dt = float(dt)
        self._t += dt
        new_z = self._z + self._vz * dt
        if new_z > self.body_z_max:
            new_z = self.body_z_max
        elif new_z < self.body_z_min:
            new_z = self.body_z_min
        self._z = new_z
        new_rz = self._rear_z + self._wy * dt
        if new_rz > self.rear_z_max:
            new_rz = self.rear_z_max
        elif new_rz < self.rear_z_min:
            new_rz = self.rear_z_min
        self._rear_z = new_rz

    def phase(self, leg_name: str) -> float:
        offset = self.PHASE_OFFSETS[leg_name]
        return (self._t * self.step_freq + offset) % 1.0

    def body_vel_xy(self) -> Tuple[float, float]:
        return (self._vx, self._vy)

    def body_yaw_rate(self) -> float:
        return self._wz

    def body_z(self) -> float:
        return self._z

    def rear_z(self) -> float:
        return self._rear_z

    def time(self) -> float:
        return self._t
```

- [ ] **Step 5: Update `kinematic_node._on_cmd_vel` call site (5-arg)**

In `dog_robot_kinematic_viz/kinematic_node.py`, replace this block:

```python
    def _on_cmd_vel(self, msg: Twist) -> None:
        self.commander.on_cmd_vel(
            msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
```

with:

```python
    def _on_cmd_vel(self, msg: Twist) -> None:
        self.commander.on_cmd_vel(
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.y, msg.angular.z)
```

- [ ] **Step 6: Run BodyCommander tests — expect green**

```bash
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws
pytest src/dog_robot_kinematic_viz/test/test_body_commander.py -v
```

Expected: all tests pass.

- [ ] **Step 7: Run the full kinematic_viz test suite as a regression check**

```bash
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws
pytest src/dog_robot_kinematic_viz/test -v
```

Expected: existing tests still pass (kinematic_node smoke still green because we updated its call site).

- [ ] **Step 8: Commit**

```bash
cd /home/nguyenvd/workspace/dog_robot_height
git add dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/body_commander.py \
        dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/kinematic_node.py \
        dog_robot_ws/src/dog_robot_kinematic_viz/test/test_body_commander.py
git commit -m "$(cat <<'EOF'
feat(body_commander): rear_z state integrated from angular.y, clamped

on_cmd_vel grows to 5 args (insert angular_y before angular_z); rear_z
integrates _wy * dt and clamps to [rear_z_min, rear_z_max] (default
±0.05 m, wider than body_z because sit needs more travel). Tests cover
clamp + crosstalk guards (vz does not affect rear_z, wy does not affect
body_z). kinematic_node._on_cmd_vel updated in lockstep so the smoke
test stays green.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: foot_target — `extra_z` arg

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/foot_target.py`
- Modify: `dog_robot_ws/src/dog_robot_kinematic_viz/test/test_foot_target.py`
- Modify: `dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/leg_driver.py` (call-site only — pass `extra_z=0.0`)

**Why bundled:** Changing the `foot_target_in_hip` signature breaks the LegDriver call site. We pass `extra_z=0.0` from `LegDriver` for now; Task 3 wires it to `rear_z`.

- [ ] **Step 1: Update the `_ft` helper in `test_foot_target.py`**

Replace the existing helper:

```python
def _ft(rest, phi, v_body, body_z=0.0, R=EYE, params=PARAMS):
    """Test helper: call foot_target_in_hip with sensible defaults."""
    return foot_target_in_hip(rest, phi, v_body, body_z, R, params)
```

with:

```python
def _ft(rest, phi, v_body, body_z=0.0, extra_z=0.0, R=EYE, params=PARAMS):
    """Test helper: call foot_target_in_hip with sensible defaults."""
    return foot_target_in_hip(rest, phi, v_body, body_z, extra_z, R, params)
```

- [ ] **Step 2: Add failing tests for `extra_z`**

Append at the end of `test/test_foot_target.py`:

```python
# --- extra_z tests ---

def test_extra_z_zero_matches_baseline():
    # Regression: extra_z=0 (new arg) preserves existing behavior at every phase.
    for phi in (0.0, 0.25, PHI_APEX, 0.75):
        p_default = _ft(REST, phi, (0.10, 0.0))
        p_explicit = _ft(REST, phi, (0.10, 0.0), extra_z=0.0)
        np.testing.assert_allclose(
            p_default, p_explicit, atol=1e-12,
            err_msg=f"phi={phi}: extra_z default != extra_z=0.0")


def test_extra_z_lifts_foot_in_body_z():
    # extra_z=+0.05 should LIFT the foot by +0.05 in body Z (R=I so body=hip).
    p = _ft(REST, 0.0, (0.0, 0.0), extra_z=0.05)
    np.testing.assert_allclose(p[2], REST[2] + 0.05, atol=1e-12)
    np.testing.assert_allclose(p[:2], REST[:2], atol=1e-12)


def test_extra_z_composes_with_body_z():
    # body_z=+0.02 drops foot -0.02, extra_z=+0.05 lifts +0.05 -> net +0.03.
    p = _ft(REST, 0.0, (0.0, 0.0), body_z=0.02, extra_z=0.05)
    np.testing.assert_allclose(p[2], REST[2] + 0.03, atol=1e-12)


def test_extra_z_composes_with_swing_lift():
    # At swing apex with full velocity: foot z = rest + swing_height + extra_z.
    p = _ft(REST, PHI_APEX, (0.10, 0.0), extra_z=0.04)
    np.testing.assert_allclose(
        p[2], REST[2] + PARAMS.swing_height + 0.04, atol=1e-12)
```

- [ ] **Step 3: Run the test file — expect failures**

```bash
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws
pytest src/dog_robot_kinematic_viz/test/test_foot_target.py -v
```

Expected: existing tests fail with `TypeError: foot_target_in_hip() got an unexpected keyword argument 'extra_z'` (via the updated `_ft` helper), and the new tests fail with the same error.

- [ ] **Step 4: Update `foot_target.py`**

Replace the entire body of `dog_robot_kinematic_viz/foot_target.py` with:

```python
"""Per-leg foot trajectory, computed in body frame and rotated to hip frame.

The gait stride and lift are physically defined in BODY frame:
- Stride: horizontal displacement in body +X/+Y plane, proportional to body velocity.
- Lift: vertical displacement in body +Z, only during swing phase.
- body_z translation: the body raises in body +Z; feet drop -body_z in body to compensate.
- extra_z translation: foot-frame-agnostic body-Z lift, added on top. Used by
  rear legs to fold toward the body for the sit pose. Sign is opposite to body_z
  because the two scalars describe different things — see the spec's
  Sign Convention subsection.

After computing the full body-frame displacement, rotate into the leg's hip
frame (using R_base_to_hip.T) and add to rest_in_hip for ik_leg.

Phase convention (phi in [0, 1)):
    0  ..  stance_phase_ratio   stance: foot drags backwards along stride
    stance_phase_ratio .. 1     swing : foot returns + lifts (sin arch),
                                lift scales linearly with |v_body_xy| up to
                                swing_activation_speed (then saturates).
Continuity is C0 across the stance/swing seam and the cycle wrap.
"""
from __future__ import annotations
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class FootTargetParams:
    stride_per_mps: float = 0.20          # stride magnitude per m/s of body vel
    swing_height: float = 0.03            # peak lift above stance plane (m)
    stance_phase_ratio: float = 0.5       # fraction of cycle spent in stance
    swing_activation_speed: float = 0.05  # m/s; |v_body| above which lift is full


def foot_target_in_hip(rest_in_hip: np.ndarray,
                       phase: float,
                       v_body_xy: tuple[float, float],
                       body_z: float,
                       extra_z: float,
                       R_base_to_hip: np.ndarray,
                       params: FootTargetParams) -> np.ndarray:
    """Return foot target in hip frame.

    rest_in_hip: fk_leg(p, (0,0,0)) for this leg, in hip frame.
    phase: in [0, 1). Wraps automatically.
    v_body_xy: body-frame XY velocity (m/s). Forward = (+vx, 0).
    body_z: body-frame Z translation (m), clamped upstream in BodyCommander.
            Subtracted from foot Z (body rising drops the foot in body frame).
    extra_z: additional body-Z foot-lift (m). Added on top of -body_z. Callers
            pass rear_z (BL/BR) or 0.0 (FL/FR).
    R_base_to_hip: hip->body rotation matrix for this leg (3x3, orthonormal).
    params: gait shape.
    """
    phi = float(phase) % 1.0
    r = params.stance_phase_ratio

    vx_body = float(v_body_xy[0])
    vy_body = float(v_body_xy[1])
    sx_body = params.stride_per_mps * vx_body
    sy_body = params.stride_per_mps * vy_body

    if phi < r:
        u = phi / r
        scale = 0.5 - u
        z_lift_body = 0.0
    else:
        u = (phi - r) / (1.0 - r)
        scale = -0.5 + u
        v_mag = float(np.hypot(vx_body, vy_body))
        s = params.swing_activation_speed
        swing_scale = 1.0 if s <= 0.0 else min(1.0, v_mag / s)
        z_lift_body = params.swing_height * np.sin(np.pi * u) * swing_scale

    # Full body-frame displacement from rest:
    #   stride (XY) + swing lift (Z) + body_z compensation - extra_z lift.
    disp_body = np.array([
        sx_body * scale,
        sy_body * scale,
        z_lift_body - float(body_z) + float(extra_z),
    ])

    # Rotate body-frame displacement into hip frame and add to rest.
    return rest_in_hip + R_base_to_hip.T @ disp_body
```

- [ ] **Step 5: Update `leg_driver.py` to pass `extra_z=0.0` (placeholder)**

In `dog_robot_kinematic_viz/leg_driver.py`, find this block:

```python
        target = foot_target_in_hip(
            self.rest_in_hip,
            phase,
            body_v_xy,
            body_z,
            self.geom.R_base_to_hip,
            self.ft,
        )
```

and replace with:

```python
        target = foot_target_in_hip(
            self.rest_in_hip,
            phase,
            body_v_xy,
            body_z,
            0.0,  # extra_z: wired to rear_z in Task 3
            self.geom.R_base_to_hip,
            self.ft,
        )
```

- [ ] **Step 6: Run the test file — expect green**

```bash
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws
pytest src/dog_robot_kinematic_viz/test/test_foot_target.py -v
```

Expected: all foot_target tests pass.

- [ ] **Step 7: Regression — full kinematic_viz test suite**

```bash
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws
pytest src/dog_robot_kinematic_viz/test -v
```

Expected: leg_driver / kinematic_node / teleop tests stay green because LegDriver still passes `extra_z=0.0`.

- [ ] **Step 8: Commit**

```bash
cd /home/nguyenvd/workspace/dog_robot_height
git add dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/foot_target.py \
        dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/leg_driver.py \
        dog_robot_ws/src/dog_robot_kinematic_viz/test/test_foot_target.py
git commit -m "$(cat <<'EOF'
feat(foot_target): add extra_z body-Z foot-lift arg (rear-fold input)

foot_target_in_hip gains a 5th scalar arg `extra_z`. disp_body.z becomes
`z_lift - body_z + extra_z`. extra_z is leg-frame-agnostic — callers
(LegDriver) decide whether to pass rear_z or 0. Sign is opposite of
body_z because they describe different things: body_z is body lift
(positive raises body, drops feet in body frame); extra_z is foot lift
(positive raises foot in body frame). LegDriver passes 0.0 placeholder
until Task 3 wires it to rear_z.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: LegDriver — `is_rear` + `rear_z` + WARN-once

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/leg_driver.py`
- Modify: `dog_robot_ws/src/dog_robot_kinematic_viz/test/test_leg_driver.py`

- [ ] **Step 1: Update `_make_drivers` helper in `test_leg_driver.py`**

Replace the existing helper:

```python
def _make_drivers():
    geoms = load_leg_geoms(URDF_JOINTS_YAML)
    return {
        name: LegDriver(geoms[name],
                        load_link_params(LINK_PARAMS_YAML, name),
                        PARAMS)
        for name in LEG_NAMES
    }
```

with:

```python
def _make_drivers(logger=None):
    geoms = load_leg_geoms(URDF_JOINTS_YAML)
    return {
        name: LegDriver(geoms[name],
                        load_link_params(LINK_PARAMS_YAML, name),
                        PARAMS,
                        is_rear=(name in ("BL", "BR")),
                        logger=logger)
        for name in LEG_NAMES
    }
```

- [ ] **Step 2: Add failing tests for `rear_z` + WARN-once**

Append at the end of `test/test_leg_driver.py`:

```python
# --- rear_z routing (is_rear flag) ---

@pytest.mark.parametrize("name", ["FL", "FR"])
def test_front_legs_ignore_rear_z(name):
    drivers = _make_drivers()
    d = drivers[name]
    q_no_rear = d.step((0.0, 0.0), 0.25, body_z=0.0, rear_z=0.0)
    d._last_joints = (0.0, 0.0, 0.0)
    q_with_rear = d.step((0.0, 0.0), 0.25, body_z=0.0, rear_z=0.05)
    np.testing.assert_allclose(
        q_no_rear, q_with_rear, atol=1e-12,
        err_msg=f"{name}: front leg responded to rear_z")


@pytest.mark.parametrize("name", ["BL", "BR"])
def test_rear_legs_respond_to_rear_z(name):
    drivers = _make_drivers()
    d = drivers[name]
    q_no_rear = d.step((0.0, 0.0), 0.0, body_z=0.0, rear_z=0.0)
    d._last_joints = (0.0, 0.0, 0.0)
    q_with_rear = d.step((0.0, 0.0), 0.0, body_z=0.0, rear_z=0.05)
    diff = max(abs(a - b) for a, b in zip(q_no_rear, q_with_rear))
    assert diff > 1e-3, \
        f"{name}: joints unchanged with rear_z=+0.05 (diff={diff})"


@pytest.mark.parametrize("name", ["BL", "BR"])
def test_rear_z_lifts_foot_in_body_z(name):
    drivers = _make_drivers()
    d = drivers[name]
    rz = 0.05
    q = d.step((0.0, 0.0), 0.0, body_z=0.0, rear_z=rz)
    foot_hip = fk_leg(d.link, q)
    foot_body = d.geom.R_base_to_hip @ foot_hip
    rest_body = d.geom.R_base_to_hip @ d.rest_in_hip
    expected_body = rest_body + np.array([0.0, 0.0, +rz])
    np.testing.assert_allclose(
        foot_body, expected_body, atol=1e-6,
        err_msg=f"{name}: foot_body={foot_body} expected={expected_body}")


def test_step_rear_z_default_matches_zero_explicit():
    drivers = _make_drivers()
    for name, d in drivers.items():
        d._last_joints = (0.0, 0.0, 0.0)
        q_default = d.step((0.05, 0.0), 0.25)
        d._last_joints = (0.0, 0.0, 0.0)
        q_explicit = d.step((0.05, 0.0), 0.25, rear_z=0.0)
        np.testing.assert_allclose(
            q_default, q_explicit, atol=1e-12,
            err_msg=f"{name}: rear_z default != explicit 0.0")


# --- WARN-once on IK saturation ---

class _CountingLogger:
    def __init__(self):
        self.warnings = []

    def warning(self, msg):
        self.warnings.append(msg)


def test_warn_logged_once_on_repeated_ik_failure():
    geoms = load_leg_geoms(URDF_JOINTS_YAML)
    log = _CountingLogger()
    d = LegDriver(geoms["FL"],
                  load_link_params(LINK_PARAMS_YAML, "FL"),
                  PARAMS,
                  is_rear=False,
                  logger=log)
    d.step((0.0, 0.0), 0.0)               # warm-up, success
    d.rest_in_hip = np.array([0.0, 0.0, -0.13])  # on yaw axis -> IK raises
    d.step((0.0, 0.0), 0.0)               # WARN #1
    d.step((0.0, 0.0), 0.0)               # already saturated, no WARN
    d.step((0.0, 0.0), 0.0)               # still saturated, no WARN
    assert len(log.warnings) == 1


def test_warn_resets_after_recovery_then_fires_again():
    geoms = load_leg_geoms(URDF_JOINTS_YAML)
    log = _CountingLogger()
    d = LegDriver(geoms["FL"],
                  load_link_params(LINK_PARAMS_YAML, "FL"),
                  PARAMS,
                  is_rear=False,
                  logger=log)
    rest_good = d.rest_in_hip.copy()
    d.step((0.0, 0.0), 0.0)               # warm-up, success
    d.rest_in_hip = np.array([0.0, 0.0, -0.13])
    d.step((0.0, 0.0), 0.0)               # WARN #1
    assert len(log.warnings) == 1
    d.rest_in_hip = rest_good
    d.step((0.0, 0.0), 0.0)               # success -> clear flag
    d.rest_in_hip = np.array([0.0, 0.0, -0.13])
    d.step((0.0, 0.0), 0.0)               # WARN #2
    assert len(log.warnings) == 2
```

- [ ] **Step 3: Run the test file — expect failures**

```bash
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws
pytest src/dog_robot_kinematic_viz/test/test_leg_driver.py -v
```

Expected: existing tests pass `_make_drivers()` already (`is_rear` and `logger` are kwargs with defaults from our pending impl); the new tests fail with `TypeError: __init__() got an unexpected keyword argument 'is_rear'`.

(If `_make_drivers()` itself errors before the new tests get a chance, that confirms the missing ctor args. Either is acceptable for the red phase.)

- [ ] **Step 4: Update `leg_driver.py`**

Replace the entire body of `dog_robot_kinematic_viz/leg_driver.py` with:

```python
"""LegDriver: per-leg foot trajectory + closed-form IK.

This is the "1 chân -> kế thừa các chân" unit: one class instantiated 4x,
once per leg. The 4 instances differ only in `geom` (per-leg base->hip),
`link_params` (per-leg LinkParams), and `is_rear` (True for BL/BR — they
respond to rear_z; FL/FR ignore it).

The foot oscillates around the leg's CAD rest pose `fk_leg(link, (0,0,0))`
so joint angles stay near zero (well inside limits) and the IK never
hits the hip-axis singularity that ik_leg raises on.

Architecture: foot_target_in_hip receives body-frame velocity and rotates
it into the hip frame internally. LegDriver is a thin wrapper: it passes
body velocity + R_base_to_hip directly to foot_target_in_hip and decides
whether to forward `rear_z` (rear legs) or 0.0 (front legs) as `extra_z`.

On IK failure (foot target unreachable, e.g. combined body_z + rear_z past
leg reach) LegDriver returns the last good joints and logs a WARN exactly
once per saturation event (cleared on next success).
"""
from __future__ import annotations
from typing import Optional, Tuple

import numpy as np

from dog_robot_kinematics.kinematics_link import LinkParams, fk_leg, ik_leg

from dog_robot_kinematic_viz.leg_geometry import LegGeom
from dog_robot_kinematic_viz.foot_target import (
    FootTargetParams, foot_target_in_hip,
)


class LegDriver:
    def __init__(self,
                 geom: LegGeom,
                 link_params: LinkParams,
                 ft_params: FootTargetParams,
                 is_rear: bool = False,
                 logger=None):
        self.geom = geom
        self.link = link_params
        self.ft = ft_params
        self.is_rear = bool(is_rear)
        self._logger = logger
        self.rest_in_hip: np.ndarray = fk_leg(link_params, (0.0, 0.0, 0.0))
        self._last_joints: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._saturated = False

    def step(self, body_v_xy: Tuple[float, float],
             phase: float,
             body_z: float = 0.0,
             rear_z: float = 0.0) -> Tuple[float, float, float]:
        extra_z = float(rear_z) if self.is_rear else 0.0
        target = foot_target_in_hip(
            self.rest_in_hip,
            phase,
            body_v_xy,
            body_z,
            extra_z,
            self.geom.R_base_to_hip,
            self.ft,
        )
        try:
            q = ik_leg(self.link, target, knee_branch=+1)
        except ValueError:
            if not self._saturated:
                self._saturated = True
                msg = "LegDriver IK saturated; holding last joints"
                if self._logger is not None:
                    self._logger.warning(msg)
                else:
                    print(f"WARNING: {msg}")
            return self._last_joints
        self._saturated = False
        self._last_joints = q
        return q

    @property
    def last_joints(self) -> Tuple[float, float, float]:
        return self._last_joints
```

- [ ] **Step 5: Run the test file — expect green**

```bash
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws
pytest src/dog_robot_kinematic_viz/test/test_leg_driver.py -v
```

Expected: all tests pass, including the new `rear_z` and WARN-once tests.

- [ ] **Step 6: Regression — full kinematic_viz test suite**

```bash
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws
pytest src/dog_robot_kinematic_viz/test -v
```

Expected: kinematic_node smoke still green; teleop tests still green.

- [ ] **Step 7: Commit**

```bash
cd /home/nguyenvd/workspace/dog_robot_height
git add dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/leg_driver.py \
        dog_robot_ws/src/dog_robot_kinematic_viz/test/test_leg_driver.py
git commit -m "$(cat <<'EOF'
feat(leg_driver): is_rear ctor + rear_z step kwarg + WARN-once on IK fail

LegDriver now decides whether to forward rear_z as extra_z based on a
per-instance is_rear flag (True for BL/BR). Front legs always pass 0.0
and are bit-for-bit unchanged. On IK failure the driver returns last
good joints and logs WARN exactly once per saturation event; the flag
clears on the next successful ik_leg call so a repeat failure logs
again. logger kwarg defaults to None (falls back to print) so unit
tests can use a counting fake.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: kinematic_node — params + LegDriver wiring + rear_z to step

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/kinematic_node.py`
- Modify: `dog_robot_ws/src/dog_robot_kinematic_viz/test/test_kinematic_node_smoke.py`

- [ ] **Step 1: Add failing smoke tests**

Append at the end of `test/test_kinematic_node_smoke.py`:

```python
def test_rear_z_range_params_passed_to_commander(rclpy_ctx):
    node = KinematicNode(parameter_overrides=_overrides(
        rear_z_min=-0.10, rear_z_max=+0.10))
    assert node.commander.rear_z_min == pytest.approx(-0.10)
    assert node.commander.rear_z_max == pytest.approx(+0.10)
    node.destroy_node()


def test_angular_y_drives_only_rear_legs(rclpy_ctx):
    # step_freq=0.0 freezes the gait clock so only rear_z can move joints.
    node = KinematicNode(parameter_overrides=_overrides(step_freq=0.0))

    listener = rclpy.create_node("rear_z_listener")
    received: list[JointState] = []
    listener.create_subscription(
        JointState, "/joint_states", lambda m: received.append(m), 10)

    publisher = rclpy.create_node("rear_z_publisher")
    pub = publisher.create_publisher(Twist, "/cmd_vel", 10)

    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(listener)
    ex.add_node(publisher)

    # Warm-up baseline.
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.4:
        ex.spin_once(timeout_sec=0.02)
    assert received
    snapshot_pre = list(received[-1].position)

    # Drive angular.y = +0.04 m/s for ~0.6 s -> rear_z ~ +0.024
    # (under the default +0.05 clamp).
    twist = Twist()
    twist.angular.y = 0.04
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.6:
        pub.publish(twist)
        ex.spin_once(timeout_sec=0.02)
    snapshot_post = list(received[-1].position)

    # Joint layout (12 floats): FL[0..3) FR[3..6) BL[6..9) BR[9..12).
    # Front legs must NOT move.
    for i in range(6):
        assert snapshot_post[i] == pytest.approx(snapshot_pre[i], abs=1e-6), (
            f"front joint {i} drifted: "
            f"{snapshot_pre[i]} -> {snapshot_post[i]}")
    # Rear legs MUST move.
    rear_delta = max(
        abs(snapshot_post[i] - snapshot_pre[i]) for i in range(6, 12))
    assert rear_delta > 1e-3, (
        f"rear joints did not respond to angular.y (max delta={rear_delta})")

    node.destroy_node()
    listener.destroy_node()
    publisher.destroy_node()


def test_only_bl_br_drivers_are_rear(rclpy_ctx):
    node = KinematicNode(parameter_overrides=_overrides())
    assert node.drivers["FL"].is_rear is False
    assert node.drivers["FR"].is_rear is False
    assert node.drivers["BL"].is_rear is True
    assert node.drivers["BR"].is_rear is True
    node.destroy_node()
```

- [ ] **Step 2: Run smoke tests — expect failures**

```bash
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws
pytest src/dog_robot_kinematic_viz/test/test_kinematic_node_smoke.py -v
```

Expected: new tests fail. `test_rear_z_range_params_passed_to_commander` fails because the node does not declare `rear_z_min/max`. `test_angular_y_drives_only_rear_legs` fails because the rear joints don't move (the node still passes `extra_z=0.0` to LegDriver). `test_only_bl_br_drivers_are_rear` fails because LegDriver construction omits `is_rear`.

- [ ] **Step 3: Update `kinematic_node.py`**

Make three localised edits.

(a) Declare the new params. After the line:

```python
        self.declare_parameter("body_z_max", +0.03)
```

insert:

```python
        self.declare_parameter("rear_z_min", -0.05)
        self.declare_parameter("rear_z_max", +0.05)
```

(b) Forward the new params to `BodyCommander`. Replace this block:

```python
        self.commander = BodyCommander(
            step_freq=float(self.get_parameter("step_freq").value),
            body_z_min=float(self.get_parameter("body_z_min").value),
            body_z_max=float(self.get_parameter("body_z_max").value))
```

with:

```python
        self.commander = BodyCommander(
            step_freq=float(self.get_parameter("step_freq").value),
            body_z_min=float(self.get_parameter("body_z_min").value),
            body_z_max=float(self.get_parameter("body_z_max").value),
            rear_z_min=float(self.get_parameter("rear_z_min").value),
            rear_z_max=float(self.get_parameter("rear_z_max").value))
```

(c) Wire `is_rear` and `logger` at LegDriver construction. Replace:

```python
        self.drivers: Dict[str, LegDriver] = {
            name: LegDriver(geoms[name],
                            load_link_params(link_yaml, name),
                            ft_params)
            for name in active
        }
```

with:

```python
        self.drivers: Dict[str, LegDriver] = {
            name: LegDriver(geoms[name],
                            load_link_params(link_yaml, name),
                            ft_params,
                            is_rear=(name in ("BL", "BR")),
                            logger=self.get_logger())
            for name in active
        }
```

(d) Pull `rear_z` and pass to each `step()`. Replace this block in `_tick`:

```python
        v_xy = self.commander.body_vel_xy()
        bz = self.commander.body_z()
        positions: List[float] = []
        for leg in LEG_NAMES:
            if leg in self.drivers:
                q = self.drivers[leg].step(
                    v_xy, self.commander.phase(leg), bz)
            else:
                q = self._idle
            positions.extend(float(x) for x in q)
```

with:

```python
        v_xy = self.commander.body_vel_xy()
        bz = self.commander.body_z()
        rz = self.commander.rear_z()
        positions: List[float] = []
        for leg in LEG_NAMES:
            if leg in self.drivers:
                q = self.drivers[leg].step(
                    v_xy, self.commander.phase(leg), bz, rz)
            else:
                q = self._idle
            positions.extend(float(x) for x in q)
```

- [ ] **Step 4: Run smoke tests — expect green**

```bash
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws
pytest src/dog_robot_kinematic_viz/test/test_kinematic_node_smoke.py -v
```

Expected: all tests pass, including the three new ones.

- [ ] **Step 5: Commit**

```bash
cd /home/nguyenvd/workspace/dog_robot_height
git add dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/kinematic_node.py \
        dog_robot_ws/src/dog_robot_kinematic_viz/test/test_kinematic_node_smoke.py
git commit -m "$(cat <<'EOF'
feat(kinematic_node): declare rear_z params, set is_rear, pass rear_z

Declare rear_z_min/rear_z_max (default ±0.05 m); forward them to
BodyCommander. Construct LegDriver with is_rear=(name in {BL,BR}) and
logger=node logger so saturation warnings appear in /rosout. _tick now
pulls commander.rear_z() and passes it to driver.step. Smoke test
verifies that publishing cmd_vel.angular.y > 0 moves BL/BR joints while
FL/FR positions stay byte-identical at step_freq=0.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: teleop_keyboard — `i/k` keys + 5-axis space-zero + `angular.y` publish

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/teleop_keyboard.py`
- Modify: `dog_robot_ws/src/dog_robot_kinematic_viz/test/test_teleop_keyboard.py`

- [ ] **Step 1: Replace the 4-axis space-zero test with 5-axis; add `i/k` tests + `angular.y` publish test**

In `test/test_teleop_keyboard.py`:

(a) Replace the existing `test_space_zeros_all_four_axes` function with:

```python
def test_space_zeros_all_five_axes(rclpy_ctx):
    node = TeleopKeyboard()
    node.on_key("w")   # vx > 0
    node.on_key("a")   # vy > 0
    node.on_key("r")   # vz > 0
    node.on_key("i")   # wy > 0   (NEW)
    node.on_key("j")   # wz > 0
    assert node._vx != 0.0
    assert node._vy != 0.0
    assert node._vz != 0.0
    assert node._wy != 0.0
    assert node._wz != 0.0
    node.on_key(" ")
    assert node._vx == 0.0
    assert node._vy == 0.0
    assert node._vz == 0.0
    assert node._wy == 0.0
    assert node._wz == 0.0
    node.destroy_node()
```

(b) Append at the end of the file:

```python
# --- i/k -> angular.y (rear-height velocity) ---

def test_i_key_increments_wy(rclpy_ctx):
    node = TeleopKeyboard()
    assert node._wy == 0.0
    assert node.on_key("i") is True
    assert node._wy == pytest.approx(LIN_STEP)
    node.destroy_node()


def test_k_key_decrements_wy(rclpy_ctx):
    node = TeleopKeyboard()
    assert node.on_key("k") is True
    assert node._wy == pytest.approx(-LIN_STEP)
    node.destroy_node()


def test_wy_clamps_to_lin_max(rclpy_ctx):
    node = TeleopKeyboard()
    for _ in range(int(LIN_MAX / LIN_STEP) + 5):
        node.on_key("i")
    assert node._wy == pytest.approx(LIN_MAX)
    node.destroy_node()


def test_wy_clamps_to_neg_lin_max(rclpy_ctx):
    node = TeleopKeyboard()
    for _ in range(int(LIN_MAX / LIN_STEP) + 5):
        node.on_key("k")
    assert node._wy == pytest.approx(-LIN_MAX)
    node.destroy_node()


def test_publish_emits_angular_y(rclpy_ctx):
    node = TeleopKeyboard()
    received: list[Twist] = []

    listener = rclpy.create_node("teleop_wy_listener")
    listener.create_subscription(
        Twist, "/cmd_vel", lambda m: received.append(m), 10)

    # Warm up DDS so the subscription is matched before publish.
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.1:
        rclpy.spin_once(node, timeout_sec=0.01)
        rclpy.spin_once(listener, timeout_sec=0.01)

    node.on_key("i")   # _wy = +LIN_STEP, also calls publish()

    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.3 and not received:
        rclpy.spin_once(listener, timeout_sec=0.02)
    assert received, "listener did not receive /cmd_vel"
    assert received[-1].angular.y == pytest.approx(LIN_STEP)

    listener.destroy_node()
    node.destroy_node()
```

- [ ] **Step 2: Run teleop tests — expect failures**

```bash
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws
pytest src/dog_robot_kinematic_viz/test/test_teleop_keyboard.py -v
```

Expected: 5-axis test fails with `AttributeError: 'TeleopKeyboard' object has no attribute '_wy'`; new `i/k` tests fail with same.

- [ ] **Step 3: Update `teleop_keyboard.py`**

Replace the entire body of `dog_robot_kinematic_viz/teleop_keyboard.py` with:

```python
"""Minimal WASD+JL+RF+IK teleop publisher for /cmd_vel.

Self-contained — no dog_robot_control / external teleop_twist_keyboard
dependency. Reads single keypresses from a TTY in raw mode; designed to be
launched inside a real terminal (e.g. prefix="gnome-terminal --" in the
launch file).

Keys:
    w / s  : linear.x  +/-   (forward / back)
    a / d  : linear.y  +/-   (strafe left / right)
    r / f  : linear.z  +/-   (body up / down — height velocity)
    i / k  : angular.y +/-   (rear up / down — rear-height velocity)
    j / l  : angular.z +/-   (yaw left / right)
    space  : zero all five axes
    q      : quit
"""
from __future__ import annotations
import sys
import termios
import tty
from select import select

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


LIN_STEP = 0.02     # m/s per keypress
ANG_STEP = 0.10     # rad/s per keypress
LIN_MAX = 0.20
ANG_MAX = 0.80


HELP = """
  Kinematic teleop — keys:
    w/s  forward / back     (linear.x)
    a/d  left / right       (linear.y)
    r/f  body up / down     (linear.z — height velocity)
    i/k  rear up / down     (angular.y — rear-height velocity)
    j/l  yaw left / right   (angular.z)
    space  zero all
    q      quit
"""


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class TeleopKeyboard(Node):

    def __init__(self):
        super().__init__("teleop_keyboard")
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._vx = 0.0
        self._vy = 0.0
        self._vz = 0.0
        self._wy = 0.0
        self._wz = 0.0

    def publish(self):
        msg = Twist()
        msg.linear.x = self._vx
        msg.linear.y = self._vy
        msg.linear.z = self._vz
        msg.angular.y = self._wy
        msg.angular.z = self._wz
        self._pub.publish(msg)

    def on_key(self, key: str) -> bool:
        """Returns True to keep running, False to quit."""
        if key == "w":
            self._vx = _clamp(self._vx + LIN_STEP, -LIN_MAX, LIN_MAX)
        elif key == "s":
            self._vx = _clamp(self._vx - LIN_STEP, -LIN_MAX, LIN_MAX)
        elif key == "a":
            self._vy = _clamp(self._vy + LIN_STEP, -LIN_MAX, LIN_MAX)
        elif key == "d":
            self._vy = _clamp(self._vy - LIN_STEP, -LIN_MAX, LIN_MAX)
        elif key == "r":
            self._vz = _clamp(self._vz + LIN_STEP, -LIN_MAX, LIN_MAX)
        elif key == "f":
            self._vz = _clamp(self._vz - LIN_STEP, -LIN_MAX, LIN_MAX)
        elif key == "i":
            self._wy = _clamp(self._wy + LIN_STEP, -LIN_MAX, LIN_MAX)
        elif key == "k":
            self._wy = _clamp(self._wy - LIN_STEP, -LIN_MAX, LIN_MAX)
        elif key == "j":
            self._wz = _clamp(self._wz + ANG_STEP, -ANG_MAX, ANG_MAX)
        elif key == "l":
            self._wz = _clamp(self._wz - ANG_STEP, -ANG_MAX, ANG_MAX)
        elif key == " ":
            self._vx = self._vy = self._vz = self._wy = self._wz = 0.0
        elif key in ("q", "\x03"):     # q or Ctrl-C
            return False
        else:
            return True
        self.publish()
        self.get_logger().info(
            f"cmd_vel: linear=({self._vx:+.2f},{self._vy:+.2f},{self._vz:+.2f})  "
            f"angular=({self._wy:+.2f},{self._wz:+.2f})")
        return True


def _read_key(timeout: float = 0.1) -> str:
    """Read one keystroke from stdin (raw mode). Returns '' on timeout."""
    r, _, _ = select([sys.stdin], [], [], timeout)
    if not r:
        return ""
    return sys.stdin.read(1)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    print(HELP)
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while rclpy.ok():
            key = _read_key()
            if key:
                if not node.on_key(key):
                    break
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run teleop tests — expect green**

```bash
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws
pytest src/dog_robot_kinematic_viz/test/test_teleop_keyboard.py -v
```

Expected: all teleop tests pass.

- [ ] **Step 5: Commit**

```bash
cd /home/nguyenvd/workspace/dog_robot_height
git add dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/teleop_keyboard.py \
        dog_robot_ws/src/dog_robot_kinematic_viz/test/test_teleop_keyboard.py
git commit -m "$(cat <<'EOF'
feat(teleop): i/k keys publish angular.y for rear-height velocity

i / k increment / decrement _wy by LIN_STEP (0.02 m/s), clamped to
±LIN_MAX (0.20 m/s). publish() sets msg.angular.y from _wy. Space-zero
now wipes 5 axes (vx, vy, vz, wy, wz). HELP text + status log line
include the new axis.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: config yaml + manual RViz verify

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_kinematic_viz/config/kinematic_params.yaml`

This task has no automated tests — the yaml defaults are already encoded in `BodyCommander.__init__`. Updating the yaml just makes the value visible / tunable from the launch side.

- [ ] **Step 1: Add `rear_z_min/max` to `kinematic_params.yaml`**

Replace the file `config/kinematic_params.yaml` with:

```yaml
kinematic_node:
  ros__parameters:
    publish_rate: 50.0
    active_legs: ["FL", "FR", "BL", "BR"]
    idle_joints: [0.0, 0.0, 0.0]
    step_freq: 1.5
    stride_per_mps: 0.20
    swing_height: 0.03
    stance_phase_ratio: 0.5
    rear_z_min: -0.05
    rear_z_max: +0.05
    # link_params_yaml + urdf_joints_yaml are injected by the launch file
    # via PathJoinSubstitution (absolute path required at node init time).
```

(`body_z_min/max` are intentionally left unspecified so they stay at the default ±0.03 — keep consistent with the existing yaml.)

- [ ] **Step 2: Full colcon build + test**

```bash
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws
pkill -f rviz2 2>/dev/null; pkill -f kinematic_node 2>/dev/null; sleep 1
colcon build --packages-select dog_robot_kinematic_viz
source install/setup.bash
colcon test --packages-select dog_robot_kinematic_viz --event-handlers console_direct+
colcon test-result --verbose --all
```

Expected: build succeeds; all tests pass.

- [ ] **Step 3: Manual RViz smoke**

```bash
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws
source install/setup.bash
ros2 launch dog_robot_kinematic_viz kinematic_teleop.launch.py
```

In the teleop terminal, verify:
1. Press `i` 5 times → BL and BR feet visibly lift toward the body in RViz; FL and FR unchanged.
2. Press `k` 5 times → BL and BR return to baseline; whole robot at nominal stand.
3. Press `r` / `f` → uniform whole-body raise/lower (existing behavior unaffected by this feature).
4. Press `i` 10 times until `_wy` saturates at `LIN_MAX` (0.20 m/s), wait until rear_z hits its `+0.05` clamp; observe `WARNING: LegDriver IK saturated; holding last joints` in the launch logs only if rear reach is exceeded (which depends on the leg geometry — saturation may or may not fire).
5. Press `space` → all axes zero, robot returns to stand.

If the rear feet do not visibly fold toward the body at `_wy = LIN_MAX`, increase the clamp via `--ros-args -p rear_z_max:=0.10` and re-run.

- [ ] **Step 4: Commit**

```bash
cd /home/nguyenvd/workspace/dog_robot_height
git add dog_robot_ws/src/dog_robot_kinematic_viz/config/kinematic_params.yaml
git commit -m "$(cat <<'EOF'
chore(config): expose rear_z_min/rear_z_max in kinematic_params.yaml

Makes the rear-Z clamp tunable from the launch side without touching
BodyCommander's defaults. Default ±0.05 m matches the in-code default.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Self-Review Notes

**Spec coverage matrix** (every spec section maps to a task):

| Spec section | Task(s) |
|---|---|
| Architecture | Tasks 2, 3, 4 (foot_target + LegDriver + kinematic_node wiring) |
| Sign convention (`extra_z` added, `body_z` subtracted) | Task 2 (formula change) + tests in Task 2 |
| `BodyCommander` (`rear_z` state, 5-arg `on_cmd_vel`, clamp) | Task 1 |
| `foot_target_in_hip` (`extra_z` arg) | Task 2 |
| `LegDriver` (`is_rear`, `rear_z`, WARN-once) | Task 3 |
| `kinematic_node` (params, forward `angular.y`, `is_rear`, pass `rear_z`) | Task 4 |
| `teleop_keyboard` (`i/k`, `_wy`, `angular.y`, 5-axis space-zero) | Task 5 |
| `config/kinematic_params.yaml` (rear_z_min/max) | Task 6 |
| Launch files (no changes) | n/a — confirmed in Task 6 (launch picks up the yaml) |
| Conflict policy (orthogonal + IK-freeze + WARN-once) | Task 1 (clamp), Task 3 (WARN-once), tests |
| Manual smoke | Task 6 step 3 |

**Type / signature consistency check:**
- `BodyCommander.on_cmd_vel(linear_x, linear_y, linear_z, angular_y, angular_z)` — used identically in Task 1 (impl), Task 1 (test updates), and Task 1 step 5 (kinematic_node call site).
- `foot_target_in_hip(rest_in_hip, phase, v_body_xy, body_z, extra_z, R_base_to_hip, params)` — used identically in Task 2 (impl), Task 2 (test `_ft` helper), and Task 2 step 5 (LegDriver call site).
- `LegDriver(geom, link_params, ft_params, is_rear=False, logger=None)` — used identically in Task 3 (impl), Task 3 (`_make_drivers`), and Task 4 step 3c (kinematic_node call site).
- `LegDriver.step(body_v_xy, phase, body_z=0.0, rear_z=0.0)` — used identically in Task 3 (impl), Task 3 (tests), and Task 4 step 3d (kinematic_node call site).

**Placeholder scan:** no TBD/TODO/"add appropriate error handling"/"similar to Task N" — every step shows the exact code or command to run.
