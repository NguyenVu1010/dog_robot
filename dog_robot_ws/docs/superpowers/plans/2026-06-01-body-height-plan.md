# Body-height control — implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let the operator raise/lower the dog robot's body in real time via `/cmd_vel.linear.z`, composable with the existing trot gait, on branch `feature/body-height` in worktree `/home/nguyenvd/workspace/dog_robot_height`.

**Architecture:** `BodyCommander` integrates `linear.z` (velocity, m/s) into a clamped `body_z` state; each tick `LegDriver.step` shifts the per-leg `rest_in_hip` by `R_base_to_hip.T @ [0,0,-body_z]` before computing the trajectory; teleop adds `r`/`f` keys; `foot_target.py` stays untouched. Defaults to current behavior when `body_z = 0`.

**Tech Stack:** ROS 2 Humble, `geometry_msgs/Twist`, `rclpy`, NumPy, pytest, colcon, ament_python.

**Spec:** `docs/superpowers/specs/2026-06-01-body-height-design.md` (commit `20ea8d8`).

---

## Working directory & branch

All work happens in worktree `/home/nguyenvd/workspace/dog_robot_height` on branch `feature/body-height`. Verify before starting:

```bash
cd /home/nguyenvd/workspace/dog_robot_height
git status            # expect: clean on feature/body-height
git log --oneline -1  # expect: 20ea8d8 docs(spec): body-height control via /cmd_vel linear.z
```

Run tests from the package directory so pytest picks up `test/` directly:

```bash
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws/src/dog_robot_kinematic_viz
python -m pytest test/ -v
```

(`pip install -e <abs-path>` for `dog_robot_kinematic_viz` should already be in place from the prior session — see `feedback_pip_editable_for_ament_python` memory. If pytest can't import the package, run `pip install -e /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws/src/dog_robot_kinematic_viz` once and retry.)

---

## File structure

Each file has one responsibility. None grows beyond what the spec requires.

**Files modified (4):**
- `dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/body_commander.py` — owns `body_z` state, integration, clamp; 4-arg `on_cmd_vel`.
- `dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/leg_driver.py` — `step(...)` gains `body_z=0.0` kwarg; per-call recompute of shifted rest. No mutation of `self.rest_in_hip`.
- `dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/kinematic_node.py` — declares `body_z_min`/`body_z_max` params, forwards `msg.linear.z` to commander, pulls `body_z()` and passes to each `driver.step`.
- `dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/teleop_keyboard.py` — adds `_vz`, `r`/`f` keys, zeroes vz on `space`, publishes `msg.linear.z`.

**Tests modified (3) / created (1):**
- `test/test_body_commander.py` — extend.
- `test/test_leg_driver.py` — extend.
- `test/test_kinematic_node_smoke.py` — extend.
- `test/test_teleop_keyboard.py` — create.

**Files untouched (per spec):**
- `foot_target.py` (stays a pure trajectory generator around a given `rest`).
- `leg_geometry.py` (`R_base_to_hip` already exposed via `LegGeom`).
- Launch files (`kinematic_teleop.launch.py` etc.) — no new launch args; the new ROS params have safe defaults.

---

## Task list

- **Task 1:** Extend `BodyCommander` with `body_z` state + 4-arg `on_cmd_vel`.
- **Task 2:** `LegDriver.step` accepts `body_z` and shifts the rest pose.
- **Task 3:** `kinematic_node` forwards `linear.z`, declares range params, passes `body_z` to drivers.
- **Task 4:** `teleop_keyboard` r/f keys + 4-axis space-zero + `linear.z` publish.
- **Task 5:** Full colcon build + colcon test + manual RViz verify.

---

### Task 1: Extend `BodyCommander` with body_z state + 4-arg `on_cmd_vel`

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/body_commander.py`
- Modify: `dog_robot_ws/src/dog_robot_kinematic_viz/test/test_body_commander.py`

- [ ] **Step 1: Update existing test and add new body_z tests**

In `dog_robot_ws/src/dog_robot_kinematic_viz/test/test_body_commander.py`:

(a) Update the existing `test_on_cmd_vel_updates_state` (currently passes 3 args) to use the 4-arg signature:

```python
def test_on_cmd_vel_updates_state():
    b = BodyCommander()
    b.on_cmd_vel(0.3, -0.1, 0.05, 0.5)
    assert b.body_vel_xy() == (0.3, -0.1)
    assert b.body_yaw_rate() == 0.5
```

(b) Append these new tests at the end of the file (after `test_unknown_leg_raises`):

```python
def test_default_body_z_is_zero():
    b = BodyCommander()
    assert b.body_z() == 0.0


def test_vz_integrates_into_body_z():
    b = BodyCommander()
    b.on_cmd_vel(0.0, 0.0, 0.02, 0.0)
    b.tick(0.1)
    assert b.body_z() == pytest.approx(0.002, abs=1e-9)
    b.tick(0.1)
    assert b.body_z() == pytest.approx(0.004, abs=1e-9)


def test_body_z_clamps_at_max():
    b = BodyCommander()  # default body_z_max = +0.04
    b.on_cmd_vel(0.0, 0.0, 0.10, 0.0)
    for _ in range(100):
        b.tick(0.01)   # total commanded delta = 0.10 * 1.0 = 0.10 m
    assert b.body_z() == pytest.approx(0.04, abs=1e-9)


def test_body_z_clamps_at_min():
    b = BodyCommander()  # default body_z_min = -0.04
    b.on_cmd_vel(0.0, 0.0, -0.10, 0.0)
    for _ in range(100):
        b.tick(0.01)
    assert b.body_z() == pytest.approx(-0.04, abs=1e-9)


def test_space_zeros_vz_halts_integration():
    b = BodyCommander()
    b.on_cmd_vel(0.0, 0.0, 0.02, 0.0)
    b.tick(0.5)
    z_after_drive = b.body_z()
    assert z_after_drive == pytest.approx(0.01, abs=1e-9)
    b.on_cmd_vel(0.0, 0.0, 0.0, 0.0)   # space
    b.tick(1.0)
    assert b.body_z() == pytest.approx(z_after_drive, abs=1e-9)


def test_body_z_min_max_params_respected():
    b = BodyCommander(body_z_min=-0.10, body_z_max=+0.10)
    b.on_cmd_vel(0.0, 0.0, 1.0, 0.0)
    for _ in range(50):
        b.tick(0.01)
    assert b.body_z() == pytest.approx(0.10, abs=1e-9)
```

- [ ] **Step 2: Run tests, confirm they fail in the expected ways**

```bash
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws/src/dog_robot_kinematic_viz
python -m pytest test/test_body_commander.py -v
```

Expected:
- `test_on_cmd_vel_updates_state` → FAIL (`TypeError: on_cmd_vel() takes 4 positional arguments but 5 were given`)
- `test_default_body_z_is_zero` → FAIL (`AttributeError: 'BodyCommander' object has no attribute 'body_z'`)
- `test_vz_integrates_into_body_z`, `test_body_z_clamps_at_max`, `test_body_z_clamps_at_min`, `test_space_zeros_vz_halts_integration` → FAIL (AttributeError on `body_z` or 4-arg `on_cmd_vel`)
- `test_body_z_min_max_params_respected` → FAIL (`TypeError: __init__() got an unexpected keyword argument 'body_z_min'`)
- All other tests (`test_default_state_is_zero`, `test_tick_accumulates_time`, `test_phase_*`, `test_trot_*`, `test_unknown_leg_raises`) → PASS unchanged

- [ ] **Step 3: Rewrite `body_commander.py` with the new state**

Replace the entire contents of `dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/body_commander.py` with:

```python
"""Body-level command state: cmd_vel + gait phase clock + body-height state.

Plain Python (no ROS). The ROS node feeds Twist values to `on_cmd_vel` and
ticks `tick(dt)` on its timer; LegDriver pulls `body_vel_xy()`,
`phase(leg_name)`, and `body_z()` each tick. Trot phase pattern: FL/BR
together, FR/BL together 180 deg out of phase.

`body_z` is integrated from `linear.z` (velocity, m/s) and clamped to
[body_z_min, body_z_max]; this class is the single source of truth for the
clamp, so downstream callers may assume the value they receive is in range.
"""
from __future__ import annotations
from typing import Tuple


class BodyCommander:
    # Trot diagonals: FL & BR move together; FR & BL are pi out of phase.
    PHASE_OFFSETS = {"FL": 0.0, "BR": 0.0, "FR": 0.5, "BL": 0.5}

    def __init__(self, step_freq: float = 1.5,
                 body_z_min: float = -0.04,
                 body_z_max: float = +0.04):
        self.step_freq = float(step_freq)
        self.body_z_min = float(body_z_min)
        self.body_z_max = float(body_z_max)
        self._t = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._vz = 0.0
        self._wz = 0.0
        self._z = 0.0

    def on_cmd_vel(self, linear_x: float, linear_y: float,
                   linear_z: float, angular_z: float) -> None:
        self._vx = float(linear_x)
        self._vy = float(linear_y)
        self._vz = float(linear_z)
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

    def phase(self, leg_name: str) -> float:
        offset = self.PHASE_OFFSETS[leg_name]
        return (self._t * self.step_freq + offset) % 1.0

    def body_vel_xy(self) -> Tuple[float, float]:
        return (self._vx, self._vy)

    def body_yaw_rate(self) -> float:
        # Reserved for future use; current LegDriver does not consume it.
        return self._wz

    def body_z(self) -> float:
        return self._z

    def time(self) -> float:
        return self._t
```

- [ ] **Step 4: Run tests, confirm all pass**

```bash
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws/src/dog_robot_kinematic_viz
python -m pytest test/test_body_commander.py -v
```

Expected: all 13 tests PASS (7 pre-existing + 6 new).

- [ ] **Step 5: Commit**

```bash
cd /home/nguyenvd/workspace/dog_robot_height
git add dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/body_commander.py \
        dog_robot_ws/src/dog_robot_kinematic_viz/test/test_body_commander.py
git commit -m "feat(body_commander): add body_z state, 4-arg on_cmd_vel, clamp range params"
```

---

### Task 2: `LegDriver.step` accepts body_z and shifts rest pose

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/leg_driver.py`
- Modify: `dog_robot_ws/src/dog_robot_kinematic_viz/test/test_leg_driver.py`

- [ ] **Step 1: Append new body_z tests**

At the end of `dog_robot_ws/src/dog_robot_kinematic_viz/test/test_leg_driver.py`, append:

```python
@pytest.mark.parametrize("name", LEG_NAMES)
def test_zero_velocity_stance_with_body_z_shifts_foot_in_body_z(name):
    # body_z > 0 means the body sits higher relative to the feet, so each
    # foot must be a distance `body_z` LOWER in body Z than at rest.
    drivers = _make_drivers()
    d = drivers[name]
    bz = 0.04
    q = d.step((0.0, 0.0), 0.0, body_z=bz)
    # FK in hip frame, rotate to body frame.
    foot_hip = fk_leg(d.link, q)
    foot_body = d.geom.R_base_to_hip @ foot_hip
    rest_body = d.geom.R_base_to_hip @ d.rest_in_hip
    expected_body = rest_body + np.array([0.0, 0.0, -bz])
    np.testing.assert_allclose(
        foot_body, expected_body, atol=1e-6,
        err_msg=f"{name}: foot_body={foot_body} expected={expected_body}")


@pytest.mark.parametrize("name", LEG_NAMES)
def test_zero_velocity_stance_with_negative_body_z_shifts_foot_up(name):
    drivers = _make_drivers()
    d = drivers[name]
    bz = -0.04
    q = d.step((0.0, 0.0), 0.0, body_z=bz)
    foot_hip = fk_leg(d.link, q)
    foot_body = d.geom.R_base_to_hip @ foot_hip
    rest_body = d.geom.R_base_to_hip @ d.rest_in_hip
    expected_body = rest_body + np.array([0.0, 0.0, -bz])  # = +0.04
    np.testing.assert_allclose(
        foot_body, expected_body, atol=1e-6,
        err_msg=f"{name}: foot_body={foot_body} expected={expected_body}")


@pytest.mark.parametrize("name", LEG_NAMES)
@pytest.mark.parametrize("bz", [+0.04, -0.04])
def test_body_z_extreme_keeps_joints_in_limits_full_cycle(name, bz):
    # Full forward-velocity cycle at the body_z clamp extremes must stay
    # within hardware joint limits.
    drivers = _make_drivers()
    d = drivers[name]
    for phi in np.linspace(0.0, 1.0, 30, endpoint=False):
        q = d.step((0.10, 0.0), float(phi), body_z=bz)
        _assert_within_limits(name, q)


def test_step_body_z_default_matches_zero_explicit_body_z():
    # Backward-compat: calling without body_z must equal body_z=0.0.
    drivers = _make_drivers()
    for name, d in drivers.items():
        # Reset internal _last_joints to avoid cross-call state leak.
        d._last_joints = (0.0, 0.0, 0.0)
        q_default = d.step((0.05, 0.0), 0.25)
        d._last_joints = (0.0, 0.0, 0.0)
        q_explicit = d.step((0.05, 0.0), 0.25, body_z=0.0)
        np.testing.assert_allclose(q_default, q_explicit, atol=1e-12)


@pytest.mark.parametrize("name", LEG_NAMES)
def test_rest_in_hip_not_mutated_by_body_z_step(name):
    # The shift must be per-call: self.rest_in_hip stays at the CAD value.
    drivers = _make_drivers()
    d = drivers[name]
    rest_before = d.rest_in_hip.copy()
    d.step((0.0, 0.0), 0.0, body_z=0.04)
    d.step((0.0, 0.0), 0.0, body_z=-0.04)
    np.testing.assert_array_equal(d.rest_in_hip, rest_before)
```

- [ ] **Step 2: Run tests, confirm new ones fail**

```bash
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws/src/dog_robot_kinematic_viz
python -m pytest test/test_leg_driver.py -v
```

Expected:
- `test_zero_velocity_stance_with_body_z_shifts_foot_in_body_z[*]` (4 cases) → FAIL (`TypeError: step() got an unexpected keyword argument 'body_z'`)
- `test_zero_velocity_stance_with_negative_body_z_shifts_foot_up[*]` (4) → FAIL (same)
- `test_body_z_extreme_keeps_joints_in_limits_full_cycle[*-*]` (8) → FAIL (same)
- `test_step_body_z_default_matches_zero_explicit_body_z` → FAIL (same)
- `test_rest_in_hip_not_mutated_by_body_z_step[*]` (4) → FAIL (same)
- All pre-existing tests → PASS unchanged

- [ ] **Step 3: Update `LegDriver.step`**

In `dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/leg_driver.py`, replace the `step` method (lines 36-56) with:

```python
    def step(self, body_v_xy: Tuple[float, float],
             phase: float,
             body_z: float = 0.0) -> Tuple[float, float, float]:
        # Rotate the body-frame XY velocity into this leg's hip frame.
        # R_base_to_hip maps a hip-frame vector to the body frame, so the
        # inverse (transpose, since R is orthonormal) takes body -> hip.
        v3 = np.array([float(body_v_xy[0]), float(body_v_xy[1]), 0.0])
        v_hip = self.geom.R_base_to_hip.T @ v3

        # body_z > 0 means body rises, so each foot must shift -body_z in
        # body Z. Rotate that body-frame shift into the hip frame and add
        # to the CAD rest pose for this call only (rest_in_hip itself is
        # never mutated — the clamp lives in BodyCommander).
        bz_shift_body = np.array([0.0, 0.0, -float(body_z)])
        bz_shift_hip = self.geom.R_base_to_hip.T @ bz_shift_body
        rest = self.rest_in_hip + bz_shift_hip

        target = foot_target_in_hip(
            rest, phase, (v_hip[0], v_hip[1]), self.ft)

        try:
            q = ik_leg(self.link, target, knee_branch=+1)
        except ValueError:
            # Unreachable / singular: hold last good joints rather than crash
            # the node. body_z clamp keeps us well inside the workspace under
            # normal operation.
            return self._last_joints

        self._last_joints = q
        return q
```

- [ ] **Step 4: Run all leg_driver tests, confirm pass**

```bash
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws/src/dog_robot_kinematic_viz
python -m pytest test/test_leg_driver.py -v
```

Expected: all tests PASS (pre-existing + 25 new parametrized cases). If `test_step_body_z_default_matches_zero_explicit_body_z` fails on a non-`FL` leg due to `_make_drivers()` rebuilding fresh drivers, double-check that the new test resets `_last_joints` before each call (the code in Step 1 already does this).

- [ ] **Step 5: Commit**

```bash
cd /home/nguyenvd/workspace/dog_robot_height
git add dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/leg_driver.py \
        dog_robot_ws/src/dog_robot_kinematic_viz/test/test_leg_driver.py
git commit -m "feat(leg_driver): step accepts body_z kwarg, shifts rest per call"
```

---

### Task 3: `kinematic_node` forwards linear.z, declares range params, passes body_z to drivers

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/kinematic_node.py`
- Modify: `dog_robot_ws/src/dog_robot_kinematic_viz/test/test_kinematic_node_smoke.py`

- [ ] **Step 1: Append new smoke test for linear.z**

At the end of `dog_robot_ws/src/dog_robot_kinematic_viz/test/test_kinematic_node_smoke.py`, append:

```python
def test_linear_z_drives_body_height_state(rclpy_ctx):
    # Publishing linear.z > 0 must move feet DOWN in body Z relative to the
    # zero-input baseline (because body_z > 0 means body rises, feet press
    # further down).  We assert the joint snapshot diverges from baseline.
    node = KinematicNode(parameter_overrides=_overrides())

    listener = rclpy.create_node("body_z_listener")
    received: list[JointState] = []
    listener.create_subscription(
        JointState, "/joint_states", lambda m: received.append(m), 10)

    publisher = rclpy.create_node("body_z_publisher")
    pub = publisher.create_publisher(Twist, "/cmd_vel", 10)

    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(listener)
    ex.add_node(publisher)

    # Baseline: spin at zero input, hold a stance-phase snapshot.
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.4:
        ex.spin_once(timeout_sec=0.02)
    assert received, "no /joint_states received during warm-up"
    snapshot_pre = list(received[-1].position)

    # Drive body up at 0.04 m/s for ~0.5 s -> body_z ~ +0.02 (well below the
    # +0.04 clamp).
    twist = Twist()
    twist.linear.z = 0.04
    pub.publish(twist)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.6:
        ex.spin_once(timeout_sec=0.02)
    snapshot_post = list(received[-1].position)

    delta = max(abs(a - b) for a, b in zip(snapshot_pre, snapshot_post))
    assert delta > 1e-3, "joints did not respond to /cmd_vel.linear.z"

    node.destroy_node()
    listener.destroy_node()
    publisher.destroy_node()


def test_body_z_range_params_passed_to_commander(rclpy_ctx):
    node = KinematicNode(parameter_overrides=_overrides(
        body_z_min=-0.10, body_z_max=+0.10))
    assert node.commander.body_z_min == pytest.approx(-0.10)
    assert node.commander.body_z_max == pytest.approx(+0.10)
    node.destroy_node()
```

`pytest` is already imported transitively via `pytest.importorskip("rclpy")` at the top of the file; if your linter complains, add `import pytest` at the top of the file.

- [ ] **Step 2: Run smoke tests, confirm new ones fail**

```bash
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws/src/dog_robot_kinematic_viz
python -m pytest test/test_kinematic_node_smoke.py -v
```

Expected:
- `test_linear_z_drives_body_height_state` → FAIL (delta near zero — `msg.linear.z` is ignored)
- `test_body_z_range_params_passed_to_commander` → FAIL (`AttributeError` on `body_z_min`, OR a ROS parameter error about unknown override)
- All pre-existing smoke tests → PASS unchanged

- [ ] **Step 3: Update `kinematic_node.py`**

Apply three edits to `dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/kinematic_node.py`.

(a) After the existing parameter declarations (after the `self.declare_parameter("stance_phase_ratio", 0.5)` line), add the two new range params and read them:

```python
        self.declare_parameter("body_z_min", -0.04)
        self.declare_parameter("body_z_max", +0.04)
```

(b) Pass them to `BodyCommander.__init__`. Replace the existing line:

```python
        self.commander = BodyCommander(
            step_freq=float(self.get_parameter("step_freq").value))
```

with:

```python
        self.commander = BodyCommander(
            step_freq=float(self.get_parameter("step_freq").value),
            body_z_min=float(self.get_parameter("body_z_min").value),
            body_z_max=float(self.get_parameter("body_z_max").value))
```

(c) Update `_on_cmd_vel` to forward `linear.z`:

```python
    def _on_cmd_vel(self, msg: Twist) -> None:
        self.commander.on_cmd_vel(
            msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
```

(d) Update `_tick` to pull `body_z` and pass it to each driver. Replace the inner loop:

```python
        v_xy = self.commander.body_vel_xy()
        positions: List[float] = []
        for leg in LEG_NAMES:
            if leg in self.drivers:
                q = self.drivers[leg].step(v_xy, self.commander.phase(leg))
            else:
                q = self._idle
            positions.extend(float(x) for x in q)
```

with:

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

- [ ] **Step 4: Run all kinematic_viz tests, confirm pass**

```bash
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws/src/dog_robot_kinematic_viz
python -m pytest test/ -v
```

Expected: all tests in the package PASS — `body_commander`, `leg_driver`, `kinematic_node_smoke`, `foot_target`, `leg_geometry`. No regressions.

- [ ] **Step 5: Commit**

```bash
cd /home/nguyenvd/workspace/dog_robot_height
git add dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/kinematic_node.py \
        dog_robot_ws/src/dog_robot_kinematic_viz/test/test_kinematic_node_smoke.py
git commit -m "feat(kinematic_node): forward linear.z, declare body_z range params"
```

---

### Task 4: `teleop_keyboard` r/f keys + 4-axis space-zero + linear.z publish

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/teleop_keyboard.py`
- Create: `dog_robot_ws/src/dog_robot_kinematic_viz/test/test_teleop_keyboard.py`

- [ ] **Step 1: Create `test_teleop_keyboard.py` with r/f/space tests**

Create `dog_robot_ws/src/dog_robot_kinematic_viz/test/test_teleop_keyboard.py` with:

```python
"""TeleopKeyboard key handling: r/f drive vz, space zeros all 4 axes, /cmd_vel
publishes linear.z. Skipped when rclpy is unavailable.
"""
import pytest

rclpy = pytest.importorskip("rclpy")

from geometry_msgs.msg import Twist           # noqa: E402

from dog_robot_kinematic_viz.teleop_keyboard import (   # noqa: E402
    TeleopKeyboard, LIN_STEP, LIN_MAX,
)


@pytest.fixture
def rclpy_ctx():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_r_key_increments_vz(rclpy_ctx):
    node = TeleopKeyboard()
    assert node._vz == 0.0
    assert node.on_key("r") is True
    assert node._vz == pytest.approx(LIN_STEP)
    node.destroy_node()


def test_f_key_decrements_vz(rclpy_ctx):
    node = TeleopKeyboard()
    assert node.on_key("f") is True
    assert node._vz == pytest.approx(-LIN_STEP)
    node.destroy_node()


def test_vz_clamps_to_lin_max(rclpy_ctx):
    node = TeleopKeyboard()
    # Press r enough times to exceed the +LIN_MAX clamp.
    for _ in range(int(LIN_MAX / LIN_STEP) + 5):
        node.on_key("r")
    assert node._vz == pytest.approx(LIN_MAX)
    node.destroy_node()


def test_vz_clamps_to_neg_lin_max(rclpy_ctx):
    node = TeleopKeyboard()
    for _ in range(int(LIN_MAX / LIN_STEP) + 5):
        node.on_key("f")
    assert node._vz == pytest.approx(-LIN_MAX)
    node.destroy_node()


def test_space_zeros_all_four_axes(rclpy_ctx):
    node = TeleopKeyboard()
    node.on_key("w")   # vx > 0
    node.on_key("a")   # vy > 0
    node.on_key("r")   # vz > 0
    node.on_key("j")   # wz > 0
    assert node._vx != 0.0
    assert node._vy != 0.0
    assert node._vz != 0.0
    assert node._wz != 0.0
    node.on_key(" ")
    assert node._vx == 0.0
    assert node._vy == 0.0
    assert node._vz == 0.0
    assert node._wz == 0.0
    node.destroy_node()


def test_publish_emits_linear_z(rclpy_ctx):
    node = TeleopKeyboard()
    received: list[Twist] = []

    listener = rclpy.create_node("teleop_listener")
    listener.create_subscription(
        Twist, "/cmd_vel", lambda m: received.append(m), 10)

    node.on_key("r")   # vz = +LIN_STEP, also calls publish()
    # Spin listener a few times to receive.
    import time
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.3 and not received:
        rclpy.spin_once(listener, timeout_sec=0.02)
    assert received, "listener did not receive /cmd_vel"
    assert received[-1].linear.z == pytest.approx(LIN_STEP)

    listener.destroy_node()
    node.destroy_node()


def test_q_key_returns_false(rclpy_ctx):
    node = TeleopKeyboard()
    assert node.on_key("q") is False
    node.destroy_node()
```

- [ ] **Step 2: Run new test file, confirm tests fail**

```bash
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws/src/dog_robot_kinematic_viz
python -m pytest test/test_teleop_keyboard.py -v
```

Expected:
- `test_r_key_increments_vz`, `test_f_key_decrements_vz`, `test_vz_clamps_*` → FAIL (`AttributeError: 'TeleopKeyboard' object has no attribute '_vz'`)
- `test_space_zeros_all_four_axes` → FAIL (same)
- `test_publish_emits_linear_z` → FAIL (msg.linear.z == 0.0)
- `test_q_key_returns_false` → PASS (already in old code)

- [ ] **Step 3: Update `teleop_keyboard.py`**

Replace the entire contents of `dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/teleop_keyboard.py` with:

```python
"""Minimal WASD+JL+RF teleop publisher for /cmd_vel.

Self-contained — no dog_robot_control / external teleop_twist_keyboard
dependency. Reads single keypresses from a TTY in raw mode; designed to be
launched inside a real terminal (e.g. prefix="gnome-terminal --" in the
launch file).

Keys:
    w / s  : linear.x  +/-   (forward / back)
    a / d  : linear.y  +/-   (strafe left / right)
    r / f  : linear.z  +/-   (body up / down — height velocity)
    j / l  : angular.z +/-   (yaw left / right)
    space  : zero all four axes
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
        self._wz = 0.0

    def publish(self):
        msg = Twist()
        msg.linear.x = self._vx
        msg.linear.y = self._vy
        msg.linear.z = self._vz
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
        elif key == "j":
            self._wz = _clamp(self._wz + ANG_STEP, -ANG_MAX, ANG_MAX)
        elif key == "l":
            self._wz = _clamp(self._wz - ANG_STEP, -ANG_MAX, ANG_MAX)
        elif key == " ":
            self._vx = self._vy = self._vz = self._wz = 0.0
        elif key in ("q", "\x03"):     # q or Ctrl-C
            return False
        else:
            return True
        self.publish()
        self.get_logger().info(
            f"cmd_vel: linear=({self._vx:+.2f},{self._vy:+.2f},{self._vz:+.2f})  "
            f"angular_z={self._wz:+.2f}")
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

- [ ] **Step 4: Run teleop tests, confirm pass**

```bash
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws/src/dog_robot_kinematic_viz
python -m pytest test/test_teleop_keyboard.py -v
```

Expected: all 7 tests PASS.

Then run the full kinematic_viz suite one more time to catch any cross-file regression:

```bash
python -m pytest test/ -v
```

Expected: ALL tests PASS across the 5 test files.

- [ ] **Step 5: Commit**

```bash
cd /home/nguyenvd/workspace/dog_robot_height
git add dog_robot_ws/src/dog_robot_kinematic_viz/dog_robot_kinematic_viz/teleop_keyboard.py \
        dog_robot_ws/src/dog_robot_kinematic_viz/test/test_teleop_keyboard.py
git commit -m "feat(teleop): r/f keys for body height, space zeros all 4 axes"
```

---

### Task 5: Full colcon build + colcon test + manual RViz verify

This is the integration gate before declaring the feature done.

- [ ] **Step 1: Kill any orphan processes from prior launches**

(See `feedback_kill_before_relaunch` and `feedback_pkill_dog_robot_orphans` memories — orphan `gzserver` / `kinematic_node` / `teleop_keyboard` from previous runs cause TF and `/cmd_vel` flakes. Use a script file, not an inline pkill, so the matcher doesn't truncate at 15 chars or self-match.)

```bash
bash /home/nguyenvd/workspace/dog_robot/dog_robot_ws/scripts/dog_relaunch_walk.sh --kill-only || true
# Fallback if the script's kill-only mode is not present:
pkill -f kinematic_node || true
pkill -f teleop_keyboard || true
pkill -f static_transform_publisher || true
pkill -f robot_state_publisher || true
pkill -f rviz2 || true
```

(Confirm with `pgrep -a kinematic_node` — should print nothing.)

- [ ] **Step 2: Full colcon build of the worktree workspace**

```bash
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws
colcon build --packages-select dog_robot_kinematic_viz dog_robot_description dog_robot_kinematics --symlink-install
```

Expected: build succeeds with no errors. If `dog_robot_kinematic_viz` fails on the setuptools 81 issue (see `feedback_setuptools_81_colcon`), either downgrade setuptools or fall back to `pip install -e <abs path>` for that package — note in the commit message which fallback was used.

- [ ] **Step 3: Full colcon test**

```bash
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws
colcon test --packages-select dog_robot_kinematic_viz --event-handlers console_direct+
colcon test-result --verbose
```

Expected: 0 errors, 0 failures across the kinematic_viz test suite. The new tests (~14 added) should appear in the count.

- [ ] **Step 4: Manual launch + RViz visual check**

```bash
cd /home/nguyenvd/workspace/dog_robot_height/dog_robot_ws
source install/setup.bash
ros2 launch dog_robot_kinematic_viz kinematic_teleop.launch.py
```

Test plan in the teleop terminal:
- Press `r` ~5 times → RViz: thigh/knee joints should extend, foot stays planted (body appears to "rise" relative to ground). Log shows `linear=(...,...,+0.10)`.
- Press `f` ~10 times → RViz: thigh/knee retract; foot stays planted; body appears to lower. Log eventually clamps at `linear=(...,...,-0.20)`, but `body_z` itself stops at `body_z_min = -0.04` (visible as joints saturating, not the published value).
- Press `space` → all axes zero; legs return to rest pose.
- Press `w` then `r` → robot walks forward AND body height changes simultaneously without IK failures (no orphan log warnings about unreachable targets).
- Press `q` to quit.

Acceptance: legs respond visibly to `r`/`f`, base_link stays at the static-TF height (which is the correct kinematics-only behavior — see spec "Out-of-scope" item 2), and pressing `space` returns to rest.

- [ ] **Step 5: Final commit (only if build/test required ancillary fixes)**

If steps 2-4 surfaced any small fixes (e.g. `setup.py` entry, README touch-up, launch arg tweak), commit those now:

```bash
cd /home/nguyenvd/workspace/dog_robot_height
git status
# add only the touched files, no -A
git add <files>
git commit -m "chore(body-height): integration fixes from manual verify"
```

Otherwise skip — no empty commits.

- [ ] **Step 6: Update the plan checkboxes and (optionally) the spec follow-ups**

If you want to leave a trail for the next session, commit the now-fully-checked plan:

```bash
cd /home/nguyenvd/workspace/dog_robot_height
git add dog_robot_ws/docs/superpowers/plans/2026-06-01-body-height-plan.md
git commit -m "docs(plan): tick off body-height implementation tasks"
```

---

## Self-review (do this before handing off)

1. **Spec coverage:**
   - User-facing contract (linear.z → body_z velocity, ±0.04 clamp, space zeros all four) → Tasks 1, 3, 4.
   - `BodyCommander` 4-arg `on_cmd_vel`, `body_z()`, clamp at source → Task 1.
   - `LegDriver.step(..., body_z=0.0)` per-call shift, rest not mutated → Task 2.
   - `kinematic_node` declares `body_z_min`/`body_z_max`, forwards `linear.z`, passes `body_z` to drivers → Task 3.
   - Teleop r/f, 4-axis space-zero, HELP/log updated → Task 4.
   - `foot_target.py` untouched → no task (correct).
   - Manual RViz acceptance criteria → Task 5 Step 4.

2. **Type / signature consistency:**
   - `BodyCommander.on_cmd_vel(linear_x, linear_y, linear_z, angular_z)` is used consistently in Task 1 (test + impl) and Task 3 (kinematic_node `_on_cmd_vel`).
   - `BodyCommander(body_z_min=..., body_z_max=...)` kwargs match in Task 1 impl and Task 3 wiring.
   - `LegDriver.step(body_v_xy, phase, body_z=0.0)` matches in Task 2 impl, Task 2 tests, and Task 3 caller (`bz` positional).
   - `commander.body_z()` getter used in Task 3 matches Task 1 impl.

3. **Range / clamp consistency:** Defaults `±0.04` appear in spec, `BodyCommander.__init__` defaults, `kinematic_node.declare_parameter` defaults, and test fixtures — all aligned.

4. **No placeholders:** All test bodies and all impl bodies are spelled out in full. No "similar to above" references.

5. **Commit cadence:** 4 feature commits + at most 1 chore + 1 docs = ≤ 6 commits, each one a self-contained green test run.

---

## Execution handoff

Plan complete and saved to `dog_robot_ws/docs/superpowers/plans/2026-06-01-body-height-plan.md` (worktree `/home/nguyenvd/workspace/dog_robot_height`, branch `feature/body-height`).
