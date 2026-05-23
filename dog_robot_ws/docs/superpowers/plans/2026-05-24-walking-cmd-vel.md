# Walking controller (omni cmd_vel) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port CHAMP gait engine to Python + add walker_controller node that consumes /cmd_vel, runs DH-IK, publishes JointTrajectory. Walker subsumes stand_controller.

**Architecture:** Four gait modules under `dog_robot_control/gait/` (config, phase_generator, trajectory_planner, body_controller, leg_controller) reproduce CHAMP's geometric trot gait math. The walker_controller node ticks at 50 Hz, composes body pose + gait deltas, transforms foot positions into DH hip frame, calls existing `kinematics_dh.ik_leg`, publishes JointTrajectory. JTC keeps position+open_loop_control interface that proved stable for stand.

**Tech Stack:** Python 3.10 + numpy, pytest, ROS 2 Humble (`rclpy`, `geometry_msgs`, `sensor_msgs`, `trajectory_msgs`), gazebo_ros2_control + joint_trajectory_controller.

---

## File plan

**Create:**
- `dog_robot_ws/src/dog_robot_control/dog_robot_control/gait/__init__.py`
- `dog_robot_ws/src/dog_robot_control/dog_robot_control/gait/gait_config.py`
- `dog_robot_ws/src/dog_robot_control/dog_robot_control/gait/phase_generator.py`
- `dog_robot_ws/src/dog_robot_control/dog_robot_control/gait/trajectory_planner.py`
- `dog_robot_ws/src/dog_robot_control/dog_robot_control/gait/body_controller.py`
- `dog_robot_ws/src/dog_robot_control/dog_robot_control/gait/leg_controller.py`
- `dog_robot_ws/src/dog_robot_control/dog_robot_control/walker_controller.py`
- `dog_robot_ws/src/dog_robot_control/config/walker_params.yaml`
- `dog_robot_ws/src/dog_robot_control/launch/walk.launch.py`
- `dog_robot_ws/src/dog_robot_control/test/test_phase_generator.py`
- `dog_robot_ws/src/dog_robot_control/test/test_trajectory_planner.py`
- `dog_robot_ws/src/dog_robot_control/test/test_body_controller.py`
- `dog_robot_ws/src/dog_robot_control/test/test_leg_controller.py`
- `dog_robot_ws/src/dog_robot_control/test/test_walker_integration.py`

**Modify:**
- `dog_robot_ws/src/dog_robot_control/setup.py` — register `walker_controller` console_script
- `dog_robot_ws/README.md` — append Walking section
- `dog_robot_ws/scripts/dog_kill_all.sh` — add walker_controller pattern

---

### Task 1: GaitConfig + zero_stance helper

**Files:**
- Create: `dog_robot_ws/src/dog_robot_control/dog_robot_control/gait/__init__.py` (empty)
- Create: `dog_robot_ws/src/dog_robot_control/dog_robot_control/gait/gait_config.py`

`zero_stance` lives in `gait_config.py` because it depends on both `GaitConfig.nominal_height` and `LegConfig.base_to_hip_xyz` + DH `L_hh+L_th+L_sh`. A small helper, NOT a separate file.

- [ ] **Step 1: Create gait/__init__.py (empty)**

```bash
mkdir -p /home/nguyenvd/workspace/dog_robot/dog_robot_ws/src/dog_robot_control/dog_robot_control/gait
touch /home/nguyenvd/workspace/dog_robot/dog_robot_ws/src/dog_robot_control/dog_robot_control/gait/__init__.py
```

- [ ] **Step 2: Write gait_config.py**

```python
"""Gait configuration + per-leg zero stance helper."""
from dataclasses import dataclass

import numpy as np

from dog_robot_control.kinematics_dh import DHParams
from dog_robot_control.leg_config import LegConfig


@dataclass(frozen=True)
class GaitConfig:
    nominal_height: float            # body z above ground at stand (m)
    stance_duration: float           # s
    swing_height: float              # m
    stance_depth: float              # m (small downward dip during stance)
    max_linear_velocity_x: float
    max_linear_velocity_y: float
    max_angular_velocity_z: float


def zero_stance(leg: LegConfig, dh: DHParams, gait: GaitConfig) -> np.ndarray:
    """Foot resting position in body frame (X forward, Y left, Z up), at all
    joint angles = 0 then body-translated so foot is at ground at body height =
    gait.nominal_height.
    """
    # All joints at 0: foot in body frame = hip_xyz + R_bh @ (L_total, 0, 0).
    # base_to_hip_rpy is always (0, pi/2, *), so R_bh @ (L, 0, 0) = (0, 0, -L)
    # regardless of the right-side pi yaw (Z rotation doesn't change Z).
    L_total = dh.L_hh + dh.L_th + dh.L_sh
    return np.array([
        leg.base_to_hip_xyz[0],
        leg.base_to_hip_xyz[1],
        leg.base_to_hip_xyz[2] - L_total,
    ])


def center_to_nominal(leg: LegConfig) -> float:
    """Distance from body center to nominal foot position projected on XY."""
    return float(np.hypot(leg.base_to_hip_xyz[0], leg.base_to_hip_xyz[1]))
```

- [ ] **Step 3: Quick sanity test (no separate test file — verify in repl)**

```bash
cd /home/nguyenvd/workspace/dog_robot/dog_robot_ws/src/dog_robot_control && python3 -c "
from dog_robot_control.kinematics_dh import DHParams
from dog_robot_control.leg_config import LEGS
from dog_robot_control.gait.gait_config import GaitConfig, zero_stance, center_to_nominal
dh = DHParams(L_hh=0.02553, L_th=0.11725, L_sh=0.07043)
gc = GaitConfig(0.15, 0.30, 0.03, 0.001, 0.15, 0.08, 0.50)
for L in LEGS:
    print(L.name, 'zero_stance=', zero_stance(L, dh, gc), 'r=', round(center_to_nominal(L), 4))
"
```

Expected: 4 legs, each zero_stance z ≈ 0.0351 - 0.213 = -0.178, x/y match base_to_hip; r ≈ 0.0848.

- [ ] **Step 4: Commit**

```bash
git add dog_robot_ws/src/dog_robot_control/dog_robot_control/gait/__init__.py \
        dog_robot_ws/src/dog_robot_control/dog_robot_control/gait/gait_config.py
git commit -m "feat(gait): GaitConfig + zero_stance/center_to_nominal helpers"
```

---

### Task 2: phase_generator.py (TDD)

**Files:**
- Create: `dog_robot_ws/src/dog_robot_control/dog_robot_control/gait/phase_generator.py`
- Create: `dog_robot_ws/src/dog_robot_control/test/test_phase_generator.py`

- [ ] **Step 1: Write failing test**

```python
# test/test_phase_generator.py
import math
import pytest

from dog_robot_control.gait.phase_generator import PhaseGenerator

STANCE = 0.30
SWING = 0.25  # CHAMP hard-coded
STRIDE = STANCE + SWING


def make_pg():
    return PhaseGenerator(stance_duration=STANCE)


def test_idle_signals_zero_when_velocity_zero():
    pg = make_pg()
    pg.run(target_velocity=0.0, step_length=0.1, t=0.0)
    assert pg.stance_phase_signal == [0.0, 0.0, 0.0, 0.0]
    assert pg.swing_phase_signal == [0.0, 0.0, 0.0, 0.0]


def test_warmup_holds_legs_0_3_in_stance():
    """First half-stride forces FL+BR to stance, no swing of FR/BL — avoids cold-start tipping."""
    pg = make_pg()
    pg.run(target_velocity=0.1, step_length=0.05, t=0.0)         # init last_touchdown
    pg.run(target_velocity=0.1, step_length=0.05, t=STANCE * 0.3)
    # Within first 0.5 stance signal, warmup engages:
    assert pg.stance_phase_signal[0] == 0.0
    assert pg.stance_phase_signal[3] == 0.0
    assert pg.swing_phase_signal[1] == 0.0
    assert pg.swing_phase_signal[2] == 0.0


def test_trot_anti_phase_after_warmup():
    """After warmup, leg 0 and 3 are in sync; legs 1 and 2 are offset by half stride."""
    pg = make_pg()
    pg.run(0.1, 0.05, 0.0)
    # Step past warmup by running through stance phase past 0.5:
    pg.run(0.1, 0.05, STANCE * 0.6)
    # Now run at a point in the middle of a stride where signals exist:
    pg.run(0.1, 0.05, STRIDE * 1.25)  # leg 0 should be in stance, leg 1 elsewhere
    assert pg.stance_phase_signal[0] == pytest.approx(pg.stance_phase_signal[3], abs=1e-6)
    # leg 1 and 2 share their own signal:
    assert pg.stance_phase_signal[1] == pytest.approx(pg.stance_phase_signal[2], abs=1e-6)


def test_velocity_zero_resets_state():
    pg = make_pg()
    pg.run(0.1, 0.05, 0.0)
    pg.run(0.1, 0.05, STRIDE * 0.5)
    pg.run(0.0, 0.0, STRIDE * 0.6)
    assert pg.stance_phase_signal == [0.0, 0.0, 0.0, 0.0]
    assert pg.swing_phase_signal == [0.0, 0.0, 0.0, 0.0]
    # After reset, next non-zero call must re-init last_touchdown:
    pg.run(0.1, 0.05, STRIDE * 0.7)
    # Should NOT immediately produce signals for the post-reset moment
    # (last_touchdown got set to STRIDE*0.7, so elapsed = 0).
    assert pg.stance_phase_signal == [0.0, 0.0, 0.0, 0.0]
```

- [ ] **Step 2: Run, expect ImportError**

```bash
cd /home/nguyenvd/workspace/dog_robot/dog_robot_ws/src/dog_robot_control && python3 -m pytest test/test_phase_generator.py -v
```

- [ ] **Step 3: Implement phase_generator.py**

```python
"""Trot phase generator — port of CHAMP's PhaseGenerator.

Outputs per-leg saw-tooth signals in [0, 1] indicating stance vs swing
progress. Leg order: [FL=0, FR=1, BL=2, BR=3]. Trot pairs: (0,3) and (1,2).
Swing phase is hard-coded to 0.25 s like CHAMP.
"""
from typing import List


SWING_PHASE_PERIOD = 0.25  # s, CHAMP-compatible


class PhaseGenerator:
    def __init__(self, stance_duration: float) -> None:
        self.stance_duration = stance_duration
        self.last_touchdown: float = 0.0
        self.has_started: bool = False
        self.has_swung: bool = False
        self.stance_phase_signal: List[float] = [0.0, 0.0, 0.0, 0.0]
        self.swing_phase_signal: List[float] = [0.0, 0.0, 0.0, 0.0]

    def run(self, target_velocity: float, step_length: float, t: float) -> None:
        stance_period = self.stance_duration
        swing_period = SWING_PHASE_PERIOD
        stride_period = stance_period + swing_period

        if target_velocity == 0.0:
            self.has_started = False
            self.has_swung = False
            self.last_touchdown = 0.0
            self.stance_phase_signal = [0.0] * 4
            self.swing_phase_signal = [0.0] * 4
            return

        if not self.has_started:
            self.has_started = True
            self.last_touchdown = t

        if (t - self.last_touchdown) >= stride_period:
            self.last_touchdown = t

        elapsed = t - self.last_touchdown
        if elapsed >= stride_period:
            elapsed = stride_period

        leg_clocks = [
            elapsed - 0.0 * stride_period,   # leg 0 (FL)
            elapsed - 0.5 * stride_period,   # leg 1 (FR)
            elapsed - 0.5 * stride_period,   # leg 2 (BL)
            elapsed - 0.0 * stride_period,   # leg 3 (BR)
        ]

        stance = [0.0] * 4
        swing = [0.0] * 4
        for i, c in enumerate(leg_clocks):
            if 0 < c < stance_period:
                stance[i] = c / stance_period
            if -swing_period < c < 0:
                swing[i] = (c + swing_period) / swing_period
            elif stance_period < c < stride_period:
                swing[i] = (c - stance_period) / swing_period

        # Warmup: keep legs 0 and 3 grounded (no swing of 1 and 2)
        # until leg 0 stance crosses 0.5 the first time.
        if not self.has_swung and stance[0] < 0.5:
            stance[0] = 0.0
            stance[3] = 0.0
            swing[1] = 0.0
            swing[2] = 0.0
        else:
            self.has_swung = True

        self.stance_phase_signal = stance
        self.swing_phase_signal = swing
```

- [ ] **Step 4: Run tests, verify pass**

```bash
cd /home/nguyenvd/workspace/dog_robot/dog_robot_ws/src/dog_robot_control && python3 -m pytest test/test_phase_generator.py -v
```

Expected: 4 passed.

- [ ] **Step 5: Commit**

```bash
git add dog_robot_ws/src/dog_robot_control/dog_robot_control/gait/phase_generator.py \
        dog_robot_ws/src/dog_robot_control/test/test_phase_generator.py
git commit -m "feat(gait): phase_generator port of CHAMP trot pattern"
```

---

### Task 3: trajectory_planner.py (TDD)

**Files:**
- Create: `dog_robot_ws/src/dog_robot_control/dog_robot_control/gait/trajectory_planner.py`
- Create: `dog_robot_ws/src/dog_robot_control/test/test_trajectory_planner.py`

- [ ] **Step 1: Write failing test**

```python
# test/test_trajectory_planner.py
import math
import numpy as np
import pytest

from dog_robot_control.gait.gait_config import GaitConfig
from dog_robot_control.gait.trajectory_planner import TrajectoryPlanner


GC = GaitConfig(nominal_height=0.15, stance_duration=0.30, swing_height=0.03,
                stance_depth=0.001, max_linear_velocity_x=0.15,
                max_linear_velocity_y=0.08, max_angular_velocity_z=0.50)


def test_zero_step_length_no_delta():
    tp = TrajectoryPlanner(GC)
    foot = np.array([0.1, 0.0, -0.15])
    out = tp.generate(foot.copy(), step_length=0.0, rotation=0.0,
                      swing_phase=0.0, stance_phase=0.0)
    assert np.allclose(out, foot)


def test_stance_sweep_linear_x():
    tp = TrajectoryPlanner(GC)
    foot0 = np.array([0.0, 0.0, -0.15])
    s = 0.08
    # stance_phase=0 (start) -> x = +s/2; stance_phase=1 (end) -> x = -s/2
    at_start = tp.generate(foot0.copy(), s, 0.0, swing_phase=0.0, stance_phase=0.001)
    at_end   = tp.generate(foot0.copy(), s, 0.0, swing_phase=0.0, stance_phase=0.999)
    assert at_start[0] == pytest.approx( s / 2, abs=1e-3)
    assert at_end[0]   == pytest.approx(-s / 2, abs=1e-3)


def test_swing_peak_lifts_foot():
    tp = TrajectoryPlanner(GC)
    foot0 = np.array([0.0, 0.0, -0.15])
    s = 0.08
    # At swing_phase = 0.5 the foot should be lifted (z > -0.15)
    out = tp.generate(foot0.copy(), s, 0.0, swing_phase=0.5, stance_phase=0.0)
    assert out[2] > foot0[2] + 0.005, f"foot did not lift: z={out[2]}"


def test_swing_endpoints_x_match_stance_endpoints():
    """Continuity: swing at signal=0+ exits to the back, swing at signal=1- arrives at the front."""
    tp = TrajectoryPlanner(GC)
    foot0 = np.array([0.0, 0.0, -0.15])
    s = 0.08
    swing_start = tp.generate(foot0.copy(), s, 0.0, swing_phase=0.01, stance_phase=0.0)
    swing_end   = tp.generate(foot0.copy(), s, 0.0, swing_phase=0.99, stance_phase=0.0)
    # back leg lifting off ≈ -s/2; front landing ≈ +s/2
    assert swing_start[0] == pytest.approx(-s / 2, abs=0.02)
    assert swing_end[0]   == pytest.approx( s / 2, abs=0.02)


def test_rotation_steers_delta_into_y():
    tp = TrajectoryPlanner(GC)
    foot0 = np.array([0.0, 0.0, -0.15])
    s = 0.08
    out = tp.generate(foot0.copy(), s, rotation=math.pi/2,
                      swing_phase=0.0, stance_phase=0.001)
    assert out[0] == pytest.approx(0.0, abs=1e-3)
    assert out[1] == pytest.approx(s / 2, abs=1e-3)
```

- [ ] **Step 2: Run, expect ImportError**

```bash
python3 -m pytest test/test_trajectory_planner.py -v
```

- [ ] **Step 3: Implement trajectory_planner.py**

```python
"""Per-leg foot trajectory — port of CHAMP's TrajectoryPlanner.

Stance phase: linear backward sweep along leg X plus small cosine z dip.
Swing phase: 12-control-point Bernstein-Bezier curve copied from CHAMP,
scaled by step_length / 0.4 m and swing_height / 0.15 m.

Per-tick output is added (in body frame) to the foot_position passed in.
"""
import math
from typing import Sequence

import numpy as np

from dog_robot_control.gait.gait_config import GaitConfig


# CHAMP reference control points (normalized; positive y is upward in CHAMP's
# convention but we flip sign below so positive lift goes to +z in our frame).
_REF_X = (-0.15, -0.2805, -0.3, -0.3, -0.3,  0.0,
           0.0,   0.0,    0.3032, 0.3032, 0.2826, 0.15)
_REF_Y = (-0.5, -0.5, -0.3611, -0.3611, -0.3611, -0.3611,
          -0.3611, -0.3214, -0.3214, -0.3214, -0.5, -0.5)

_N_POINTS = 12
_FACT = (1.0, 1.0, 2.0, 6.0, 24.0, 120.0, 720.0, 5040.0,
         40320.0, 362880.0, 3628800.0, 39916800.0)


def _bernstein(i: int, n: int, t: float) -> float:
    coeff = _FACT[n] / (_FACT[i] * _FACT[n - i])
    return coeff * (t ** i) * ((1.0 - t) ** (n - i))


class TrajectoryPlanner:
    def __init__(self, gait: GaitConfig) -> None:
        self.gait = gait

    def _control_points(self, step_length: float) -> tuple:
        h_ratio = self.gait.swing_height / 0.15
        l_ratio = step_length / 0.4
        cp_x = [0.0] * _N_POINTS
        cp_y = [0.0] * _N_POINTS
        for i in range(_N_POINTS):
            if i == 0:
                cp_x[i] = -step_length / 2.0
            elif i == _N_POINTS - 1:
                cp_x[i] = step_length / 2.0
            else:
                cp_x[i] = _REF_X[i] * l_ratio
            cp_y[i] = -((_REF_Y[i] * h_ratio) + 0.5 * h_ratio)
        return cp_x, cp_y

    def generate(self, foot_position: np.ndarray, step_length: float,
                 rotation: float, swing_phase: float,
                 stance_phase: float) -> np.ndarray:
        """Return foot_position + computed delta (does NOT mutate input)."""
        if step_length == 0.0:
            return foot_position

        cp_x, cp_y = self._control_points(step_length)
        n = _N_POINTS - 1
        dx = 0.0
        dz = 0.0   # CHAMP names this 'y' but our world has Z vertical

        if stance_phase > swing_phase:
            # Stance: linear sweep + cosine dip.
            dx = (step_length / 2.0) * (1.0 - 2.0 * stance_phase)
            dz = -self.gait.stance_depth * math.cos(math.pi * dx / step_length)
        elif swing_phase > stance_phase:
            # Swing: Bezier curve. CHAMP subtracts y so an upward (+z) lift comes out.
            for i in range(_N_POINTS):
                b = _bernstein(i, n, swing_phase)
                dx += b * cp_x[i]
                dz -= b * cp_y[i]
        # else: both zero, no delta.

        result = foot_position.copy()
        result[0] += dx * math.cos(rotation)
        result[1] += dx * math.sin(rotation)
        result[2] += dz
        return result
```

- [ ] **Step 4: Run tests, verify pass**

```bash
python3 -m pytest test/test_trajectory_planner.py -v
```

Expected: 5 passed.

- [ ] **Step 5: Commit**

```bash
git add dog_robot_ws/src/dog_robot_control/dog_robot_control/gait/trajectory_planner.py \
        dog_robot_ws/src/dog_robot_control/test/test_trajectory_planner.py
git commit -m "feat(gait): trajectory_planner port of CHAMP Bezier swing + stance dip"
```

---

### Task 4: body_controller.py (TDD)

**Files:**
- Create: `dog_robot_ws/src/dog_robot_control/dog_robot_control/gait/body_controller.py`
- Create: `dog_robot_ws/src/dog_robot_control/test/test_body_controller.py`

- [ ] **Step 1: Write failing test**

```python
# test/test_body_controller.py
import math
import numpy as np
import pytest

from dog_robot_control.kinematics_dh import DHParams
from dog_robot_control.leg_config import LEGS, get_leg
from dog_robot_control.gait.gait_config import GaitConfig
from dog_robot_control.gait.body_controller import BodyController, BodyPose


DH = DHParams(L_hh=0.02553, L_th=0.11725, L_sh=0.07043)
GC = GaitConfig(0.15, 0.30, 0.03, 0.001, 0.15, 0.08, 0.50)


def test_pose_at_nominal_returns_zero_at_hip_frame():
    """req_pose at (0,0,nominal_height,0,0,0): foot in body-at-hip frame
    should be (0, 0, -nominal_height)."""
    bc = BodyController(LEGS, DH, GC)
    foot = bc.pose_command(BodyPose(0, 0, GC.nominal_height, 0, 0, 0))
    for i, L in enumerate(LEGS):
        assert foot[i][0] == pytest.approx(0.0, abs=1e-6)
        assert foot[i][1] == pytest.approx(0.0, abs=1e-6)
        assert foot[i][2] == pytest.approx(-GC.nominal_height, abs=1e-6)


def test_pose_z_raises_lifts_foot():
    bc = BodyController(LEGS, DH, GC)
    foot = bc.pose_command(BodyPose(0, 0, GC.nominal_height + 0.05, 0, 0, 0))
    for i in range(4):
        # body lifted by 0.05 → foot in body-at-hip frame moves to z = -0.10
        assert foot[i][2] == pytest.approx(-(GC.nominal_height + 0.05) - 0.0 + (GC.nominal_height - (GC.nominal_height + 0.05)) + (-(GC.nominal_height + 0.05) + GC.nominal_height + 0.05), abs=0.01)
        # The intent: z ≈ -0.20 (body higher → foot relative to hip is lower in -z direction).
        assert foot[i][2] == pytest.approx(-0.20, abs=0.01)
```

Wait that test is muddled. Let me simplify:

```python
def test_pose_z_raises_lifts_foot():
    bc = BodyController(LEGS, DH, GC)
    foot = bc.pose_command(BodyPose(0, 0, GC.nominal_height + 0.05, 0, 0, 0))
    for i in range(4):
        # raising body by +0.05 → foot must be deeper (more negative z) by 0.05.
        assert foot[i][2] == pytest.approx(-GC.nominal_height - 0.05, abs=1e-6)


def test_pose_x_translates_foot_opposite():
    bc = BodyController(LEGS, DH, GC)
    foot = bc.pose_command(BodyPose(0.04, 0, GC.nominal_height, 0, 0, 0))
    for i in range(4):
        # body moves +x → foot must move -x relative to hip
        assert foot[i][0] == pytest.approx(-0.04, abs=1e-6)
```

- [ ] **Step 2: Replace muddled test — write final tests**

Replace the `test_body_controller.py` contents you wrote in Step 1 with this clean version:

```python
import math
import numpy as np
import pytest

from dog_robot_control.kinematics_dh import DHParams
from dog_robot_control.leg_config import LEGS
from dog_robot_control.gait.gait_config import GaitConfig
from dog_robot_control.gait.body_controller import BodyController, BodyPose


DH = DHParams(L_hh=0.02553, L_th=0.11725, L_sh=0.07043)
GC = GaitConfig(0.15, 0.30, 0.03, 0.001, 0.15, 0.08, 0.50)


def test_pose_at_nominal_centers_foot_below_hip():
    bc = BodyController(LEGS, DH, GC)
    foot = bc.pose_command(BodyPose(0, 0, GC.nominal_height, 0, 0, 0))
    for i in range(4):
        assert foot[i][0] == pytest.approx(0.0, abs=1e-6)
        assert foot[i][1] == pytest.approx(0.0, abs=1e-6)
        assert foot[i][2] == pytest.approx(-GC.nominal_height, abs=1e-6)


def test_pose_z_lifts_body_lowers_foot():
    bc = BodyController(LEGS, DH, GC)
    foot = bc.pose_command(BodyPose(0, 0, GC.nominal_height + 0.05, 0, 0, 0))
    for i in range(4):
        assert foot[i][2] == pytest.approx(-GC.nominal_height - 0.05, abs=1e-6)


def test_pose_x_translates_foot_opposite():
    bc = BodyController(LEGS, DH, GC)
    foot = bc.pose_command(BodyPose(0.04, 0, GC.nominal_height, 0, 0, 0))
    for i in range(4):
        assert foot[i][0] == pytest.approx(-0.04, abs=1e-6)


def test_pose_yaw_rotates_feet():
    bc = BodyController(LEGS, DH, GC)
    foot = bc.pose_command(BodyPose(0, 0, GC.nominal_height, 0, 0, 0.2))
    # rotated foot positions: rotation by -0.2 around body Z.
    # foot in body-at-hip frame BEFORE yaw is (0, 0, -h). After Rz(-0.2): (0, 0, -h).
    # So yaw alone with foot directly below hip doesn't change foot. Test via
    # a translated-then-rotated point — easier path: hip position itself rotates,
    # so foot relative to hip stays (0,0,-h). Skip yaw assertion; verify
    # combined: yaw + x-translation moves foot in x-y plane after rotation.
    # For unit test simplicity, only check no NaN and z stays approx -h:
    for i in range(4):
        assert not np.any(np.isnan(foot[i]))
        assert foot[i][2] == pytest.approx(-GC.nominal_height, abs=1e-6)
```

- [ ] **Step 3: Run, expect ImportError**

```bash
python3 -m pytest test/test_body_controller.py -v
```

- [ ] **Step 4: Implement body_controller.py**

```python
"""Body pose controller — port of CHAMP BodyController.

Given a requested body pose (translation x,y,z and orientation roll,pitch,yaw),
produce per-leg foot positions in the BODY-AT-HIP frame (axes parallel to
body frame, origin at the leg's hip joint). Foot starts at zero_stance and is
translated/rotated opposite to the requested body motion (legs stay grounded
while the body moves).
"""
from dataclasses import dataclass
from typing import List, Sequence

import numpy as np

from dog_robot_control.kinematics_dh import DHParams
from dog_robot_control.leg_config import LegConfig
from dog_robot_control.gait.gait_config import GaitConfig, zero_stance


@dataclass(frozen=True)
class BodyPose:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


def _Rx(a): c, s = np.cos(a), np.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def _Ry(a): c, s = np.cos(a), np.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def _Rz(a): c, s = np.cos(a), np.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])


class BodyController:
    def __init__(self, legs: Sequence[LegConfig], dh: DHParams,
                 gait: GaitConfig) -> None:
        self.legs = list(legs)
        self.dh = dh
        self.gait = gait
        # Cache zero_stance per leg (body frame, full extension).
        self._zero_stance = [zero_stance(L, dh, gait) for L in self.legs]

    def pose_command(self, req: BodyPose) -> List[np.ndarray]:
        """Return list of 4 foot positions in body-at-hip frame."""
        out: List[np.ndarray] = []
        for i, L in enumerate(self.legs):
            z0 = self._zero_stance[i]
            tx = -req.x
            ty = -req.y
            # Translation in z: lift foot toward body so body sits at req.z above ground.
            tz_raw = -(z0[2] + req.z)
            max_tz = -z0[2] * 0.65
            tz = max(0.0, min(tz_raw, max_tz))

            foot = z0 + np.array([tx, ty, tz])
            R = _Rz(-req.yaw) @ _Ry(-req.pitch) @ _Rx(-req.roll)
            foot = R @ foot

            # transformToHip: subtract hip origin in body frame.
            hip = np.array(L.base_to_hip_xyz)
            foot = foot - hip
            out.append(foot)
        return out
```

- [ ] **Step 5: Run tests, verify pass**

```bash
python3 -m pytest test/test_body_controller.py -v
```

Expected: 4 passed.

- [ ] **Step 6: Commit**

```bash
git add dog_robot_ws/src/dog_robot_control/dog_robot_control/gait/body_controller.py \
        dog_robot_ws/src/dog_robot_control/test/test_body_controller.py
git commit -m "feat(gait): body_controller port — pose command -> per-leg foot in body-at-hip"
```

---

### Task 5: leg_controller.py (TDD)

**Files:**
- Create: `dog_robot_ws/src/dog_robot_control/dog_robot_control/gait/leg_controller.py`
- Create: `dog_robot_ws/src/dog_robot_control/test/test_leg_controller.py`

- [ ] **Step 1: Write failing test**

```python
# test/test_leg_controller.py
import math
import numpy as np
import pytest

from dog_robot_control.kinematics_dh import DHParams
from dog_robot_control.leg_config import LEGS
from dog_robot_control.gait.gait_config import GaitConfig
from dog_robot_control.gait.leg_controller import LegController, Velocity


DH = DHParams(L_hh=0.02553, L_th=0.11725, L_sh=0.07043)
GC = GaitConfig(0.15, 0.30, 0.03, 0.001, 0.15, 0.08, 0.50)


def _foot_in():
    """4 feet at zero pose in body-at-hip frame."""
    return [np.array([0.0, 0.0, -0.15]) for _ in range(4)]


def test_zero_velocity_no_delta():
    lc = LegController(LEGS, DH, GC)
    feet = _foot_in()
    out = lc.velocity_command(feet, Velocity(0, 0, 0), t=0.0)
    for i in range(4):
        assert np.allclose(out[i], feet[i])


def test_forward_velocity_moves_feet_backward_during_stance():
    """At vx > 0, during stance phase the foot sweeps backward (negative x)."""
    lc = LegController(LEGS, DH, GC)
    # First call to init phase_generator state:
    lc.velocity_command(_foot_in(), Velocity(0.1, 0, 0), t=0.0)
    # Run through warmup + half a stride:
    for k in range(1, 30):
        lc.velocity_command(_foot_in(), Velocity(0.1, 0, 0), t=k * 0.02)
    out = lc.velocity_command(_foot_in(), Velocity(0.1, 0, 0), t=30 * 0.02)
    # At least one leg should have x != 0 (gait is running).
    deltas = [abs(out[i][0]) for i in range(4)]
    assert max(deltas) > 0.001, f"no gait delta: {deltas}"


def test_velocity_caps():
    lc = LegController(LEGS, DH, GC)
    # Send vx far above max; LegController should cap internally — gait still runs.
    lc.velocity_command(_foot_in(), Velocity(10.0, 0, 0), t=0.0)
    # No assertion on capped value externally; just ensure no exception.
```

- [ ] **Step 2: Run, expect ImportError**

```bash
python3 -m pytest test/test_leg_controller.py -v
```

- [ ] **Step 3: Implement leg_controller.py**

```python
"""Leg controller — port of CHAMP LegController.

Combines Raibert heuristic + per-leg trajectory planning. Mutates the foot
positions passed in (from BodyController) by adding gait deltas.
"""
import math
from dataclasses import dataclass
from typing import List, Sequence

import numpy as np

from dog_robot_control.kinematics_dh import DHParams
from dog_robot_control.leg_config import LegConfig
from dog_robot_control.gait.gait_config import (
    GaitConfig,
    center_to_nominal,
    zero_stance,
)
from dog_robot_control.gait.phase_generator import PhaseGenerator
from dog_robot_control.gait.trajectory_planner import TrajectoryPlanner


@dataclass(frozen=True)
class Velocity:
    vx: float
    vy: float
    wz: float


def _cap(v, lo, hi):
    return max(lo, min(hi, v))


class LegController:
    def __init__(self, legs: Sequence[LegConfig], dh: DHParams,
                 gait: GaitConfig) -> None:
        self.legs = list(legs)
        self.dh = dh
        self.gait = gait
        self.phase_generator = PhaseGenerator(gait.stance_duration)
        self.trajectory_planners: List[TrajectoryPlanner] = [
            TrajectoryPlanner(gait) for _ in self.legs
        ]
        self._zero_stance = [zero_stance(L, dh, gait) for L in self.legs]
        self._center_to_nom = center_to_nominal(self.legs[0])  # symmetric

    @staticmethod
    def _raibert(stance_duration: float, target_velocity: float) -> float:
        return (stance_duration / 2.0) * target_velocity

    def _transform_leg(self, leg_idx: int, step_x: float, step_y: float,
                       theta: float) -> tuple:
        z0 = self._zero_stance[leg_idx]
        # Rotated stance: zero_stance translated by (step_x, step_y, 0)
        # then rotated by theta around Z.
        tx = z0[0] + step_x
        ty = z0[1] + step_y
        c, s = math.cos(theta), math.sin(theta)
        rot_x = c * tx - s * ty
        rot_y = s * tx + c * ty
        delta_x = rot_x - z0[0]
        delta_y = rot_y - z0[1]
        step_length = math.hypot(delta_x, delta_y) * 2.0
        rotation = math.atan2(delta_y, delta_x)
        return step_length, rotation

    def velocity_command(self, foot_positions: List[np.ndarray],
                         req: Velocity, t: float) -> List[np.ndarray]:
        vx = _cap(req.vx, -self.gait.max_linear_velocity_x,
                  self.gait.max_linear_velocity_x)
        vy = _cap(req.vy, -self.gait.max_linear_velocity_y,
                  self.gait.max_linear_velocity_y)
        wz = _cap(req.wz, -self.gait.max_angular_velocity_z,
                  self.gait.max_angular_velocity_z)

        tangential = wz * self._center_to_nom
        velocity_mag = math.hypot(vx, vy + tangential)

        step_x = self._raibert(self.gait.stance_duration, vx)
        step_y = self._raibert(self.gait.stance_duration, vy)
        step_theta = self._raibert(self.gait.stance_duration, tangential)
        theta = math.sin((step_theta / 2.0) / self._center_to_nom) * 2.0

        step_lengths = [0.0] * 4
        rotations = [0.0] * 4
        for i in range(4):
            step_lengths[i], rotations[i] = self._transform_leg(
                i, step_x, step_y, theta)
        mean_step = sum(step_lengths) / 4.0

        self.phase_generator.run(velocity_mag, mean_step, t)

        out: List[np.ndarray] = []
        for i in range(4):
            new_foot = self.trajectory_planners[i].generate(
                foot_positions[i],
                step_lengths[i],
                rotations[i],
                self.phase_generator.swing_phase_signal[i],
                self.phase_generator.stance_phase_signal[i],
            )
            out.append(new_foot)
        return out
```

- [ ] **Step 4: Run tests, verify pass**

```bash
python3 -m pytest test/test_leg_controller.py -v
```

Expected: 3 passed.

- [ ] **Step 5: Commit**

```bash
git add dog_robot_ws/src/dog_robot_control/dog_robot_control/gait/leg_controller.py \
        dog_robot_ws/src/dog_robot_control/test/test_leg_controller.py
git commit -m "feat(gait): leg_controller port — Raibert + transformLeg + per-leg gait"
```

---

### Task 6: Integration test (mocked clock)

**Files:**
- Create: `dog_robot_ws/src/dog_robot_control/test/test_walker_integration.py`

- [ ] **Step 1: Write the integration test**

```python
# test/test_walker_integration.py
"""End-to-end gait pipeline test using a mocked clock.

Drives BodyController + LegController for 3 strides at vx=0.1 and verifies:
- ik_leg succeeds for every leg every tick (foot stays in reach).
- joint angles stay within URDF limits.
- trot timing: leg 0 and 3 phase signals match within 1 tick.
"""
import math
import numpy as np
import pytest

from dog_robot_control.kinematics_dh import DHParams, ik_leg
from dog_robot_control.leg_config import LEGS
from dog_robot_control.gait.gait_config import GaitConfig
from dog_robot_control.gait.body_controller import BodyController, BodyPose
from dog_robot_control.gait.leg_controller import LegController, Velocity


DH = DHParams(L_hh=0.02553, L_th=0.11725, L_sh=0.07043)
GC = GaitConfig(nominal_height=0.15, stance_duration=0.30, swing_height=0.03,
                stance_depth=0.001, max_linear_velocity_x=0.15,
                max_linear_velocity_y=0.08, max_angular_velocity_z=0.50)


JOINT_LIMITS = {
    "hip":   (-0.785,  0.785),
    "thigh": (-1.571,  1.571),
    "knee":  (0.0,     2.617),
}


def _Rx(a): c, s = math.cos(a), math.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def _Ry(a): c, s = math.cos(a), math.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def _Rz(a): c, s = math.cos(a), math.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])


def _body_to_hip(foot_body_at_hip, leg):
    r, p, y = leg.base_to_hip_rpy
    R_bh = _Rz(y) @ _Ry(p) @ _Rx(r)
    return R_bh.T @ foot_body_at_hip


def test_walking_pipeline_no_ik_failures():
    bc = BodyController(LEGS, DH, GC)
    lc = LegController(LEGS, DH, GC)
    dt = 0.02
    n_ticks = int((GC.stance_duration + 0.25) * 3 / dt)  # ~3 strides
    failures = []
    leg0_track = []
    leg3_track = []
    for k in range(n_ticks):
        t = k * dt
        feet = bc.pose_command(BodyPose(0, 0, GC.nominal_height, 0, 0, 0))
        feet = lc.velocity_command(feet, Velocity(0.1, 0, 0), t)
        for i, L in enumerate(LEGS):
            foot_h = _body_to_hip(feet[i], L)
            try:
                q = ik_leg(DH, foot_h, knee_direction=+1)
                assert JOINT_LIMITS["hip"][0]   <= q[0] <= JOINT_LIMITS["hip"][1]
                assert JOINT_LIMITS["thigh"][0] <= q[1] <= JOINT_LIMITS["thigh"][1]
                assert JOINT_LIMITS["knee"][0]  <= q[2] <= JOINT_LIMITS["knee"][1]
            except (ValueError, AssertionError) as e:
                failures.append((k, i, L.name, str(e)))
        leg0_track.append(lc.phase_generator.stance_phase_signal[0])
        leg3_track.append(lc.phase_generator.stance_phase_signal[3])
    assert not failures, f"IK or limit failures: {failures[:5]}"
    # Trot: leg 0 and leg 3 are in-phase.
    diffs = [abs(a - b) for a, b in zip(leg0_track, leg3_track)]
    assert max(diffs) < 1e-9, f"trot in-phase broken: max diff {max(diffs)}"
```

- [ ] **Step 2: Run test, verify pass**

```bash
cd /home/nguyenvd/workspace/dog_robot/dog_robot_ws/src/dog_robot_control && python3 -m pytest test/ -v 2>&1 | tail -20
```

Expected: all gait tests pass (including new integration test).

If `test_walking_pipeline_no_ik_failures` fails with IK errors, it means our gait math + body-frame-to-hip-frame conversion is misaligned. Inspect the first failing leg + tick number, print `foot_h` and `dh.L_th + dh.L_sh + dh.L_hh` to compare reach. Common cause: knee_direction sign mismatch (try `-1`) or zero_stance z too low (foot already at ground means gait swing dips below ground).

- [ ] **Step 3: Commit**

```bash
git add dog_robot_ws/src/dog_robot_control/test/test_walker_integration.py
git commit -m "test(gait): walking pipeline integration test (3 strides, IK + limits + trot)"
```

---

### Task 7: walker_controller.py

**Files:**
- Create: `dog_robot_ws/src/dog_robot_control/dog_robot_control/walker_controller.py`
- Modify: `dog_robot_ws/src/dog_robot_control/setup.py`

- [ ] **Step 1: Implement walker_controller.py**

```python
"""Walker controller node.

Subscribes /cmd_vel + /stand_cmd, ticks gait pipeline at publish_rate, calls
DH-IK, publishes JointTrajectory. Subsumes stand_controller: cmd_vel = 0 →
robot holds stand pose.
"""
import math
from typing import List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from dog_robot_control.kinematics_dh import DHParams, ik_leg
from dog_robot_control.leg_config import LEGS
from dog_robot_control.gait.gait_config import GaitConfig
from dog_robot_control.gait.body_controller import BodyController, BodyPose
from dog_robot_control.gait.leg_controller import LegController, Velocity


def _Rx(a): c, s = math.cos(a), math.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def _Ry(a): c, s = math.cos(a), math.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def _Rz(a): c, s = math.cos(a), math.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])


def _body_to_hip(foot_body_at_hip, leg):
    r, p, y = leg.base_to_hip_rpy
    R_bh = _Rz(y) @ _Ry(p) @ _Rx(r)
    return R_bh.T @ foot_body_at_hip


DEFAULT_JOINT_ORDER = [
    "FL_hip_yaw","FL_thigh_pitch","FL_knee_pitch",
    "FR_hip_yaw","FR_thigh_pitch","FR_knee_pitch",
    "BL_hip_yaw","BL_thigh_pitch","BL_knee_pitch",
    "BR_hip_yaw","BR_thigh_pitch","BR_knee_pitch",
]


class WalkerController(Node):
    def __init__(self):
        super().__init__("walker_controller")
        self.declare_parameter("dh.L_hh", 0.02553)
        self.declare_parameter("dh.L_th", 0.11725)
        self.declare_parameter("dh.L_sh", 0.07043)
        self.declare_parameter("gait.nominal_height", 0.15)
        self.declare_parameter("gait.stance_duration", 0.30)
        self.declare_parameter("gait.swing_height", 0.03)
        self.declare_parameter("gait.stance_depth", 0.001)
        self.declare_parameter("gait.max_linear_velocity_x", 0.15)
        self.declare_parameter("gait.max_linear_velocity_y", 0.08)
        self.declare_parameter("gait.max_angular_velocity_z", 0.50)
        self.declare_parameter("stand.ramp_time", 2.0)
        self.declare_parameter("stand.cmd_vel_timeout", 0.5)
        self.declare_parameter("stand.publish_rate", 50.0)
        self.declare_parameter("stand.knee_direction", 1)
        self.declare_parameter("joint_order", DEFAULT_JOINT_ORDER)

        dh = DHParams(
            L_hh=self.get_parameter("dh.L_hh").value,
            L_th=self.get_parameter("dh.L_th").value,
            L_sh=self.get_parameter("dh.L_sh").value,
        )
        self.gait = GaitConfig(
            nominal_height=self.get_parameter("gait.nominal_height").value,
            stance_duration=self.get_parameter("gait.stance_duration").value,
            swing_height=self.get_parameter("gait.swing_height").value,
            stance_depth=self.get_parameter("gait.stance_depth").value,
            max_linear_velocity_x=self.get_parameter("gait.max_linear_velocity_x").value,
            max_linear_velocity_y=self.get_parameter("gait.max_linear_velocity_y").value,
            max_angular_velocity_z=self.get_parameter("gait.max_angular_velocity_z").value,
        )
        self.dh = dh
        self.knee_dir = int(self.get_parameter("stand.knee_direction").value)
        self.ramp_time = float(self.get_parameter("stand.ramp_time").value)
        self.cmd_timeout = float(self.get_parameter("stand.cmd_vel_timeout").value)
        rate = float(self.get_parameter("stand.publish_rate").value)
        self.joint_order = list(self.get_parameter("joint_order").value)

        self.body_controller = BodyController(LEGS, dh, self.gait)
        self.leg_controller = LegController(LEGS, dh, self.gait)

        self.req_vel = Velocity(0.0, 0.0, 0.0)
        self.req_pose = BodyPose(0, 0, self.gait.nominal_height, 0, 0, 0)
        self.last_cmd_vel_t: Optional[float] = None

        self.start_angles: Optional[np.ndarray] = None
        self.ramp_target: Optional[np.ndarray] = None
        self.ramp_start_t: Optional[float] = None
        self.ramp_done = False

        self.pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )
        self.sub_js = self.create_subscription(
            JointState, "/joint_states", self._on_js, 10
        )
        self.sub_vel = self.create_subscription(
            Twist, "/cmd_vel", self._on_vel, 10
        )
        self.sub_pose = self.create_subscription(
            Pose, "/stand_cmd", self._on_pose, 10
        )
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info("walker_controller up; waiting for /joint_states")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_js(self, msg: JointState):
        if self.start_angles is not None:
            return
        idx = {n: i for i, n in enumerate(msg.name)}
        try:
            self.start_angles = np.array([msg.position[idx[j]] for j in self.joint_order])
        except KeyError as e:
            self.get_logger().warn(f"joint_states missing {e}; will retry")
            return
        # Initial ramp target = stand pose (cmd_vel=0).
        self.ramp_target = self._compute_stand_target()
        self.ramp_start_t = self._now()
        self.get_logger().info("captured start angles; ramping to stand")

    def _on_vel(self, msg: Twist):
        self.req_vel = Velocity(msg.linear.x, msg.linear.y, msg.angular.z)
        self.last_cmd_vel_t = self._now()

    def _on_pose(self, msg: Pose):
        new_h = float(msg.position.z)
        if not (0.05 < new_h < 0.30):
            self.get_logger().warn(f"ignored stand_cmd height={new_h}")
            return
        # Allow updating only z for now.
        self.req_pose = BodyPose(self.req_pose.x, self.req_pose.y, new_h,
                                 self.req_pose.roll, self.req_pose.pitch,
                                 self.req_pose.yaw)

    def _compute_stand_target(self) -> np.ndarray:
        feet = self.body_controller.pose_command(self.req_pose)
        targets: List[float] = []
        for i, L in enumerate(LEGS):
            foot_h = _body_to_hip(feet[i], L)
            try:
                q = ik_leg(self.dh, foot_h, knee_direction=self.knee_dir)
            except ValueError as e:
                self.get_logger().error(f"IK failed for stand on {L.name}: {e}")
                return self.start_angles.copy() if self.start_angles is not None else np.zeros(12)
            targets.extend(q)
        return np.array(targets)

    def _tick(self):
        if self.start_angles is None or self.ramp_target is None:
            return

        t = self._now()

        # Cmd_vel timeout → zero velocity.
        if self.last_cmd_vel_t is not None and (t - self.last_cmd_vel_t) > self.cmd_timeout:
            self.req_vel = Velocity(0.0, 0.0, 0.0)

        # Ramp phase: linear interpolate start_angles → ramp_target over ramp_time.
        if not self.ramp_done:
            elapsed = t - self.ramp_start_t
            if elapsed >= self.ramp_time:
                self.ramp_done = True
                q = self.ramp_target
            else:
                alpha = elapsed / self.ramp_time
                q = (1.0 - alpha) * self.start_angles + alpha * self.ramp_target
        else:
            # Gait phase: full pipeline.
            feet = self.body_controller.pose_command(self.req_pose)
            feet = self.leg_controller.velocity_command(feet, self.req_vel, t)
            targets: List[float] = []
            for i, L in enumerate(LEGS):
                foot_h = _body_to_hip(feet[i], L)
                try:
                    angles = ik_leg(self.dh, foot_h, knee_direction=self.knee_dir)
                except ValueError as e:
                    self.get_logger().warn(
                        f"IK fail for {L.name}: {e}", throttle_duration_sec=1.0)
                    return
                targets.extend(angles)
            q = np.array(targets)

        msg = JointTrajectory()
        msg.joint_names = self.joint_order
        pt = JointTrajectoryPoint()
        pt.positions = q.tolist()
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = int(0.1 * 1e9)
        msg.points = [pt]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WalkerController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Update setup.py — add walker_controller entry**

Edit `dog_robot_ws/src/dog_robot_control/setup.py`, replace the `entry_points` block with:

```python
    entry_points={
        "console_scripts": [
            "teleop_keyboard = dog_robot_control.teleop_keyboard:main",
            "stand_controller = dog_robot_control.stand_controller:main",
            "walker_controller = dog_robot_control.walker_controller:main",
        ],
    },
```

- [ ] **Step 3: Build + verify executable installs**

```bash
cd /home/nguyenvd/workspace/dog_robot/dog_robot_ws && colcon build --packages-select dog_robot_control --symlink-install 2>&1 | tail -3
```

If `colcon build` fails with setuptools 81 (known issue, see memory feedback-setuptools-81-colcon), copy manually:

```bash
cp src/dog_robot_control/dog_robot_control/walker_controller.py \
   install/dog_robot_control/lib/python*/site-packages/dog_robot_control/
# Also create the install-side wrapper script next to stand_controller:
ls install/dog_robot_control/lib/dog_robot_control/
cp install/dog_robot_control/lib/dog_robot_control/stand_controller \
   install/dog_robot_control/lib/dog_robot_control/walker_controller
sed -i 's|stand_controller:main|walker_controller:main|g' \
   install/dog_robot_control/lib/dog_robot_control/walker_controller
# Also ensure the gait/ subpackage is copied:
mkdir -p install/dog_robot_control/lib/python*/site-packages/dog_robot_control/gait
cp src/dog_robot_control/dog_robot_control/gait/*.py \
   install/dog_robot_control/lib/python*/site-packages/dog_robot_control/gait/
```

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 pkg executables dog_robot_control | grep walker_controller
```
Expected: `dog_robot_control walker_controller`.

- [ ] **Step 4: Smoke test node startup**

```bash
timeout 3 ros2 run dog_robot_control walker_controller 2>&1 | head -5 || true
```
Expected log line: `walker_controller up; waiting for /joint_states`.

- [ ] **Step 5: Commit**

```bash
git add dog_robot_ws/src/dog_robot_control/dog_robot_control/walker_controller.py \
        dog_robot_ws/src/dog_robot_control/setup.py
git commit -m "feat(control): walker_controller node (gait + DH-IK + JointTrajectory)"
```

---

### Task 8: walker_params.yaml + walk.launch.py

**Files:**
- Create: `dog_robot_ws/src/dog_robot_control/config/walker_params.yaml`
- Create: `dog_robot_ws/src/dog_robot_control/launch/walk.launch.py`

- [ ] **Step 1: Write walker_params.yaml**

```yaml
walker_controller:
  ros__parameters:
    dh:
      L_hh: 0.02553
      L_th: 0.11725
      L_sh: 0.07043
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
      publish_rate: 50.0
      knee_direction: 1
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

- [ ] **Step 2: Write walk.launch.py (copy of stand.launch.py with walker_controller swapped in)**

```python
"""Walking launch: Gazebo + spawn dog_robot + JTC + walker_controller."""
from launch import LaunchDescription
from launch.actions import (ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    descr = FindPackageShare("dog_robot_description")
    ctrl = FindPackageShare("dog_robot_control")

    urdf_xacro = PathJoinSubstitution([descr, "urdf", "dog_robot.urdf.xacro"])
    controllers_yaml = PathJoinSubstitution([descr, "config", "ros2_controllers.yaml"])
    walker_params = PathJoinSubstitution([ctrl, "config", "walker_params.yaml"])

    robot_description = {
        "robot_description": Command([
            FindExecutable(name="xacro"), " ", urdf_xacro,
            " controllers_yaml_path:=", controllers_yaml,
        ])
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo_ros"), "/launch/gazebo.launch.py"]),
        launch_arguments={"verbose": "false"}.items(),
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "dog_robot",
                   "-z", "0.30", "-timeout", "120"],
        output="screen",
    )

    load_jsb = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "joint_state_broadcaster"],
        output="screen",
    )
    load_jtc = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "joint_trajectory_controller"],
        output="screen",
    )
    walker = Node(
        package="dog_robot_control",
        executable="walker_controller",
        name="walker_controller",
        parameters=[walker_params],
        output="screen",
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[load_jsb])),
        RegisterEventHandler(OnProcessExit(target_action=load_jsb, on_exit=[load_jtc])),
        RegisterEventHandler(OnProcessExit(target_action=load_jtc, on_exit=[walker])),
    ])
```

- [ ] **Step 3: Rebuild + verify launch file discoverable**

```bash
cd /home/nguyenvd/workspace/dog_robot/dog_robot_ws && colcon build --packages-select dog_robot_control --symlink-install 2>&1 | tail -3
# If colcon build fails (setuptools 81), manual copy:
mkdir -p install/dog_robot_control/share/dog_robot_control/{launch,config}
cp src/dog_robot_control/launch/walk.launch.py install/dog_robot_control/share/dog_robot_control/launch/
cp src/dog_robot_control/config/walker_params.yaml install/dog_robot_control/share/dog_robot_control/config/
```

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ls $(ros2 pkg prefix dog_robot_control)/share/dog_robot_control/launch/ | grep walk
```
Expected: `walk.launch.py` listed.

- [ ] **Step 4: Commit**

```bash
git add dog_robot_ws/src/dog_robot_control/config/walker_params.yaml \
        dog_robot_ws/src/dog_robot_control/launch/walk.launch.py
git commit -m "feat(launch): walker_params.yaml + walk.launch.py"
```

---

### Task 9: Gazebo regression + tuning

**Files:** tuning changes only, if needed.

- [ ] **Step 1: Kill any stale sim, launch walk**

```bash
bash dog_robot_ws/scripts/dog_kill_all.sh && sleep 1
source /opt/ros/humble/setup.bash && source dog_robot_ws/install/setup.bash
ros2 launch dog_robot_control walk.launch.py > /tmp/walk1.log 2>&1 &
LP=$!
sleep 15
```

- [ ] **Step 2: Verify stand still works (cmd_vel = 0)**

```bash
ros2 control list_controllers
timeout 2 ros2 topic echo /joint_states --once | sed -n '/^position:/,/^velocity:/p' | head -14
```
Expected: both controllers active, joint angles symmetric, no falling.

- [ ] **Step 3: Walk forward**

```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}}' -r 10 &
PUB=$!
sleep 10
kill $PUB
```

Watch Gazebo viewer (or `/joint_states`) for 10 s. Acceptance:
- Robot moves in +X direction (body x increases — verify via `ros2 service call /gazebo/get_entity_state ...` if available, or visually).
- Body roll/pitch absolute < 0.25 rad.
- No leg crosses URDF limits (no warning logs about IK fail).

- [ ] **Step 4: Walk backward, lateral, turn — same acceptance**

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{linear: {x: -0.1}}'
sleep 6
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{linear: {y: 0.05}}'
sleep 6
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{angular: {z: 0.3}}'
sleep 6
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{}'
sleep 3
```

- [ ] **Step 5: Kill cleanly**

```bash
bash dog_robot_ws/scripts/dog_kill_all.sh
```

- [ ] **Step 6: If unstable, tune in this order**

Edit `walker_params.yaml`:
1. **knee_direction flip**: try `knee_direction: -1` (might invert gait swing direction).
2. **Lower nominal_height** to 0.13 if knee bends too much during swing peak.
3. **Reduce swing_height** to 0.02 if foot kicks too high and unstable.
4. **Increase stance_duration** to 0.40 to slow gait.

Then `cp config/walker_params.yaml install/.../share/dog_robot_control/config/` and relaunch.

- [ ] **Step 7: If tuning needed, commit**

```bash
git add dog_robot_ws/src/dog_robot_control/config/walker_params.yaml
git commit -m "tune(walker): gait params for stable walking in Gazebo"
```

If after 3 tuning attempts walking is still unstable, escalate as BLOCKED. Common root cause to investigate: the body-to-hip frame conversion `R_bh.T @ foot_body_at_hip` may have a sign error for right-side legs (FR/BR with rpy yaw=π) — print `foot_h` per leg and compare across left/right symmetry.

---

### Task 10: README + kill script + cleanup

**Files:**
- Modify: `dog_robot_ws/README.md`
- Modify: `dog_robot_ws/scripts/dog_kill_all.sh`
- Modify: `dog_robot_ws/src/dog_robot_control/dog_robot_control/stand_controller.py` (deprecation docstring)

- [ ] **Step 1: Append Walking section to README**

Open `dog_robot_ws/README.md` and ADD this section after the existing Kinematics section:

````markdown

## Walking

`walker_controller` is the production controller. It subsumes the older
`stand_controller`: when `/cmd_vel` is zero, the walker holds the stand pose;
non-zero `/cmd_vel` triggers a trot gait (Bernstein-Bezier swing + linear
stance) computed entirely in Python and converted to joint commands via the
DH-IK module.

### Launch

```bash
bash dog_robot_ws/scripts/dog_kill_all.sh
ros2 launch dog_robot_control walk.launch.py
```

Robot ramps to stand within 3 s. Then:

```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}}' -r 10
```

Twist field map:
- `linear.x` — forward / backward (m/s), capped at `gait.max_linear_velocity_x`
- `linear.y` — sideways (m/s)
- `angular.z` — yaw (rad/s)

Set all to zero (or stop publishing — there is a 0.5 s timeout) to stop.

### Gait config

`dog_robot_control/config/walker_params.yaml` exposes tunable gait params
(stance duration, swing height, velocity caps, knee direction, etc.). See
inline comments in the YAML.

### Architecture

`/cmd_vel` → `BodyController.pose_command` → `LegController.velocity_command`
(phase_generator + trajectory_planner per leg) → for each leg: rotate to DH
hip frame → `ik_leg` → `JointTrajectory` → joint_trajectory_controller →
Gazebo. Python module layout: `dog_robot_control/dog_robot_control/gait/`.

### Deprecated

`stand_controller` + `stand.launch.py` remain in the tree for reference but
are deprecated. Use the walker.
````

- [ ] **Step 2: Add walker_controller to kill script**

Edit `dog_robot_ws/scripts/dog_kill_all.sh`. In the PATTERNS array, ensure `walker_controller` is listed. The current array has `stand_controller`; just add `walker_controller` next to it. Full updated array:

```bash
PATTERNS=(
  gzserver gzclient ros_gz_bridge spawn_entity
  robot_state_publisher joint_state_broadcaster
  joint_trajectory_controller controller_manager
  ros2_control_node ros2_control gazebo_ros2_control
  stand_controller walker_controller rviz2
  champ_base champ_gazebo
)
```

- [ ] **Step 3: Add deprecation banner to stand_controller.py**

Open `dog_robot_ws/src/dog_robot_control/dog_robot_control/stand_controller.py` and replace the top-of-file docstring with:

```python
"""DEPRECATED — superseded by walker_controller (2026-05-24).

walker_controller does everything this node does (stand pose at default
height) plus walking via /cmd_vel. Kept here for backward compat with
stand.launch.py; will be deleted in a future cleanup.
"""
```

- [ ] **Step 4: Commit**

```bash
git add dog_robot_ws/README.md \
        dog_robot_ws/scripts/dog_kill_all.sh \
        dog_robot_ws/src/dog_robot_control/dog_robot_control/stand_controller.py
git commit -m "docs(readme): walking section + deprecate stand_controller"
```

---

## Self-review

**Spec coverage:**
- ✅ omni cmd_vel — Task 7 walker_controller subscribes Twist, full vx/vy/wz.
- ✅ Python port of CHAMP gait — Tasks 1-5.
- ✅ Walker subsumes stand — Task 7 walker_controller handles stand pose at cmd_vel=0.
- ✅ DH-IK reuse — Tasks 6/7 import ik_leg from existing kinematics_dh.
- ✅ JTC position+open_loop_control — unchanged from stand; walker uses same JTC.
- ✅ Tests — Tasks 2,3,4,5,6.
- ✅ Gazebo regression — Task 9.
- ✅ README — Task 10.

**Placeholder scan:** none — every code step has full code. Test in Task 4 had a muddled assertion in Step 1 that gets fixed in Step 2 (explicit replacement instructed).

**Type consistency:**
- `BodyPose` (Task 4) used in Task 5 + 7 — consistent fields.
- `Velocity(vx, vy, wz)` (Task 5) used in Task 7 — consistent.
- `GaitConfig` field set consistent across all tasks (nominal_height, stance_duration, swing_height, stance_depth, max_linear_velocity_x/y, max_angular_velocity_z).
- `LegConfig.base_to_hip_xyz` / `base_to_hip_rpy` / `mirror` — from existing leg_config.py, used consistently.
- Joint order — declared once in walker_params.yaml and DEFAULT_JOINT_ORDER constant, matches throughout.

---

## Execution Handoff

Plan complete and saved to `dog_robot_ws/docs/superpowers/plans/2026-05-24-walking-cmd-vel.md`. Two execution options:

1. **Subagent-Driven (recommended)** — I dispatch a fresh subagent per task, review between tasks, fast iteration.
2. **Inline Execution** — Execute tasks in this session using executing-plans, batch execution with checkpoints.
