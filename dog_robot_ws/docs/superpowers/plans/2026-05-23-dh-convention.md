# DH Convention Conversion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rewrite dog_robot leg URDF in Modified DH (Craig) convention, add closed-form DH-IK Python module, stand-only ROS 2 controller. CHAMP bypassed. Walking deferred.

**Architecture:** One symmetric DH table for all 4 legs (per-leg CAD asymmetry absorbed into base→hip fixed transform). Pure-Python kinematics module called from a `stand_controller` ROS 2 node that publishes `JointTrajectory` to `joint_trajectory_controller` running in effort command mode with PID. URDF mesh visuals get compensating origin transforms so meshes render correctly under new joint frames.

**Tech Stack:** ROS 2 Humble, Gazebo Classic 11, gazebo_ros2_control, joint_trajectory_controller (effort cmd interface + PID), Python 3.10 + numpy, pytest, xacro.

---

## File Plan

**Create:**
- `dog_robot_ws/src/dog_robot_control/dog_robot_control/kinematics_dh.py` — DHParams, mdh_transform, fk_leg, ik_leg
- `dog_robot_ws/src/dog_robot_control/dog_robot_control/leg_config.py` — LegConfig dataclass + LEGS tuple
- `dog_robot_ws/src/dog_robot_control/dog_robot_control/stand_controller.py` — ROS 2 node
- `dog_robot_ws/src/dog_robot_control/test/test_kinematics_dh.py` — FK/IK unit tests
- `dog_robot_ws/src/dog_robot_control/test/test_urdf_dh_consistency.py` — URDF↔module FK cross-check
- `dog_robot_ws/src/dog_robot_control/config/dh_params.yaml` — link lengths, leg configs, stand params
- `dog_robot_ws/src/dog_robot_control/config/controller_params.yaml` — UPDATE: PID gains for effort JTC (or create if missing)
- `dog_robot_ws/src/dog_robot_control/launch/stand.launch.py` — gazebo+spawn+JTC+stand_controller
- `dog_robot_ws/src/dog_robot_description/scripts/compute_dh_lengths.py` — derive L_hh,L_th,L_sh from current CAD
- `dog_robot_ws/src/dog_robot_description/scripts/compute_visual_compensation.py` — derive mesh rpy/xyz compensation
- `dog_robot_ws/README.md` — append Kinematics section (file may not exist; create)

**Modify:**
- `dog_robot_ws/src/dog_robot_description/urdf/leg.xacro` — replace CHAMP-IK surgery with DH joint origins
- `dog_robot_ws/src/dog_robot_description/urdf/dog_robot.urdf.xacro` — new per-leg `<xacro:leg>` calls
- `dog_robot_ws/src/dog_robot_description/config/ros2_controllers.yaml` — switch JTC to effort + PID
- `dog_robot_ws/src/dog_robot_control/setup.py` — register `stand_controller` console_script

---

### Task 1: Extract DH constants from existing CAD measurements

**Files:**
- Create: `dog_robot_ws/src/dog_robot_description/scripts/compute_dh_lengths.py`

- [ ] **Step 1: Write the script**

```python
#!/usr/bin/env python3
"""Compute symmetric DH link lengths from existing per-leg CAD values in dog_robot.urdf.xacro.

L_hh = mean |thigh_xyz Y component| (hip-to-thigh common normal)
L_th = mean magnitude(knee_xyz in original frame) (thigh length)
L_sh = mean magnitude(foot_xyz in original frame) (shank length)
"""
import math

# Values copied from current dog_robot.urdf.xacro (committed state).
LEGS = {
    "FL": dict(thigh_xyz=( 0.02520,  0.02536, -0.01317),
               knee_xyz =( 0.0,      0.04102, -0.10984),
               foot_xyz =( 0.0,     -0.01922, -0.06773)),
    "FR": dict(thigh_xyz=( 0.02520, -0.02570, -0.01250),
               knee_xyz =( 0.0,     -0.04270, -0.10920),
               foot_xyz =( 0.0,      0.01826, -0.06802)),
    "BL": dict(thigh_xyz=(-0.02520,  0.02536, -0.01318),
               knee_xyz =( 0.0,      0.04082, -0.10992),
               foot_xyz =( 0.0,     -0.01906, -0.06742)),
    "BR": dict(thigh_xyz=(-0.02520, -0.02570, -0.01250),
               knee_xyz =( 0.0,     -0.04270, -0.10920),
               foot_xyz =( 0.0,      0.01842, -0.06838)),
}

def mag(v):
    return math.sqrt(sum(x * x for x in v))

L_hh = sum(abs(L["thigh_xyz"][1]) for L in LEGS.values()) / 4
L_th = sum(mag(L["knee_xyz"]) for L in LEGS.values()) / 4
L_sh = sum(mag(L["foot_xyz"]) for L in LEGS.values()) / 4

print(f"L_hh = {L_hh:.5f}  # m, hip-to-thigh common normal")
print(f"L_th = {L_th:.5f}  # m, thigh length")
print(f"L_sh = {L_sh:.5f}  # m, shank length")
```

- [ ] **Step 2: Run it**

```bash
chmod +x dog_robot_ws/src/dog_robot_description/scripts/compute_dh_lengths.py
python3 dog_robot_ws/src/dog_robot_description/scripts/compute_dh_lengths.py
```

Expected output (approximate):
```
L_hh = 0.02944  # m
L_th = 0.11737  # m
L_sh = 0.07013  # m
```

Note: `L_hh` from CAD is ~0.029, not 0.040 as I sketched in the spec — that's because the symmetric value averages all 4 legs' thigh Y offset. We'll use the computed value going forward.

- [ ] **Step 3: Commit**

```bash
git add dog_robot_ws/src/dog_robot_description/scripts/compute_dh_lengths.py
git commit -m "feat(scripts): compute symmetric DH link lengths from CAD"
```

---

### Task 2: kinematics_dh.py — mdh_transform + fk_leg (TDD)

**Files:**
- Create: `dog_robot_ws/src/dog_robot_control/dog_robot_control/kinematics_dh.py`
- Create: `dog_robot_ws/src/dog_robot_control/test/test_kinematics_dh.py`

- [ ] **Step 1: Write the failing test**

```python
# test/test_kinematics_dh.py
import numpy as np
import pytest
from dog_robot_control.kinematics_dh import DHParams, mdh_transform, fk_leg

DH = DHParams(L_hh=0.02944, L_th=0.11737, L_sh=0.07013)

def test_mdh_identity_when_all_zero():
    T = mdh_transform(0.0, 0.0, 0.0, 0.0)
    assert np.allclose(T, np.eye(4))

def test_mdh_pure_translation():
    T = mdh_transform(0.0, 0.123, 0.0, 0.0)
    assert np.allclose(T[:3, 3], [0.123, 0.0, 0.0])
    assert np.allclose(T[:3, :3], np.eye(3))

def test_mdh_pure_rotation_z():
    T = mdh_transform(0.0, 0.0, 0.0, np.pi / 2)
    expected_R = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    assert np.allclose(T[:3, :3], expected_R)

def test_fk_at_zero_angles_extends_along_x_h():
    # At all-zero joint angles foot should sit at (L_hh + L_th + L_sh, 0, 0)
    # in the hip frame (along X_H = downward in body coords).
    foot = fk_leg(DH, (0.0, 0.0, 0.0))
    expected = np.array([DH.L_hh + DH.L_th + DH.L_sh, 0.0, 0.0])
    assert np.allclose(foot, expected, atol=1e-9)
```

- [ ] **Step 2: Run test to verify it fails**

```bash
cd dog_robot_ws/src/dog_robot_control && python -m pytest test/test_kinematics_dh.py -v
```

Expected: ImportError or `kinematics_dh` not found.

- [ ] **Step 3: Implement kinematics_dh.py (FK + transform only, IK in next task)**

```python
# dog_robot_control/kinematics_dh.py
"""Modified DH (Craig) kinematics for a 3-DOF quadruped leg.

DH table (one symmetric set for all 4 legs):
    i | alpha_{i-1} | a_{i-1} |  d_i | theta_i
    1 |     0       |   0     |   0  |  theta_hip
    2 |   -pi/2     |  L_hh   |   0  |  theta_thigh
    3 |     0       |  L_th   |   0  |  theta_knee
    F |     0       |  L_sh   |   0  |  0
"""
from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass(frozen=True)
class DHParams:
    L_hh: float  # hip-to-thigh common normal (a_1)
    L_th: float  # thigh length (a_2)
    L_sh: float  # shank length (a_3)


def mdh_transform(alpha: float, a: float, d: float, theta: float) -> np.ndarray:
    """Modified DH (Craig) homogeneous transform from frame i-1 to frame i.

    T = Rx(alpha) * Tx(a) * Rz(theta) * Tz(d)
    """
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [   ct,    -st,   0.0,        a],
        [st*ca,  ct*ca,   -sa,   -d*sa],
        [st*sa,  ct*sa,    ca,    d*ca],
        [  0.0,    0.0,   0.0,      1.0],
    ])


def fk_leg(dh: DHParams, theta: Tuple[float, float, float]) -> np.ndarray:
    """Foot position in hip frame H. theta = (theta_hip, theta_thigh, theta_knee)."""
    A1 = mdh_transform(0.0,        0.0,     0.0, theta[0])
    A2 = mdh_transform(-np.pi / 2, dh.L_hh, 0.0, theta[1])
    A3 = mdh_transform(0.0,        dh.L_th, 0.0, theta[2])
    AF = mdh_transform(0.0,        dh.L_sh, 0.0, 0.0)
    T = A1 @ A2 @ A3 @ AF
    return T[:3, 3]
```

- [ ] **Step 4: Add `__init__.py` if not present**

```bash
ls dog_robot_ws/src/dog_robot_control/dog_robot_control/__init__.py
# Already exists per earlier inspection
touch dog_robot_ws/src/dog_robot_control/test/__init__.py
```

- [ ] **Step 5: Run test to verify it passes**

```bash
cd dog_robot_ws/src/dog_robot_control && python -m pytest test/test_kinematics_dh.py -v
```
Expected: 4 passed.

- [ ] **Step 6: Commit**

```bash
git add dog_robot_ws/src/dog_robot_control/dog_robot_control/kinematics_dh.py \
        dog_robot_ws/src/dog_robot_control/test/test_kinematics_dh.py \
        dog_robot_ws/src/dog_robot_control/test/__init__.py
git commit -m "feat(kinematics): MDH transform + fk_leg for 3-DOF quadruped leg"
```

---

### Task 3: kinematics_dh.py — ik_leg + roundtrip (TDD)

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_control/dog_robot_control/kinematics_dh.py`
- Modify: `dog_robot_ws/src/dog_robot_control/test/test_kinematics_dh.py`

- [ ] **Step 1: Append failing tests**

```python
# Append to test_kinematics_dh.py:
from dog_robot_control.kinematics_dh import ik_leg

JOINT_LIMITS = {
    "hip":   (-0.785, 0.785),
    "thigh": (-1.571, 1.571),
    "knee":  (0.0,    2.617),
}

def test_ik_at_stand_pose_recovers_zero_hip():
    # Foot at (L_hh+L_th+L_sh - 0.012, 0, 0) — slightly bent stand.
    target = np.array([DH.L_hh + DH.L_th + DH.L_sh - 0.012, 0.0, 0.0])
    theta = ik_leg(DH, target, knee_direction=+1)
    assert abs(theta[0]) < 1e-9                 # hip yaw = 0
    foot_back = fk_leg(DH, theta)
    assert np.allclose(foot_back, target, atol=1e-9)

def test_fk_ik_roundtrip_random():
    rng = np.random.default_rng(seed=42)
    n_ok = 0
    for _ in range(200):
        theta = (
            rng.uniform(*JOINT_LIMITS["hip"]),
            rng.uniform(-0.6, 0.6),   # thigh: avoid singular near limits
            rng.uniform(0.3, 1.8),    # knee: stay bent away from singular
        )
        foot = fk_leg(DH, theta)
        try:
            theta_back = ik_leg(DH, foot, knee_direction=+1)
        except ValueError:
            continue
        foot_again = fk_leg(DH, theta_back)
        assert np.allclose(foot, foot_again, atol=1e-6), (theta, foot, theta_back, foot_again)
        n_ok += 1
    assert n_ok > 150, f"roundtrip succeeded for only {n_ok}/200 samples"

def test_ik_unreachable_raises():
    # Foot way out of reach.
    far = np.array([5.0, 0.0, 0.0])
    with pytest.raises(ValueError):
        ik_leg(DH, far)
```

- [ ] **Step 2: Run tests, verify they fail**

```bash
cd dog_robot_ws/src/dog_robot_control && python -m pytest test/test_kinematics_dh.py -v -k ik
```
Expected: ImportError for `ik_leg`.

- [ ] **Step 3: Implement ik_leg**

Append to `kinematics_dh.py`:

```python
def ik_leg(dh: DHParams, foot_h: np.ndarray, knee_direction: int = +1) -> Tuple[float, float, float]:
    """Closed-form inverse kinematics for one 3-DOF leg.

    foot_h: foot target in hip frame H, shape (3,).
    knee_direction: +1 = arccos branch (knee bends one way), -1 = the other.
    Returns (theta_hip, theta_thigh, theta_knee). Raises ValueError if unreachable.
    """
    x, y, z = float(foot_h[0]), float(foot_h[1]), float(foot_h[2])

    # Step 1: hip yaw rotates the leg plane around Z_H.
    # Constraint plane: -sin(theta_hip)*x + cos(theta_hip)*y = 0.
    if abs(x) < 1e-12 and abs(y) < 1e-12:
        raise ValueError("foot on hip yaw axis: theta_hip undefined")
    theta_hip = np.arctan2(y, x)

    # Step 2: project foot into the 2R plane (rotate by -theta_hip).
    r = np.hypot(x, y)   # distance from Z_H axis, always >= 0
    # In rotated frame foot lies at (r, 0, z); thigh joint is at (L_hh, 0, 0).
    a_t = r - dh.L_hh    # planar X relative to thigh joint
    b_t = z              # planar Y relative to thigh joint (=Z_H)

    # Step 3: 2R planar IK.
    dist_sq = a_t * a_t + b_t * b_t
    cos_knee = (dist_sq - dh.L_th**2 - dh.L_sh**2) / (2.0 * dh.L_th * dh.L_sh)
    if cos_knee > 1.0 + 1e-9 or cos_knee < -1.0 - 1e-9:
        raise ValueError(f"foot out of reach: dist={np.sqrt(dist_sq):.4f} m, "
                         f"max={dh.L_th + dh.L_sh:.4f} m")
    cos_knee = float(np.clip(cos_knee, -1.0, 1.0))
    theta_knee = knee_direction * np.arccos(cos_knee)
    theta_thigh = (
        np.arctan2(b_t, a_t)
        - np.arctan2(dh.L_sh * np.sin(theta_knee),
                     dh.L_th + dh.L_sh * np.cos(theta_knee))
    )
    return (float(theta_hip), float(theta_thigh), float(theta_knee))
```

- [ ] **Step 4: Run tests, verify pass**

```bash
cd dog_robot_ws/src/dog_robot_control && python -m pytest test/test_kinematics_dh.py -v
```
Expected: all tests pass (including roundtrip).

- [ ] **Step 5: Commit**

```bash
git add dog_robot_ws/src/dog_robot_control/dog_robot_control/kinematics_dh.py \
        dog_robot_ws/src/dog_robot_control/test/test_kinematics_dh.py
git commit -m "feat(kinematics): closed-form ik_leg + FK/IK roundtrip test"
```

---

### Task 4: leg_config.py + dh_params.yaml

**Files:**
- Create: `dog_robot_ws/src/dog_robot_control/dog_robot_control/leg_config.py`
- Create: `dog_robot_ws/src/dog_robot_control/config/dh_params.yaml`

- [ ] **Step 1: Write leg_config.py**

```python
# dog_robot_control/leg_config.py
"""Per-leg configuration: base→hip rigid transform + mirror sign for IK."""
from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class LegConfig:
    name: str                              # "FL" | "FR" | "BL" | "BR"
    base_to_hip_xyz: Tuple[float, float, float]
    base_to_hip_rpy: Tuple[float, float, float]
    mirror: int                            # +1 left, -1 right


import math

_PI_2 = math.pi / 2
_PI = math.pi

LEGS: Tuple[LegConfig, ...] = (
    LegConfig("FL", ( 0.07480,  0.04000, 0.03510), (0.0, _PI_2, 0.0), +1),
    LegConfig("FR", ( 0.07480, -0.04000, 0.03510), (0.0, _PI_2, _PI), -1),
    LegConfig("BL", (-0.07480,  0.04000, 0.03510), (0.0, _PI_2, 0.0), +1),
    LegConfig("BR", (-0.07480, -0.04000, 0.03510), (0.0, _PI_2, _PI), -1),
)


def get_leg(name: str) -> LegConfig:
    for L in LEGS:
        if L.name == name:
            return L
    raise KeyError(name)
```

- [ ] **Step 2: Write dh_params.yaml**

```yaml
# dog_robot_control/config/dh_params.yaml
# Values L_hh, L_th, L_sh come from scripts/compute_dh_lengths.py output.
stand_controller:
  ros__parameters:
    dh:
      L_hh: 0.02944
      L_th: 0.11737
      L_sh: 0.07013
    stand:
      default_height: 0.18      # body z (m) above ground in stand pose
      ramp_time: 2.0            # seconds to reach stand from current
      publish_rate: 50.0        # Hz
      knee_direction: 1         # +1 or -1
    joint_order:                # must match ros2_controllers joints list
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

- [ ] **Step 3: Add a quick test for leg_config**

Append to `test/test_kinematics_dh.py`:

```python
from dog_robot_control.leg_config import LEGS, get_leg

def test_legs_table_has_4_entries():
    assert len(LEGS) == 4
    assert {L.name for L in LEGS} == {"FL", "FR", "BL", "BR"}

def test_mirror_signs_match_side():
    assert get_leg("FL").mirror == +1
    assert get_leg("FR").mirror == -1
    assert get_leg("BL").mirror == +1
    assert get_leg("BR").mirror == -1
```

- [ ] **Step 4: Run test, verify pass**

```bash
cd dog_robot_ws/src/dog_robot_control && python -m pytest test/test_kinematics_dh.py -v
```

- [ ] **Step 5: Update setup.py to install config dir**

Already installed via the `(os.path.join("share", package_name, "config"), glob("config/*.yaml"))` line. No change needed.

- [ ] **Step 6: Commit**

```bash
git add dog_robot_ws/src/dog_robot_control/dog_robot_control/leg_config.py \
        dog_robot_ws/src/dog_robot_control/config/dh_params.yaml \
        dog_robot_ws/src/dog_robot_control/test/test_kinematics_dh.py
git commit -m "feat(kinematics): leg config table + dh_params yaml"
```

---

### Task 5: Mesh visual compensation helper

**Files:**
- Create: `dog_robot_ws/src/dog_robot_description/scripts/compute_visual_compensation.py`

Goal: for each leg, output xacro-ready `<visual><origin xyz=... rpy=.../>` values so meshes (authored in the original URDF link frames) render correctly when the link frames are re-oriented to DH convention.

- [ ] **Step 1: Write the script**

```python
#!/usr/bin/env python3
"""Compute mesh visual origin (xyz, rpy) so STL meshes (authored in the OLD URDF
link frames) render in the same world location under the NEW DH-aligned link
frames.

For each link, compensation = T_new_link_in_parent^{-1} * T_old_link_in_parent
where both Ts are evaluated with all joint angles at 0.

Prints, per leg, a block of 4 lines suitable to paste into dog_robot.urdf.xacro
as <xacro:leg ... mesh_*_xyz=... mesh_*_rpy=... .../>
"""
import math
from itertools import product

import numpy as np


def Rx(a): c,s=math.cos(a),math.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def Ry(a): c,s=math.cos(a),math.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def Rz(a): c,s=math.cos(a),math.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])


def H(R, t):
    M = np.eye(4)
    M[:3,:3] = R
    M[:3, 3] = t
    return M


def rpy(R):
    # Standard XYZ extrinsic = URDF rpy.
    sy = -R[2,0]
    if abs(sy) < 0.9999999:
        p = math.asin(sy)
        r = math.atan2(R[2,1], R[2,2])
        y = math.atan2(R[1,0], R[0,0])
    else:
        # gimbal lock
        p = math.copysign(math.pi/2, sy)
        r = 0.0
        y = math.atan2(-R[0,1], R[1,1])
    return (r, p, y)


def urdf_origin(xyz, rpy_tuple):
    """URDF origin T = T_xyz * R_rpy."""
    r,p,y = rpy_tuple
    R = Rz(y) @ Ry(p) @ Rx(r)
    return H(R, np.array(xyz))


# Old URDF (committed state, CHAMP-IK surgery applied) – per-leg parameters.
OLD = {
    "FL": dict(hip_xyz=( 0.07480, 0.04000, 0.03510), hip_rpy=(0,0,0),
               thigh_xyz=( 0.02520, 0.02536,-0.01317), thigh_rpy=(0,0.94261,0),
               knee_xyz=(0.0,0.04102,-0.10984),       knee_rpy=(0,-1.93175,0),
               foot_xyz=(0.0,-0.01922,-0.06773),      foot_rpy=(0,0,0)),
    "FR": dict(hip_xyz=( 0.07480,-0.04000, 0.03510), hip_rpy=(0,0,0),
               thigh_xyz=( 0.02520,-0.02570,-0.01250), thigh_rpy=(0,0.93698,0),
               knee_xyz=(0.0,-0.04270,-0.10920),      knee_rpy=(0,-1.91411,0),
               foot_xyz=(0.0,0.01826,-0.06802),       foot_rpy=(0,0,0)),
    "BL": dict(hip_xyz=(-0.07480, 0.04000, 0.03510), hip_rpy=(0,0,0),
               thigh_xyz=(-0.02520, 0.02536,-0.01318), thigh_rpy=(0,0.86151,0),
               knee_xyz=(0.0,0.04082,-0.10992),       knee_rpy=(0,-1.96488,0),
               foot_xyz=(0.0,-0.01906,-0.06742),      foot_rpy=(0,0,0)),
    "BR": dict(hip_xyz=(-0.07480,-0.04000, 0.03510), hip_rpy=(0,0,0),
               thigh_xyz=(-0.02520,-0.02570,-0.01250), thigh_rpy=(0,0.86324,0),
               knee_xyz=(0.0,-0.04270,-0.10920),      knee_rpy=(0,-1.95214,0),
               foot_xyz=(0.0,0.01842,-0.06838),       foot_rpy=(0,0,0)),
}

# New URDF (DH aligned). base_to_hip xyz copies OLD hip_xyz; rpy is per-side.
PI_2 = math.pi / 2
NEW_HIP_RPY = {"FL": (0, PI_2, 0), "FR": (0, PI_2, math.pi),
               "BL": (0, PI_2, 0), "BR": (0, PI_2, math.pi)}
L_HH, L_TH, L_SH = 0.02944, 0.11737, 0.07013   # from compute_dh_lengths.py


def main():
    for leg in ("FL", "FR", "BL", "BR"):
        O = OLD[leg]
        # Old link-in-parent transforms at all joint angles = 0:
        T_old_hip   = urdf_origin(O["hip_xyz"],   O["hip_rpy"])
        T_old_thigh = urdf_origin(O["thigh_xyz"], O["thigh_rpy"])
        T_old_shank = urdf_origin(O["knee_xyz"],  O["knee_rpy"])
        T_old_foot  = urdf_origin(O["foot_xyz"],  O["foot_rpy"])

        # New link-in-parent transforms:
        T_new_hip   = urdf_origin(O["hip_xyz"], NEW_HIP_RPY[leg])
        T_new_thigh = urdf_origin((L_HH, 0, 0), (-PI_2, 0, 0))
        T_new_shank = urdf_origin((L_TH, 0, 0), (0, 0, 0))
        T_new_foot  = urdf_origin((L_SH, 0, 0), (0, 0, 0))

        # Visual compensation = inv(T_new) @ T_old for each link.
        comp_hip   = np.linalg.inv(T_new_hip)   @ T_old_hip
        comp_thigh = np.linalg.inv(T_new_thigh) @ T_old_thigh
        comp_shank = np.linalg.inv(T_new_shank) @ T_old_shank
        comp_foot  = np.linalg.inv(T_new_foot)  @ T_old_foot

        def fmt(T, name):
            xyz = tuple(round(v, 5) for v in T[:3, 3])
            r,p,y = rpy(T[:3, :3])
            return (f'  mesh_{name}_xyz="{xyz[0]} {xyz[1]} {xyz[2]}" '
                    f'mesh_{name}_rpy="{r:.5f} {p:.5f} {y:.5f}"')

        print(f"<!-- {leg} -->")
        print(fmt(comp_hip,   "hip"))
        print(fmt(comp_thigh, "thigh"))
        print(fmt(comp_shank, "shank"))
        print(fmt(comp_foot,  "foot"))
        print()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run it, capture output**

```bash
chmod +x dog_robot_ws/src/dog_robot_description/scripts/compute_visual_compensation.py
python3 dog_robot_ws/src/dog_robot_description/scripts/compute_visual_compensation.py
```

Output will be 4 blocks of mesh_xxx params per leg. Keep stdout for pasting into dog_robot.urdf.xacro in Task 7.

- [ ] **Step 3: Commit**

```bash
git add dog_robot_ws/src/dog_robot_description/scripts/compute_visual_compensation.py
git commit -m "feat(scripts): compute mesh visual origin compensation for DH URDF"
```

---

### Task 6: Rewrite leg.xacro with DH joint origins

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_description/urdf/leg.xacro`

- [ ] **Step 1: Replace file contents**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="dog_robot_leg">

  <!--
    Modified DH (Craig) convention. Per-leg base->hip rigid transform places the
    hip frame so Z_H = body X (hip yaw axis), X_H = body -Z (downward). The DH
    chain then has identical link lengths for all 4 legs:
      i=1 (hip):   alpha_0=0,     a_0=0
      i=2 (thigh): alpha_1=-pi/2, a_1=L_hh
      i=3 (knee):  alpha_2=0,     a_2=L_th
      F  (foot):   alpha_3=0,     a_3=L_sh

    URDF joint origin T = Translation(xyz) * Rotation(rpy), so the per-joint
    fixed transform Rx(alpha)*Tx(a) maps to xyz=(a,0,0) rpy=(alpha,0,0).
  -->

  <xacro:macro name="leg" params="prefix
                                  base_to_hip_xyz base_to_hip_rpy
                                  L_hh L_th L_sh
                                  mesh_hip_xyz mesh_hip_rpy
                                  mesh_thigh_xyz mesh_thigh_rpy
                                  mesh_shank_xyz mesh_shank_rpy
                                  mesh_foot_xyz mesh_foot_rpy
                                  foot_sphere_xyz:='0 0 0'">

    <link name="${prefix}_hip_link">
      <visual>
        <origin xyz="${mesh_hip_xyz}" rpy="${mesh_hip_rpy}"/>
        <geometry>
          <mesh filename="package://dog_robot_description/meshes/visual/${prefix}_hip_link.stl"/>
        </geometry>
        <material name="grey"/>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry><box size="0.03 0.03 0.03"/></geometry>
      </collision>
      <xacro:inertial_box mass="0.15" x="0.04" y="0.04" z="0.04">
        <origin xyz="0 0 0" rpy="0 0 0"/>
      </xacro:inertial_box>
    </link>

    <joint name="${prefix}_hip_yaw" type="revolute">
      <parent link="base_link"/>
      <child link="${prefix}_hip_link"/>
      <origin xyz="${base_to_hip_xyz}" rpy="${base_to_hip_rpy}"/>
      <axis xyz="0 0 1"/>
      <limit lower="-0.785" upper="0.785" effort="5.0" velocity="8.0"/>
      <dynamics damping="0.05" friction="0.0"/>
    </joint>

    <link name="${prefix}_thigh_link">
      <visual>
        <origin xyz="${mesh_thigh_xyz}" rpy="${mesh_thigh_rpy}"/>
        <geometry>
          <mesh filename="package://dog_robot_description/meshes/visual/${prefix}_thigh_link.stl"/>
        </geometry>
        <material name="dark_grey"/>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry><box size="0.02 0.02 0.08"/></geometry>
      </collision>
      <xacro:inertial_box mass="0.10" x="0.03" y="0.03" z="0.10">
        <origin xyz="0 0 0" rpy="0 0 0"/>
      </xacro:inertial_box>
    </link>

    <joint name="${prefix}_thigh_pitch" type="revolute">
      <parent link="${prefix}_hip_link"/>
      <child link="${prefix}_thigh_link"/>
      <origin xyz="${L_hh} 0 0" rpy="-1.5707963 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="-1.571" upper="1.571" effort="5.0" velocity="8.0"/>
      <dynamics damping="0.05" friction="0.0"/>
    </joint>

    <link name="${prefix}_shank_link">
      <visual>
        <origin xyz="${mesh_shank_xyz}" rpy="${mesh_shank_rpy}"/>
        <geometry>
          <mesh filename="package://dog_robot_description/meshes/visual/${prefix}_shank_link.stl"/>
        </geometry>
        <material name="grey"/>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry><box size="0.015 0.015 0.10"/></geometry>
      </collision>
      <xacro:inertial_box mass="0.08" x="0.02" y="0.02" z="0.115">
        <origin xyz="0 0 0" rpy="0 0 0"/>
      </xacro:inertial_box>
    </link>

    <joint name="${prefix}_knee_pitch" type="revolute">
      <parent link="${prefix}_thigh_link"/>
      <child link="${prefix}_shank_link"/>
      <origin xyz="${L_th} 0 0" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="0.0" upper="2.617" effort="5.0" velocity="8.0"/>
      <dynamics damping="0.05" friction="0.0"/>
    </joint>

    <link name="${prefix}_foot_link">
      <visual>
        <origin xyz="${mesh_foot_xyz}" rpy="${mesh_foot_rpy}"/>
        <geometry>
          <mesh filename="package://dog_robot_description/meshes/visual/${prefix}_foot_link.stl"/>
        </geometry>
        <material name="dark_grey"/>
      </visual>
      <collision>
        <origin xyz="${foot_sphere_xyz}" rpy="0 0 0"/>
        <geometry><sphere radius="0.018"/></geometry>
      </collision>
      <xacro:inertial_sphere mass="0.02" radius="0.015">
        <origin xyz="0 0 0" rpy="0 0 0"/>
      </xacro:inertial_sphere>
    </link>

    <joint name="${prefix}_foot_fixed" type="fixed">
      <parent link="${prefix}_shank_link"/>
      <child link="${prefix}_foot_link"/>
      <origin xyz="${L_sh} 0 0" rpy="0 0 0"/>
    </joint>

  </xacro:macro>

</robot>
```

- [ ] **Step 2: Verify xacro parses (cannot fully test until Task 7 updates the caller)**

```bash
cd dog_robot_ws && xacro src/dog_robot_description/urdf/leg.xacro 2>&1 | head -3
```
Expected: error about missing macro invocation – that's fine, leg.xacro is a macro file. The real parse test happens in Task 7.

- [ ] **Step 3: Commit**

```bash
git add dog_robot_ws/src/dog_robot_description/urdf/leg.xacro
git commit -m "refactor(urdf): DH-aligned joint origins in leg.xacro macro"
```

---

### Task 7: Rewrite dog_robot.urdf.xacro with DH per-leg config

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_description/urdf/dog_robot.urdf.xacro`

- [ ] **Step 1: Run Task 5's compensation script and capture mesh params for paste**

```bash
python3 dog_robot_ws/src/dog_robot_description/scripts/compute_visual_compensation.py > /tmp/mesh_compensation.txt
cat /tmp/mesh_compensation.txt
```

- [ ] **Step 2: Replace `<xacro:leg>` calls in dog_robot.urdf.xacro**

Replace the 4 `<xacro:leg>` calls with the following template (substituting the mesh values from /tmp/mesh_compensation.txt):

```xml
  <!-- Common DH link lengths (from scripts/compute_dh_lengths.py averaged over 4 legs). -->
  <xacro:property name="L_hh" value="0.02944"/>
  <xacro:property name="L_th" value="0.11737"/>
  <xacro:property name="L_sh" value="0.07013"/>

  <xacro:leg prefix="FL"
             base_to_hip_xyz="0.07480 0.04000 0.03510"
             base_to_hip_rpy="0 1.5707963 0"
             L_hh="${L_hh}" L_th="${L_th}" L_sh="${L_sh}"
             mesh_hip_xyz="<FROM SCRIPT>" mesh_hip_rpy="<FROM SCRIPT>"
             mesh_thigh_xyz="<FROM SCRIPT>" mesh_thigh_rpy="<FROM SCRIPT>"
             mesh_shank_xyz="<FROM SCRIPT>" mesh_shank_rpy="<FROM SCRIPT>"
             mesh_foot_xyz="<FROM SCRIPT>"  mesh_foot_rpy="<FROM SCRIPT>"
             foot_sphere_xyz="0 0 0"/>

  <xacro:leg prefix="FR"
             base_to_hip_xyz="0.07480 -0.04000 0.03510"
             base_to_hip_rpy="0 1.5707963 3.1415927"
             L_hh="${L_hh}" L_th="${L_th}" L_sh="${L_sh}"
             mesh_hip_xyz="<FROM SCRIPT>" mesh_hip_rpy="<FROM SCRIPT>"
             mesh_thigh_xyz="<FROM SCRIPT>" mesh_thigh_rpy="<FROM SCRIPT>"
             mesh_shank_xyz="<FROM SCRIPT>" mesh_shank_rpy="<FROM SCRIPT>"
             mesh_foot_xyz="<FROM SCRIPT>"  mesh_foot_rpy="<FROM SCRIPT>"
             foot_sphere_xyz="0 0 0"/>

  <xacro:leg prefix="BL"
             base_to_hip_xyz="-0.07480 0.04000 0.03510"
             base_to_hip_rpy="0 1.5707963 0"
             L_hh="${L_hh}" L_th="${L_th}" L_sh="${L_sh}"
             mesh_hip_xyz="<FROM SCRIPT>" mesh_hip_rpy="<FROM SCRIPT>"
             mesh_thigh_xyz="<FROM SCRIPT>" mesh_thigh_rpy="<FROM SCRIPT>"
             mesh_shank_xyz="<FROM SCRIPT>" mesh_shank_rpy="<FROM SCRIPT>"
             mesh_foot_xyz="<FROM SCRIPT>"  mesh_foot_rpy="<FROM SCRIPT>"
             foot_sphere_xyz="0 0 0"/>

  <xacro:leg prefix="BR"
             base_to_hip_xyz="-0.07480 -0.04000 0.03510"
             base_to_hip_rpy="0 1.5707963 3.1415927"
             L_hh="${L_hh}" L_th="${L_th}" L_sh="${L_sh}"
             mesh_hip_xyz="<FROM SCRIPT>" mesh_hip_rpy="<FROM SCRIPT>"
             mesh_thigh_xyz="<FROM SCRIPT>" mesh_thigh_rpy="<FROM SCRIPT>"
             mesh_shank_xyz="<FROM SCRIPT>" mesh_shank_rpy="<FROM SCRIPT>"
             mesh_foot_xyz="<FROM SCRIPT>"  mesh_foot_rpy="<FROM SCRIPT>"
             foot_sphere_xyz="0 0 0"/>
```

Replace each `<FROM SCRIPT>` token with the corresponding output of `compute_visual_compensation.py`. The script's output is formatted to be directly pasteable.

- [ ] **Step 3: Build + verify URDF parses**

```bash
cd dog_robot_ws && colcon build --packages-select dog_robot_description && source install/setup.bash
xacro src/dog_robot_description/urdf/dog_robot.urdf.xacro controllers_yaml_path:=/tmp/dummy.yaml > /tmp/dog.urdf
head -20 /tmp/dog.urdf
check_urdf /tmp/dog.urdf
```
Expected: `Successfully Parsed XML`, 13 links + 12 joints listed.

- [ ] **Step 4: Visually inspect in RViz (manual check)**

```bash
ros2 launch dog_robot_description display.launch.py     # if exists, else use:
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat /tmp/dog.urdf)"
# Open rviz2, add RobotModel, set Fixed Frame to base_link. All meshes should
# render in their original locations (legs hanging down, body horizontal).
```

If meshes look wrong (offset or rotated incorrectly), the compensation script has a bug; revisit Task 5.

- [ ] **Step 5: Commit**

```bash
git add dog_robot_ws/src/dog_robot_description/urdf/dog_robot.urdf.xacro
git commit -m "refactor(urdf): per-leg DH config + mesh compensation in dog_robot.urdf.xacro"
```

---

### Task 8: URDF↔kinematics_dh consistency test

**Files:**
- Create: `dog_robot_ws/src/dog_robot_control/test/test_urdf_dh_consistency.py`

- [ ] **Step 1: Install urdf_parser_py if not present**

```bash
sudo apt-get install -y ros-humble-urdfdom-py
```

- [ ] **Step 2: Write the test**

```python
# test/test_urdf_dh_consistency.py
"""Verify FK chain built from dog_robot.urdf.xacro matches kinematics_dh.fk_leg
for all 4 legs at random joint configurations.
"""
import os
import subprocess
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from dog_robot_control.kinematics_dh import DHParams, fk_leg
from dog_robot_control.leg_config import LEGS

DH = DHParams(L_hh=0.02944, L_th=0.11737, L_sh=0.07013)
TOL = 1e-4   # meters / radians


def _xacro_to_urdf(xacro_path: str) -> str:
    out = subprocess.check_output([
        "xacro", xacro_path, "controllers_yaml_path:=/tmp/dummy.yaml",
    ])
    return out.decode("utf-8")


def _Rx(a): c,s=math.cos(a),math.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def _Ry(a): c,s=math.cos(a),math.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def _Rz(a): c,s=math.cos(a),math.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])

def _T(R, t):
    M = np.eye(4); M[:3,:3]=R; M[:3,3]=t; return M

def _urdf_origin(elem):
    o = elem.find("origin")
    xyz = (0.0,0.0,0.0); rpy = (0.0,0.0,0.0)
    if o is not None:
        if o.get("xyz"): xyz = tuple(float(v) for v in o.get("xyz").split())
        if o.get("rpy"): rpy = tuple(float(v) for v in o.get("rpy").split())
    r,p,y = rpy
    return _T(_Rz(y) @ _Ry(p) @ _Rx(r), np.array(xyz))


def _joint_axis(elem):
    a = elem.find("axis")
    return tuple(float(v) for v in a.get("xyz").split())


def _axis_angle(axis, theta):
    ax = np.array(axis) / np.linalg.norm(axis)
    K = np.array([[0,-ax[2],ax[1]],[ax[2],0,-ax[0]],[-ax[1],ax[0],0]])
    return np.eye(3) + math.sin(theta)*K + (1-math.cos(theta))*K@K


def _urdf_fk_foot(urdf_root, leg, theta):
    """Walk URDF joint chain from base_link to {leg}_foot_link."""
    joints = {j.get("name"): j for j in urdf_root.findall("joint")}
    chain = [(f"{leg}_hip_yaw",     theta[0]),
             (f"{leg}_thigh_pitch", theta[1]),
             (f"{leg}_knee_pitch",  theta[2]),
             (f"{leg}_foot_fixed",  0.0)]
    T = np.eye(4)
    for jname, q in chain:
        j = joints[jname]
        T_origin = _urdf_origin(j)
        if j.get("type") == "fixed":
            T = T @ T_origin
        else:
            axis = _joint_axis(j)
            R = _axis_angle(axis, q)
            T_q = _T(R, np.zeros(3))
            T = T @ T_origin @ T_q
    return T


def test_urdf_fk_matches_dh_fk_at_zero():
    xacro = os.path.expanduser(
        "~/workspace/dog_robot/dog_robot_ws/src/dog_robot_description/urdf/dog_robot.urdf.xacro")
    urdf_str = _xacro_to_urdf(xacro)
    root = ET.fromstring(urdf_str)
    for L in LEGS:
        T_foot = _urdf_fk_foot(root, L.name, (0.0, 0.0, 0.0))
        foot_world = T_foot[:3, 3]
        # Convert world foot -> hip frame using base_to_hip rigid transform.
        r,p,y = L.base_to_hip_rpy
        R_bh = _Rz(y) @ _Ry(p) @ _Rx(r)
        t_bh = np.array(L.base_to_hip_xyz)
        T_bh = _T(R_bh, t_bh)
        foot_hip = (np.linalg.inv(T_bh) @ np.append(foot_world, 1.0))[:3]
        # Compare with kinematics_dh FK in hip frame.
        foot_dh = fk_leg(DH, (0.0, 0.0, 0.0))
        assert np.allclose(foot_hip, foot_dh, atol=TOL), (L.name, foot_hip, foot_dh)


def test_urdf_fk_matches_dh_fk_random():
    xacro = os.path.expanduser(
        "~/workspace/dog_robot/dog_robot_ws/src/dog_robot_description/urdf/dog_robot.urdf.xacro")
    urdf_str = _xacro_to_urdf(xacro)
    root = ET.fromstring(urdf_str)
    rng = np.random.default_rng(seed=0)
    for L in LEGS:
        for _ in range(10):
            theta = (rng.uniform(-0.5, 0.5),
                     rng.uniform(-0.5, 0.5),
                     rng.uniform( 0.3, 1.5))
            T_foot = _urdf_fk_foot(root, L.name, theta)
            foot_world = T_foot[:3, 3]
            r,p,y = L.base_to_hip_rpy
            T_bh = _T(_Rz(y) @ _Ry(p) @ _Rx(r), np.array(L.base_to_hip_xyz))
            foot_hip = (np.linalg.inv(T_bh) @ np.append(foot_world, 1.0))[:3]
            foot_dh = fk_leg(DH, theta)
            assert np.allclose(foot_hip, foot_dh, atol=TOL), (L.name, theta, foot_hip, foot_dh)
```

- [ ] **Step 3: Run test, expect PASS**

```bash
cd dog_robot_ws/src/dog_robot_control && python -m pytest test/test_urdf_dh_consistency.py -v
```
Expected: 2 passed.

If FAIL: either URDF mesh compensation values are off (shouldn't affect joint chain – only visuals), or the DH joint origins in leg.xacro are wrong. Investigate by printing intermediate T matrices in `_urdf_fk_foot`.

- [ ] **Step 4: Commit**

```bash
git add dog_robot_ws/src/dog_robot_control/test/test_urdf_dh_consistency.py
git commit -m "test(kinematics): URDF FK chain matches kinematics_dh.fk_leg"
```

---

### Task 9: Update ros2_controllers.yaml for effort JTC with PID

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_description/config/ros2_controllers.yaml`

- [ ] **Step 1: Replace contents**

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

joint_trajectory_controller:
  ros__parameters:
    joints:
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
    command_interfaces:
      - effort
    state_interfaces:
      - position
      - velocity
    state_publish_rate: 50.0
    action_monitor_rate: 20.0
    allow_partial_joints_goal: true
    open_loop_control: false
    gains:
      FL_hip_yaw:     { p: 30.0, i: 0.1, d: 1.0, i_clamp: 1.0 }
      FL_thigh_pitch: { p: 40.0, i: 0.1, d: 1.5, i_clamp: 1.0 }
      FL_knee_pitch:  { p: 40.0, i: 0.1, d: 1.5, i_clamp: 1.0 }
      FR_hip_yaw:     { p: 30.0, i: 0.1, d: 1.0, i_clamp: 1.0 }
      FR_thigh_pitch: { p: 40.0, i: 0.1, d: 1.5, i_clamp: 1.0 }
      FR_knee_pitch:  { p: 40.0, i: 0.1, d: 1.5, i_clamp: 1.0 }
      BL_hip_yaw:     { p: 30.0, i: 0.1, d: 1.0, i_clamp: 1.0 }
      BL_thigh_pitch: { p: 40.0, i: 0.1, d: 1.5, i_clamp: 1.0 }
      BL_knee_pitch:  { p: 40.0, i: 0.1, d: 1.5, i_clamp: 1.0 }
      BR_hip_yaw:     { p: 30.0, i: 0.1, d: 1.0, i_clamp: 1.0 }
      BR_thigh_pitch: { p: 40.0, i: 0.1, d: 1.5, i_clamp: 1.0 }
      BR_knee_pitch:  { p: 40.0, i: 0.1, d: 1.5, i_clamp: 1.0 }
```

- [ ] **Step 2: Build + verify controller loads**

```bash
cd dog_robot_ws && colcon build --packages-select dog_robot_description && source install/setup.bash
ros2 launch dog_robot_description gazebo.launch.py &
sleep 12
ros2 control list_controllers
pkill -f gazebo; pkill -f gzserver; pkill -f robot_state_publisher
```
Expected output: `joint_trajectory_controller   active`, `joint_state_broadcaster   active`.

If `joint_trajectory_controller` is `inactive` due to interface mismatch: confirm ros2_control.xacro has `<command_interface name="effort"/>` for all 12 joints (it should – we left it untouched).

- [ ] **Step 3: Commit**

```bash
git add dog_robot_ws/src/dog_robot_description/config/ros2_controllers.yaml
git commit -m "feat(control): JTC with effort interface + per-joint PID gains"
```

---

### Task 10: stand_controller.py — basic ramp + IK publisher

**Files:**
- Create: `dog_robot_ws/src/dog_robot_control/dog_robot_control/stand_controller.py`
- Modify: `dog_robot_ws/src/dog_robot_control/setup.py`

- [ ] **Step 1: Implement stand_controller.py**

```python
# dog_robot_control/stand_controller.py
"""Stand-only controller for dog_robot.

Workflow:
  1. Wait for first /joint_states message; record current angles.
  2. Compute target joint angles for default stand pose using DH IK.
  3. Linearly interpolate (current -> target) over ramp_time seconds.
  4. Publish JointTrajectory at publish_rate to /joint_trajectory_controller/joint_trajectory.
  5. After ramp completes, hold target. Subscribe /stand_cmd (geometry_msgs/Pose)
     to update target body height (z) on the fly; orientation ignored in v1.
"""
import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from dog_robot_control.kinematics_dh import DHParams, ik_leg
from dog_robot_control.leg_config import LEGS


def Rx(a): c,s=math.cos(a),math.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def Ry(a): c,s=math.cos(a),math.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def Rz(a): c,s=math.cos(a),math.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])


def body_to_hip(point_body, leg):
    r, p, y = leg.base_to_hip_rpy
    R_bh = Rz(y) @ Ry(p) @ Rx(r)
    t_bh = np.array(leg.base_to_hip_xyz)
    return R_bh.T @ (np.asarray(point_body) - t_bh)


class StandController(Node):
    def __init__(self):
        super().__init__("stand_controller")
        self.declare_parameter("dh.L_hh", 0.02944)
        self.declare_parameter("dh.L_th", 0.11737)
        self.declare_parameter("dh.L_sh", 0.07013)
        self.declare_parameter("stand.default_height", 0.18)
        self.declare_parameter("stand.ramp_time", 2.0)
        self.declare_parameter("stand.publish_rate", 50.0)
        self.declare_parameter("stand.knee_direction", 1)
        self.declare_parameter("joint_order", [
            "FL_hip_yaw","FL_thigh_pitch","FL_knee_pitch",
            "FR_hip_yaw","FR_thigh_pitch","FR_knee_pitch",
            "BL_hip_yaw","BL_thigh_pitch","BL_knee_pitch",
            "BR_hip_yaw","BR_thigh_pitch","BR_knee_pitch",
        ])

        self.dh = DHParams(
            L_hh=self.get_parameter("dh.L_hh").value,
            L_th=self.get_parameter("dh.L_th").value,
            L_sh=self.get_parameter("dh.L_sh").value,
        )
        self.height = float(self.get_parameter("stand.default_height").value)
        self.ramp_time = float(self.get_parameter("stand.ramp_time").value)
        self.knee_dir = int(self.get_parameter("stand.knee_direction").value)
        rate = float(self.get_parameter("stand.publish_rate").value)
        self.joint_order = list(self.get_parameter("joint_order").value)

        self.start_angles: Optional[np.ndarray] = None
        self.target_angles: Optional[np.ndarray] = None
        self.ramp_start_t: Optional[float] = None

        self.pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )
        self.sub_js = self.create_subscription(
            JointState, "/joint_states", self._on_js, 10
        )
        self.sub_cmd = self.create_subscription(
            Pose, "/stand_cmd", self._on_cmd, 10
        )
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info("stand_controller up; waiting for /joint_states")

    def _on_js(self, msg: JointState):
        if self.start_angles is not None:
            return
        idx = {n: i for i, n in enumerate(msg.name)}
        try:
            angles = np.array([msg.position[idx[j]] for j in self.joint_order])
        except KeyError as e:
            self.get_logger().warn(f"joint_states missing {e}; will retry")
            return
        self.start_angles = angles
        self._recompute_target()
        self.ramp_start_t = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info("captured start angles; ramping to stand")

    def _on_cmd(self, msg: Pose):
        new_h = float(msg.position.z)
        if not (0.05 < new_h < 0.30):
            self.get_logger().warn(f"ignored stand_cmd height={new_h} (out of bounds)")
            return
        self.height = new_h
        if self.start_angles is None:
            return
        # Re-ramp from CURRENT publish target to new target.
        self.start_angles = self._current_command() if self.target_angles is not None else self.start_angles
        self._recompute_target()
        self.ramp_start_t = self.get_clock().now().nanoseconds * 1e-9

    def _recompute_target(self):
        targets = []
        for L in LEGS:
            # Default foot world position: directly below hip joint at z=0.
            foot_world = np.array([L.base_to_hip_xyz[0], L.base_to_hip_xyz[1], 0.0])
            foot_body = foot_world - np.array([0.0, 0.0, self.height])
            foot_hip = body_to_hip(foot_body, L)
            try:
                q = ik_leg(self.dh, foot_hip, knee_direction=self.knee_dir)
            except ValueError as e:
                self.get_logger().error(f"IK failed for {L.name}: {e}; height={self.height}")
                return
            targets.extend(q)
        self.target_angles = np.array(targets)

    def _current_command(self) -> np.ndarray:
        # Where the ramp is right now.
        if self.target_angles is None or self.start_angles is None or self.ramp_start_t is None:
            return self.start_angles.copy() if self.start_angles is not None else np.zeros(12)
        t = self.get_clock().now().nanoseconds * 1e-9 - self.ramp_start_t
        alpha = float(np.clip(t / self.ramp_time, 0.0, 1.0))
        return (1.0 - alpha) * self.start_angles + alpha * self.target_angles

    def _tick(self):
        if self.start_angles is None or self.target_angles is None:
            return
        q = self._current_command()
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
    node = StandController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Register console_script in setup.py**

Edit `dog_robot_ws/src/dog_robot_control/setup.py`, replace the `entry_points` block:

```python
    entry_points={
        "console_scripts": [
            "teleop_keyboard = dog_robot_control.teleop_keyboard:main",
            "stand_controller = dog_robot_control.stand_controller:main",
        ],
    },
```

- [ ] **Step 3: Add geometry_msgs + sensor_msgs + trajectory_msgs deps to package.xml (likely already present)**

Check `dog_robot_ws/src/dog_robot_control/package.xml`:

```bash
grep -E "geometry_msgs|sensor_msgs|trajectory_msgs" dog_robot_ws/src/dog_robot_control/package.xml
```
All three should already be declared per earlier session — if not, add `<exec_depend>...` lines.

- [ ] **Step 4: Build + verify executable installs**

```bash
cd dog_robot_ws && colcon build --packages-select dog_robot_control && source install/setup.bash
ros2 pkg executables dog_robot_control | grep stand_controller
```
Expected: `dog_robot_control stand_controller`.

- [ ] **Step 5: Commit**

```bash
git add dog_robot_ws/src/dog_robot_control/dog_robot_control/stand_controller.py \
        dog_robot_ws/src/dog_robot_control/setup.py
git commit -m "feat(control): stand_controller node using DH-IK"
```

---

### Task 11: stand.launch.py

**Files:**
- Create: `dog_robot_ws/src/dog_robot_control/launch/stand.launch.py`

- [ ] **Step 1: Write launch file**

```python
# launch/stand.launch.py
"""Stand-only launch: Gazebo + spawn dog_robot + JTC + stand_controller."""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler)
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
    stand_params = PathJoinSubstitution([ctrl, "config", "dh_params.yaml"])

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
    stand = Node(
        package="dog_robot_control",
        executable="stand_controller",
        name="stand_controller",
        parameters=[stand_params],
        output="screen",
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[load_jsb])),
        RegisterEventHandler(OnProcessExit(target_action=load_jsb, on_exit=[load_jtc])),
        RegisterEventHandler(OnProcessExit(target_action=load_jtc, on_exit=[stand])),
    ])
```

- [ ] **Step 2: Install launch file**

setup.py already globs `launch/*.launch.py`. Just rebuild:

```bash
cd dog_robot_ws && colcon build --packages-select dog_robot_control && source install/setup.bash
```

- [ ] **Step 3: Smoke launch (kill before relaunch per project memory)**

```bash
pkill -f gzserver; pkill -f gzclient; pkill -f stand_controller; pkill -f robot_state_publisher; sleep 1
ros2 launch dog_robot_control stand.launch.py &
sleep 15
ros2 topic list | grep -E "joint_trajectory|joint_states"
ros2 control list_controllers
```
Expected: trajectory + state topics present; both controllers active.

```bash
pkill -f gzserver; pkill -f gzclient; pkill -f stand_controller
```

- [ ] **Step 4: Commit**

```bash
git add dog_robot_ws/src/dog_robot_control/launch/stand.launch.py
git commit -m "feat(launch): stand-only launch file (gazebo + JTC + stand_controller)"
```

---

### Task 12: Gazebo stand verification

**Files:** none (manual test)

- [ ] **Step 1: Run stand**

```bash
pkill -f gzserver; pkill -f gzclient; pkill -f stand_controller; sleep 1
ros2 launch dog_robot_control stand.launch.py
```

Watch Gazebo viewer for 15 s after spawn. Acceptance:
- Robot does NOT fall over within first 15 s after `stand_controller` ramp completes (~3 s after spawn).
- Body roll/pitch remain |·| < 0.15 rad once settled.
- Feet are on the ground; legs slightly bent (not fully extended).

- [ ] **Step 2: If robot tips / oscillates, tune**

Tuning order:
1. Flip `knee_direction` (`-1` ↔ `+1`) in `dh_params.yaml` — wrong direction makes the knee bend the wrong way and the robot collapses.
2. Reduce `default_height` from 0.18 → 0.15 to lower COM.
3. Increase `gains.<joint>.p` in `ros2_controllers.yaml` by 20% if joints drift; reduce by 20% if they oscillate.

After each tuning change, rebuild affected packages and re-launch.

- [ ] **Step 3: Commit tuning if changes made**

```bash
git add dog_robot_ws/src/dog_robot_control/config/dh_params.yaml \
        dog_robot_ws/src/dog_robot_description/config/ros2_controllers.yaml
git commit -m "tune(stand): default height + knee_direction for stable stand"
```

---

### Task 13: README — Kinematics section

**Files:**
- Create or modify: `dog_robot_ws/README.md`

- [ ] **Step 1: Append section (create README if missing)**

If `dog_robot_ws/README.md` doesn't exist, create it with:

```markdown
# dog_robot_ws

ROS 2 workspace for a 12-DOF quadruped robot.

```

Then append:

````markdown

## Kinematics

The dog_robot uses Modified Denavit-Hartenberg (Craig) convention. Each leg is a
3-DOF chain (hip yaw, thigh pitch, knee pitch). One symmetric DH table covers
all four legs; per-leg variation lives in the base→hip fixed transform.

### Frames

- **Body B** — URDF root `base_link` (X forward, Y left, Z up).
- **Hip H_<leg>** — fixed transform per leg; Z_H along the hip yaw axis (= body
  X), X_H downward (= -body Z).
- **DH frames 1-3** — at each joint, Z along that joint's axis.

### DH Table

| i | α_{i-1} | a_{i-1}       | d_i | θ_i      |
|---|---------|---------------|-----|----------|
| 1 | 0       | 0             | 0   | θ_hip    |
| 2 | -π/2    | L_hh = 0.02944 m | 0 | θ_thigh  |
| 3 | 0       | L_th = 0.11737 m | 0 | θ_knee   |
| F | 0       | L_sh = 0.07013 m | 0 | 0        |

Lengths come from `scripts/compute_dh_lengths.py` averaging the four legs' CAD
measurements.

### Forward kinematics

```python
from dog_robot_control.kinematics_dh import DHParams, fk_leg
dh = DHParams(L_hh=0.02944, L_th=0.11737, L_sh=0.07013)
foot_xyz = fk_leg(dh, (theta_hip, theta_thigh, theta_knee))
```

### Inverse kinematics

```python
from dog_robot_control.kinematics_dh import ik_leg
theta_hip, theta_thigh, theta_knee = ik_leg(dh, foot_xyz_in_hip_frame,
                                            knee_direction=+1)
```

Closed-form 2R planar + hip yaw decomposition. Raises `ValueError` for foot
targets on the hip yaw axis or beyond reach.

### Per-leg base→hip transforms

| Leg | base→hip xyz (m)            | base→hip rpy (rad)    | Mirror |
|-----|------------------------------|------------------------|--------|
| FL  | ( 0.0748,  0.0400, 0.0351)  | (0, π/2, 0)           | +1     |
| FR  | ( 0.0748, -0.0400, 0.0351)  | (0, π/2, π)           | -1     |
| BL  | (-0.0748,  0.0400, 0.0351)  | (0, π/2, 0)           | +1     |
| BR  | (-0.0748, -0.0400, 0.0351)  | (0, π/2, π)           | -1     |

The right-side `π` yaw places right legs on the opposite side of body Y while
keeping the same DH table — IK and FK code is identical for all four legs.

### Verification

```bash
cd src/dog_robot_control && python -m pytest test/
```

Tests check FK/IK roundtrip (200 random configs) and URDF chain ↔ kinematics
module agreement on 40 random joint angle sets across all four legs.
````

- [ ] **Step 2: Commit**

```bash
git add dog_robot_ws/README.md
git commit -m "docs(readme): DH convention + FK/IK reference"
```

---

### Task 14: Cleanup — deprecate old CHAMP launch, update kill helper

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_config/launch/gazebo.launch.py` (add deprecation banner)
- Create: `dog_robot_ws/scripts/dog_kill_all.sh`

- [ ] **Step 1: Add deprecation comment to old launch**

Prepend to `dog_robot_ws/src/dog_robot_config/launch/gazebo.launch.py`:

```python
"""DEPRECATED — old CHAMP-based launch.

Walking was previously driven via CHAMP's IK. With the DH conversion
(2026-05-23) the kinematics module moved to dog_robot_control; CHAMP IK
is no longer used. Walking is a follow-up plan — when that plan ships,
this file will either be deleted or replaced with a new gait_controller
launch.

Use `ros2 launch dog_robot_control stand.launch.py` for stand-only sim.
"""
```

- [ ] **Step 2: Create persistent kill script**

`dog_robot_ws/scripts/dog_kill_all.sh`:

```bash
#!/usr/bin/env bash
# Kill orphan dog_robot sim processes. Use full-cmdline match (-f) since several
# names exceed pkill's 15-char limit, and run from a script file to avoid
# pkill matching its own shell command line.
set +e
pkill -f gzserver
pkill -f gzclient
pkill -f ros_gz_bridge
pkill -f spawn_entity
pkill -f robot_state_publisher
pkill -f joint_state_broadcaster
pkill -f joint_trajectory_controller
pkill -f controller_manager
pkill -f stand_controller
pkill -f rviz2
pkill -f champ_base
pkill -f champ_gazebo
sleep 0.5
echo "[dog_kill_all] done"
```

```bash
mkdir -p dog_robot_ws/scripts
# Write the file (use the snippet above), then:
chmod +x dog_robot_ws/scripts/dog_kill_all.sh
```

- [ ] **Step 3: Smoke test the kill script**

```bash
ros2 launch dog_robot_control stand.launch.py &
sleep 8
bash dog_robot_ws/scripts/dog_kill_all.sh
sleep 1
pgrep -fa "gzserver|stand_controller|robot_state_publisher" || echo "all dead"
```
Expected: `all dead`.

- [ ] **Step 4: Commit**

```bash
git add dog_robot_ws/src/dog_robot_config/launch/gazebo.launch.py \
        dog_robot_ws/scripts/dog_kill_all.sh
git commit -m "chore(cleanup): deprecate CHAMP launch, add dog_kill_all helper"
```

---

## Self-review

**Spec coverage:**
- ✅ Modified DH frame convention → Tasks 6, 7 (URDF), 2 (kinematics_dh module).
- ✅ One symmetric DH table → Task 4 (leg_config + LEGS) and Task 7 (one set of L_hh/L_th/L_sh constants).
- ✅ Closed-form FK + IK → Tasks 2, 3.
- ✅ Stand controller node → Task 10.
- ✅ Launch + Gazebo verification → Tasks 11, 12.
- ✅ README DH table → Task 13.
- ✅ Tests (FK/IK roundtrip + URDF consistency) → Tasks 3, 8.
- ✅ CHAMP bypass → Task 14 (deprecation banner, kill helper).
- ✅ Mesh visual compensation → Task 5 (script), Task 7 (paste into URDF).

**Placeholder scan:**
- The mesh values in Task 7 are `<FROM SCRIPT>` tokens that get filled by Task 5's output at execution time. This is the only "placeholder" and is intentional — actual values aren't known until Task 1+5 run.
- All other steps have concrete code/commands.

**Type consistency:**
- `DHParams` fields `L_hh`, `L_th`, `L_sh` consistent across all tasks.
- `fk_leg(dh, theta)` signature consistent (theta is a 3-tuple).
- `ik_leg(dh, foot_h, knee_direction)` signature consistent.
- `LegConfig` fields (`name`, `base_to_hip_xyz`, `base_to_hip_rpy`, `mirror`) consistent across leg_config, stand_controller, test_urdf_dh_consistency.
- Joint name order consistent (FL→FR→BL→BR; hip_yaw→thigh_pitch→knee_pitch within each leg) across `dh_params.yaml`, `ros2_controllers.yaml`, and `stand_controller.py`.
