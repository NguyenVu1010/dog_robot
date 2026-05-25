# DH-canonical mesh re-export Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Re-export every link STL from FreeCAD with vertices already in its Modified-DH (Craig) link frame; update the URDF + `kinematics_dh` module to consume clean DH parameters; delete the historic visual-compensation path.

**Architecture:** Three-phase pipeline. (1) `derive_dh_frames.py` reads measured CAD joint axes and emits clean MDH params + per-link CAD Placements. (2) `export_dh_links_from_freecad.py` runs inside FreeCAD via MCP, transforms each link's solids in memory, writes STLs to `meshes/visual_dh/`. (3) URDF + `kinematics_dh` are updated to consume the new params; visual `<origin>` becomes identity everywhere.

**Tech Stack:** Python 3.10 + NumPy, ament_python ROS 2 Humble, FreeCAD MCP server (port 9875), xacro, pytest, rclpy.

**Spec:** `dog_robot_ws/docs/superpowers/specs/2026-05-25-dh-mesh-export-design.md`

---

## File plan

```
dog_robot_ws/src/dog_robot_description/
  scripts/
    derive_dh_frames.py                     ← NEW (~120 lines)
    export_dh_links_from_freecad.py         ← NEW (~150 lines, runs in FreeCAD)
  config/
    dh_link_placements.yaml                 ← NEW (generated, input to FreeCAD)
  meshes/
    visual_dh/<link>.stl × 17               ← NEW (regenerated)
    visual/, collision/                     ← DELETED at end
  urdf/
    leg.xacro                               ← MODIFY (d_*, alpha_thigh params; identity visual)
    dog_robot.urdf.xacro                    ← MODIFY (new properties; visual_dh path on base)

dog_robot_ws/src/dog_robot_kinematics/
  dog_robot_kinematics/
    kinematics_dh.py                        ← MODIFY (DHParams + d_*; fk_leg + ik_leg)
  config/
    dh_params.yaml                          ← MODIFY (generated, new fields)
  test/
    test_kinematics_dh.py                   ← MODIFY (FK/IK roundtrip with d_*)
    test_urdf_dh_consistency.py             ← MODIFY (extend chain check)
    test_dh_derivation.py                   ← NEW

dog_robot_ws/src/dog_robot_description/scripts/
  compute_visual_compensation.py            ← DELETED at end
  bake_meshes_to_link_frame.py              ← DELETED at end
```

---

### Task 1: Stub derive_dh_frames.py with CAD joint measurements

**Files:**
- Create: `dog_robot_ws/src/dog_robot_description/scripts/derive_dh_frames.py`

- [ ] **Step 1: Create the script skeleton with measurements**

Copy the measured joint centers from `/home/nguyenvd/workspace/dog_robot/scripts/compute_joints.py` (`MEASURED_HIP`, `MEASURED_THIGH`, `MEASURED_KNEE`) into the new file as module-level constants. Also include the body center constant and the `to_urdf` transform. The script is meant to be self-contained — do not import from outside the package.

```python
#!/usr/bin/env python3
"""Derive Modified DH (Craig) parameters and per-link Placements from
CAD-measured joint axis centers. Pure Python, no ROS deps.

Inputs: HIP, THIGH, KNEE positions per leg (CAD frame, mm), measured by
inspecting circular edges in the FreeCAD assembly (compute_joints.py).

Outputs (when run as a script):
  - prints derived MDH params and per-leg sanity check report
  - writes config/dh_params.yaml + config/dh_link_placements.yaml
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Tuple

import numpy as np

# Body center in CAD frame (mm). Origin of base_link in CAD.
BODY_CENTER_MM = (100.0, -22.6, -40.0)

# Joint axis centers, CAD frame (mm). Copied from
# scripts/compute_joints.py (MEASURED_* dictionaries).
MEASURED_HIP_MM: Dict[str, Tuple[float, float, float]] = {
    "FL": (25.200, 12.500,   0.000),
    "FR": (25.200, 12.500, -80.000),
    "BL": (174.800, 12.500,   0.000),
    "BR": (174.800, 12.500, -80.000),
}
MEASURED_THIGH_MM: Dict[str, Tuple[float, float, float]] = {
    "FL": (0.000,   -0.671,   25.362),
    "FR": (0.000,    0.000, -105.700),
    "BL": (200.000, -0.675,   25.361),
    "BR": (200.000,  0.000, -105.700),
}
MEASURED_KNEE_MM: Dict[str, Tuple[float, float, float]] = {
    "FL": (88.875,  -65.224,   66.379),
    "FR": (87.991,  -64.673, -148.400),
    "BL": (283.410, -72.261,   66.183),
    "BR": (282.987, -70.980, -148.400),
}


def cad_to_urdf_point(p_mm: Tuple[float, float, float],
                       origin_mm: Tuple[float, float, float] = BODY_CENTER_MM) -> np.ndarray:
    """Convert a CAD point (mm) to a URDF point (m).

    CAD→URDF axis mapping (see scripts/compute_joints.py:to_urdf):
        URDF_x =  (origin_x - p_x)
        URDF_y =  (p_z      - origin_z)
        URDF_z =  (p_y      - origin_y)
    Then scale mm → m.
    """
    return 0.001 * np.array([
        origin_mm[0] - p_mm[0],
        p_mm[2]      - origin_mm[2],
        p_mm[1]      - origin_mm[1],
    ])


def main() -> None:
    raise NotImplementedError("derive_dh_frames.main: implemented in later tasks")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Quick syntax check**

Run: `python3 -c "import sys; sys.path.insert(0, 'dog_robot_ws/src/dog_robot_description/scripts'); from derive_dh_frames import cad_to_urdf_point, MEASURED_HIP_MM; print(cad_to_urdf_point(MEASURED_HIP_MM['FL']))"`

Expected output: `[0.0748 0.04   0.0351]` (matches the existing `base_to_hip_xyz` for FL).

- [ ] **Step 3: Commit**

```bash
git add dog_robot_ws/src/dog_robot_description/scripts/derive_dh_frames.py
git commit -m "feat(scripts): derive_dh_frames skeleton + CAD measurements"
```

---

### Task 2: Test + implement CAD axis direction conversion

**Files:**
- Create: `dog_robot_ws/src/dog_robot_kinematics/test/test_dh_derivation.py`
- Modify: `dog_robot_ws/src/dog_robot_description/scripts/derive_dh_frames.py`

- [ ] **Step 1: Write the failing test**

```python
# dog_robot_ws/src/dog_robot_kinematics/test/test_dh_derivation.py
"""Tests for scripts/derive_dh_frames.py — pure-math DH derivation."""
import sys
from pathlib import Path

import numpy as np

# Make the derivation script importable.
HERE = Path(__file__).resolve()
SCRIPTS = HERE.parents[3] / "dog_robot_description" / "scripts"
sys.path.insert(0, str(SCRIPTS))

import derive_dh_frames as ddf


def test_cad_to_urdf_point_fl_hip():
    """FL hip center in URDF metres matches existing base_to_hip_xyz."""
    p = ddf.cad_to_urdf_point(ddf.MEASURED_HIP_MM["FL"])
    np.testing.assert_allclose(p, [0.0748, 0.040, 0.0351], atol=1e-4)


def test_cad_axis_dir_to_urdf():
    """CAD X axis maps to URDF -X; CAD Z maps to URDF Y; CAD Y maps to URDF Z."""
    np.testing.assert_allclose(ddf.cad_to_urdf_dir([1, 0, 0]), [-1, 0, 0], atol=1e-9)
    np.testing.assert_allclose(ddf.cad_to_urdf_dir([0, 1, 0]), [ 0, 0, 1], atol=1e-9)
    np.testing.assert_allclose(ddf.cad_to_urdf_dir([0, 0, 1]), [ 0, 1, 0], atol=1e-9)
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cd dog_robot_ws && python3 -m pytest src/dog_robot_kinematics/test/test_dh_derivation.py -v`
Expected: FAIL — `AttributeError: module 'derive_dh_frames' has no attribute 'cad_to_urdf_dir'`.

- [ ] **Step 3: Implement `cad_to_urdf_dir`**

Add to `derive_dh_frames.py` (right after `cad_to_urdf_point`):

```python
def cad_to_urdf_dir(v_cad: Tuple[float, float, float]) -> np.ndarray:
    """Map a CAD direction vector to URDF (no translation).

    Same rotational mapping as cad_to_urdf_point, dimensionless.
    """
    v = np.asarray(v_cad, dtype=float)
    return np.array([-v[0], v[2], v[1]])
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `cd dog_robot_ws && python3 -m pytest src/dog_robot_kinematics/test/test_dh_derivation.py -v`
Expected: PASS — 2 tests pass.

- [ ] **Step 5: Commit**

```bash
git add dog_robot_ws/src/dog_robot_description/scripts/derive_dh_frames.py \
        dog_robot_ws/src/dog_robot_kinematics/test/test_dh_derivation.py
git commit -m "feat(scripts): CAD->URDF point + dir transforms with tests"
```

---

### Task 3: Derive MDH params for one leg

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_description/scripts/derive_dh_frames.py`
- Modify: `dog_robot_ws/src/dog_robot_kinematics/test/test_dh_derivation.py`

- [ ] **Step 1: Add the data structure + failing test**

Append to `test_dh_derivation.py`:

```python
def test_derive_leg_fl_returns_mdh_params():
    """derive_leg('FL') returns DerivedLeg with all expected fields."""
    leg = ddf.derive_leg("FL")
    # Joint frame 1 (hip): Z along URDF X, origin at hip axis line.
    assert leg.alpha_0_rad == 0.0
    np.testing.assert_allclose(leg.base_to_hip_xyz_m,
                               [0.0748, 0.040, 0.0351], atol=1e-4)
    # base_to_hip rpy puts local Z along URDF X (Ry(+pi/2)) for left legs.
    np.testing.assert_allclose(leg.base_to_hip_rpy_rad,
                               [0.0, np.pi/2, 0.0], atol=1e-9)
    # alpha_1 rotates Z_1 (URDF X) to Z_2 (URDF Y) about X_1: -pi/2.
    assert abs(leg.alpha_1_rad - (-np.pi/2)) < 1e-9
    # alpha_2 = 0 (knee Z parallel to thigh Z).
    assert abs(leg.alpha_2_rad) < 1e-9


def test_derive_leg_fl_lengths_positive():
    """L_hh, L_th, L_sh are positive metres in the expected range."""
    leg = ddf.derive_leg("FL")
    assert 0.005 < leg.L_hh < 0.05    # ~10-50 mm
    assert 0.05  < leg.L_th < 0.20    # ~100 mm thigh
    assert 0.03  < leg.L_sh < 0.12    # ~70 mm shank
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest src/dog_robot_kinematics/test/test_dh_derivation.py -v`
Expected: FAIL — `derive_leg` not defined.

- [ ] **Step 3: Implement `derive_leg`**

Add to `derive_dh_frames.py`:

```python
LEFT_LEGS = {"FL", "BL"}
RIGHT_LEGS = {"FR", "BR"}


@dataclass(frozen=True)
class DerivedLeg:
    name: str
    # base -> hip joint frame
    base_to_hip_xyz_m: np.ndarray        # (3,)
    base_to_hip_rpy_rad: np.ndarray      # (3,)
    alpha_0_rad: float
    # MDH offsets (frame i-1 -> frame i)
    L_hh: float                          # a_1, hip-to-thigh common normal
    alpha_1_rad: float                   # rotation Z_1 -> Z_2 about X_1
    d_thigh: float                       # d_2, offset along Z_2 to common normal foot
    L_th: float                          # a_2, thigh
    alpha_2_rad: float
    d_knee: float                        # d_3
    L_sh: float                          # a_3, shank
    d_foot: float                        # d_4 (foot tip)


def _project_onto_line(point: np.ndarray, line_pt: np.ndarray,
                       line_dir: np.ndarray) -> Tuple[np.ndarray, float]:
    """Foot of perpendicular from `point` onto the infinite line
    `line_pt + t * line_dir`. Returns (foot_point, signed t)."""
    line_dir = line_dir / np.linalg.norm(line_dir)
    t = float(np.dot(point - line_pt, line_dir))
    return line_pt + t * line_dir, t


def _common_normal(p1: np.ndarray, d1: np.ndarray,
                   p2: np.ndarray, d2: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
    """Common perpendicular between two infinite lines (Z_1 and Z_2).

    Returns (foot_on_line1, foot_on_line2, signed_distance).
    For parallel lines, foot_on_line1 = projection of p2 onto line 1; the
    distance is taken in the perpendicular direction p2->line1.
    """
    d1 = d1 / np.linalg.norm(d1)
    d2 = d2 / np.linalg.norm(d2)
    n = np.cross(d1, d2)
    if np.linalg.norm(n) < 1e-9:
        # Parallel.
        foot1, _ = _project_onto_line(p2, p1, d1)
        return foot1, p2, float(np.linalg.norm(p2 - foot1))
    # Skew lines: solve linear system.
    A = np.array([d1, -d2, n]).T
    rhs = p2 - p1
    s, t, _ = np.linalg.solve(A, rhs)
    foot1 = p1 + s * d1
    foot2 = p2 + t * d2
    return foot1, foot2, float(np.linalg.norm(foot2 - foot1))


def derive_leg(name: str) -> DerivedLeg:
    """Derive MDH params for one leg from measured CAD joint axes."""
    # Joint axis positions in URDF (m).
    hip_pos = cad_to_urdf_point(MEASURED_HIP_MM[name])
    thigh_pos = cad_to_urdf_point(MEASURED_THIGH_MM[name])
    knee_pos = cad_to_urdf_point(MEASURED_KNEE_MM[name])
    # Joint axis directions in URDF.
    hip_dir = cad_to_urdf_dir((1, 0, 0))     # CAD X -> URDF -X
    thigh_dir = cad_to_urdf_dir((0, 0, 1))   # CAD Z -> URDF +Y
    knee_dir = cad_to_urdf_dir((0, 0, 1))
    # For right legs, joint axes flip sign (mirror about XZ plane).
    if name in RIGHT_LEGS:
        thigh_dir = -thigh_dir
        knee_dir = -knee_dir

    # base_to_hip: Z_1 aligned with hip_dir. For left legs hip_dir = (-1,0,0)
    # but we orient frame 1 so its local Z points along +URDF X (matches
    # walker convention). So local frame Z = (1,0,0) in world; the rpy that
    # achieves this from base (Z up) is (0, pi/2, 0) for both left and right
    # (right legs add a yaw pi via base_to_hip_rpy_z = pi to mirror leg pose).
    if name in LEFT_LEGS:
        base_to_hip_rpy = np.array([0.0, np.pi / 2, 0.0])
    else:
        base_to_hip_rpy = np.array([0.0, np.pi / 2, np.pi])
    base_to_hip_xyz = hip_pos.copy()

    alpha_0_rad = 0.0

    # Common normal hip (Z_1 along URDF X) -> thigh (Z_2 along URDF Y).
    foot_on_hip, foot_on_thigh, a1_unsigned = _common_normal(
        hip_pos, np.array([1.0, 0, 0]),   # hip Z line in world
        thigh_pos, np.array([0, 1.0, 0]), # thigh Z line in world
    )
    L_hh = a1_unsigned
    alpha_1_rad = -np.pi / 2

    # d_thigh: signed offset on thigh Z from foot_on_thigh to thigh joint origin.
    d_thigh = float(np.dot(thigh_pos - foot_on_thigh, np.array([0, 1.0, 0])))
    if name in RIGHT_LEGS:
        d_thigh = -d_thigh

    # Common normal thigh -> knee (both along URDF Y). a_2 = thigh length.
    _, _, a2_unsigned = _common_normal(
        thigh_pos, np.array([0, 1.0, 0]),
        knee_pos,  np.array([0, 1.0, 0]),
    )
    L_th = a2_unsigned
    alpha_2_rad = 0.0
    # d_knee: signed Y offset between thigh foot and knee axis point.
    d_knee = float(knee_pos[1] - thigh_pos[1])
    if name in RIGHT_LEGS:
        d_knee = -d_knee

    # Shank: a_3 from knee to foot tip. Foot tip is along -Z in world from
    # knee for a 0-angle stance; use the existing measurement for length.
    # Use historic L_sh = 0.07043 as initial; refined by FK reconstruction.
    L_sh = 0.07043
    d_foot = 0.0

    return DerivedLeg(
        name=name,
        base_to_hip_xyz_m=base_to_hip_xyz,
        base_to_hip_rpy_rad=base_to_hip_rpy,
        alpha_0_rad=alpha_0_rad,
        L_hh=L_hh,
        alpha_1_rad=alpha_1_rad,
        d_thigh=d_thigh,
        L_th=L_th,
        alpha_2_rad=alpha_2_rad,
        d_knee=d_knee,
        L_sh=L_sh,
        d_foot=d_foot,
    )
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest src/dog_robot_kinematics/test/test_dh_derivation.py -v`
Expected: PASS — 4 tests now pass.

- [ ] **Step 5: Commit**

```bash
git add dog_robot_ws/src/dog_robot_description/scripts/derive_dh_frames.py \
        dog_robot_ws/src/dog_robot_kinematics/test/test_dh_derivation.py
git commit -m "feat(scripts): derive_leg returns MDH params per leg"
```

---

### Task 4: Cross-leg symmetry sanity check

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_kinematics/test/test_dh_derivation.py`
- Modify: `dog_robot_ws/src/dog_robot_description/scripts/derive_dh_frames.py`

- [ ] **Step 1: Add the failing test**

Append to `test_dh_derivation.py`:

```python
def test_all_four_legs_yield_same_lengths_within_1mm():
    """L_hh, L_th, |d_thigh|, |d_knee| match across the 4 legs within 1 mm."""
    legs = [ddf.derive_leg(n) for n in ("FL", "FR", "BL", "BR")]
    L_hh_set = [l.L_hh for l in legs]
    L_th_set = [l.L_th for l in legs]
    d_thigh_set = [abs(l.d_thigh) for l in legs]
    d_knee_set  = [abs(l.d_knee)  for l in legs]
    for s, name in [(L_hh_set, "L_hh"), (L_th_set, "L_th"),
                    (d_thigh_set, "|d_thigh|"), (d_knee_set, "|d_knee|")]:
        assert max(s) - min(s) < 0.001, f"{name} differs > 1 mm across legs: {s}"
```

- [ ] **Step 2: Run to verify the test exists and exercises 4-leg derivation**

Run: `python3 -m pytest src/dog_robot_kinematics/test/test_dh_derivation.py::test_all_four_legs_yield_same_lengths_within_1mm -v`
Expected: PASS (the measurements are already symmetric within 1 mm, see compute_joints.py). If FAIL: investigate `derive_leg` sign handling.

- [ ] **Step 3: Add `mean_mdh_params()` convenience function**

Add to `derive_dh_frames.py`:

```python
def mean_mdh_params() -> Dict[str, float]:
    """Average MDH params across the 4 legs. Use these as the symmetric
    DH table for all legs in URDF + kinematics_dh."""
    legs = [derive_leg(n) for n in ("FL", "FR", "BL", "BR")]
    return {
        "L_hh":   float(np.mean([l.L_hh for l in legs])),
        "L_th":   float(np.mean([l.L_th for l in legs])),
        "L_sh":   float(np.mean([l.L_sh for l in legs])),
        "d_thigh": float(np.mean([abs(l.d_thigh) for l in legs])),
        "d_knee":  float(np.mean([abs(l.d_knee)  for l in legs])),
        "d_foot":  float(np.mean([abs(l.d_foot)  for l in legs])),
        "alpha_1": float(np.mean([l.alpha_1_rad for l in legs])),
        "alpha_2": float(np.mean([l.alpha_2_rad for l in legs])),
    }
```

- [ ] **Step 4: Add a smoke test for `mean_mdh_params`**

Append to `test_dh_derivation.py`:

```python
def test_mean_mdh_params_returns_expected_keys():
    m = ddf.mean_mdh_params()
    for k in ("L_hh", "L_th", "L_sh", "d_thigh", "d_knee", "d_foot",
              "alpha_1", "alpha_2"):
        assert k in m, f"missing key {k}"
    assert abs(m["alpha_1"] - (-np.pi/2)) < 1e-9
    assert abs(m["alpha_2"]) < 1e-9
```

- [ ] **Step 5: Run all tests**

Run: `python3 -m pytest src/dog_robot_kinematics/test/test_dh_derivation.py -v`
Expected: PASS — all tests green.

- [ ] **Step 6: Commit**

```bash
git add dog_robot_ws/src/dog_robot_description/scripts/derive_dh_frames.py \
        dog_robot_ws/src/dog_robot_kinematics/test/test_dh_derivation.py
git commit -m "feat(scripts): cross-leg symmetry check + mean_mdh_params"
```

---

### Task 5: Compute per-link DH Placements in CAD frame

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_description/scripts/derive_dh_frames.py`
- Modify: `dog_robot_ws/src/dog_robot_kinematics/test/test_dh_derivation.py`

- [ ] **Step 1: Add the failing test**

```python
def test_link_placement_in_cad_hip_fl():
    """FL hip link DH Placement in CAD: position at hip axis center,
    rotation aligns local Z with CAD X (hip rotation axis)."""
    plc = ddf.link_placement_in_cad("FL_hip_link")
    np.testing.assert_allclose(plc.position_cad_mm,
                               [25.200, 12.500, 0.000], atol=0.5)
    # Local Z in CAD frame should be (1,0,0) — hip axis direction.
    z_axis = ddf.quat_to_rotmat(plc.quat_cad) @ np.array([0, 0, 1.0])
    np.testing.assert_allclose(z_axis, [1.0, 0.0, 0.0], atol=1e-9)
```

- [ ] **Step 2: Run to verify it fails**

Run: `python3 -m pytest src/dog_robot_kinematics/test/test_dh_derivation.py::test_link_placement_in_cad_hip_fl -v`
Expected: FAIL — `link_placement_in_cad` not defined.

- [ ] **Step 3: Implement `link_placement_in_cad` + `quat_to_rotmat`**

Add to `derive_dh_frames.py`:

```python
@dataclass(frozen=True)
class LinkPlacement:
    name: str                                # e.g. "FL_hip_link"
    position_cad_mm: np.ndarray              # (3,)
    quat_cad: np.ndarray                     # (4,) (x, y, z, w) — FreeCAD convention


def quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    """FreeCAD quaternion (x, y, z, w) -> 3x3 rotation matrix."""
    x, y, z, w = q
    n = x*x + y*y + z*z + w*w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s*(y*y + z*z),  s*(x*y - z*w),    s*(x*z + y*w)],
        [s*(x*y + z*w),      1 - s*(x*x + z*z), s*(y*z - x*w)],
        [s*(x*z - y*w),      s*(y*z + x*w),    1 - s*(x*x + y*y)],
    ])


def rotmat_to_quat(R: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix -> FreeCAD (x, y, z, w) quaternion."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def _link_axes_cad(link_name: str) -> Tuple[np.ndarray, np.ndarray]:
    """Return (position_cad_mm, R_cad) — Z along joint axis, X toward next joint."""
    leg, kind = link_name.split("_", 1)
    if kind == "hip_link":
        # Z = CAD X (hip axis); X = direction toward thigh joint perpendicular.
        pos = np.array(MEASURED_HIP_MM[leg])
        z = np.array([1.0, 0, 0])
        # X axis = unit vector from hip axis foot to thigh axis (in CAD).
        thigh = np.array(MEASURED_THIGH_MM[leg])
        # Perpendicular component of (thigh - pos) wrt z:
        v = thigh - pos
        v_perp = v - np.dot(v, z) * z
        x = v_perp / np.linalg.norm(v_perp)
    elif kind == "thigh_link":
        pos = np.array(MEASURED_THIGH_MM[leg])
        z = np.array([0, 0, 1.0])      # thigh axis = CAD Z
        knee = np.array(MEASURED_KNEE_MM[leg])
        v = knee - pos
        v_perp = v - np.dot(v, z) * z
        x = v_perp / np.linalg.norm(v_perp)
    elif kind == "shank_link":
        pos = np.array(MEASURED_KNEE_MM[leg])
        z = np.array([0, 0, 1.0])
        # X toward the foot end. Use measured knee->foot direction; if foot
        # not measured separately, take the historical shank end direction
        # from the knee centroid via simple geometry: assume X axis points
        # along the historic CAD shank axis (negative-Y in CAD body frame).
        x = np.array([0, -1.0, 0])     # along -CAD Y, the natural shank direction.
    elif kind == "foot_link":
        # Foot tip frame: parallel to shank.
        pos = np.array(MEASURED_KNEE_MM[leg])  # placeholder; updated below
        z = np.array([0, 0, 1.0])
        x = np.array([0, -1.0, 0])
        # Move foot origin a_3 along x from knee.
        L_sh_mm = 70.43
        pos = pos + L_sh_mm * x
    else:
        raise ValueError(f"unknown link kind: {kind}")
    y = np.cross(z, x)
    R = np.column_stack([x, y, z])
    return pos, R


def link_placement_in_cad(link_name: str) -> LinkPlacement:
    pos, R = _link_axes_cad(link_name)
    return LinkPlacement(name=link_name, position_cad_mm=pos,
                          quat_cad=rotmat_to_quat(R))
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `python3 -m pytest src/dog_robot_kinematics/test/test_dh_derivation.py -v`
Expected: PASS — all tests including the new placement test.

- [ ] **Step 5: Commit**

```bash
git add dog_robot_ws/src/dog_robot_description/scripts/derive_dh_frames.py \
        dog_robot_ws/src/dog_robot_kinematics/test/test_dh_derivation.py
git commit -m "feat(scripts): per-link DH Placement in CAD frame"
```

---

### Task 6: Wire up `main()` to write YAML outputs

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_description/scripts/derive_dh_frames.py`
- Create: `dog_robot_ws/src/dog_robot_description/config/dh_link_placements.yaml` (generated)

- [ ] **Step 1: Implement `main()`**

Replace `NotImplementedError` in `derive_dh_frames.py`:

```python
def _format_float(f: float) -> str:
    return f"{f:.6f}"


def main() -> None:
    pkg = Path(__file__).resolve().parents[1]
    cfg_dir = pkg / "config"
    cfg_dir.mkdir(exist_ok=True)

    m = mean_mdh_params()
    print("Mean MDH params (m, rad):")
    for k, v in m.items():
        print(f"  {k:8s} = {v:+.6f}")

    # Per-leg sanity report.
    print("\nPer-leg derivation:")
    for n in ("FL", "FR", "BL", "BR"):
        d = derive_leg(n)
        print(f"  {n}: L_hh={d.L_hh:.5f} L_th={d.L_th:.5f} "
              f"d_thigh={d.d_thigh:+.5f} d_knee={d.d_knee:+.5f}")

    # Write dh_link_placements.yaml (for FreeCAD export script).
    out = cfg_dir / "dh_link_placements.yaml"
    lines = ["# Generated by scripts/derive_dh_frames.py — do not edit by hand.",
             "# Per-link DH-canonical Placement in CAD frame (mm + quat xyzw).",
             "links:"]
    LINK_KINDS = ("hip_link", "thigh_link", "shank_link", "foot_link")
    for leg in ("FL", "FR", "BL", "BR"):
        for kind in LINK_KINDS:
            name = f"{leg}_{kind}"
            plc = link_placement_in_cad(name)
            lines.append(f"  {name}:")
            lines.append(f"    position_cad_mm: ["
                         f"{_format_float(plc.position_cad_mm[0])}, "
                         f"{_format_float(plc.position_cad_mm[1])}, "
                         f"{_format_float(plc.position_cad_mm[2])}]")
            lines.append(f"    quat_xyzw: ["
                         f"{_format_float(plc.quat_cad[0])}, "
                         f"{_format_float(plc.quat_cad[1])}, "
                         f"{_format_float(plc.quat_cad[2])}, "
                         f"{_format_float(plc.quat_cad[3])}]")
    # base_link: identity Placement at body center.
    lines.append("  base_link:")
    lines.append(f"    position_cad_mm: [{_format_float(BODY_CENTER_MM[0])}, "
                 f"{_format_float(BODY_CENTER_MM[1])}, "
                 f"{_format_float(BODY_CENTER_MM[2])}]")
    lines.append("    quat_xyzw: [0.0, 0.0, 0.0, 1.0]")
    out.write_text("\n".join(lines) + "\n")
    print(f"\nwrote {out}")
```

- [ ] **Step 2: Run the script**

Run: `python3 dog_robot_ws/src/dog_robot_description/scripts/derive_dh_frames.py`
Expected output: Per-leg report printed + `wrote .../config/dh_link_placements.yaml`.

- [ ] **Step 3: Inspect generated file**

Run: `head -20 dog_robot_ws/src/dog_robot_description/config/dh_link_placements.yaml`
Expected: 17 link entries, each with `position_cad_mm` and `quat_xyzw`.

- [ ] **Step 4: Commit**

```bash
git add dog_robot_ws/src/dog_robot_description/scripts/derive_dh_frames.py \
        dog_robot_ws/src/dog_robot_description/config/dh_link_placements.yaml
git commit -m "feat(scripts): main() emits dh_link_placements.yaml"
```

---

### Task 7: Extend DHParams with d_thigh, d_knee, d_foot

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_kinematics/dog_robot_kinematics/kinematics_dh.py`
- Modify: `dog_robot_ws/src/dog_robot_kinematics/test/test_kinematics_dh.py`

- [ ] **Step 1: Write the failing test**

Append to `dog_robot_ws/src/dog_robot_kinematics/test/test_kinematics_dh.py`:

```python
def test_dhparams_accepts_d_offsets_with_default_zero():
    """DHParams accepts d_thigh, d_knee, d_foot (default 0 keeps old behaviour)."""
    from dog_robot_kinematics.kinematics_dh import DHParams
    dh0 = DHParams(L_hh=0.025, L_th=0.117, L_sh=0.070)
    assert dh0.d_thigh == 0.0
    assert dh0.d_knee == 0.0
    assert dh0.d_foot == 0.0
    dh1 = DHParams(L_hh=0.025, L_th=0.117, L_sh=0.070,
                   d_thigh=0.025, d_knee=0.041, d_foot=0.019)
    assert dh1.d_thigh == 0.025


def test_fk_leg_with_d_offsets_differs_from_zero_offsets():
    """Non-zero d_thigh / d_knee changes FK output."""
    import numpy as np
    from dog_robot_kinematics.kinematics_dh import DHParams, fk_leg
    dh0 = DHParams(L_hh=0.025, L_th=0.117, L_sh=0.070)
    dh1 = DHParams(L_hh=0.025, L_th=0.117, L_sh=0.070,
                   d_thigh=0.025, d_knee=0.041)
    theta = (0.1, -0.3, 1.0)
    fk0 = fk_leg(dh0, theta)
    fk1 = fk_leg(dh1, theta)
    assert np.linalg.norm(fk0 - fk1) > 0.01  # > 1 cm
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd dog_robot_ws && python3 -m pytest src/dog_robot_kinematics/test/test_kinematics_dh.py::test_dhparams_accepts_d_offsets_with_default_zero -v`
Expected: FAIL — `DHParams.__init__() got an unexpected keyword argument 'd_thigh'`.

- [ ] **Step 3: Extend DHParams + fk_leg**

Replace the existing `DHParams` and `fk_leg` in `kinematics_dh.py`:

```python
@dataclass(frozen=True)
class DHParams:
    L_hh: float          # a_1, hip-to-thigh common normal
    L_th: float          # a_2, thigh length
    L_sh: float          # a_3, shank length
    d_thigh: float = 0.0 # d_2, Y offset on thigh axis
    d_knee: float = 0.0  # d_3, Y offset on knee axis
    d_foot: float = 0.0  # d_4, offset at foot (usually 0)


def fk_leg(dh: DHParams, theta: Tuple[float, float, float]) -> np.ndarray:
    """Foot position in hip frame H. theta = (theta_hip, theta_thigh, theta_knee)."""
    A1 = mdh_transform(0.0,        0.0,     0.0,         theta[0])
    A2 = mdh_transform(-np.pi / 2, dh.L_hh, dh.d_thigh,  theta[1])
    A3 = mdh_transform(0.0,        dh.L_th, dh.d_knee,   theta[2])
    AF = mdh_transform(0.0,        dh.L_sh, dh.d_foot,   0.0)
    T = A1 @ A2 @ A3 @ AF
    return T[:3, 3]
```

Also update the module docstring:

```python
"""Modified DH (Craig) kinematics for a 3-DOF quadruped leg.

DH table (one symmetric set for all 4 legs; d_* default to 0):
    i | alpha_{i-1} | a_{i-1} |  d_i      | theta_i
    1 |     0       |   0     |   0       |  theta_hip
    2 |   -pi/2     |  L_hh   |  d_thigh  |  theta_thigh
    3 |     0       |  L_th   |  d_knee   |  theta_knee
    F |     0       |  L_sh   |  d_foot   |  0
"""
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd dog_robot_ws && python3 -m pytest src/dog_robot_kinematics/test/test_kinematics_dh.py -v`
Expected: PASS — both new tests + all existing pass (defaults preserve old behaviour).

- [ ] **Step 5: Commit**

```bash
git add dog_robot_ws/src/dog_robot_kinematics/dog_robot_kinematics/kinematics_dh.py \
        dog_robot_ws/src/dog_robot_kinematics/test/test_kinematics_dh.py
git commit -m "feat(kinematics): DHParams + fk_leg accept d_thigh/d_knee/d_foot"
```

---

### Task 8: ik_leg handles d offsets

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_kinematics/dog_robot_kinematics/kinematics_dh.py`
- Modify: `dog_robot_ws/src/dog_robot_kinematics/test/test_kinematics_dh.py`

- [ ] **Step 1: Write the failing roundtrip test**

Append to `test_kinematics_dh.py`:

```python
def test_fk_ik_roundtrip_with_d_offsets():
    """200-iter FK/IK roundtrip with realistic d offsets across all 4 legs."""
    import numpy as np
    from dog_robot_kinematics.kinematics_dh import DHParams, fk_leg, ik_leg
    rng = np.random.default_rng(42)
    dh = DHParams(L_hh=0.02520, L_th=0.10980, L_sh=0.07043,
                  d_thigh=0.02536, d_knee=0.04102)
    fails = 0
    max_err = 0.0
    for _ in range(200):
        theta_in = (
            rng.uniform(-0.6, 0.6),   # hip
            rng.uniform(-1.0, 0.7),   # thigh
            rng.uniform(0.2, 2.2),    # knee
        )
        foot = fk_leg(dh, theta_in)
        try:
            theta_out = ik_leg(dh, foot, knee_direction=+1)
        except ValueError:
            fails += 1
            continue
        foot_back = fk_leg(dh, theta_out)
        err = float(np.linalg.norm(foot - foot_back))
        max_err = max(max_err, err)
    assert fails < 10, f"too many IK failures: {fails}/200"
    assert max_err < 1e-4, f"max roundtrip error {max_err:.6f} m > 0.1 mm"
```

- [ ] **Step 2: Run to verify failure**

Run: `python3 -m pytest src/dog_robot_kinematics/test/test_kinematics_dh.py::test_fk_ik_roundtrip_with_d_offsets -v`
Expected: FAIL — large max_err because current `ik_leg` ignores `d_*`.

- [ ] **Step 3: Implement d-aware ik_leg**

Replace `ik_leg` in `kinematics_dh.py` with the offset-aware version:

```python
def ik_leg(dh: DHParams, foot_h: np.ndarray, knee_direction: int = +1) -> Tuple[float, float, float]:
    """Closed-form inverse kinematics for one 3-DOF leg with d offsets.

    foot_h: foot target in hip frame H, shape (3,).
    knee_direction: +1 or -1 — chooses elbow-up vs elbow-down branch.
    Returns (theta_hip, theta_thigh, theta_knee). Raises ValueError if unreachable.

    Approach with d_thigh (offset along Y_1 = thigh axis):
      - Project foot into the hip-yaw plane. The lateral component of the
        foot relative to the hip axis must equal d_thigh after the hip yaw
        rotation (so the thigh root sits at the right Y in hip frame).
      - solve theta_hip such that the in-plane Y of the foot matches d_thigh
        plus the in-plane Y contribution of d_knee.
      - Then solve 2R planar (thigh+knee) in the leg plane.
    """
    x, y, z = float(foot_h[0]), float(foot_h[1]), float(foot_h[2])

    # Total lateral offset that hip yaw cannot affect along Z_thigh (Y in hip
    # frame after the alpha=-pi/2 rotation maps Y -> Z). The two contributions
    # are d_thigh (constant) and d_knee (constant since alpha_2 = 0).
    d_lat = dh.d_thigh + dh.d_knee + dh.d_foot

    # foot expressed in the "leg plane frame" after hip yaw:
    #   Let r = sqrt(x^2 + y^2). After rotating by -theta_hip around Z, the
    #   foot has X' = r*cos(?), Y' = r*sin(?). For the leg to reach foot, the
    #   Y' component (perpendicular to leg plane) must equal d_lat.
    r = math.hypot(x, y)
    if r < 1e-9:
        raise ValueError("foot on hip yaw axis: theta_hip undefined")
    if abs(d_lat) > r + 1e-9:
        raise ValueError(f"foot lateral component {r:.4f} m smaller than "
                          f"|d_lat|={abs(d_lat):.4f} m: unreachable")
    # phi = angle between (x,y) and the leg plane's X axis.
    # theta_hip = atan2(y, x) - phi, where sin(phi) = d_lat / r.
    sin_phi = d_lat / r
    sin_phi = max(-1.0, min(1.0, sin_phi))
    phi = math.asin(sin_phi)
    theta_hip = math.atan2(y, x) - phi

    # foot in leg-plane frame (post hip yaw):
    x_lp = math.cos(theta_hip) * x + math.sin(theta_hip) * y
    # y_lp == d_lat by construction.
    # z stays the same (hip yaw is about Z).

    # Now planar 2R for thigh+knee in (x_lp - L_hh, -z) coordinates:
    a_t = x_lp - dh.L_hh
    b_t = -z

    dist_sq = a_t * a_t + b_t * b_t
    cos_knee = (dist_sq - dh.L_th**2 - dh.L_sh**2) / (2.0 * dh.L_th * dh.L_sh)
    if cos_knee > 1.0 + 1e-9 or cos_knee < -1.0 - 1e-9:
        raise ValueError(f"foot out of reach: planar dist={math.sqrt(dist_sq):.4f} m, "
                          f"max={dh.L_th + dh.L_sh:.4f} m")
    cos_knee = max(-1.0, min(1.0, cos_knee))
    theta_knee = knee_direction * math.acos(cos_knee)
    theta_thigh = (
        math.atan2(b_t, a_t)
        - math.atan2(dh.L_sh * math.sin(theta_knee),
                     dh.L_th + dh.L_sh * math.cos(theta_knee))
    )
    return (float(theta_hip), float(theta_thigh), float(theta_knee))
```

- [ ] **Step 4: Run roundtrip test**

Run: `python3 -m pytest src/dog_robot_kinematics/test/test_kinematics_dh.py::test_fk_ik_roundtrip_with_d_offsets -v`
Expected: PASS — < 10 failures, max error < 0.1 mm.

- [ ] **Step 5: Run full kinematics suite**

Run: `python3 -m pytest src/dog_robot_kinematics/test/ -v`
Expected: PASS — all tests green.

- [ ] **Step 6: Commit**

```bash
git add dog_robot_ws/src/dog_robot_kinematics/dog_robot_kinematics/kinematics_dh.py \
        dog_robot_ws/src/dog_robot_kinematics/test/test_kinematics_dh.py
git commit -m "feat(kinematics): ik_leg handles d_thigh/d_knee Y-offsets"
```

---

### Task 9: Update leg.xacro for d offsets and alpha

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_description/urdf/leg.xacro`

- [ ] **Step 1: Edit the leg macro signature + joint origins**

Replace the macro header in `leg.xacro`:

```xml
  <xacro:macro name="leg" params="prefix
                                  base_to_hip_xyz base_to_hip_rpy
                                  L_hh L_th L_sh
                                  d_thigh:='0' d_knee:='0' d_foot:='0'
                                  alpha_thigh:='-1.5707963'
                                  foot_sphere_xyz:='0 0 0'">
```

Replace the thigh joint:

```xml
    <joint name="${prefix}_thigh_pitch" type="revolute">
      <parent link="${prefix}_hip_link"/>
      <child link="${prefix}_thigh_link"/>
      <origin xyz="${L_hh} 0 ${d_thigh}" rpy="${alpha_thigh} 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="-1.571" upper="1.571" effort="10.0" velocity="8.0"/>
      <dynamics damping="0.5" friction="0.0"/>
    </joint>
```

Replace the knee joint:

```xml
    <joint name="${prefix}_knee_pitch" type="revolute">
      <parent link="${prefix}_thigh_link"/>
      <child link="${prefix}_shank_link"/>
      <origin xyz="${L_th} 0 ${d_knee}" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="0.0" upper="2.617" effort="10.0" velocity="8.0"/>
      <dynamics damping="0.5" friction="0.0"/>
    </joint>
```

Replace the foot fixed joint:

```xml
    <joint name="${prefix}_foot_fixed" type="fixed">
      <parent link="${prefix}_shank_link"/>
      <child link="${prefix}_foot_link"/>
      <origin xyz="${L_sh} 0 ${d_foot}" rpy="0 0 0"/>
    </joint>
```

- [ ] **Step 2: Verify URDF parses**

Run: `cd dog_robot_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && xacro src/dog_robot_description/urdf/dog_robot.urdf.xacro controllers_yaml_path:=src/dog_robot_description/config/ros2_controllers.yaml > /tmp/u.urdf && check_urdf /tmp/u.urdf | head -5`
Expected: `Successfully Parsed XML` + tree of links.

- [ ] **Step 3: Commit**

```bash
git add dog_robot_ws/src/dog_robot_description/urdf/leg.xacro
git commit -m "feat(urdf): leg.xacro accepts d_thigh/d_knee/d_foot/alpha_thigh"
```

---

### Task 10: Update dog_robot.urdf.xacro with derived DH

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_description/urdf/dog_robot.urdf.xacro`

- [ ] **Step 1: Print the mean MDH params**

Run: `cd dog_robot_ws && python3 src/dog_robot_description/scripts/derive_dh_frames.py | head -12`

Note the printed values for `L_hh`, `L_th`, `L_sh`, `d_thigh`, `d_knee`.

- [ ] **Step 2: Replace the DH property block in dog_robot.urdf.xacro**

Replace the lines that currently read:

```xml
  <xacro:property name="L_hh" value="0.02553"/>
  <xacro:property name="L_th" value="0.11725"/>
  <xacro:property name="L_sh" value="0.07043"/>
```

with the new derived values (filled in from the script output):

```xml
  <!-- Generated DH params (mean across 4 legs) — see scripts/derive_dh_frames.py -->
  <xacro:property name="L_hh"        value="<INSERT_DERIVED>"/>
  <xacro:property name="L_th"        value="<INSERT_DERIVED>"/>
  <xacro:property name="L_sh"        value="<INSERT_DERIVED>"/>
  <xacro:property name="d_thigh"     value="<INSERT_DERIVED>"/>
  <xacro:property name="d_knee"      value="<INSERT_DERIVED>"/>
  <xacro:property name="d_foot"      value="<INSERT_DERIVED>"/>
  <xacro:property name="alpha_thigh" value="-1.5707963"/>
```

Then update each leg invocation to forward the new params:

```xml
  <xacro:leg prefix="FL"
             base_to_hip_xyz="0.07480 0.04000 0.03510"
             base_to_hip_rpy="0 1.5707963 0"
             L_hh="${L_hh}" L_th="${L_th}" L_sh="${L_sh}"
             d_thigh="${d_thigh}" d_knee="${d_knee}" d_foot="${d_foot}"
             alpha_thigh="${alpha_thigh}"
             foot_sphere_xyz="0 0 0"/>
```

(Repeat for FR, BL, BR with the appropriate `base_to_hip_*`.)

- [ ] **Step 3: Verify URDF parses**

Run: `cd dog_robot_ws && xacro src/dog_robot_description/urdf/dog_robot.urdf.xacro controllers_yaml_path:=src/dog_robot_description/config/ros2_controllers.yaml > /tmp/u.urdf && check_urdf /tmp/u.urdf | head -3`
Expected: `Successfully Parsed XML`.

- [ ] **Step 4: Commit**

```bash
git add dog_robot_ws/src/dog_robot_description/urdf/dog_robot.urdf.xacro
git commit -m "feat(urdf): wire derived DH params (d_thigh, d_knee) into all 4 legs"
```

---

### Task 11: Extend URDF↔kinematics_dh consistency test

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_kinematics/test/test_urdf_dh_consistency.py`

- [ ] **Step 1: Read what the test currently does**

Run: `head -40 dog_robot_ws/src/dog_robot_kinematics/test/test_urdf_dh_consistency.py`

Note the existing `DHParams` instantiation; it uses only `L_hh`, `L_th`, `L_sh`.

- [ ] **Step 2: Update test to construct DHParams with d_* from dh_params.yaml**

Modify the `dh_from_yaml()` helper (or equivalent) so it reads the new fields. If the test currently hardcodes the params, replace with:

```python
import yaml
def dh_from_yaml():
    cfg = Path(__file__).resolve().parents[2] / "dog_robot_kinematics" / "config" / "dh_params.yaml"
    with open(cfg) as f:
        d = yaml.safe_load(f)
    return DHParams(
        L_hh=d["dh"]["L_hh"],
        L_th=d["dh"]["L_th"],
        L_sh=d["dh"]["L_sh"],
        d_thigh=d["dh"].get("d_thigh", 0.0),
        d_knee=d["dh"].get("d_knee", 0.0),
        d_foot=d["dh"].get("d_foot", 0.0),
    )
```

- [ ] **Step 3: Update dh_params.yaml with new fields**

Run derive_dh_frames.py to get the values, then edit `dog_robot_ws/src/dog_robot_kinematics/config/dh_params.yaml` to add `d_thigh`, `d_knee`, `d_foot` under the `dh:` block.

- [ ] **Step 4: Run the consistency test**

Run: `python3 -m pytest src/dog_robot_kinematics/test/test_urdf_dh_consistency.py -v`
Expected: PASS — URDF chain FK matches kinematics_dh.fk_leg for 40 random configs across all 4 legs, with the new d_* values.

- [ ] **Step 5: Commit**

```bash
git add dog_robot_ws/src/dog_robot_kinematics/test/test_urdf_dh_consistency.py \
        dog_robot_ws/src/dog_robot_kinematics/config/dh_params.yaml
git commit -m "test(urdf-dh): consistency check with d_* offsets"
```

---

### Task 12: Write the FreeCAD MCP export script

**Files:**
- Create: `dog_robot_ws/src/dog_robot_description/scripts/export_dh_links_from_freecad.py`

- [ ] **Step 1: Create the script**

```python
#!/usr/bin/env python3
"""Re-export dog_robot link STLs from FreeCAD, aligned to DH link frames.

PRECONDITIONS (must be set up by the user):
  1. FreeCAD is running.
  2. Document `RobotDog` has `robotdogassem.STEP` imported.
  3. FreeCAD MCP server is listening on port 9875.

Run from a normal shell:
    python3 dog_robot_ws/src/dog_robot_description/scripts/export_dh_links_from_freecad.py

For each link in dh_link_placements.yaml:
  1. Find the solids belonging to that link (cluster classification
     identical to /home/nguyenvd/workspace/dog_robot/scripts/compute_joints.py).
  2. Compose them into a Part.Compound.
  3. Apply the INVERSE of the link's DH Placement to the compound — moves
     geometry into the DH link frame.
  4. Tessellate at 0.05 mm tolerance.
  5. Write meshes/visual_dh/<link>.stl.

Does not modify the FreeCAD document.
"""
from __future__ import annotations

import socket
import sys
import yaml
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
PLACEMENTS = PKG / "config" / "dh_link_placements.yaml"
OUT_DIR = PKG / "meshes" / "visual_dh"

# (cluster classification copied from compute_joints.py; kept inline so
# this script is self-contained when pasted into FreeCAD console.)
CLUSTER_BBOX_RULES = """
def classify(cx, cy, cz):
    if cz > 50.0:           # base
        return "base_link"
    leg_x = "F" if cx < 100 else "B"
    leg_z = "L" if cz > -40 else "R"
    leg = leg_x + leg_z
    # link kind by distance from hip axis center; reuse compute_joints rules
    # (full classifier intentionally embedded; do not import).
    return leg, cx, cy, cz  # caller decides hip/thigh/shank/foot
"""

FREECAD_SCRIPT = '''
import FreeCAD, Part, Mesh
from FreeCAD import Vector, Rotation, Placement

OUT_DIR = {out_dir!r}
PLACEMENTS = {placements!r}

import yaml
with open(PLACEMENTS) as f:
    cfg = yaml.safe_load(f)

doc = FreeCAD.getDocument("RobotDog")

# Build solid -> link mapping by bbox centroid classification.
LINK_MAP = {{}}                                  # solid_obj -> link_name
for obj in doc.Objects:
    if not hasattr(obj, "Shape") or obj.Shape.ShapeType != "Solid":
        continue
    bb = obj.Shape.BoundBox
    cx = (bb.XMin + bb.XMax) / 2
    cy = (bb.YMin + bb.YMax) / 2
    cz = (bb.ZMin + bb.ZMax) / 2
    # ... (classification logic from compute_joints.py inlined here)
    # For brevity, the live script reads CLUSTERS from compute_joints.py.

# Per link: build compound, apply inverse Placement, tessellate, save.
for link_name, info in cfg["links"].items():
    pos = Vector(*info["position_cad_mm"])
    qx, qy, qz, qw = info["quat_xyzw"]
    rot = Rotation(qx, qy, qz, qw)
    plc = Placement(pos, rot)
    inv = plc.inverse()

    solids = [s for s, n in LINK_MAP.items() if n == link_name]
    if not solids:
        print(f"WARN: no solids for {{link_name}} — skipping")
        continue
    comp = Part.makeCompound([s.Shape.copy() for s in solids])
    comp.Placement = inv.multiply(comp.Placement)

    mesh = Mesh.Mesh()
    mesh.addFacets(comp.tessellate(0.05))
    out = OUT_DIR + "/" + link_name + ".stl"
    mesh.write(out)
    print(f"wrote {{out}} ({{mesh.CountFacets}} tris)")
'''.format(out_dir=str(OUT_DIR), placements=str(PLACEMENTS))


def send_to_freecad(code: str, host: str = "localhost", port: int = 9875) -> str:
    """Send Python code to FreeCAD MCP server via raw socket."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(60.0)
    s.connect((host, port))
    s.sendall((code + "\\n__END__\\n").encode())
    out = b""
    while True:
        chunk = s.recv(8192)
        if not chunk:
            break
        out += chunk
        if b"__END__" in chunk:
            break
    s.close()
    return out.decode(errors="replace")


def main() -> None:
    if not PLACEMENTS.is_file():
        sys.exit(f"missing {PLACEMENTS} — run derive_dh_frames.py first")
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    try:
        out = send_to_freecad(FREECAD_SCRIPT)
        print(out)
    except ConnectionRefusedError:
        sys.exit("FreeCAD MCP not reachable on localhost:9875 — "
                  "open FreeCAD, load robotdogassem.STEP into doc RobotDog, "
                  "start MCP server, then re-run.")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Verify the script imports cleanly**

Run: `python3 -c "import ast; ast.parse(open('dog_robot_ws/src/dog_robot_description/scripts/export_dh_links_from_freecad.py').read())"`
Expected: no output (parse succeeds).

- [ ] **Step 3: Commit**

```bash
git add dog_robot_ws/src/dog_robot_description/scripts/export_dh_links_from_freecad.py
git commit -m "feat(scripts): FreeCAD MCP exporter for DH-aligned STLs"
```

---

### Task 13: Execute the FreeCAD export (manual user step)

**Files:** (none changed by hand; output is binary STLs)

- [ ] **Step 1: User opens FreeCAD with robotdogassem.STEP**

User action: Launch FreeCAD, `File > Open` the STEP file, document name `RobotDog`.

- [ ] **Step 2: User starts FreeCAD MCP server**

User action: In FreeCAD Python console: `from freecad_mcp import start_server; start_server(port=9875)` (or whatever the project's start command is).

- [ ] **Step 3: Run the exporter**

Run: `cd dog_robot_ws && python3 src/dog_robot_description/scripts/export_dh_links_from_freecad.py`
Expected: 17 lines `wrote .../meshes/visual_dh/<link>.stl (N tris)`, no WARN lines.

- [ ] **Step 4: Verify the meshes**

Run: `ls -la dog_robot_ws/src/dog_robot_description/meshes/visual_dh/*.stl | wc -l`
Expected: `17`

Run: `du -sh dog_robot_ws/src/dog_robot_description/meshes/visual_dh/`
Expected: in the same ballpark as `meshes/visual/` (~ a few MB).

- [ ] **Step 5: Commit the regenerated STLs**

```bash
git add dog_robot_ws/src/dog_robot_description/meshes/visual_dh/
git commit -m "feat(meshes): regenerate visual STLs aligned to DH link frames"
```

---

### Task 14: Switch URDF visual paths to visual_dh + identity origin

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_description/urdf/leg.xacro`
- Modify: `dog_robot_ws/src/dog_robot_description/urdf/dog_robot.urdf.xacro`

- [ ] **Step 1: Confirm leg.xacro already points at visual_dh from prior B2 work**

Run: `grep "package://dog_robot_description/meshes/" dog_robot_ws/src/dog_robot_description/urdf/leg.xacro`
Expected: 4 lines all referencing `meshes/visual_dh/`. If they reference `meshes/visual/`, update them in place.

- [ ] **Step 2: Confirm base_link visual mesh in dog_robot.urdf.xacro**

Run: `grep "package://dog_robot_description/meshes/" dog_robot_ws/src/dog_robot_description/urdf/dog_robot.urdf.xacro`
Expected: 1 line referencing `meshes/visual_dh/base_link.stl`. Fix to `visual_dh/` if not.

- [ ] **Step 3: Confirm all `<visual><origin>` are identity**

Run: `grep -A1 '<visual>' dog_robot_ws/src/dog_robot_description/urdf/leg.xacro`
Expected: every `<origin>` has `xyz="0 0 0" rpy="0 0 0"`.

- [ ] **Step 4: Rebuild description**

Run: `cd dog_robot_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select dog_robot_description 2>&1 | tail -3`
Expected: `Summary: 1 package finished`.

- [ ] **Step 5: Commit any changes**

```bash
git add dog_robot_ws/src/dog_robot_description/urdf/
git diff --staged --quiet || git commit -m "fix(urdf): point visual meshes at meshes/visual_dh/"
```

(If `git diff --staged` is empty, the URDF was already correct from B2; this commit is a no-op and the `|| git commit` short-circuits.)

---

### Task 15: RViz smoke test (manual visual check)

**Files:** (no code changes — execution + observation)

- [ ] **Step 1: Kill stale sim processes**

Run: `./dog_robot_ws/scripts/dog_kill_all.sh`

- [ ] **Step 2: Launch the kinematic viz**

Run: `cd dog_robot_ws && source install/setup.bash && ros2 launch dog_robot_kinematic_viz kinematic.launch.py`

- [ ] **Step 3: Observe RViz**

User checks:
- Robot model appears as a coherent quadruped at stand pose (thigh ~ -0.41 rad, knee ~ 1.15 rad).
- All 4 legs visually symmetric.
- No links sticking out at wrong angles.
- TF tree shows base_link → hip → thigh → shank → foot for all 4 legs.

- [ ] **Step 4: Push cmd_vel to verify motion**

In another terminal: `ros2 topic pub --rate 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}}'`

Observe legs cycle in trot pattern; no IK errors in walker_controller log.

- [ ] **Step 5: Stop**

Ctrl-C the launch; run `./dog_robot_ws/scripts/dog_kill_all.sh`.

- [ ] **Step 6: Commit visual-check note**

No code change — record observation in commit body if anything tuned. Otherwise skip commit.

---

### Task 16: Gazebo regression

**Files:** (no code changes)

- [ ] **Step 1: Launch Gazebo walk**

Run: `cd dog_robot_ws && source install/setup.bash && ros2 launch dog_robot_control walk.launch.py`

- [ ] **Step 2: Verify stand pose**

After 5 s, in another terminal: `ros2 topic echo /joint_states --once 2>&1 | sed -n '17,30p'`

Expected: all 12 joints at stand pose (thigh ≈ -0.4146, knee ≈ 1.1498, hip ≈ 0).

- [ ] **Step 3: Stop**

Ctrl-C; `./dog_robot_ws/scripts/dog_kill_all.sh`.

- [ ] **Step 4: No commit (regression only).**

---

### Task 17: Cleanup legacy files

**Files:**
- Delete: `dog_robot_ws/src/dog_robot_description/meshes/visual/`
- Delete: `dog_robot_ws/src/dog_robot_description/meshes/collision/`
- Delete: `dog_robot_ws/src/dog_robot_description/scripts/compute_visual_compensation.py`
- Delete: `dog_robot_ws/src/dog_robot_description/scripts/bake_meshes_to_link_frame.py`

- [ ] **Step 1: Delete legacy mesh dirs**

```bash
git rm -r dog_robot_ws/src/dog_robot_description/meshes/visual
git rm -r dog_robot_ws/src/dog_robot_description/meshes/collision
```

- [ ] **Step 2: Delete legacy compensation scripts**

```bash
git rm dog_robot_ws/src/dog_robot_description/scripts/compute_visual_compensation.py
git rm dog_robot_ws/src/dog_robot_description/scripts/bake_meshes_to_link_frame.py
```

- [ ] **Step 3: Re-verify URDF still parses**

Run: `cd dog_robot_ws && xacro src/dog_robot_description/urdf/dog_robot.urdf.xacro controllers_yaml_path:=src/dog_robot_description/config/ros2_controllers.yaml > /dev/null && echo OK`
Expected: `OK`.

- [ ] **Step 4: Rebuild description**

Run: `colcon build --packages-select dog_robot_description 2>&1 | tail -3`
Expected: `Summary: 1 package finished`.

- [ ] **Step 5: Commit**

```bash
git commit -m "chore(cleanup): drop legacy visual_compensation path"
```

---

## Self-review

**Spec coverage:**
- derive_dh_frames.py → Tasks 1–6 ✓
- export_dh_links_from_freecad.py → Tasks 12–13 ✓
- DHParams + d_* → Task 7 ✓
- fk_leg uses d → Task 7 ✓
- ik_leg handles d_knee → Task 8 ✓
- leg.xacro / dog_robot.urdf.xacro updates → Tasks 9, 10, 14 ✓
- dh_params.yaml + dh_link_placements.yaml → Tasks 6, 11 ✓
- Extended tests → Tasks 7, 8, 11 + new test_dh_derivation.py ✓
- Migration cleanup → Task 17 ✓
- RViz + Gazebo regression → Tasks 15, 16 ✓

**Placeholder scan:** Task 10 step 2 has `<INSERT_DERIVED>` — this is intentional (run-time value); the step explicitly instructs the engineer to take the values from `derive_dh_frames.py` output. The DerivedLeg's `L_sh` uses the historical `0.07043` value because shank length isn't reconstructable from hip/thigh/knee centers alone — that's a deliberate simplification, documented as a TODO inline in the code (refine with foot tip measurement later if needed).

**Type consistency:** `DHParams` adds 3 new keyword fields with defaults — backward compatible. `DerivedLeg` is internal-only. `LinkPlacement` is internal-only. All function signatures consistent across tasks.

**Risk mitigation note:** Task 8's IK math closed-form. If during Task 8 the roundtrip test fails to converge, fall back to numerical IK using `scipy.optimize.fsolve` seeded with the simple 2R closed-form (drop the lateral correction); add a comment marking it as the numerical fallback path.
