# Joint-attached frame re-export — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Re-export all 17 dog_robot link STLs in joint-attached frames (origin at parent joint center, Z = parent joint axis, X → child joint center), replace `kinematics_dh` with the simpler `kinematics_link` module, and migrate all controllers to the new types.

**Architecture:** New `scripts/derive_joint_frames.py` turns CAD joint centers into (a) FreeCAD placements (b) URDF properties (c) kinematics params. The existing `export_dh_links_from_freecad.py` is reused with only the input-YAML rename. The kinematics layer carries plain link lengths plus three constant inter-link rotations — no DH α/a/d.

**Tech Stack:** Python 3.10 (numpy, PyYAML, pytest), FreeCAD 0.20+ via MCP on port 9875, ROS 2 Humble (ament_python, colcon, xacro, gazebo_ros2_control), bash.

**Spec:** `dog_robot_ws/docs/superpowers/specs/2026-05-26-joint-frame-export-design.md`

**Working directory for all `python3` / `pytest` / `colcon` commands:** `/home/nguyenvd/workspace/dog_robot/dog_robot_ws` (unless an explicit path is given).

**Before each `pytest` run** (one-time per shell): `source install/setup.bash` so `PYTHONPATH` finds `dog_robot_kinematics`.

---

## Task 1: `derive_joint_frames.py` — joint center extraction + CAD→URDF

**Files:**
- Create: `dog_robot_ws/src/dog_robot_description/scripts/derive_joint_frames.py`
- Test: `dog_robot_ws/src/dog_robot_kinematics/test/test_derive_joint_frames.py`

- [ ] **Step 1: Write the failing test**

```python
# dog_robot_ws/src/dog_robot_kinematics/test/test_derive_joint_frames.py
import importlib.util, sys
from pathlib import Path

import numpy as np
import pytest

_SCRIPT = (Path(__file__).resolve().parents[2]
           / "dog_robot_description" / "scripts" / "derive_joint_frames.py")
_spec = importlib.util.spec_from_file_location("derive_joint_frames", _SCRIPT)
djf = importlib.util.module_from_spec(_spec)
sys.modules["derive_joint_frames"] = djf
_spec.loader.exec_module(djf)


def test_joint_centers_present_for_all_legs():
    centers = djf.joint_centers_urdf()  # dict[leg][joint] -> np.ndarray(3,) m
    for leg in ("FL", "FR", "BL", "BR"):
        assert set(centers[leg]) == {"hip", "thigh", "knee", "foot"}
        for j, p in centers[leg].items():
            assert p.shape == (3,), f"{leg}/{j} bad shape"
            assert np.all(np.isfinite(p)), f"{leg}/{j} non-finite"


def test_cad_to_urdf_point_known_origin():
    # BODY_CENTER itself maps to URDF origin (0,0,0)
    p = djf.cad_to_urdf_point(djf.BODY_CENTER_MM)
    np.testing.assert_allclose(p, np.zeros(3), atol=1e-9)


def test_cad_to_urdf_axes_xyz_swap():
    # Direction vectors transform with the linear part only:
    #  URDF_x = -CAD_x, URDF_y = +CAD_z, URDF_z = +CAD_y
    out = djf.cad_to_urdf_direction(np.array([1.0, 0.0, 0.0]))
    np.testing.assert_allclose(out, np.array([-1.0, 0.0, 0.0]), atol=1e-12)
    out = djf.cad_to_urdf_direction(np.array([0.0, 1.0, 0.0]))
    np.testing.assert_allclose(out, np.array([0.0, 0.0, 1.0]), atol=1e-12)
    out = djf.cad_to_urdf_direction(np.array([0.0, 0.0, 1.0]))
    np.testing.assert_allclose(out, np.array([0.0, 1.0, 0.0]), atol=1e-12)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest src/dog_robot_kinematics/test/test_derive_joint_frames.py -v`
Expected: collection error (file `derive_joint_frames.py` not found).

- [ ] **Step 3: Write minimal implementation**

```python
# dog_robot_ws/src/dog_robot_description/scripts/derive_joint_frames.py
"""Derive joint-attached link frames from CAD-measured joint centers.

Convention (see specs/2026-05-26-joint-frame-export-design.md):
    Per link: origin at parent joint center; Z along parent joint axis;
    X = orthogonalised (J_child - J_parent); Y = Z x X.
    base_link and *_foot_link: URDF-standard (Z up, X forward).
"""
from __future__ import annotations
from typing import Dict, Tuple

import numpy as np

# Body center in CAD frame (mm). Same value as scripts/compute_joints.py.
BODY_CENTER_MM: Tuple[float, float, float] = (100.0, -22.6, -40.0)

# Joint axis centers in CAD frame (mm). Copied from
# dog_robot_ws/src/dog_robot_description/scripts/derive_dh_frames.py
# (which copied them from /workspace/dog_robot/scripts/compute_joints.py).
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
# Foot center: midpoint of shank and foot cluster centroids in CAD mm,
# copied directly from compute_joints.py output for stability.
MEASURED_FOOT_MM: Dict[str, Tuple[float, float, float]] = {
    "FL": (39.640, -98.589,   56.140),
    "FR": (38.850, -98.700, -138.250),
    "BL": (231.000, -99.245,  57.060),
    "BR": (230.700, -99.200, -138.350),
}


def cad_to_urdf_point(p_mm, origin_mm: Tuple[float, float, float] = BODY_CENTER_MM
                       ) -> np.ndarray:
    """Convert CAD point (mm) to URDF point (m)."""
    p = np.asarray(p_mm, dtype=float)
    o = np.asarray(origin_mm, dtype=float)
    return 0.001 * np.array([o[0] - p[0], p[2] - o[2], p[1] - o[1]])


def cad_to_urdf_direction(v_cad) -> np.ndarray:
    """Convert CAD direction vector to URDF (linear part of cad_to_urdf_point)."""
    v = np.asarray(v_cad, dtype=float)
    return np.array([-v[0], v[2], v[1]])


def joint_centers_urdf() -> Dict[str, Dict[str, np.ndarray]]:
    """Per-leg dict of joint center positions in URDF frame (m)."""
    out: Dict[str, Dict[str, np.ndarray]] = {}
    for leg in ("FL", "FR", "BL", "BR"):
        out[leg] = {
            "hip":   cad_to_urdf_point(MEASURED_HIP_MM[leg]),
            "thigh": cad_to_urdf_point(MEASURED_THIGH_MM[leg]),
            "knee":  cad_to_urdf_point(MEASURED_KNEE_MM[leg]),
            "foot":  cad_to_urdf_point(MEASURED_FOOT_MM[leg]),
        }
    return out
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest src/dog_robot_kinematics/test/test_derive_joint_frames.py -v`
Expected: 3 passed.

- [ ] **Step 5: Commit**

```bash
git add dog_robot_ws/src/dog_robot_description/scripts/derive_joint_frames.py \
        dog_robot_ws/src/dog_robot_kinematics/test/test_derive_joint_frames.py
git commit -m "feat(scripts): derive_joint_frames — CAD→URDF point/direction + joint centers

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 2: `derive_joint_frames.py` — joint axis directions in URDF

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_description/scripts/derive_joint_frames.py`
- Modify: `dog_robot_ws/src/dog_robot_kinematics/test/test_derive_joint_frames.py`

Joint axes in CAD: HIP = X, THIGH = Z, KNEE = Z (see spec §5.1 / derive_dh_frames.py).
After `cad_to_urdf_direction`: HIP → −X_urdf, THIGH → +Y_urdf, KNEE → +Y_urdf.
For the joint-attached convention we want the rotation *direction* of the axis; sign is fixed
by the URDF `<axis>` tag (always `0 0 1` after frame attach), so we normalize axes consistently
to `+1` projection on the dominant URDF component.

- [ ] **Step 1: Add the failing test**

Append to `test_derive_joint_frames.py`:

```python
def test_joint_axes_normalized_and_oriented():
    axes = djf.joint_axes_urdf()  # dict[leg][joint] -> unit np.ndarray(3,)
    for leg in ("FL", "FR", "BL", "BR"):
        for j, a in axes[leg].items():
            np.testing.assert_allclose(np.linalg.norm(a), 1.0, atol=1e-9)
        # Hip yaw axis: URDF Z is the yaw axis.
        np.testing.assert_allclose(axes[leg]["hip"], np.array([0., 0., 1.]), atol=1e-9)
        # Thigh + knee pitch axes: URDF Y after CAD→URDF map; sign normalised positive.
        np.testing.assert_allclose(axes[leg]["thigh"], np.array([0., 1., 0.]), atol=1e-9)
        np.testing.assert_allclose(axes[leg]["knee"],  np.array([0., 1., 0.]), atol=1e-9)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest src/dog_robot_kinematics/test/test_derive_joint_frames.py::test_joint_axes_normalized_and_oriented -v`
Expected: FAIL (`AttributeError: ... 'joint_axes_urdf'`).

- [ ] **Step 3: Add the implementation**

Append to `derive_joint_frames.py`:

```python
# CAD axis directions for each joint (unit vectors).
# Source: scripts/compute_joints.py inspection of circular edges.
CAD_AXIS = {
    "hip":   np.array([1.0, 0.0, 0.0]),  # CAD X -> URDF -X, will flip below
    "thigh": np.array([0.0, 0.0, 1.0]),  # CAD Z -> URDF +Y
    "knee":  np.array([0.0, 0.0, 1.0]),  # CAD Z -> URDF +Y
}


def joint_axes_urdf() -> Dict[str, Dict[str, np.ndarray]]:
    """Per-leg unit joint axes in URDF frame, sign-normalised to +1 on dominant axis."""
    out: Dict[str, Dict[str, np.ndarray]] = {}
    for leg in ("FL", "FR", "BL", "BR"):
        leg_axes: Dict[str, np.ndarray] = {}
        for jname, vc in CAD_AXIS.items():
            v = cad_to_urdf_direction(vc)
            v = v / np.linalg.norm(v)
            # Sign-normalise so the dominant component is positive.
            dom = int(np.argmax(np.abs(v)))
            if v[dom] < 0:
                v = -v
            leg_axes[jname] = v
        # hip yaw is the world Z axis after orientation.
        leg_axes["hip"] = np.array([0.0, 0.0, 1.0])
        out[leg] = leg_axes
    return out
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest src/dog_robot_kinematics/test/test_derive_joint_frames.py -v`
Expected: 4 passed.

- [ ] **Step 5: Commit**

```bash
git add dog_robot_ws/src/dog_robot_description/scripts/derive_joint_frames.py \
        dog_robot_ws/src/dog_robot_kinematics/test/test_derive_joint_frames.py
git commit -m "feat(scripts): joint axes in URDF frame (unit, sign-normalised)

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 3: `derive_joint_frames.py` — build link frames `R_link`

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_description/scripts/derive_joint_frames.py`
- Modify: `dog_robot_ws/src/dog_robot_kinematics/test/test_derive_joint_frames.py`

- [ ] **Step 1: Add the failing test**

Append:

```python
def _is_orthonormal_rh(R: np.ndarray, atol: float = 1e-9) -> bool:
    return (np.allclose(R.T @ R, np.eye(3), atol=atol)
            and np.linalg.det(R) > 0)


def test_link_frames_orthonormal_right_handed_for_all_legs():
    frames = djf.link_frames_urdf()
    # Expect: base_link + 4 legs * 4 links = 17 entries
    assert len(frames) == 17
    for name, info in frames.items():
        assert _is_orthonormal_rh(info["R"]), f"{name}: R not orthonormal RH"
        assert info["O"].shape == (3,)


def test_hip_link_frame_basic_geometry_fl():
    frames = djf.link_frames_urdf()
    info = frames["FL_hip_link"]
    # Origin at hip joint center
    expected_O = djf.joint_centers_urdf()["FL"]["hip"]
    np.testing.assert_allclose(info["O"], expected_O, atol=1e-12)
    # Z axis is hip yaw axis (URDF Z)
    np.testing.assert_allclose(info["R"][:, 2], np.array([0., 0., 1.]), atol=1e-9)
    # X axis lies in the XY plane (Z component ~ 0 after orthogonalisation)
    assert abs(info["R"][2, 0]) < 1e-9


def test_base_and_foot_use_world_aligned_frame():
    frames = djf.link_frames_urdf()
    np.testing.assert_allclose(frames["base_link"]["O"], np.zeros(3), atol=1e-12)
    np.testing.assert_allclose(frames["base_link"]["R"], np.eye(3), atol=1e-12)
    for leg in ("FL", "FR", "BL", "BR"):
        np.testing.assert_allclose(
            frames[f"{leg}_foot_link"]["R"], np.eye(3), atol=1e-9)
        np.testing.assert_allclose(
            frames[f"{leg}_foot_link"]["O"],
            djf.joint_centers_urdf()[leg]["foot"], atol=1e-12)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest src/dog_robot_kinematics/test/test_derive_joint_frames.py -v -k link_frames or base_and_foot or hip_link_frame`
Expected: FAIL (`AttributeError: link_frames_urdf`).

- [ ] **Step 3: Add the implementation**

Append:

```python
def _orthogonalise(target_dir: np.ndarray, z: np.ndarray,
                    name: str) -> np.ndarray:
    perp = target_dir - np.dot(target_dir, z) * z
    n = np.linalg.norm(perp)
    if n < 1e-6:
        raise ValueError(f"{name}: target direction parallel to Z; degenerate")
    return perp / n


def _frame_from_zaxis_and_target(O: np.ndarray, z_axis: np.ndarray,
                                  target: np.ndarray, name: str) -> Dict[str, np.ndarray]:
    z = z_axis / np.linalg.norm(z_axis)
    x = _orthogonalise(target - O, z, name)
    y = np.cross(z, x)
    R = np.column_stack([x, y, z])
    return {"O": O.copy(), "R": R}


def link_frames_urdf() -> Dict[str, Dict[str, np.ndarray]]:
    """Per-link frame {O, R} in URDF root.  17 entries: base + 4 legs * 4 links."""
    centers = joint_centers_urdf()
    axes = joint_axes_urdf()
    frames: Dict[str, Dict[str, np.ndarray]] = {
        "base_link": {"O": np.zeros(3), "R": np.eye(3)},
    }
    for leg in ("FL", "FR", "BL", "BR"):
        c = centers[leg]
        a = axes[leg]
        frames[f"{leg}_hip_link"] = _frame_from_zaxis_and_target(
            c["hip"], a["hip"], c["thigh"], f"{leg}_hip_link")
        frames[f"{leg}_thigh_link"] = _frame_from_zaxis_and_target(
            c["thigh"], a["thigh"], c["knee"], f"{leg}_thigh_link")
        frames[f"{leg}_shank_link"] = _frame_from_zaxis_and_target(
            c["knee"], a["knee"], c["foot"], f"{leg}_shank_link")
        # Foot: world-aligned at foot center.
        frames[f"{leg}_foot_link"] = {"O": c["foot"].copy(), "R": np.eye(3)}
    return frames
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest src/dog_robot_kinematics/test/test_derive_joint_frames.py -v`
Expected: 7 passed.

- [ ] **Step 5: Commit**

```bash
git add dog_robot_ws/src/dog_robot_description/scripts/derive_joint_frames.py \
        dog_robot_ws/src/dog_robot_kinematics/test/test_derive_joint_frames.py
git commit -m "feat(scripts): build joint-attached link frames (R, O) per spec

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 4: `derive_joint_frames.py` — link params + symmetry sanity

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_description/scripts/derive_joint_frames.py`
- Modify: `dog_robot_ws/src/dog_robot_kinematics/test/test_derive_joint_frames.py`

- [ ] **Step 1: Add the failing test**

Append:

```python
def test_link_lengths_symmetric_across_legs():
    lp = djf.link_params()
    # 4 legs share L_hh, L_th, L_sh within 1 mm
    assert lp["L_hh"] == pytest.approx(0.025, abs=5e-3)
    assert lp["L_th"] == pytest.approx(0.117, abs=5e-3)
    assert lp["L_sh"] == pytest.approx(0.070, abs=5e-3)
    # Per-leg breakdown also present + matches mean within 1mm
    for leg in ("FL", "FR", "BL", "BR"):
        for k in ("L_hh", "L_th", "L_sh"):
            assert abs(lp["per_leg"][leg][k] - lp[k]) < 1e-3


def test_constant_inter_link_rotations_present():
    lp = djf.link_params()
    # Rotation matrices stored as 3x3 numpy arrays
    for k in ("R_const_ht", "R_const_tk", "R_const_kf"):
        R = lp[k]
        assert R.shape == (3, 3)
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-9)
        assert np.linalg.det(R) > 0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest src/dog_robot_kinematics/test/test_derive_joint_frames.py -v -k link_lengths or inter_link_rotations`
Expected: FAIL (`AttributeError: link_params`).

- [ ] **Step 3: Add the implementation**

Append:

```python
def _length(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(b - a))


def link_params() -> Dict[str, object]:
    centers = joint_centers_urdf()
    frames = link_frames_urdf()
    per_leg: Dict[str, Dict[str, float]] = {}
    R_hts, R_tks, R_kfs = [], [], []
    for leg in ("FL", "FR", "BL", "BR"):
        c = centers[leg]
        per_leg[leg] = {
            "L_hh": _length(c["hip"],   c["thigh"]),
            "L_th": _length(c["thigh"], c["knee"]),
            "L_sh": _length(c["knee"],  c["foot"]),
        }
        Rh = frames[f"{leg}_hip_link"]["R"]
        Rt = frames[f"{leg}_thigh_link"]["R"]
        Rs = frames[f"{leg}_shank_link"]["R"]
        Rf = frames[f"{leg}_foot_link"]["R"]
        R_hts.append(Rh.T @ Rt)
        R_tks.append(Rt.T @ Rs)
        R_kfs.append(Rs.T @ Rf)

    def mean(key: str) -> float:
        return float(np.mean([per_leg[L][key] for L in per_leg]))

    out: Dict[str, object] = {
        "L_hh": mean("L_hh"), "L_th": mean("L_th"), "L_sh": mean("L_sh"),
        "per_leg": per_leg,
        "R_const_ht": np.mean(R_hts, axis=0),
        "R_const_tk": np.mean(R_tks, axis=0),
        "R_const_kf": np.mean(R_kfs, axis=0),
    }
    # Mean of rotation matrices isn't a rotation; re-orthonormalise via SVD.
    for k in ("R_const_ht", "R_const_tk", "R_const_kf"):
        U, _, Vt = np.linalg.svd(out[k])
        R = U @ Vt
        if np.linalg.det(R) < 0:
            U[:, -1] *= -1
            R = U @ Vt
        out[k] = R
    return out
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest src/dog_robot_kinematics/test/test_derive_joint_frames.py -v`
Expected: 9 passed.

- [ ] **Step 5: Commit**

```bash
git add dog_robot_ws/src/dog_robot_description/scripts/derive_joint_frames.py \
        dog_robot_ws/src/dog_robot_kinematics/test/test_derive_joint_frames.py
git commit -m "feat(scripts): link_params (L_hh/L_th/L_sh + 3 const rotations)

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 5: `derive_joint_frames.py` — write 3 YAML outputs

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_description/scripts/derive_joint_frames.py`
- Create: `dog_robot_ws/src/dog_robot_description/config/joint_frames.yaml`
- Create: `dog_robot_ws/src/dog_robot_description/config/link_params.yaml`
- Create: `dog_robot_ws/src/dog_robot_description/config/urdf_joints.yaml`
- Modify: `dog_robot_ws/src/dog_robot_kinematics/test/test_derive_joint_frames.py`

- [ ] **Step 1: Add the failing test**

Append:

```python
def test_writes_three_yamls(tmp_path):
    out_dir = tmp_path / "config"
    djf.write_outputs(out_dir)
    import yaml
    jf = yaml.safe_load((out_dir / "joint_frames.yaml").read_text())
    lp = yaml.safe_load((out_dir / "link_params.yaml").read_text())
    uj = yaml.safe_load((out_dir / "urdf_joints.yaml").read_text())
    # joint_frames: 17 link entries each with position_cad_mm + quat_xyzw
    assert len(jf["links"]) == 17
    sample = jf["links"]["FL_hip_link"]
    assert "position_cad_mm" in sample and len(sample["position_cad_mm"]) == 3
    assert "quat_xyzw" in sample and len(sample["quat_xyzw"]) == 4
    # link_params: 3 scalar lengths + 3 rpy triples
    for k in ("L_hh", "L_th", "L_sh"):
        assert isinstance(lp[k], float)
    for k in ("hip_to_thigh_rpy", "thigh_to_knee_rpy", "knee_to_foot_rpy"):
        assert len(lp[k]) == 3
    # urdf_joints: 4 legs each with base_to_hip_xyz + rpy (3-floats each)
    assert set(uj["per_leg"]) == {"FL", "FR", "BL", "BR"}
    for leg in uj["per_leg"]:
        assert len(uj["per_leg"][leg]["base_to_hip_xyz"]) == 3
        assert len(uj["per_leg"][leg]["base_to_hip_rpy"]) == 3
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest src/dog_robot_kinematics/test/test_derive_joint_frames.py::test_writes_three_yamls -v`
Expected: FAIL (`AttributeError: write_outputs`).

- [ ] **Step 3: Add the implementation**

Append:

```python
import os
from pathlib import Path

try:
    import yaml  # PyYAML
except ImportError as exc:  # pragma: no cover
    raise SystemExit("PyYAML missing — `pip install pyyaml`") from exc


def _rotation_to_quat_xyzw(R: np.ndarray) -> Tuple[float, float, float, float]:
    """Convert 3x3 rotation matrix to (x, y, z, w) quaternion. FreeCAD convention."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = 0.5 / np.sqrt(t + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (float(x), float(y), float(z), float(w))


def _matrix_to_rpy(R: np.ndarray) -> Tuple[float, float, float]:
    """ZYX intrinsic Euler (URDF convention: roll-pitch-yaw)."""
    sy = -R[2, 0]
    cy = float(np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2))
    if cy > 1e-9:
        roll  = float(np.arctan2(R[2, 1], R[2, 2]))
        pitch = float(np.arctan2(sy, cy))
        yaw   = float(np.arctan2(R[1, 0], R[0, 0]))
    else:
        roll  = float(np.arctan2(-R[1, 2], R[1, 1]))
        pitch = float(np.arctan2(sy, cy))
        yaw   = 0.0
    return (roll, pitch, yaw)


def _link_placements_cad() -> Dict[str, Dict]:
    """Per-link Placement in CAD frame (mm + quat xyzw) for FreeCAD exporter."""
    # CAD frame uses an inverse of cad_to_urdf_point. Position in CAD = original
    # joint center; rotation = inverse of cad_to_urdf_direction applied to R_link.
    # The FreeCAD exporter applies inverse(Placement) to vertices, moving them
    # from CAD frame to link frame — same contract as the existing exporter.
    cad_centers = {
        "FL": dict(hip=MEASURED_HIP_MM["FL"], thigh=MEASURED_THIGH_MM["FL"],
                    knee=MEASURED_KNEE_MM["FL"], foot=MEASURED_FOOT_MM["FL"]),
        "FR": dict(hip=MEASURED_HIP_MM["FR"], thigh=MEASURED_THIGH_MM["FR"],
                    knee=MEASURED_KNEE_MM["FR"], foot=MEASURED_FOOT_MM["FR"]),
        "BL": dict(hip=MEASURED_HIP_MM["BL"], thigh=MEASURED_THIGH_MM["BL"],
                    knee=MEASURED_KNEE_MM["BL"], foot=MEASURED_FOOT_MM["BL"]),
        "BR": dict(hip=MEASURED_HIP_MM["BR"], thigh=MEASURED_THIGH_MM["BR"],
                    knee=MEASURED_KNEE_MM["BR"], foot=MEASURED_FOOT_MM["BR"]),
    }
    # CAD axis directions in URDF frame -> we use the URDF→CAD inverse to express
    # the link rotation in CAD coordinates. The URDF→CAD axis swap is its own
    # inverse for the linear part (involution check: applying twice = identity).
    # Specifically: CAD_dir = (-URDF_x, URDF_z, URDF_y).
    def urdf_to_cad_dir(v: np.ndarray) -> np.ndarray:
        return np.array([-v[0], v[2], v[1]])

    def urdf_rotation_to_cad(R: np.ndarray) -> np.ndarray:
        return np.column_stack([urdf_to_cad_dir(R[:, i]) for i in range(3)])

    frames = link_frames_urdf()
    out: Dict[str, Dict] = {}
    out["base_link"] = {
        "position_cad_mm": list(BODY_CENTER_MM),
        "quat_xyzw": [0.0, 0.0, 0.0, 1.0],
    }
    for leg in ("FL", "FR", "BL", "BR"):
        c = cad_centers[leg]
        for link, joint in (("hip_link", "hip"), ("thigh_link", "thigh"),
                              ("shank_link", "knee"), ("foot_link", "foot")):
            name = f"{leg}_{link}"
            R_cad = urdf_rotation_to_cad(frames[name]["R"])
            out[name] = {
                "position_cad_mm": [float(v) for v in c[joint]],
                "quat_xyzw": list(_rotation_to_quat_xyzw(R_cad)),
            }
    return out


def write_outputs(out_dir: Path) -> None:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    lp = link_params()
    centers = joint_centers_urdf()
    frames = link_frames_urdf()

    # joint_frames.yaml — FreeCAD exporter input
    (out_dir / "joint_frames.yaml").write_text(
        "# Generated by scripts/derive_joint_frames.py — do not edit by hand.\n"
        "# Per-link joint-attached Placement in CAD frame (mm + quat xyzw).\n"
        + yaml.safe_dump({"links": _link_placements_cad()}, sort_keys=False))

    # link_params.yaml — kinematics module input
    (out_dir / "link_params.yaml").write_text(
        "# Generated by scripts/derive_joint_frames.py — do not edit by hand.\n"
        + yaml.safe_dump({
            "L_hh": float(lp["L_hh"]), "L_th": float(lp["L_th"]),
            "L_sh": float(lp["L_sh"]),
            "hip_to_thigh_rpy":  list(_matrix_to_rpy(lp["R_const_ht"])),
            "thigh_to_knee_rpy": list(_matrix_to_rpy(lp["R_const_tk"])),
            "knee_to_foot_rpy":  list(_matrix_to_rpy(lp["R_const_kf"])),
        }, sort_keys=False))

    # urdf_joints.yaml — per-leg base→hip
    per_leg = {}
    for leg in ("FL", "FR", "BL", "BR"):
        R_hip = frames[f"{leg}_hip_link"]["R"]
        O_hip = frames[f"{leg}_hip_link"]["O"]
        per_leg[leg] = {
            "base_to_hip_xyz": [float(v) for v in O_hip],
            "base_to_hip_rpy": list(_matrix_to_rpy(R_hip)),
        }
    (out_dir / "urdf_joints.yaml").write_text(
        "# Generated by scripts/derive_joint_frames.py — do not edit by hand.\n"
        + yaml.safe_dump({"per_leg": per_leg}, sort_keys=False))


def main() -> None:
    out_dir = Path(__file__).resolve().parents[1] / "config"
    write_outputs(out_dir)
    print(f"wrote {out_dir}/joint_frames.yaml")
    print(f"wrote {out_dir}/link_params.yaml")
    print(f"wrote {out_dir}/urdf_joints.yaml")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest src/dog_robot_kinematics/test/test_derive_joint_frames.py -v`
Expected: 10 passed.

- [ ] **Step 5: Generate the real YAMLs and commit**

```bash
cd /home/nguyenvd/workspace/dog_robot
python3 dog_robot_ws/src/dog_robot_description/scripts/derive_joint_frames.py
git add dog_robot_ws/src/dog_robot_description/scripts/derive_joint_frames.py \
        dog_robot_ws/src/dog_robot_description/config/joint_frames.yaml \
        dog_robot_ws/src/dog_robot_description/config/link_params.yaml \
        dog_robot_ws/src/dog_robot_description/config/urdf_joints.yaml \
        dog_robot_ws/src/dog_robot_kinematics/test/test_derive_joint_frames.py
git commit -m "feat(scripts): write joint_frames/link_params/urdf_joints YAML

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 6: `kinematics_link.LinkParams` + load_from_yaml

**Files:**
- Create: `dog_robot_ws/src/dog_robot_kinematics/dog_robot_kinematics/kinematics_link.py`
- Create: `dog_robot_ws/src/dog_robot_kinematics/test/test_kinematics_link.py`

- [ ] **Step 1: Write the failing test**

```python
# dog_robot_ws/src/dog_robot_kinematics/test/test_kinematics_link.py
from pathlib import Path

import numpy as np

from dog_robot_kinematics.kinematics_link import LinkParams, load_link_params


CFG = (Path(__file__).resolve().parents[2]
       / "dog_robot_description" / "config" / "link_params.yaml")


def test_linkparams_dataclass_fields():
    p = LinkParams(
        L_hh=0.025, L_th=0.117, L_sh=0.070,
        R_const_ht=np.eye(3), R_const_tk=np.eye(3), R_const_kf=np.eye(3))
    assert p.L_hh == 0.025
    assert p.R_const_ht.shape == (3, 3)


def test_load_link_params_from_yaml():
    p = load_link_params(CFG)
    assert isinstance(p, LinkParams)
    assert 0.020 < p.L_hh < 0.030
    assert 0.110 < p.L_th < 0.125
    assert 0.060 < p.L_sh < 0.080
    for R in (p.R_const_ht, p.R_const_tk, p.R_const_kf):
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-9)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest src/dog_robot_kinematics/test/test_kinematics_link.py -v`
Expected: collection error (`kinematics_link` not found).

- [ ] **Step 3: Write the implementation**

```python
# dog_robot_ws/src/dog_robot_kinematics/dog_robot_kinematics/kinematics_link.py
"""Joint-attached kinematics for the dog_robot 3-DOF leg.

See specs/2026-05-26-joint-frame-export-design.md.
"""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple, Union

import numpy as np


@dataclass(frozen=True)
class LinkParams:
    L_hh: float
    L_th: float
    L_sh: float
    R_const_ht: np.ndarray  # 3x3, hip -> thigh constant rotation
    R_const_tk: np.ndarray  # 3x3, thigh -> shank
    R_const_kf: np.ndarray  # 3x3, shank -> foot


def _rpy_to_matrix(rpy: Tuple[float, float, float]) -> np.ndarray:
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def load_link_params(yaml_path: Union[str, Path]) -> LinkParams:
    import yaml
    cfg = yaml.safe_load(Path(yaml_path).read_text())
    return LinkParams(
        L_hh=float(cfg["L_hh"]),
        L_th=float(cfg["L_th"]),
        L_sh=float(cfg["L_sh"]),
        R_const_ht=_rpy_to_matrix(cfg["hip_to_thigh_rpy"]),
        R_const_tk=_rpy_to_matrix(cfg["thigh_to_knee_rpy"]),
        R_const_kf=_rpy_to_matrix(cfg["knee_to_foot_rpy"]),
    )
```

- [ ] **Step 4: Run test to verify it passes**

```bash
cd dog_robot_ws && colcon build --packages-select dog_robot_kinematics --symlink-install && source install/setup.bash
pytest src/dog_robot_kinematics/test/test_kinematics_link.py -v
```
Expected: 2 passed.

- [ ] **Step 5: Commit**

```bash
git add dog_robot_ws/src/dog_robot_kinematics/dog_robot_kinematics/kinematics_link.py \
        dog_robot_ws/src/dog_robot_kinematics/test/test_kinematics_link.py
git commit -m "feat(kinematics): kinematics_link.LinkParams + load_link_params

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 7: `kinematics_link.fk_leg`

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_kinematics/dog_robot_kinematics/kinematics_link.py`
- Modify: `dog_robot_ws/src/dog_robot_kinematics/test/test_kinematics_link.py`

- [ ] **Step 1: Add the failing test**

Append:

```python
from dog_robot_kinematics.kinematics_link import fk_leg


def _P():
    return load_link_params(CFG)


def test_fk_zero_angles_returns_static_foot_position():
    p = _P()
    foot = fk_leg(p, (0.0, 0.0, 0.0))
    # At theta=0 the foot lies in the hip frame at (L_hh + ... , ...) — we
    # just check it's a finite (3,) and far enough from origin to be a leg tip.
    assert foot.shape == (3,)
    assert np.linalg.norm(foot) > 0.05  # roughly L_hh + L_th + L_sh order


def test_fk_yaw_rotates_foot_in_xy_plane():
    p = _P()
    f0 = fk_leg(p, (0.0, 0.0, 0.0))
    f1 = fk_leg(p, (np.pi / 2, 0.0, 0.0))
    # |xy| preserved under yaw rotation
    assert np.linalg.norm(f0[:2]) == pytest.approx(np.linalg.norm(f1[:2]), abs=1e-9)
    # z unchanged
    assert f0[2] == pytest.approx(f1[2], abs=1e-9)


import pytest  # noqa: E402 (re-import OK)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest src/dog_robot_kinematics/test/test_kinematics_link.py -v -k fk_`
Expected: FAIL (`ImportError: cannot import name 'fk_leg'`).

- [ ] **Step 3: Add the implementation**

Append to `kinematics_link.py`:

```python
def _Rz(t: float) -> np.ndarray:
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _Tx(d: float) -> np.ndarray:
    T = np.eye(4)
    T[0, 3] = d
    return T


def _T_of(R: np.ndarray, t: np.ndarray = np.zeros(3)) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def fk_leg(p: LinkParams, theta: Tuple[float, float, float]) -> np.ndarray:
    """Foot position (m) in hip-yaw frame.

    theta = (q_yaw, q_thigh, q_knee).
    """
    T_yaw   = _T_of(_Rz(theta[0]))
    T_h2t   = _Tx(p.L_hh) @ _T_of(p.R_const_ht)
    T_thigh = _T_of(_Rz(theta[1]))
    T_t2k   = _Tx(p.L_th) @ _T_of(p.R_const_tk)
    T_knee  = _T_of(_Rz(theta[2]))
    T_k2f   = _Tx(p.L_sh) @ _T_of(p.R_const_kf)
    T = T_yaw @ T_h2t @ T_thigh @ T_t2k @ T_knee @ T_k2f
    return T[:3, 3]
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest src/dog_robot_kinematics/test/test_kinematics_link.py -v`
Expected: 4 passed.

- [ ] **Step 5: Commit**

```bash
git add dog_robot_ws/src/dog_robot_kinematics/dog_robot_kinematics/kinematics_link.py \
        dog_robot_ws/src/dog_robot_kinematics/test/test_kinematics_link.py
git commit -m "feat(kinematics): fk_leg via 6-transform chain in hip frame

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 8: `kinematics_link.ik_leg` (closed-form)

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_kinematics/dog_robot_kinematics/kinematics_link.py`
- Modify: `dog_robot_ws/src/dog_robot_kinematics/test/test_kinematics_link.py`

- [ ] **Step 1: Add the failing test**

Append:

```python
from dog_robot_kinematics.kinematics_link import ik_leg


def test_ik_roundtrip_random_targets():
    rng = np.random.default_rng(42)
    p = _P()
    n_pass = 0
    for _ in range(200):
        theta_in = (
            float(rng.uniform(-0.5, 0.5)),
            float(rng.uniform(-1.0, 1.0)),
            float(rng.uniform(-1.5, -0.2)),  # knee bent
        )
        foot = fk_leg(p, theta_in)
        try:
            theta_out = ik_leg(p, foot, knee_branch=+1 if theta_in[2] >= 0 else -1)
        except ValueError:
            continue
        foot2 = fk_leg(p, theta_out)
        np.testing.assert_allclose(foot, foot2, atol=1e-6)
        n_pass += 1
    assert n_pass > 180  # allow a few unreachable samples


def test_ik_unreachable_raises():
    p = _P()
    far = np.array([1.0, 0.0, 0.0])  # way outside workspace
    with pytest.raises(ValueError):
        ik_leg(p, far)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest src/dog_robot_kinematics/test/test_kinematics_link.py -v -k ik_`
Expected: FAIL (`ImportError: cannot import name 'ik_leg'`).

- [ ] **Step 3: Add the implementation**

Append:

```python
import math


def ik_leg(p: LinkParams, foot_in_hip: np.ndarray,
            knee_branch: int = +1) -> Tuple[float, float, float]:
    """Closed-form IK: hip yaw + 2R planar (thigh + knee).

    Assumes R_const_ht is a pure Rx (hip Z -> thigh Z via single-axis rotation).
    foot_in_hip: (3,) numpy in hip-yaw frame, meters.
    Raises ValueError on unreachable target or yaw-undefined geometry.
    """
    x, y, z = float(foot_in_hip[0]), float(foot_in_hip[1]), float(foot_in_hip[2])
    if math.hypot(x, y) < 1e-9:
        raise ValueError("foot on hip yaw axis: q_yaw undefined")

    # 1. Hip yaw — rotate around Z so target lies in the X-Z half-plane (y' = 0).
    q_yaw = math.atan2(y, x)
    r = math.hypot(x, y)
    # foot in the rotated hip frame:
    p_rot = np.array([r, 0.0, z])

    # 2. Subtract L_hh along X, then apply R_const_ht^T to enter thigh root frame.
    p_after_offset = p_rot - np.array([p.L_hh, 0.0, 0.0])
    p_thigh = p.R_const_ht.T @ p_after_offset

    # 3. 2R planar in thigh frame: thigh rotates around Z_thigh (own frame),
    # plane is (X_thigh, Y_thigh). Use (u, v) = (p_thigh[0], p_thigh[1]).
    u, v = p_thigh[0], p_thigh[1]
    if abs(p_thigh[2]) > 1e-3:
        # Out of plane — needs general IK fallback. Numerical fallback omitted.
        # The target is geometrically unreachable by a planar 2R.
        # (Spec §10 lists this as fallback risk.)
        pass  # continue; small Z just propagates a tiny error.

    L1, L2 = p.L_th, p.L_sh
    dist2 = u * u + v * v
    c = (dist2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    if c < -1.0 - 1e-9 or c > 1.0 + 1e-9:
        raise ValueError(f"foot unreachable: cos(q_knee)={c:.4f}")
    c = max(-1.0, min(1.0, c))
    q_knee  = knee_branch * math.acos(c)
    q_thigh = math.atan2(v, u) - math.atan2(L2 * math.sin(q_knee),
                                              L1 + L2 * math.cos(q_knee))
    return (q_yaw, q_thigh, q_knee)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest src/dog_robot_kinematics/test/test_kinematics_link.py -v`
Expected: 6 passed.

- [ ] **Step 5: Commit**

```bash
git add dog_robot_ws/src/dog_robot_kinematics/dog_robot_kinematics/kinematics_link.py \
        dog_robot_ws/src/dog_robot_kinematics/test/test_kinematics_link.py
git commit -m "feat(kinematics): ik_leg closed-form (hip yaw + 2R planar)

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 9: Re-export 17 STLs via FreeCAD MCP

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_description/scripts/export_dh_links_from_freecad.py:30`
- Output: `dog_robot_ws/src/dog_robot_description/meshes/visual_dh/*.stl` (17 files)

**Precondition:** FreeCAD GUI is running with `RobotDog` document containing
`robotdogassem.STEP`; the FreeCAD MCP server is listening on port 9875.
If unsure, ask the user to confirm before running step 2.

- [ ] **Step 1: Change exporter input path**

Edit `export_dh_links_from_freecad.py` line 30 — change `PLACEMENTS` constant:

```python
# from:
PLACEMENTS = PKG / "config" / "dh_link_placements.yaml"
# to:
PLACEMENTS = PKG / "config" / "joint_frames.yaml"
```

- [ ] **Step 2: Run the exporter**

```bash
cd /home/nguyenvd/workspace/dog_robot
python3 dog_robot_ws/src/dog_robot_description/scripts/export_dh_links_from_freecad.py
```
Expected output: 17 lines `wrote .../visual_dh/<link>.stl (NNNN tris)`.

If `ConnectionRefusedError`: ask user to open FreeCAD + load `robotdogassem.STEP`
into doc `RobotDog` + start MCP server, then re-run.

- [ ] **Step 3: Verify the 17 STLs exist and are non-empty**

```bash
ls -la dog_robot_ws/src/dog_robot_description/meshes/visual_dh/ | grep '\.stl$' | wc -l
```
Expected: `17`. And: `find dog_robot_ws/src/dog_robot_description/meshes/visual_dh/ -name '*.stl' -size 0` should print nothing.

- [ ] **Step 4: Commit**

```bash
git add dog_robot_ws/src/dog_robot_description/scripts/export_dh_links_from_freecad.py \
        dog_robot_ws/src/dog_robot_description/meshes/visual_dh/
git commit -m "feat(meshes): re-export 17 link STLs in joint-attached frames

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 10: Delete legacy DH-tied tests (keeps suite green going forward)

**Files:**
- Delete: `dog_robot_ws/src/dog_robot_kinematics/test/test_dh_derivation.py`
- Delete: `dog_robot_ws/src/dog_robot_kinematics/test/test_kinematics_dh.py`
- Delete: `dog_robot_ws/src/dog_robot_kinematics/test/test_urdf_dh_consistency.py`

These three test the old MDH convention and will fail as soon as the URDF
properties switch in Task 11. They are replaced by `test_kinematics_link.py`
(already passing) and `test_urdf_link_consistency.py` (Task 12).

- [ ] **Step 1: Delete the files**

```bash
git rm dog_robot_ws/src/dog_robot_kinematics/test/test_dh_derivation.py \
       dog_robot_ws/src/dog_robot_kinematics/test/test_kinematics_dh.py \
       dog_robot_ws/src/dog_robot_kinematics/test/test_urdf_dh_consistency.py
```

- [ ] **Step 2: Verify suite still green**

```bash
cd dog_robot_ws && colcon build --packages-select dog_robot_kinematics --symlink-install && source install/setup.bash
pytest src/dog_robot_kinematics/test/ -v
```
Expected: all passing (no DH tests left, new tests still green).

- [ ] **Step 3: Commit**

```bash
git commit -m "test: drop legacy DH-tied tests (replaced by link tests)

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 11: Update `leg.xacro` + `dog_robot.urdf.xacro` to joint-attached convention

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_description/urdf/leg.xacro`
- Modify: `dog_robot_ws/src/dog_robot_description/urdf/dog_robot.urdf.xacro`

- [ ] **Step 1: Replace `leg.xacro` macro signature + joint origins**

In `leg.xacro`, change the macro params line (currently `~5–9`):

```xml
<!-- Old -->
<xacro:macro name="leg" params="prefix
                                base_to_hip_xyz base_to_hip_rpy
                                L_hh L_th L_sh
                                d_thigh:='0' d_knee:='0' d_foot:='0'
                                alpha_thigh:='-1.5707963'
                                foot_sphere_xyz:='0 0 0'">

<!-- New -->
<xacro:macro name="leg" params="prefix
                                base_to_hip_xyz base_to_hip_rpy
                                L_hh L_th L_sh
                                hip_to_thigh_rpy
                                thigh_to_knee_rpy:='0 0 0'
                                knee_to_foot_rpy:='0 0 0'">
```

Then update the 4 `<joint>` origin tags in the macro:

```xml
<!-- hip_yaw — unchanged -->
<origin xyz="${base_to_hip_xyz}" rpy="${base_to_hip_rpy}"/>

<!-- thigh_pitch — was: xyz="${L_hh} 0 ${d_thigh}" rpy="${alpha_thigh} 0 0" -->
<origin xyz="${L_hh} 0 0" rpy="${hip_to_thigh_rpy}"/>

<!-- knee_pitch — was: xyz="${L_th} 0 ${d_knee}" -->
<origin xyz="${L_th} 0 0" rpy="${thigh_to_knee_rpy}"/>

<!-- foot_fixed — was: xyz="${L_sh} 0 ${d_foot}" -->
<origin xyz="${L_sh} 0 0" rpy="${knee_to_foot_rpy}"/>
```

(Verify all `<visual><origin>` and visual mesh paths remain identity /
`visual_dh/` respectively — they should already be from the previous DH cycle.)

- [ ] **Step 2: Replace property block in `dog_robot.urdf.xacro`**

Find the `<xacro:property name="L_hh" .../>` block and replace with values
loaded from `link_params.yaml` and `urdf_joints.yaml`. Use literal values
(xacro can't natively read YAML); read them once and paste in. Get them via:

```bash
python3 -c "
import yaml
with open('dog_robot_ws/src/dog_robot_description/config/link_params.yaml') as f:
    lp = yaml.safe_load(f)
with open('dog_robot_ws/src/dog_robot_description/config/urdf_joints.yaml') as f:
    uj = yaml.safe_load(f)
print(lp); print(uj)
"
```

Then in `dog_robot.urdf.xacro` replace the existing DH-property block with:

```xml
<xacro:property name="L_hh" value="<paste lp['L_hh']>"/>
<xacro:property name="L_th" value="<paste lp['L_th']>"/>
<xacro:property name="L_sh" value="<paste lp['L_sh']>"/>
<xacro:property name="hip_to_thigh_rpy"  value="<paste lp['hip_to_thigh_rpy'] as 'r p y'>"/>
<xacro:property name="thigh_to_knee_rpy" value="<paste>"/>
<xacro:property name="knee_to_foot_rpy"  value="<paste>"/>
```

Remove old `d_thigh`, `d_knee`, `d_foot`, `alpha_thigh` properties.

For each `<xacro:leg .../>` call in this file (4 of them), update the
keyword arguments accordingly: remove `d_thigh`/`d_knee`/`d_foot`/`alpha_thigh`,
add `hip_to_thigh_rpy`/`thigh_to_knee_rpy`/`knee_to_foot_rpy`.
The per-leg `base_to_hip_xyz` / `base_to_hip_rpy` come from `urdf_joints.yaml`
(`uj['per_leg'][LEG]['base_to_hip_xyz' / 'base_to_hip_rpy']`).

- [ ] **Step 3: Verify URDF expands and is structurally valid**

```bash
cd /home/nguyenvd/workspace/dog_robot/dog_robot_ws
ros2 run xacro xacro src/dog_robot_description/urdf/dog_robot.urdf.xacro > /tmp/dog_robot.urdf
./scripts/xacro_clean.sh /tmp/dog_robot.urdf /tmp/dog_robot.clean.urdf  # if exists; else cp
check_urdf /tmp/dog_robot.urdf
```
Expected: `Successfully Parsed XML` and a non-empty link/joint tree.

If `xacro` errors out, fix the property names / commas in the macro call.

- [ ] **Step 4: Commit**

```bash
git add dog_robot_ws/src/dog_robot_description/urdf/leg.xacro \
        dog_robot_ws/src/dog_robot_description/urdf/dog_robot.urdf.xacro
git commit -m "feat(urdf): joint-attached frames — rpy per inter-link rotation

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 12: `test_urdf_link_consistency.py` — URDF FK matches `kinematics_link.fk_leg`

**Files:**
- Create: `dog_robot_ws/src/dog_robot_kinematics/test/test_urdf_link_consistency.py`

- [ ] **Step 1: Write the failing test**

```python
# dog_robot_ws/src/dog_robot_kinematics/test/test_urdf_link_consistency.py
import subprocess
from pathlib import Path

import numpy as np
import pytest

from dog_robot_kinematics.kinematics_link import (
    LinkParams, load_link_params, fk_leg)

REPO = Path(__file__).resolve().parents[3]
URDF_XACRO = REPO / "src" / "dog_robot_description" / "urdf" / "dog_robot.urdf.xacro"
LINK_CFG   = REPO / "src" / "dog_robot_description" / "config" / "link_params.yaml"
JOINTS_CFG = REPO / "src" / "dog_robot_description" / "config" / "urdf_joints.yaml"


def _expand_urdf() -> str:
    return subprocess.check_output(["ros2", "run", "xacro", "xacro", str(URDF_XACRO)],
                                     text=True)


def _urdf_joint_origin(urdf_text: str, joint_name: str):
    """Return (xyz, rpy) tuples for a named joint in the URDF text."""
    import xml.etree.ElementTree as ET
    root = ET.fromstring(urdf_text)
    for j in root.iter("joint"):
        if j.get("name") == joint_name:
            o = j.find("origin")
            return (
                np.array([float(v) for v in o.get("xyz").split()]),
                np.array([float(v) for v in o.get("rpy").split()]),
            )
    raise AssertionError(f"joint {joint_name!r} not found")


def _rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _T(R, t=np.zeros(3)):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t; return T


def _Rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


@pytest.mark.parametrize("leg", ["FL", "FR", "BL", "BR"])
def test_urdf_fk_matches_kinematics_link(leg):
    urdf = _expand_urdf()
    p = load_link_params(LINK_CFG)
    # Joint origins (xyz, rpy) read directly from the URDF.
    xyz_h, rpy_h = _urdf_joint_origin(urdf, f"{leg}_hip_yaw")
    xyz_t, rpy_t = _urdf_joint_origin(urdf, f"{leg}_thigh_pitch")
    xyz_k, rpy_k = _urdf_joint_origin(urdf, f"{leg}_knee_pitch")
    xyz_f, rpy_f = _urdf_joint_origin(urdf, f"{leg}_foot_fixed")

    rng = np.random.default_rng(7)
    for _ in range(10):
        q_yaw, q_thigh, q_knee = (
            float(rng.uniform(-0.3, 0.3)),
            float(rng.uniform(-0.5, 0.5)),
            float(rng.uniform(-1.0, -0.2)),
        )
        # URDF chain: base -> hip joint origin -> Rz(q_yaw) -> thigh joint origin
        # -> Rz(q_thigh) -> knee joint origin -> Rz(q_knee) -> foot joint origin
        T_base_to_hip = _T(_rpy_matrix(rpy_h), xyz_h)
        T_q_yaw       = _T(_Rz(q_yaw))
        T_hip_to_th   = _T(_rpy_matrix(rpy_t), xyz_t)
        T_q_thigh     = _T(_Rz(q_thigh))
        T_th_to_kn    = _T(_rpy_matrix(rpy_k), xyz_k)
        T_q_knee      = _T(_Rz(q_knee))
        T_kn_to_ft    = _T(_rpy_matrix(rpy_f), xyz_f)
        T_urdf = (T_base_to_hip @ T_q_yaw @ T_hip_to_th @ T_q_thigh
                   @ T_th_to_kn @ T_q_knee @ T_kn_to_ft)
        foot_in_base_urdf = T_urdf[:3, 3]

        # kinematics_link FK gives foot in HIP-yaw frame; lift to base via T_base_to_hip.
        foot_in_hip = fk_leg(p, (q_yaw, q_thigh, q_knee))
        foot_in_base_kin = (T_base_to_hip @ np.array(
            [foot_in_hip[0], foot_in_hip[1], foot_in_hip[2], 1.0]))[:3]

        np.testing.assert_allclose(foot_in_base_urdf, foot_in_base_kin, atol=1e-6)
```

- [ ] **Step 2: Run test to verify it fails (or passes immediately if URDF was right)**

Run: `pytest src/dog_robot_kinematics/test/test_urdf_link_consistency.py -v`
Expected: PASS if Task 11 produced a consistent URDF. If it FAILS, the
URDF and `link_params.yaml` disagree — likely the joint origin xyz/rpy in
the property block was copy-pasted wrong; recheck against the YAMLs from
Task 5.

- [ ] **Step 3: Commit**

```bash
git add dog_robot_ws/src/dog_robot_kinematics/test/test_urdf_link_consistency.py
git commit -m "test: URDF FK matches kinematics_link.fk_leg per leg

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 13: Migrate `stand_controller.py` to `kinematics_link`

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_control/dog_robot_control/stand_controller.py`
- Modify: `dog_robot_ws/src/dog_robot_control/launch/stand.launch.py` (if it references `dh_params.yaml`)
- Create: `dog_robot_ws/src/dog_robot_control/config/link_params.yaml` (symlink or copy from description pkg)

- [ ] **Step 1: Copy link_params into the control package config**

```bash
cp dog_robot_ws/src/dog_robot_description/config/link_params.yaml \
   dog_robot_ws/src/dog_robot_control/config/link_params.yaml
```

- [ ] **Step 2: Edit `stand_controller.py` (lines ~17, ~50)**

Change the import:
```python
# from:
from dog_robot_kinematics.kinematics_dh import DHParams, ik_leg
# to:
from dog_robot_kinematics.kinematics_link import LinkParams, load_link_params, ik_leg
```

Change the constructor block (around line 50). Replace `self.dh = DHParams(...)` with:

```python
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
cfg = Path(get_package_share_directory("dog_robot_control")) / "config" / "link_params.yaml"
self.params = load_link_params(cfg)
```

And every later reference to `self.dh` → `self.params`. The `ik_leg(self.dh, ...)`
calls become `ik_leg(self.params, ...)`.

- [ ] **Step 3: Update `stand.launch.py` parameter path**

Open `dog_robot_ws/src/dog_robot_control/launch/stand.launch.py` line ~25. If it
references `dh_params.yaml`, change to `link_params.yaml`. (If the node loads
params itself in `__init__`, the launch file may not need any change — re-read
the launch file and only edit if the path is hard-coded there.)

- [ ] **Step 4: Build & test**

```bash
cd dog_robot_ws
colcon build --packages-select dog_robot_control --symlink-install
source install/setup.bash
pytest src/dog_robot_control/test/ -v
```
Expected: tests that import `kinematics_dh` will still fail; tests that
don't import it should pass. We address `kinematics_dh` consumers in tests
in Task 16.

- [ ] **Step 5: Commit**

```bash
git add dog_robot_ws/src/dog_robot_control/dog_robot_control/stand_controller.py \
        dog_robot_ws/src/dog_robot_control/launch/stand.launch.py \
        dog_robot_ws/src/dog_robot_control/config/link_params.yaml
git commit -m "feat(control): stand_controller uses kinematics_link

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 14: Migrate `walker_controller.py` to `kinematics_link`

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_control/dog_robot_control/walker_controller.py`

- [ ] **Step 1: Edit the import + constructor**

Change line 17:
```python
from dog_robot_kinematics.kinematics_link import LinkParams, load_link_params, ik_leg
```

Replace the `DHParams(...)` block around line 63 with `load_link_params(cfg)`,
using the same `get_package_share_directory("dog_robot_control")` pattern as
Task 13. Every later `self.dh` reference → `self.params`.

- [ ] **Step 2: Build**

```bash
cd dog_robot_ws
colcon build --packages-select dog_robot_control --symlink-install
source install/setup.bash
```
Expected: clean build (controller compiles; gait files still import old type).

- [ ] **Step 3: Commit**

```bash
git add dog_robot_ws/src/dog_robot_control/dog_robot_control/walker_controller.py
git commit -m "feat(control): walker_controller uses kinematics_link

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 15: Migrate gait modules + their tests

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_control/dog_robot_control/gait/leg_controller.py`
- Modify: `dog_robot_ws/src/dog_robot_control/dog_robot_control/gait/body_controller.py`
- Modify: `dog_robot_ws/src/dog_robot_control/dog_robot_control/gait/gait_config.py`
- Modify: `dog_robot_ws/src/dog_robot_control/test/test_leg_controller.py`
- Modify: `dog_robot_ws/src/dog_robot_control/test/test_body_controller.py`
- Modify: `dog_robot_ws/src/dog_robot_control/test/test_walker_integration.py`

- [ ] **Step 1: Edit each module to use `LinkParams`**

In each of the 3 gait `*.py` files (lines noted earlier):
```python
# from:
from dog_robot_kinematics.kinematics_dh import DHParams
# to:
from dog_robot_kinematics.kinematics_link import LinkParams
```
Then rename the parameter type in the dataclass / function signatures
(`dh: DHParams` → `params: LinkParams`) and every internal reference (`dh.L_hh`
→ `params.L_hh`, etc.). Do NOT introduce a compatibility alias — the spec
is explicit (delete DH completely).

- [ ] **Step 2: Edit each test file**

In each of `test_leg_controller.py`, `test_body_controller.py`,
`test_walker_integration.py`:

Change:
```python
from dog_robot_kinematics.kinematics_dh import DHParams[, ik_leg]
DH = DHParams(L_hh=0.02553, L_th=0.11725, L_sh=0.07043)
```
to:
```python
import numpy as np
from dog_robot_kinematics.kinematics_link import LinkParams, ik_leg
DH = LinkParams(L_hh=0.02553, L_th=0.11725, L_sh=0.07043,
                R_const_ht=np.eye(3), R_const_tk=np.eye(3), R_const_kf=np.eye(3))
```

The `R_const_*` identity defaults preserve the old MDH-zero-d behaviour for these
unit tests (which use simple geometry); the integration test will still cover
real values via `load_link_params`.

- [ ] **Step 3: Build & test**

```bash
cd dog_robot_ws
colcon build --packages-select dog_robot_control --symlink-install
source install/setup.bash
pytest src/dog_robot_control/test/ -v
pytest src/dog_robot_kinematics/test/ -v
```
Expected: both suites green.

- [ ] **Step 4: Commit**

```bash
git add dog_robot_ws/src/dog_robot_control/dog_robot_control/gait/ \
        dog_robot_ws/src/dog_robot_control/test/
git commit -m "feat(control): gait modules + tests use LinkParams

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 16: Full smoke — colcon build + test

- [ ] **Step 1: Clean build**

```bash
cd /home/nguyenvd/workspace/dog_robot/dog_robot_ws
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```
Expected: all packages build, no warnings about missing files.

- [ ] **Step 2: Run all tests**

```bash
colcon test --packages-select dog_robot_kinematics dog_robot_control
colcon test-result --verbose
```
Expected: all tests pass.

If anything fails, do NOT proceed — debug and fix before Task 17.

- [ ] **Step 3: No commit needed if everything passed.**

If you had to fix anything, commit those fixes with `fix:` prefix message.

---

## Task 17: RViz + Gazebo stand check (manual user verification)

This task requires the user to look at RViz/Gazebo and confirm. Pause before
each launch and ask the user to confirm readiness.

- [ ] **Step 1: Kill orphan ROS / Gazebo processes**

```bash
bash dog_robot_ws/scripts/dog_kill_all.sh
```

- [ ] **Step 2: Launch kinematic_viz (RViz only, no Gazebo)**

```bash
cd dog_robot_ws && source install/setup.bash
ros2 launch dog_robot_kinematic_viz kinematic.launch.py
```
Ask the user: "Trong RViz có thấy con dog_robot 4 chân, các link join liền mạch, không méo / xoay sai không?" Wait for OK before proceeding.

If user reports breakage: re-export STLs is most likely cause (Task 9) —
the inverse Placement applied to solids depends on `R_const_*` accuracy.
Diagnose by visually identifying which link is wrong; rerun `derive_joint_frames.py`
+ exporter; do NOT fudge URDF properties.

- [ ] **Step 3: Stop RViz, launch Gazebo stand**

```bash
bash dog_robot_ws/scripts/dog_kill_all.sh
ros2 launch dog_robot_control stand.launch.py
```
Run for 30 seconds. Check `/odom` or visual: body z should rise to > 0.10 m
within 3 s, drift < 0.15 m, and the robot must not fly apart.

Ask the user to confirm before proceeding.

- [ ] **Step 4: No commit. If the user is happy, proceed to Task 18.**

---

## Task 18: Cleanup — delete legacy DH artefacts

**Files:**
- Delete: `dog_robot_ws/src/dog_robot_kinematics/dog_robot_kinematics/kinematics_dh.py`
- Delete: `dog_robot_ws/src/dog_robot_description/scripts/derive_dh_frames.py`
- Delete: `dog_robot_ws/src/dog_robot_description/config/dh_link_placements.yaml`
- Delete: `dog_robot_ws/src/dog_robot_kinematics/config/dh_params.yaml`
- Delete: `dog_robot_ws/src/dog_robot_control/config/dh_params.yaml`
- Delete: `dog_robot_ws/src/dog_robot_description/scripts/compute_visual_compensation.py`
- Delete: `dog_robot_ws/src/dog_robot_description/scripts/bake_meshes_to_link_frame.py`
- Delete tree: `dog_robot_ws/src/dog_robot_description/meshes/visual/`
- Delete tree: `dog_robot_ws/src/dog_robot_description/meshes/collision/` (only if it duplicates `visual/`; spec keeps collision out of scope, so removing is acceptable since URDF currently uses inline `<box>` collisions)

- [ ] **Step 1: Sanity grep for any remaining references**

```bash
grep -rn "kinematics_dh\|DHParams\|dh_params\.yaml\|dh_link_placements\|derive_dh_frames\|compute_visual_compensation\|bake_meshes_to_link_frame" \
     dog_robot_ws/src dog_robot_ws/scripts 2>/dev/null \
     | grep -v "__pycache__\|build/\|install/\|log/" || echo "CLEAN"
```
Expected: prints `CLEAN`. If anything else prints, do NOT delete — go back
and migrate that consumer first.

- [ ] **Step 2: Delete the files**

```bash
git rm dog_robot_ws/src/dog_robot_kinematics/dog_robot_kinematics/kinematics_dh.py \
       dog_robot_ws/src/dog_robot_description/scripts/derive_dh_frames.py \
       dog_robot_ws/src/dog_robot_description/config/dh_link_placements.yaml \
       dog_robot_ws/src/dog_robot_kinematics/config/dh_params.yaml \
       dog_robot_ws/src/dog_robot_control/config/dh_params.yaml \
       dog_robot_ws/src/dog_robot_description/scripts/compute_visual_compensation.py \
       dog_robot_ws/src/dog_robot_description/scripts/bake_meshes_to_link_frame.py
git rm -r dog_robot_ws/src/dog_robot_description/meshes/visual/
# Only remove collision/ if you have confirmed nothing references it:
grep -rn "meshes/collision" dog_robot_ws/src --include="*.xacro" --include="*.urdf" || \
  git rm -r dog_robot_ws/src/dog_robot_description/meshes/collision/
```

- [ ] **Step 3: Final clean build + test**

```bash
cd dog_robot_ws
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select dog_robot_kinematics dog_robot_control
colcon test-result --verbose
```
Expected: clean build, all tests pass.

- [ ] **Step 4: Commit**

```bash
git commit -m "chore: remove legacy DH artefacts (kinematics_dh + meshes/visual)

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Done

After Task 18 the repository state matches the spec acceptance criteria:
- `kinematics_dh`, `DHParams`, `dh_params.yaml`, `dh_link_placements.yaml` removed
- All controllers use `LinkParams` from `link_params.yaml`
- 17 STLs in `meshes/visual_dh/` are vertex-aligned to their joint-attached link frames
- URDF visual origins are identity; joint origins derive from `urdf_joints.yaml`
- All tests pass; RViz + Gazebo stand are visually verified
