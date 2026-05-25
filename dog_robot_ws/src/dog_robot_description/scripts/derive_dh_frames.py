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


def cad_to_urdf_dir(v_cad: Tuple[float, float, float]) -> np.ndarray:
    """Map a CAD direction vector to URDF (no translation).

    Same rotational mapping as cad_to_urdf_point, dimensionless.
    """
    v = np.asarray(v_cad, dtype=float)
    return np.array([-v[0], v[2], v[1]])


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


def main() -> None:
    raise NotImplementedError("derive_dh_frames.main: implemented in later tasks")


if __name__ == "__main__":
    main()
