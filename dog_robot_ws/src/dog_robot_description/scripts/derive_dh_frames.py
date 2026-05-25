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


if __name__ == "__main__":
    main()
