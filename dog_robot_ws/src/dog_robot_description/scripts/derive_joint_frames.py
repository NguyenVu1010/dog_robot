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
