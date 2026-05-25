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
# Foot center: midpoint of shank-cluster and foot-cluster centroids in CAD mm.
# Source: scripts/compute_joints.py output ("CAD_JOINT_POSITIONS" section,
# *_foot_fixed entries). To regenerate, run
#   python3 scripts/compute_joints.py 2>&1 | grep foot_fixed
MEASURED_FOOT_MM: Dict[str, Tuple[float, float, float]] = {
    "FL": (32.3,  -102.4,   47.2),
    "FR": (31.6,  -102.7, -130.1),
    "BL": (223.2, -102.6,   47.1),
    "BR": (222.4, -102.7, -130.0),
}


def cad_to_urdf_point(p_mm: Tuple[float, float, float],
                       origin_mm: Tuple[float, float, float] = BODY_CENTER_MM
                       ) -> np.ndarray:
    """Convert CAD point (mm) to URDF point (m)."""
    p = np.asarray(p_mm, dtype=float)
    o = np.asarray(origin_mm, dtype=float)
    return 0.001 * np.array([o[0] - p[0], p[2] - o[2], p[1] - o[1]])


def cad_to_urdf_direction(v_cad) -> np.ndarray:
    """Convert CAD direction vector to URDF (linear part of cad_to_urdf_point)."""
    v = np.asarray(v_cad, dtype=float)
    return np.array([-v[0], v[2], v[1]])


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
