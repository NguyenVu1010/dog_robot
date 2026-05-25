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
