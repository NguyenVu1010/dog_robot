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
