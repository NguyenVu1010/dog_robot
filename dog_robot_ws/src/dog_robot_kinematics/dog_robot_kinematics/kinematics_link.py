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
