"""Joint-attached kinematics for the dog_robot 3-DOF leg.

See specs/2026-05-26-joint-frame-export-design.md.
"""
from __future__ import annotations
import math
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


def _T_of(R: np.ndarray, t: np.ndarray | None = None) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    if t is not None:
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


def ik_leg(p: LinkParams, foot_in_hip: np.ndarray,
           knee_branch: int = +1) -> Tuple[float, float, float]:
    """Closed-form IK: hip yaw + 2R planar (thigh + knee).

    Exploits two invariants under Rz(q_yaw): the foot Z component and the foot XY
    magnitude. These yield a quadratic in vx (thigh-frame X), from which the 2R
    problem and q_yaw are recovered analytically.

    foot_in_hip: (3,) numpy array in hip-yaw frame, meters.
    knee_branch: +1 or -1 — selects the quadratic root in vx (two distinct leg
        configurations, not the classical elbow-up/down sign). +1 recovers the
        natural FK config for a forward-thigh / bent-knee standing pose and is
        the branch controllers should use.
    Raises ValueError on unreachable target or yaw-undefined geometry.

    Note: R_const_ht is a general 3D rotation for this robot (not the pure Rx the
    original design assumed), so q_yaw cannot be read off directly; it is solved
    from the two Rz(q_yaw)-invariants below.
    """
    x, y, z = float(foot_in_hip[0]), float(foot_in_hip[1]), float(foot_in_hip[2])

    c0 = p.R_const_ht[:, 0]   # first column of hip-to-thigh rotation
    c1 = p.R_const_ht[:, 1]   # second column
    # R_const_tk is a pure Rz; extract its rotation angle.
    alpha_tk = math.atan2(p.R_const_tk[1, 0], p.R_const_tk[0, 0])

    r_xy = math.hypot(x, y)
    if r_xy < 1e-9:
        raise ValueError("foot on hip yaw axis: q_yaw undefined")

    # v = [vx; vy; 0] is the foot position in the thigh root frame.
    # Two q_yaw-invariant constraints:
    #   (1) z = c0[2]*vx + c1[2]*vy
    #   (2) r_xy^2 = (L_hh + c0[0]*vx + c1[0]*vy)^2 + (c0[1]*vx + c1[1]*vy)^2
    # From (1): vy = a*vx + b
    a_coef = -c0[2] / c1[2]
    b_coef = z / c1[2]

    # Substitute into (2) -> quadratic in vx: qa*vx^2 + qb*vx + qc = 0
    A = p.L_hh + c1[0] * b_coef
    B = c0[0] + c1[0] * a_coef
    C = c1[1] * b_coef
    D = c0[1] + c1[1] * a_coef
    qa = B * B + D * D
    qb = 2.0 * (A * B + C * D)
    qc = A * A + C * C - r_xy * r_xy
    disc = qb * qb - 4.0 * qa * qc
    if disc < -1e-12:
        raise ValueError(f"foot unreachable: discriminant={disc:.6f}")
    disc = max(0.0, disc)

    # knee_branch selects which of the two quadratic roots to use.
    vx = (-qb + knee_branch * math.sqrt(disc)) / (2.0 * qa)
    vy = a_coef * vx + b_coef

    # 2R IK in thigh frame: the combined knee angle is alpha_kn = alpha_tk + q_kn.
    dist2 = vx * vx + vy * vy
    c_kn = (dist2 - p.L_th ** 2 - p.L_sh ** 2) / (2.0 * p.L_th * p.L_sh)
    if c_kn < -1.0 - 1e-9 or c_kn > 1.0 + 1e-9:
        raise ValueError(f"foot unreachable: cos(alpha_kn)={c_kn:.4f}")
    c_kn = max(-1.0, min(1.0, c_kn))
    # Negative branch keeps q_kn in the bent-knee (negative) range for this robot.
    alpha_kn = -math.acos(c_kn)
    q_kn = alpha_kn - alpha_tk
    q_th = (math.atan2(vy, vx)
            - math.atan2(p.L_sh * math.sin(alpha_kn),
                         p.L_th + p.L_sh * math.cos(alpha_kn)))

    # Recover q_yaw from the angle of foot_at_q_yaw0 vs the measured foot angle.
    foot0 = np.array([p.L_hh, 0.0, 0.0]) + p.R_const_ht @ np.array([vx, vy, 0.0])
    q_yaw = math.atan2(y, x) - math.atan2(foot0[1], foot0[0])

    return (q_yaw, q_th, q_kn)
