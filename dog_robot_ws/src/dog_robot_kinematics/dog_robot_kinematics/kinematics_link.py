"""Joint-attached kinematics for the dog_robot 3-DOF leg.

See specs/2026-05-26-joint-frame-export-design.md.

Each leg joint is a fixed transform F_i = Trans(t_i) . Rot(R_i) (maps a
child-frame point p to the parent as R_i @ p + t_i) followed by a revolute
Z rotation. Transforms are stored per side (left = FL+BL, right = FR+BR)
because the along-axis joint offsets mirror between sides.
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple, Union

import numpy as np


@dataclass(frozen=True)
class LinkParams:
    # Scalar link lengths (full 3D joint-to-joint distance), for gait geometry.
    L_hh: float
    L_th: float
    L_sh: float
    # Fixed parent->child joint transforms for one side.
    t_ht: np.ndarray
    R_ht: np.ndarray   # hip   -> thigh
    t_tk: np.ndarray
    R_tk: np.ndarray   # thigh -> shank  (pure Rz)
    t_kf: np.ndarray
    R_kf: np.ndarray   # shank -> foot


def _rpy_to_matrix(rpy: Tuple[float, float, float]) -> np.ndarray:
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def load_link_params(yaml_path: Union[str, Path], side: str) -> LinkParams:
    """Load the joint transforms for one side ("left" or "right")."""
    import yaml
    cfg = yaml.safe_load(Path(yaml_path).read_text())
    s = cfg[side]

    def tr(key):
        return (np.asarray(s[key]["xyz"], dtype=float),
                _rpy_to_matrix(s[key]["rpy"]))

    t_ht, R_ht = tr("hip_to_thigh")
    t_tk, R_tk = tr("thigh_to_knee")
    t_kf, R_kf = tr("knee_to_foot")
    return LinkParams(
        L_hh=float(cfg["L_hh"]), L_th=float(cfg["L_th"]), L_sh=float(cfg["L_sh"]),
        t_ht=t_ht, R_ht=R_ht, t_tk=t_tk, R_tk=R_tk, t_kf=t_kf, R_kf=R_kf)


def _Rz(t: float) -> np.ndarray:
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _T(R: np.ndarray, t: np.ndarray | None = None) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    if t is not None:
        T[:3, 3] = t
    return T


def fk_leg(p: LinkParams, theta: Tuple[float, float, float]) -> np.ndarray:
    """Foot position (m) in hip-yaw frame. theta = (q_yaw, q_thigh, q_knee)."""
    T = (_T(_Rz(theta[0]))
         @ _T(p.R_ht, p.t_ht) @ _T(_Rz(theta[1]))
         @ _T(p.R_tk, p.t_tk) @ _T(_Rz(theta[2]))
         @ _T(p.R_kf, p.t_kf))
    return T[:3, 3]


def ik_leg(p: LinkParams, foot_in_hip: np.ndarray,
           knee_branch: int = +1) -> Tuple[float, float, float]:
    """Closed-form IK for the per-side joint-attached chain.

    Geometry (see spec 5.3): the foot in the thigh-root frame has a constant
    component wz along the (parallel) thigh/knee Z axis and an in-plane (wx, wy)
    that is a 2R with effective links |t_tk_xy|, |t_kf_xy| and offset angle from
    R_tk. q_yaw is solved from the two Rz(q_yaw)-invariants (foot Z and XY
    radius), which include the non-zero t_ht offset and the constant wz term.

    knee_branch: +1 or -1 selects the quadratic root in wx. +1 recovers the
        natural forward-thigh / bent-knee config and is what controllers use.
    Raises ValueError on unreachable target or yaw-undefined geometry.
    """
    x, y, z = float(foot_in_hip[0]), float(foot_in_hip[1]), float(foot_in_hip[2])
    r_xy = math.hypot(x, y)
    if r_xy < 1e-9:
        raise ValueError("foot on hip yaw axis: q_yaw undefined")

    c0 = p.R_ht[:, 0]
    c1 = p.R_ht[:, 1]
    c2 = p.R_ht[:, 2]
    if abs(c1[2]) < 1e-12:
        raise ValueError("degenerate R_ht: c1[2] ~ 0")

    # Out-of-plane (thigh-Z) component of the foot in the thigh-root frame —
    # constant because both pitch rotations and R_tk preserve that axis.
    wz = p.t_tk[2] + p.t_kf[2]

    # (1) foot_z invariant:  c0[2]*wx + c1[2]*wy + wz*c2[2] + t_ht[2] = z
    rhs_z = z - p.t_ht[2] - wz * c2[2]
    a_coef = -c0[2] / c1[2]
    b_coef = rhs_z / c1[2]

    # (2) foot XY radius invariant. With u = R_ht @ (wx, wy, wz) + t_ht:
    Kx = wz * c2[0] + p.t_ht[0]
    Ky = wz * c2[1] + p.t_ht[1]
    A = c1[0] * b_coef + Kx          # u_x = B*wx + A
    B = c0[0] + c1[0] * a_coef
    C = c1[1] * b_coef + Ky          # u_y = D*wx + C
    D = c0[1] + c1[1] * a_coef
    qa = B * B + D * D
    qb = 2.0 * (A * B + C * D)
    qc = A * A + C * C - r_xy * r_xy
    disc = qb * qb - 4.0 * qa * qc
    if disc < -1e-12:
        raise ValueError(f"foot unreachable: discriminant={disc:.6f}")
    disc = max(0.0, disc)
    wx = (-qb + knee_branch * math.sqrt(disc)) / (2.0 * qa)
    wy = a_coef * wx + b_coef

    # In-plane 2R: effective links are the X-projections of t_tk and t_kf,
    # elbow offset is the R_tk rotation angle.
    a1 = p.t_tk[0]
    a2 = p.t_kf[0]
    a_tk = math.atan2(p.R_tk[1, 0], p.R_tk[0, 0])
    dist2 = wx * wx + wy * wy
    c_phi = (dist2 - a1 * a1 - a2 * a2) / (2.0 * a1 * a2)
    if c_phi < -1.0 - 1e-9 or c_phi > 1.0 + 1e-9:
        raise ValueError(f"foot unreachable: cos(elbow)={c_phi:.4f}")
    c_phi = max(-1.0, min(1.0, c_phi))
    phi = -math.acos(c_phi)                      # elbow angle (thigh->shank link)
    q_knee = phi - a_tk
    q_thigh = (math.atan2(wy, wx)
               - math.atan2(a2 * math.sin(phi), a1 + a2 * math.cos(phi)))

    # q_yaw from foot = Rz(q_yaw) @ u.
    u_x = B * wx + A
    u_y = D * wx + C
    q_yaw = math.atan2(y, x) - math.atan2(u_y, u_x)
    # wrap to (-pi, pi]
    q_yaw = (q_yaw + math.pi) % (2.0 * math.pi) - math.pi

    return (q_yaw, q_thigh, q_knee)
