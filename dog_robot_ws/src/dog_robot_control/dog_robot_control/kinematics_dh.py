"""Modified DH (Craig) kinematics for a 3-DOF quadruped leg.

DH table (one symmetric set for all 4 legs):
    i | alpha_{i-1} | a_{i-1} |  d_i | theta_i
    1 |     0       |   0     |   0  |  theta_hip
    2 |   -pi/2     |  L_hh   |   0  |  theta_thigh
    3 |     0       |  L_th   |   0  |  theta_knee
    F |     0       |  L_sh   |   0  |  0
"""
from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass(frozen=True)
class DHParams:
    L_hh: float  # hip-to-thigh common normal (a_1)
    L_th: float  # thigh length (a_2)
    L_sh: float  # shank length (a_3)


def mdh_transform(alpha: float, a: float, d: float, theta: float) -> np.ndarray:
    """Modified DH (Craig) homogeneous transform from frame i-1 to frame i.

    T = Rx(alpha) * Tx(a) * Rz(theta) * Tz(d)
    """
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [   ct,    -st,   0.0,        a],
        [st*ca,  ct*ca,   -sa,   -d*sa],
        [st*sa,  ct*sa,    ca,    d*ca],
        [  0.0,    0.0,   0.0,      1.0],
    ])


def fk_leg(dh: DHParams, theta: Tuple[float, float, float]) -> np.ndarray:
    """Foot position in hip frame H. theta = (theta_hip, theta_thigh, theta_knee)."""
    A1 = mdh_transform(0.0,        0.0,     0.0, theta[0])
    A2 = mdh_transform(-np.pi / 2, dh.L_hh, 0.0, theta[1])
    A3 = mdh_transform(0.0,        dh.L_th, 0.0, theta[2])
    AF = mdh_transform(0.0,        dh.L_sh, 0.0, 0.0)
    T = A1 @ A2 @ A3 @ AF
    return T[:3, 3]


def ik_leg(dh: DHParams, foot_h: np.ndarray, knee_direction: int = +1) -> Tuple[float, float, float]:
    """Closed-form inverse kinematics for one 3-DOF leg.

    foot_h: foot target in hip frame H, shape (3,).
    knee_direction: +1 or -1 — chooses elbow-up vs elbow-down branch.
    Returns (theta_hip, theta_thigh, theta_knee). Raises ValueError if unreachable.
    """
    x, y, z = float(foot_h[0]), float(foot_h[1]), float(foot_h[2])

    if abs(x) < 1e-12 and abs(y) < 1e-12:
        raise ValueError("foot on hip yaw axis: theta_hip undefined")
    theta_hip = np.arctan2(y, x)

    r = np.hypot(x, y)
    a_t = r - dh.L_hh
    b_t = -z  # alpha=-pi/2 in A2 negates z in the hip frame vs. the planar leg plane

    dist_sq = a_t * a_t + b_t * b_t
    cos_knee = (dist_sq - dh.L_th**2 - dh.L_sh**2) / (2.0 * dh.L_th * dh.L_sh)
    if cos_knee > 1.0 + 1e-9 or cos_knee < -1.0 - 1e-9:
        raise ValueError(f"foot out of reach: dist={np.sqrt(dist_sq):.4f} m, "
                         f"max={dh.L_th + dh.L_sh:.4f} m")
    cos_knee = float(np.clip(cos_knee, -1.0, 1.0))
    theta_knee = knee_direction * np.arccos(cos_knee)
    theta_thigh = (
        np.arctan2(b_t, a_t)
        - np.arctan2(dh.L_sh * np.sin(theta_knee),
                     dh.L_th + dh.L_sh * np.cos(theta_knee))
    )
    return (float(theta_hip), float(theta_thigh), float(theta_knee))
