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
