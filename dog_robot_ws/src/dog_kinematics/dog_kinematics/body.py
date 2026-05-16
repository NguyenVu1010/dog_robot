"""Body kinematics: 6-DOF body pose → 4 hip frames + leg-frame conversion."""
import math
import numpy as np
from .constants import BODY_LENGTH as L, BODY_WIDTH as W


def _Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0, 0],
                     [0, c, -s, 0],
                     [0, s,  c, 0],
                     [0, 0, 0, 1]])


def _Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[ c, 0, s, 0],
                     [ 0, 1, 0, 0],
                     [-s, 0, c, 0],
                     [ 0, 0, 0, 1]])


def _Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0, 0],
                     [s,  c, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])


def _T(x, y, z):
    M = np.eye(4)
    M[0, 3] = x; M[1, 3] = y; M[2, 3] = z
    return M


_HALF_PI = np.pi / 2


def _hip_local_transform(x_offset, z_offset):
    """Standard hip frame attached to body corner (matches TestIK orientation)."""
    return np.array([
        [math.cos(_HALF_PI),  0, math.sin(_HALF_PI), x_offset],
        [0,                    1, 0,                  0],
        [-math.sin(_HALF_PI), 0, math.cos(_HALF_PI), z_offset],
        [0,                    0, 0,                  1.0],
    ])


def bodyIK(omega, phi, psi, xm, ym, zm):
    """Return (Tlf, Trf, Tlb, Trb, Tm): 4 hip frames + body matrix."""
    Tm = _T(xm, ym, zm) @ _Rx(omega) @ _Ry(phi) @ _Rz(psi)
    Tlf = Tm @ _hip_local_transform( L/2,  W/2)
    Trf = Tm @ _hip_local_transform( L/2, -W/2)
    Tlb = Tm @ _hip_local_transform(-L/2,  W/2)
    Trb = Tm @ _hip_local_transform(-L/2, -W/2)
    return Tlf, Trf, Tlb, Trb, Tm


_IX = np.diag([-1.0, 1.0, 1.0, 1.0])


def world_to_leg(T_leg, foot_world, is_right=False):
    """Convert foot world coords to leg-local frame; flip X for right legs."""
    if is_right:
        return _IX @ np.linalg.inv(T_leg) @ foot_world
    return np.linalg.inv(T_leg) @ foot_world
