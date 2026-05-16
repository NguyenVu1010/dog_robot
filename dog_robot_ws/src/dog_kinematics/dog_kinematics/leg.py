"""1-leg inverse + forward kinematics (meters, IK frame).

IK frame convention (matches TestIK/):
  x: fore-aft  (positive = forward)
  y: vertical  (negative = down, foot pos has y < 0)
  z: lateral   (positive = leg-outward direction)
"""
import math
from .constants import L2, L3, L4


class OutOfWorkspace(ValueError):
    """Raised when foot target is outside reachable workspace."""


def _clamp(v, lo, hi):
    return max(lo, min(v, hi))


def legIK(x, y, z):
    """Return (omega, theta, phi, D, G) for foot at (x, y, z) in leg-frame meters.

    Raises OutOfWorkspace if unreachable.
    """
    C = y*y + z*z
    if C <= L2*L2:
        raise OutOfWorkspace(f"Y^2 + Z^2 = {C:.6f} <= L2^2 = {L2*L2:.6f}")
    D = math.sqrt(C - L2*L2)
    G = math.sqrt(D*D + x*x)
    if G > (L3 + L4):
        raise OutOfWorkspace(f"G = {G:.6f} > L3+L4 = {L3+L4:.6f}")
    if G < abs(L3 - L4):
        raise OutOfWorkspace(f"G = {G:.6f} < |L3-L4| = {abs(L3-L4):.6f}")

    omega = math.atan2(z, y) + math.atan2(D, L2)
    cos_phi = (G*G - L3*L3 - L4*L4) / (-2.0 * L3 * L4)
    cos_phi = _clamp(cos_phi, -1.0, 1.0)
    phi = math.acos(cos_phi)
    sin_term = (L4 * math.sin(phi)) / G
    sin_term = _clamp(sin_term, -1.0, 1.0)
    theta = math.atan2(x, D) + math.asin(sin_term)

    return omega, theta, phi, D, G


def calcLegPoints(omega, theta, phi, D):
    """Forward kinematics: return [P0, P1, P2, P3] joint positions in leg-frame meters.

    P0 = hip (origin)
    P1 = thigh_pitch (after L2 offset)
    P2 = knee
    P3 = foot
    """
    P0 = (0.0, 0.0, 0.0)
    Ay = L2 * math.cos(omega)
    Az = L2 * math.sin(omega)
    P1 = (0.0, Ay, Az)

    beta = omega - math.atan2(D, L2)
    r = math.sqrt(L2*L2 + D*D)
    y_foot = r * math.cos(beta)
    z_foot = r * math.sin(beta)

    vy = y_foot - Ay
    vz = z_foot - Az
    norm_v = math.sqrt(vy*vy + vz*vz)
    if norm_v < 1e-9:
        uy, uz = 1.0, 0.0
    else:
        uy, uz = vy / norm_v, vz / norm_v

    xk = L3 * math.sin(theta)
    dk = L3 * math.cos(theta)
    P2 = (xk, Ay + dk * uy, Az + dk * uz)

    xf = L3 * math.sin(theta) + L4 * math.sin(theta + phi - math.pi)
    df = L3 * math.cos(theta) + L4 * math.cos(theta + phi - math.pi)
    P3 = (xf, Ay + df * uy, Az + df * uz)

    return [P0, P1, P2, P3]
