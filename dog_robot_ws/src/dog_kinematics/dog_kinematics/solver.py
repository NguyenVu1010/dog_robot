"""High-level: body pose + 4 foot targets → 12 joint angles dict."""
import numpy as np
from .body import bodyIK, world_to_leg
from .leg import legIK


def solve_all_legs(body_pose, foot_targets_world):
    """
    Args:
        body_pose: (omega, phi, psi, xm, ym, zm)
        foot_targets_world: dict {"FL": (x,y,z,1.0), ...} world coords (homogeneous)

    Returns:
        dict {"FL_hip_yaw": rad, "FL_thigh_pitch": rad, ...} 12 entries
    """
    Tlf, Trf, Tlb, Trb, _ = bodyIK(*body_pose)
    legs = {
        "FL": (Tlf, False),
        "FR": (Trf, True),
        "BL": (Tlb, False),
        "BR": (Trb, True),
    }
    out = {}
    for name, (T_leg, is_right) in legs.items():
        foot = np.array(foot_targets_world[name], dtype=float)
        if foot.shape == (3,):
            foot = np.append(foot, 1.0)
        Q = world_to_leg(T_leg, foot, is_right=is_right)
        omega, theta, phi, _, _ = legIK(Q[0], Q[1], Q[2])
        out[f"{name}_hip_yaw"] = omega
        out[f"{name}_thigh_pitch"] = theta
        out[f"{name}_knee_pitch"] = phi
    return out
