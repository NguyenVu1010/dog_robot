"""Per-leg foot trajectory in hip frame, anchored at the leg's CAD rest pose.

Design choice: each leg's "neutral" foot position is `fk_leg(p, 0)` — the
foot location when all 3 joints are zero. The gait oscillates around that
neutral, so q stays near zero (well inside joint limits) and the IK never
sees the foot on the hip yaw axis (which raises in `kinematics_link.ik_leg`).
The body XY velocity is rotated into the leg's hip frame (by the caller)
and scaled to a per-cycle stride vector.

Phase convention (phi in [0, 1)):
    0  ..  stance_phase_ratio   stance: foot drags backwards along stride
    stance_phase_ratio .. 1     swing : foot returns + lifts (sin arch)
Continuity is enforced at phi = stance_phase_ratio and at phi = 1 -> 0:
    foot_target is C0 continuous across both seams.
"""
from __future__ import annotations
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class FootTargetParams:
    stride_per_mps: float = 0.20          # stride magnitude per m/s of body vel
    swing_height: float = 0.03            # peak lift above stance plane (m)
    stance_phase_ratio: float = 0.5       # fraction of cycle spent in stance


def foot_target_in_hip(rest_in_hip: np.ndarray,
                       phase: float,
                       v_hip_xy: tuple[float, float],
                       params: FootTargetParams) -> np.ndarray:
    """Return foot target (x, y, z) in the leg's hip frame at the given phase.

    rest_in_hip: fk_leg(p, (0,0,0)) for this leg; the foot oscillates around
        this point so joint angles stay near zero.
    phase: in [0, 1). Wraps automatically (uses phase % 1.0).
    v_hip_xy: body XY velocity expressed in the hip frame (caller rotates).
    """
    phi = float(phase) % 1.0
    r = params.stance_phase_ratio
    sx = params.stride_per_mps * float(v_hip_xy[0])
    sy = params.stride_per_mps * float(v_hip_xy[1])

    if phi < r:
        # Stance: linear forward -> backward along stride (scale +0.5 -> -0.5).
        u = phi / r
        scale = 0.5 - u
        z_lift = 0.0
    else:
        # Swing: linear backward -> forward (scale -0.5 -> +0.5) with sin lift.
        u = (phi - r) / (1.0 - r)
        scale = -0.5 + u
        z_lift = params.swing_height * np.sin(np.pi * u)

    return np.array([
        rest_in_hip[0] + sx * scale,
        rest_in_hip[1] + sy * scale,
        rest_in_hip[2] + z_lift,
    ])
