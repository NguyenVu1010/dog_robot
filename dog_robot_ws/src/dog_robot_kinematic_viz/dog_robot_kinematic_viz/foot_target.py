"""Per-leg foot trajectory, computed in body frame and rotated to hip frame.

The gait stride and lift are physically defined in BODY frame:
- Stride: horizontal displacement in body +X/+Y plane, proportional to body velocity.
- Lift: vertical displacement in body +Z, only during swing phase.
- body_z translation: the body raises in body +Z; feet drop -body_z in body to compensate.
- extra_z translation: leg-frame-agnostic body-Z foot-lift, added on top. Used
  by rear legs (+pitch_amount, fold toward body) and front legs (-pitch_amount,
  extend away) — see the spec's Sign Convention subsection. Sign is opposite
  to body_z because the two scalars describe different things.

After computing the full body-frame displacement, rotate into the leg's hip
frame (using R_base_to_hip.T) and add to rest_in_hip for ik_leg.

Phase convention (phi in [0, 1)):
    0  ..  stance_phase_ratio   stance: foot drags backwards along stride
    stance_phase_ratio .. 1     swing : foot returns + lifts (sin arch),
                                lift scales linearly with |v_body_xy| up to
                                swing_activation_speed (then saturates).
Continuity is C0 across the stance/swing seam and the cycle wrap.
"""
from __future__ import annotations
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class FootTargetParams:
    stride_per_mps: float = 0.20          # stride magnitude per m/s of body vel
    swing_height: float = 0.03            # peak lift above stance plane (m)
    stance_phase_ratio: float = 0.5       # fraction of cycle spent in stance
    swing_activation_speed: float = 0.05  # m/s; |v_body| above which lift is full


def foot_target_in_hip(rest_in_hip: np.ndarray,
                       phase: float,
                       v_body_xy: tuple[float, float],
                       body_z: float,
                       extra_z: float,
                       R_base_to_hip: np.ndarray,
                       params: FootTargetParams) -> np.ndarray:
    """Return foot target in hip frame.

    rest_in_hip: fk_leg(p, (0,0,0)) for this leg, in hip frame.
    phase: in [0, 1). Wraps automatically.
    v_body_xy: body-frame XY velocity (m/s). Forward = (+vx, 0).
    body_z: body-frame Z translation (m), clamped upstream in BodyCommander.
            Subtracted from foot Z (body rising drops the foot in body frame).
    extra_z: leg-frame-agnostic body-Z foot-lift (m). Added on top of -body_z.
            Callers pass +pitch_amount (rear legs, fold) or -pitch_amount
            (front legs, extend).
    R_base_to_hip: hip->body rotation matrix for this leg (3x3, orthonormal).
    params: gait shape.
    """
    phi = float(phase) % 1.0
    r = params.stance_phase_ratio

    vx_body = float(v_body_xy[0])
    vy_body = float(v_body_xy[1])
    sx_body = params.stride_per_mps * vx_body
    sy_body = params.stride_per_mps * vy_body

    if phi < r:
        u = phi / r
        scale = 0.5 - u
        z_lift_body = 0.0
    else:
        u = (phi - r) / (1.0 - r)
        scale = -0.5 + u
        v_mag = float(np.hypot(vx_body, vy_body))
        s = params.swing_activation_speed
        swing_scale = 1.0 if s <= 0.0 else min(1.0, v_mag / s)
        z_lift_body = params.swing_height * np.sin(np.pi * u) * swing_scale

    # Full body-frame displacement from rest:
    #   stride (XY) + swing lift (Z) - body_z compensation + extra_z lift.
    disp_body = np.array([
        sx_body * scale,
        sy_body * scale,
        z_lift_body - float(body_z) + float(extra_z),
    ])

    # Rotate body-frame displacement into hip frame and add to rest.
    return rest_in_hip + R_base_to_hip.T @ disp_body
