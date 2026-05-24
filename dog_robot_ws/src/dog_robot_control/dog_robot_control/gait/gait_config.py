"""Gait configuration + per-leg zero stance helper."""
from dataclasses import dataclass

import numpy as np

from dog_robot_kinematics.kinematics_dh import DHParams
from dog_robot_kinematics.leg_config import LegConfig


@dataclass(frozen=True)
class GaitConfig:
    nominal_height: float            # body z above ground at stand (m)
    stance_duration: float           # s
    swing_height: float              # m
    stance_depth: float              # m (small downward dip during stance)
    max_linear_velocity_x: float
    max_linear_velocity_y: float
    max_angular_velocity_z: float


def zero_stance(leg: LegConfig, dh: DHParams, gait: GaitConfig) -> np.ndarray:
    """Foot resting position in body frame (X forward, Y left, Z up), at all
    joint angles = 0 then body-translated so foot is at ground at body height =
    gait.nominal_height.
    """
    # All joints at 0: foot in body frame = hip_xyz + R_bh @ (L_total, 0, 0).
    # base_to_hip_rpy is always (0, pi/2, *), so R_bh @ (L, 0, 0) = (0, 0, -L)
    # regardless of the right-side pi yaw (Z rotation doesn't change Z).
    L_total = dh.L_hh + dh.L_th + dh.L_sh
    return np.array([
        leg.base_to_hip_xyz[0],
        leg.base_to_hip_xyz[1],
        leg.base_to_hip_xyz[2] - L_total,
    ])


def center_to_nominal(leg: LegConfig) -> float:
    """Distance from body center to nominal foot position projected on XY."""
    return float(np.hypot(leg.base_to_hip_xyz[0], leg.base_to_hip_xyz[1]))
