"""LegDriver: per-leg foot trajectory + closed-form IK.

This is the "1 chân -> kế thừa các chân" unit: one class instantiated 4x,
once per leg. The 4 instances differ only in `geom` (per-leg base->hip),
`link_params` (per-leg LinkParams), and `is_rear` (True for BL/BR — the
sign of pitch_amount's contribution to extra_z flips between front and
rear).

The foot oscillates around the leg's CAD rest pose `fk_leg(link, (0,0,0))`
so joint angles stay near zero (well inside limits) and the IK never
hits the hip-axis singularity that ik_leg raises on.

Architecture: foot_target_in_hip receives body-frame velocity and rotates
it into the hip frame internally. LegDriver is a thin wrapper: it passes
body velocity + R_base_to_hip directly to foot_target_in_hip and chooses
the sign of pitch_amount per leg — `+pitch_amount` on rear legs (fold),
`-pitch_amount` on front legs (extend). This gives a virtual body pitch
on a single scalar input.

On IK failure (foot target unreachable, e.g. combined body_z + pitch past
leg reach) LegDriver returns the last good joints and logs a WARN exactly
once per saturation event (cleared on next success).
"""
from __future__ import annotations
from typing import Tuple

import numpy as np

from dog_robot_kinematics.kinematics_link import LinkParams, fk_leg, ik_leg

from dog_robot_kinematic_viz.leg_geometry import LegGeom
from dog_robot_kinematic_viz.foot_target import (
    FootTargetParams, foot_target_in_hip,
)


class LegDriver:
    def __init__(self,
                 geom: LegGeom,
                 link_params: LinkParams,
                 ft_params: FootTargetParams,
                 is_rear: bool = False,
                 logger=None):
        self.geom = geom
        self.link = link_params
        self.ft = ft_params
        self.is_rear = bool(is_rear)
        self._logger = logger
        self.rest_in_hip: np.ndarray = fk_leg(link_params, (0.0, 0.0, 0.0))
        self._last_joints: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._saturated = False

    def step(self, body_v_xy: Tuple[float, float],
             phase: float,
             body_z: float = 0.0,
             pitch_amount: float = 0.0) -> Tuple[float, float, float]:
        p = float(pitch_amount)
        extra_z = p if self.is_rear else -p
        target = foot_target_in_hip(
            self.rest_in_hip,
            phase,
            body_v_xy,
            body_z,
            extra_z,
            self.geom.R_base_to_hip,
            self.ft,
        )
        try:
            q = ik_leg(self.link, target, knee_branch=+1)
        except ValueError:
            if not self._saturated:
                self._saturated = True
                msg = "LegDriver IK saturated; holding last joints"
                if self._logger is not None:
                    self._logger.warning(msg)
                else:
                    print(f"WARNING: {msg}")
            return self._last_joints
        self._saturated = False
        self._last_joints = q
        return q

    @property
    def last_joints(self) -> Tuple[float, float, float]:
        return self._last_joints
