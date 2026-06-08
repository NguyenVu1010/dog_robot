"""LegDriver: per-leg foot trajectory + closed-form IK.

This is the "1 chân -> kế thừa các chân" unit: one class instantiated 4x,
once per leg. The 4 instances differ only in `geom` (per-leg base->hip)
and `link_params` (per-leg LinkParams). No per-leg conditionals inside
the class itself.

The foot oscillates around the leg's CAD rest pose `fk_leg(link, (0,0,0))`
so joint angles stay near zero (well inside limits) and the IK never
hits the hip-axis singularity that ik_leg raises on.

Architecture: foot_target_in_hip receives body-frame velocity and rotates
it into the hip frame internally. LegDriver is a thin wrapper: it passes
body velocity + R_base_to_hip directly to foot_target_in_hip.
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
                 ft_params: FootTargetParams):
        self.geom = geom
        self.link = link_params
        self.ft = ft_params
        self.rest_in_hip: np.ndarray = fk_leg(link_params, (0.0, 0.0, 0.0))
        self._last_joints: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    def step(self, body_v_xy: Tuple[float, float],
             phase: float,
             body_z: float = 0.0) -> Tuple[float, float, float]:
        target = foot_target_in_hip(
            self.rest_in_hip,
            phase,
            body_v_xy,
            body_z,
            self.geom.R_base_to_hip,
            self.ft,
        )
        try:
            q = ik_leg(self.link, target, knee_branch=+1)
        except ValueError:
            # Unreachable / singular: hold last good joints rather than crash.
            return self._last_joints
        self._last_joints = q
        return q

    @property
    def last_joints(self) -> Tuple[float, float, float]:
        return self._last_joints
