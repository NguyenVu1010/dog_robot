"""Leg controller — port of CHAMP LegController.

Combines Raibert heuristic + per-leg trajectory planning. Returns updated
foot positions (does not mutate input).
"""
import math
from dataclasses import dataclass
from typing import List, Sequence

import numpy as np

from dog_robot_control.kinematics_dh import DHParams
from dog_robot_control.leg_config import LegConfig
from dog_robot_control.gait.gait_config import (
    GaitConfig,
    center_to_nominal,
    zero_stance,
)
from dog_robot_control.gait.phase_generator import PhaseGenerator
from dog_robot_control.gait.trajectory_planner import TrajectoryPlanner


@dataclass(frozen=True)
class Velocity:
    vx: float
    vy: float
    wz: float


def _cap(v, lo, hi):
    return max(lo, min(hi, v))


class LegController:
    def __init__(self, legs: Sequence[LegConfig], dh: DHParams,
                 gait: GaitConfig) -> None:
        self.legs = list(legs)
        self.dh = dh
        self.gait = gait
        self.phase_generator = PhaseGenerator(gait.stance_duration)
        self.trajectory_planners: List[TrajectoryPlanner] = [
            TrajectoryPlanner(gait) for _ in self.legs
        ]
        self._zero_stance = [zero_stance(L, dh, gait) for L in self.legs]
        self._center_to_nom = center_to_nominal(self.legs[0])

    @staticmethod
    def _raibert(stance_duration: float, target_velocity: float) -> float:
        return (stance_duration / 2.0) * target_velocity

    def _transform_leg(self, leg_idx: int, step_x: float, step_y: float,
                       theta: float) -> tuple:
        z0 = self._zero_stance[leg_idx]
        tx = z0[0] + step_x
        ty = z0[1] + step_y
        c, s = math.cos(theta), math.sin(theta)
        rot_x = c * tx - s * ty
        rot_y = s * tx + c * ty
        delta_x = rot_x - z0[0]
        delta_y = rot_y - z0[1]
        step_length = math.hypot(delta_x, delta_y) * 2.0
        rotation = math.atan2(delta_y, delta_x)
        return step_length, rotation

    def velocity_command(self, foot_positions: List[np.ndarray],
                         req: Velocity, t: float) -> List[np.ndarray]:
        vx = _cap(req.vx, -self.gait.max_linear_velocity_x,
                  self.gait.max_linear_velocity_x)
        vy = _cap(req.vy, -self.gait.max_linear_velocity_y,
                  self.gait.max_linear_velocity_y)
        wz = _cap(req.wz, -self.gait.max_angular_velocity_z,
                  self.gait.max_angular_velocity_z)

        tangential = wz * self._center_to_nom
        velocity_mag = math.hypot(vx, vy + tangential)

        step_x = self._raibert(self.gait.stance_duration, vx)
        step_y = self._raibert(self.gait.stance_duration, vy)
        step_theta = self._raibert(self.gait.stance_duration, tangential)
        theta = math.sin((step_theta / 2.0) / self._center_to_nom) * 2.0

        step_lengths = [0.0] * 4
        rotations = [0.0] * 4
        for i in range(4):
            step_lengths[i], rotations[i] = self._transform_leg(
                i, step_x, step_y, theta)
        mean_step = sum(step_lengths) / 4.0

        self.phase_generator.run(velocity_mag, mean_step, t)

        out: List[np.ndarray] = []
        for i in range(4):
            new_foot = self.trajectory_planners[i].generate(
                foot_positions[i],
                step_lengths[i],
                rotations[i],
                self.phase_generator.swing_phase_signal[i],
                self.phase_generator.stance_phase_signal[i],
            )
            out.append(new_foot)
        return out
