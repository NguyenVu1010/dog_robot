"""Per-leg foot trajectory — port of CHAMP's TrajectoryPlanner.

Stance phase: linear backward sweep along leg X plus small cosine z dip.
Swing phase: 12-control-point Bernstein-Bezier curve copied from CHAMP,
scaled by step_length / 0.4 m and swing_height / 0.15 m.

Per-tick output is added (in body frame) to the foot_position passed in.
"""
import math
from typing import Sequence

import numpy as np

from dog_robot_control.gait.gait_config import GaitConfig


_REF_X = (-0.15, -0.2805, -0.3, -0.3, -0.3,  0.0,
           0.0,   0.0,    0.3032, 0.3032, 0.2826, 0.15)
_REF_Y = (-0.5, -0.5, -0.3611, -0.3611, -0.3611, -0.3611,
          -0.3611, -0.3214, -0.3214, -0.3214, -0.5, -0.5)

_N_POINTS = 12
_FACT = (1.0, 1.0, 2.0, 6.0, 24.0, 120.0, 720.0, 5040.0,
         40320.0, 362880.0, 3628800.0, 39916800.0)


def _bernstein(i: int, n: int, t: float) -> float:
    coeff = _FACT[n] / (_FACT[i] * _FACT[n - i])
    return coeff * (t ** i) * ((1.0 - t) ** (n - i))


class TrajectoryPlanner:
    def __init__(self, gait: GaitConfig) -> None:
        self.gait = gait

    def _control_points(self, step_length: float) -> tuple:
        h_ratio = self.gait.swing_height / 0.15
        l_ratio = step_length / 0.4
        cp_x = [0.0] * _N_POINTS
        cp_y = [0.0] * _N_POINTS
        for i in range(_N_POINTS):
            if i == 0:
                cp_x[i] = -step_length / 2.0
            elif i == _N_POINTS - 1:
                cp_x[i] = step_length / 2.0
            else:
                cp_x[i] = _REF_X[i] * l_ratio
            cp_y[i] = -((_REF_Y[i] * h_ratio) + 0.5 * h_ratio)
        return cp_x, cp_y

    def generate(self, foot_position: np.ndarray, step_length: float,
                 rotation: float, swing_phase: float,
                 stance_phase: float) -> np.ndarray:
        """Return foot_position + computed delta (does NOT mutate input)."""
        if step_length == 0.0:
            return foot_position

        cp_x, cp_y = self._control_points(step_length)
        n = _N_POINTS - 1
        dx = 0.0
        dz = 0.0

        if stance_phase > swing_phase:
            dx = (step_length / 2.0) * (1.0 - 2.0 * stance_phase)
            dz = -self.gait.stance_depth * math.cos(math.pi * dx / step_length)
        elif swing_phase > stance_phase:
            for i in range(_N_POINTS):
                b = _bernstein(i, n, swing_phase)
                dx += b * cp_x[i]
                dz -= b * cp_y[i]

        result = foot_position.copy()
        result[0] += dx * math.cos(rotation)
        result[1] += dx * math.sin(rotation)
        result[2] += dz
        return result
