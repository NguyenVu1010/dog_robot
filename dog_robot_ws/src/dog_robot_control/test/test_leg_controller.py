import math
import numpy as np
import pytest

from dog_robot_kinematics.kinematics_dh import DHParams
from dog_robot_kinematics.leg_config import LEGS
from dog_robot_control.gait.gait_config import GaitConfig
from dog_robot_control.gait.leg_controller import LegController, Velocity


DH = DHParams(L_hh=0.02553, L_th=0.11725, L_sh=0.07043)
GC = GaitConfig(0.15, 0.30, 0.03, 0.001, 0.15, 0.08, 0.50)


def _foot_in():
    return [np.array([0.0, 0.0, -0.15]) for _ in range(4)]


def test_zero_velocity_no_delta():
    lc = LegController(LEGS, DH, GC)
    feet = _foot_in()
    out = lc.velocity_command(feet, Velocity(0, 0, 0), t=0.0)
    for i in range(4):
        assert np.allclose(out[i], feet[i])


def test_forward_velocity_moves_feet_during_gait():
    lc = LegController(LEGS, DH, GC)
    lc.velocity_command(_foot_in(), Velocity(0.1, 0, 0), t=0.0)
    for k in range(1, 30):
        lc.velocity_command(_foot_in(), Velocity(0.1, 0, 0), t=k * 0.02)
    out = lc.velocity_command(_foot_in(), Velocity(0.1, 0, 0), t=30 * 0.02)
    deltas = [abs(out[i][0]) for i in range(4)]
    assert max(deltas) > 0.001, f"no gait delta: {deltas}"


def test_velocity_caps():
    lc = LegController(LEGS, DH, GC)
    out = lc.velocity_command(_foot_in(), Velocity(10.0, 0, 0), t=0.0)
    for foot in out:
        assert not np.any(np.isnan(foot))
