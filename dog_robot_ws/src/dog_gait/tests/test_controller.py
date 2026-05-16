# tests/test_controller.py
import numpy as np
from dog_gait.controller import GaitController


def test_controller_stand_returns_12_angles():
    ctrl = GaitController()
    ctrl.enable()
    angles = ctrl.tick(cmd_vel=(0, 0, 0), body_pose=(0, 0, 0, 0, 0, 0), dt=0.02)
    assert len(angles) == 12


def test_controller_trot_returns_12_angles():
    ctrl = GaitController()
    ctrl.enable()
    angles = ctrl.tick(cmd_vel=(0.1, 0, 0), body_pose=(0, 0, 0, 0, 0, 0), dt=0.02)
    assert len(angles) == 12
    for v in angles.values():
        assert not np.isnan(v)


def test_controller_disabled_returns_none():
    ctrl = GaitController()
    # not enabled
    angles = ctrl.tick(cmd_vel=(0, 0, 0), body_pose=(0, 0, 0, 0, 0, 0), dt=0.02)
    assert angles is None


def test_phase_advances():
    ctrl = GaitController()
    ctrl.enable()
    p0 = ctrl.phase
    ctrl.tick(cmd_vel=(0.1, 0, 0), body_pose=(0, 0, 0, 0, 0, 0), dt=0.02)
    assert ctrl.phase != p0
