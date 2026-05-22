import numpy as np
import pytest
from dog_robot_control.kinematics_dh import DHParams, mdh_transform, fk_leg

DH = DHParams(L_hh=0.02553, L_th=0.11725, L_sh=0.07043)

def test_mdh_identity_when_all_zero():
    T = mdh_transform(0.0, 0.0, 0.0, 0.0)
    assert np.allclose(T, np.eye(4))

def test_mdh_pure_translation():
    T = mdh_transform(0.0, 0.123, 0.0, 0.0)
    assert np.allclose(T[:3, 3], [0.123, 0.0, 0.0])
    assert np.allclose(T[:3, :3], np.eye(3))

def test_mdh_pure_rotation_z():
    T = mdh_transform(0.0, 0.0, 0.0, np.pi / 2)
    expected_R = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    assert np.allclose(T[:3, :3], expected_R)

def test_fk_at_zero_angles_extends_along_x_h():
    # At all-zero joint angles foot should sit at (L_hh + L_th + L_sh, 0, 0)
    # in the hip frame (along X_H = downward in body coords).
    foot = fk_leg(DH, (0.0, 0.0, 0.0))
    expected = np.array([DH.L_hh + DH.L_th + DH.L_sh, 0.0, 0.0])
    assert np.allclose(foot, expected, atol=1e-9)
