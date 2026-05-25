from pathlib import Path

import numpy as np
import pytest

from dog_robot_kinematics.kinematics_link import LinkParams, load_link_params, fk_leg


CFG = (Path(__file__).resolve().parents[2]
       / "dog_robot_description" / "config" / "link_params.yaml")


def test_linkparams_dataclass_fields():
    p = LinkParams(
        L_hh=0.025, L_th=0.117, L_sh=0.070,
        R_const_ht=np.eye(3), R_const_tk=np.eye(3), R_const_kf=np.eye(3))
    assert p.L_hh == 0.025
    assert p.R_const_ht.shape == (3, 3)


def test_load_link_params_from_yaml():
    p = load_link_params(CFG)
    assert isinstance(p, LinkParams)
    assert 0.020 < p.L_hh < 0.050   # actual value is ~0.038 (3D distance)
    assert 0.110 < p.L_th < 0.125
    assert 0.060 < p.L_sh < 0.080
    for R in (p.R_const_ht, p.R_const_tk, p.R_const_kf):
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-9)


def _P():
    return load_link_params(CFG)


def test_fk_zero_angles_returns_static_foot_position():
    p = _P()
    foot = fk_leg(p, (0.0, 0.0, 0.0))
    assert foot.shape == (3,)
    assert np.linalg.norm(foot) > 0.05  # roughly L_hh + L_th + L_sh order


def test_fk_yaw_rotates_foot_in_xy_plane():
    p = _P()
    f0 = fk_leg(p, (0.0, 0.0, 0.0))
    f1 = fk_leg(p, (np.pi / 2, 0.0, 0.0))
    # |xy| preserved under yaw rotation
    assert np.linalg.norm(f0[:2]) == pytest.approx(np.linalg.norm(f1[:2]), abs=1e-9)
    # z unchanged
    assert f0[2] == pytest.approx(f1[2], abs=1e-9)
