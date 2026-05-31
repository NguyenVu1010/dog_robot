"""Verify load_leg_geoms returns all 4 legs with correct shape + orthonormal R."""
from pathlib import Path

import numpy as np
import pytest

from dog_robot_kinematic_viz.leg_geometry import (
    LegGeom, LEG_NAMES, load_leg_geoms, _rpy_to_matrix,
)


URDF_JOINTS = (Path(__file__).resolve().parents[2]
               / "dog_robot_description" / "config" / "urdf_joints.yaml")


def test_load_all_four_legs():
    geoms = load_leg_geoms(URDF_JOINTS)
    assert set(geoms.keys()) == set(LEG_NAMES)
    for name, g in geoms.items():
        assert isinstance(g, LegGeom)
        assert g.name == name
        assert g.base_to_hip_xyz.shape == (3,)
        assert len(g.base_to_hip_rpy) == 3
        assert g.R_base_to_hip.shape == (3, 3)


def test_R_is_orthonormal():
    geoms = load_leg_geoms(URDF_JOINTS)
    I = np.eye(3)
    for g in geoms.values():
        np.testing.assert_allclose(g.R_base_to_hip.T @ g.R_base_to_hip, I, atol=1e-9)
        np.testing.assert_allclose(np.linalg.det(g.R_base_to_hip), 1.0, atol=1e-9)


def test_hip_local_z_aligns_with_body_x_for_all_legs():
    # Per REP-103, the hip joint axis (local Z, since URDF <axis>=0 0 1) maps
    # to body +X (forward) for every leg.
    geoms = load_leg_geoms(URDF_JOINTS)
    for g in geoms.values():
        # Column 2 of R_base_to_hip is the hip frame's Z axis in body coords.
        np.testing.assert_allclose(
            g.R_base_to_hip[:, 2], np.array([1.0, 0.0, 0.0]), atol=1e-9,
            err_msg=f"{g.name} hip Z axis != body +X")


def test_front_legs_y_sign_mirrors():
    geoms = load_leg_geoms(URDF_JOINTS)
    # FL has +y; FR has -y (and similarly back legs).
    assert geoms["FL"].base_to_hip_xyz[1] > 0
    assert geoms["FR"].base_to_hip_xyz[1] < 0
    assert geoms["BL"].base_to_hip_xyz[1] > 0
    assert geoms["BR"].base_to_hip_xyz[1] < 0
    # Front legs have +x, back legs have -x.
    assert geoms["FL"].base_to_hip_xyz[0] > 0
    assert geoms["FR"].base_to_hip_xyz[0] > 0
    assert geoms["BL"].base_to_hip_xyz[0] < 0
    assert geoms["BR"].base_to_hip_xyz[0] < 0


def test_missing_leg_raises():
    import tempfile
    bad = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    bad.write("per_leg:\n  FL: {base_to_hip_xyz: [0,0,0], base_to_hip_rpy: [0,0,0]}\n")
    bad.close()
    with pytest.raises(KeyError, match="FR"):
        load_leg_geoms(bad.name)


def test_rpy_to_matrix_helper_matches_known_rotation():
    R = _rpy_to_matrix((0.0, 0.0, np.pi / 2))
    expected = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    np.testing.assert_allclose(R, expected, atol=1e-12)
