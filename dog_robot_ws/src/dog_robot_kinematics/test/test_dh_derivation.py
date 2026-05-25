"""Tests for scripts/derive_dh_frames.py — pure-math DH derivation."""
import sys
from pathlib import Path

import numpy as np

# Make the derivation script importable.
HERE = Path(__file__).resolve()
SCRIPTS = HERE.parents[2] / "dog_robot_description" / "scripts"
sys.path.insert(0, str(SCRIPTS))

import derive_dh_frames as ddf


def test_cad_to_urdf_point_fl_hip():
    """FL hip center in URDF metres matches existing base_to_hip_xyz."""
    p = ddf.cad_to_urdf_point(ddf.MEASURED_HIP_MM["FL"])
    np.testing.assert_allclose(p, [0.0748, 0.040, 0.0351], atol=1e-4)


def test_cad_axis_dir_to_urdf():
    """CAD X axis maps to URDF -X; CAD Z maps to URDF Y; CAD Y maps to URDF Z."""
    np.testing.assert_allclose(ddf.cad_to_urdf_dir([1, 0, 0]), [-1, 0, 0], atol=1e-9)
    np.testing.assert_allclose(ddf.cad_to_urdf_dir([0, 1, 0]), [ 0, 0, 1], atol=1e-9)
    np.testing.assert_allclose(ddf.cad_to_urdf_dir([0, 0, 1]), [ 0, 1, 0], atol=1e-9)


def test_derive_leg_fl_returns_mdh_params():
    """derive_leg('FL') returns DerivedLeg with all expected fields."""
    leg = ddf.derive_leg("FL")
    # Joint frame 1 (hip): Z along URDF X, origin at hip axis line.
    assert leg.alpha_0_rad == 0.0
    np.testing.assert_allclose(leg.base_to_hip_xyz_m,
                               [0.0748, 0.040, 0.0351], atol=1e-4)
    # base_to_hip rpy puts local Z along URDF X (Ry(+pi/2)) for left legs.
    np.testing.assert_allclose(leg.base_to_hip_rpy_rad,
                               [0.0, np.pi/2, 0.0], atol=1e-9)
    # alpha_1 rotates Z_1 (URDF X) to Z_2 (URDF Y) about X_1: -pi/2.
    assert abs(leg.alpha_1_rad - (-np.pi/2)) < 1e-9
    # alpha_2 = 0 (knee Z parallel to thigh Z).
    assert abs(leg.alpha_2_rad) < 1e-9


def test_derive_leg_fl_lengths_positive():
    """L_hh, L_th, L_sh are positive metres in the expected range."""
    leg = ddf.derive_leg("FL")
    assert 0.005 < leg.L_hh < 0.05    # ~10-50 mm
    assert 0.05  < leg.L_th < 0.20    # ~100 mm thigh
    assert 0.03  < leg.L_sh < 0.12    # ~70 mm shank
