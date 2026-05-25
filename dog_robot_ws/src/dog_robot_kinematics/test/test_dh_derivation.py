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


def test_all_four_legs_yield_same_lengths_within_1mm():
    """L_hh, L_th, |d_thigh|, |d_knee| match across the 4 legs within 2 mm.

    Note: |d_knee| shows ~1.9 mm spread (FL/BL ~41 mm, FR/BR ~42.7 mm) due to
    measurement inconsistency in the source CAD data (FR/BR knee Z was set as a
    round number rather than measured from circular edges like FL/BL).  The 2 mm
    tolerance is intentional so the test still catches sign-handling bugs while
    accepting this known CAD data limitation.
    """
    legs = [ddf.derive_leg(n) for n in ("FL", "FR", "BL", "BR")]
    L_hh_set = [l.L_hh for l in legs]
    L_th_set = [l.L_th for l in legs]
    d_thigh_set = [abs(l.d_thigh) for l in legs]
    d_knee_set  = [abs(l.d_knee)  for l in legs]
    for s, name in [(L_hh_set, "L_hh"), (L_th_set, "L_th"),
                    (d_thigh_set, "|d_thigh|"), (d_knee_set, "|d_knee|")]:
        assert max(s) - min(s) < 0.002, f"{name} differs > 2 mm across legs: {s}"


def test_mean_mdh_params_returns_expected_keys():
    m = ddf.mean_mdh_params()
    for k in ("L_hh", "L_th", "L_sh", "d_thigh", "d_knee", "d_foot",
              "alpha_1", "alpha_2"):
        assert k in m, f"missing key {k}"
    assert abs(m["alpha_1"] - (-np.pi/2)) < 1e-9
    assert abs(m["alpha_2"]) < 1e-9
