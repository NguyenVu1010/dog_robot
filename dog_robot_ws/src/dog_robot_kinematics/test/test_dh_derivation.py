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
