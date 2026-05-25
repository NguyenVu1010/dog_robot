import importlib.util, sys
from pathlib import Path

import numpy as np
import pytest

_SCRIPT = (Path(__file__).resolve().parents[2]
           / "dog_robot_description" / "scripts" / "derive_joint_frames.py")
_spec = importlib.util.spec_from_file_location("derive_joint_frames", _SCRIPT)
djf = importlib.util.module_from_spec(_spec)
sys.modules["derive_joint_frames"] = djf
_spec.loader.exec_module(djf)


def test_joint_centers_present_for_all_legs():
    centers = djf.joint_centers_urdf()  # dict[leg][joint] -> np.ndarray(3,) m
    for leg in ("FL", "FR", "BL", "BR"):
        assert set(centers[leg]) == {"hip", "thigh", "knee", "foot"}
        for j, p in centers[leg].items():
            assert p.shape == (3,), f"{leg}/{j} bad shape"
            assert np.all(np.isfinite(p)), f"{leg}/{j} non-finite"


def test_cad_to_urdf_point_known_origin():
    # BODY_CENTER itself maps to URDF origin (0,0,0)
    p = djf.cad_to_urdf_point(djf.BODY_CENTER_MM)
    np.testing.assert_allclose(p, np.zeros(3), atol=1e-9)


def test_cad_to_urdf_axes_xyz_swap():
    # Direction vectors transform with the linear part only:
    #  URDF_x = -CAD_x, URDF_y = +CAD_z, URDF_z = +CAD_y
    out = djf.cad_to_urdf_direction(np.array([1.0, 0.0, 0.0]))
    np.testing.assert_allclose(out, np.array([-1.0, 0.0, 0.0]), atol=1e-12)
    out = djf.cad_to_urdf_direction(np.array([0.0, 1.0, 0.0]))
    np.testing.assert_allclose(out, np.array([0.0, 0.0, 1.0]), atol=1e-12)
    out = djf.cad_to_urdf_direction(np.array([0.0, 0.0, 1.0]))
    np.testing.assert_allclose(out, np.array([0.0, 1.0, 0.0]), atol=1e-12)
