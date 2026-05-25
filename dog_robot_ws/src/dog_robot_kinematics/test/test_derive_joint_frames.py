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


def test_joint_axes_normalized_and_oriented():
    axes = djf.joint_axes_urdf()  # dict[leg][joint] -> unit np.ndarray(3,)
    for leg in ("FL", "FR", "BL", "BR"):
        for j, a in axes[leg].items():
            np.testing.assert_allclose(np.linalg.norm(a), 1.0, atol=1e-9)
        # Hip yaw axis: URDF Z is the yaw axis.
        np.testing.assert_allclose(axes[leg]["hip"], np.array([0., 0., 1.]), atol=1e-9)
        # Thigh + knee pitch axes: URDF Y after CAD→URDF map; sign normalised positive.
        np.testing.assert_allclose(axes[leg]["thigh"], np.array([0., 1., 0.]), atol=1e-9)
        np.testing.assert_allclose(axes[leg]["knee"],  np.array([0., 1., 0.]), atol=1e-9)


def _is_orthonormal_rh(R: np.ndarray, atol: float = 1e-9) -> bool:
    return (np.allclose(R.T @ R, np.eye(3), atol=atol)
            and np.linalg.det(R) > 0)


def test_link_frames_orthonormal_right_handed_for_all_legs():
    frames = djf.link_frames_urdf()
    # Expect: base_link + 4 legs * 4 links = 17 entries
    assert len(frames) == 17
    for name, info in frames.items():
        assert _is_orthonormal_rh(info["R"]), f"{name}: R not orthonormal RH"
        assert info["O"].shape == (3,)


def test_hip_link_frame_basic_geometry_fl():
    frames = djf.link_frames_urdf()
    info = frames["FL_hip_link"]
    expected_O = djf.joint_centers_urdf()["FL"]["hip"]
    np.testing.assert_allclose(info["O"], expected_O, atol=1e-12)
    # Z axis is hip yaw axis (URDF Z)
    np.testing.assert_allclose(info["R"][:, 2], np.array([0., 0., 1.]), atol=1e-9)
    # X axis lies in the XY plane (Z component ~ 0 after orthogonalisation)
    assert abs(info["R"][2, 0]) < 1e-9


def test_base_and_foot_use_world_aligned_frame():
    frames = djf.link_frames_urdf()
    np.testing.assert_allclose(frames["base_link"]["O"], np.zeros(3), atol=1e-12)
    np.testing.assert_allclose(frames["base_link"]["R"], np.eye(3), atol=1e-12)
    for leg in ("FL", "FR", "BL", "BR"):
        np.testing.assert_allclose(
            frames[f"{leg}_foot_link"]["R"], np.eye(3), atol=1e-9)
        np.testing.assert_allclose(
            frames[f"{leg}_foot_link"]["O"],
            djf.joint_centers_urdf()[leg]["foot"], atol=1e-12)
