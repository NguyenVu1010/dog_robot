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
        # Hip roll axis: URDF X (forward, ROS REP-103).
        np.testing.assert_allclose(axes[leg]["hip"], np.array([1., 0., 0.]), atol=1e-9)
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
    # Z axis is the hip rotation axis = body +X (REP-103)
    np.testing.assert_allclose(info["R"][:, 2], np.array([1., 0., 0.]), atol=1e-9)
    # X axis lies in the YZ plane (X component ~ 0 after orthogonalisation)
    assert abs(info["R"][0, 0]) < 1e-9


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


def test_thigh_and_shank_origins_match_joint_centers():
    frames = djf.link_frames_urdf()
    centers = djf.joint_centers_urdf()
    for leg in ("FL", "FR", "BL", "BR"):
        np.testing.assert_allclose(
            frames[f"{leg}_thigh_link"]["O"],
            centers[leg]["thigh"], atol=1e-12,
            err_msg=f"{leg}_thigh_link origin mismatch")
        np.testing.assert_allclose(
            frames[f"{leg}_shank_link"]["O"],
            centers[leg]["knee"], atol=1e-12,
            err_msg=f"{leg}_shank_link origin mismatch")


def test_orthogonalise_raises_on_parallel_vectors():
    z = np.array([0.0, 0.0, 1.0])
    target_parallel = np.array([0.0, 0.0, 5.0])  # exact multiple of z
    with pytest.raises(ValueError, match="degenerate"):
        djf._orthogonalise(target_parallel, z, "test_case")


def test_link_lengths_symmetric_across_legs():
    lp = djf.link_params()
    # 4 legs share L_hh, L_th, L_sh within 1 mm
    # L_hh ~38 mm (3D distance hip→thigh including lateral offset)
    assert lp["L_hh"] == pytest.approx(0.038, abs=5e-3)
    assert lp["L_th"] == pytest.approx(0.117, abs=5e-3)
    assert lp["L_sh"] == pytest.approx(0.070, abs=5e-3)
    # Per-leg breakdown also present + matches mean within 1mm
    for leg in ("FL", "FR", "BL", "BR"):
        for k in ("L_hh", "L_th", "L_sh"):
            assert abs(lp["per_leg"][leg][k] - lp[k]) < 1e-3


def test_constant_inter_link_rotations_present():
    lp = djf.link_params()
    # Rotation matrices stored as 3x3 numpy arrays
    for k in ("R_const_ht", "R_const_tk", "R_const_kf"):
        R = lp[k]
        assert R.shape == (3, 3)
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-9)
        assert np.linalg.det(R) > 0


def test_leg_joint_transforms_exact():
    lt = djf.leg_joint_transforms()
    fr = djf.link_frames_urdf()
    assert set(lt) == {"FL", "FR", "BL", "BR"}
    pc = {"hip_to_thigh": ("hip_link", "thigh_link"),
          "thigh_to_knee": ("thigh_link", "shank_link"),
          "knee_to_foot": ("shank_link", "foot_link")}
    for leg in ("FL", "FR", "BL", "BR"):
        for key, (par, ch) in pc.items():
            xyz = np.asarray(lt[leg][key]["xyz"])
            R = lt[leg][key]["R"]
            assert abs(xyz[1]) < 1e-6           # Y ~ 0 by frame construction
            np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-9)
            # exact: reproduces the child origin in the parent frame
            Op, Rp = fr[f"{leg}_{par}"]["O"], fr[f"{leg}_{par}"]["R"]
            Oc = fr[f"{leg}_{ch}"]["O"]
            np.testing.assert_allclose(xyz, Rp.T @ (Oc - Op), atol=1e-12)
    # Front and back legs differ at the hip (slightly different thigh-offset
    # pitch geometry), so per-leg storage is required. With hip axis uniformly
    # body+X (REP-103), the FL vs BL difference is small but non-zero.
    R_fl = lt["FL"]["hip_to_thigh"]["R"]
    R_bl = lt["BL"]["hip_to_thigh"]["R"]
    ang = np.degrees(np.arccos(np.clip((np.trace(R_fl.T @ R_bl) - 1) / 2, -1, 1)))
    assert ang > 1.0


def test_writes_three_yamls(tmp_path):
    out_dir = tmp_path / "config"
    djf.write_outputs(out_dir)
    import yaml
    jf = yaml.safe_load((out_dir / "joint_frames.yaml").read_text())
    lp = yaml.safe_load((out_dir / "link_params.yaml").read_text())
    uj = yaml.safe_load((out_dir / "urdf_joints.yaml").read_text())
    # joint_frames: 17 link entries each with position_cad_mm + quat_xyzw
    assert len(jf["links"]) == 17
    sample = jf["links"]["FL_hip_link"]
    assert "position_cad_mm" in sample and len(sample["position_cad_mm"]) == 3
    assert "quat_xyzw" in sample and len(sample["quat_xyzw"]) == 4
    # link_params: 3 scalar lengths + per-leg joint transforms (xyz + rpy)
    for k in ("L_hh", "L_th", "L_sh"):
        assert isinstance(lp[k], float)
    for leg in ("FL", "FR", "BL", "BR"):
        assert set(lp[leg]) == {"hip_to_thigh", "thigh_to_knee", "knee_to_foot"}
        for key in lp[leg]:
            assert len(lp[leg][key]["xyz"]) == 3
            assert len(lp[leg][key]["rpy"]) == 3
    # Quaternion norm should be ~1.0 for every link
    import math
    for name, info in jf["links"].items():
        q = info["quat_xyzw"]
        n = math.sqrt(sum(c * c for c in q))
        assert abs(n - 1.0) < 1e-6, f"{name} quat not unit: {q} (|q|={n})"
    # Lengths in meters: catch a missing mm→m scaling regression
    assert 0.01 < lp["L_hh"] < 0.10
    assert 0.05 < lp["L_th"] < 0.20
    assert 0.03 < lp["L_sh"] < 0.15
    # urdf_joints: 4 legs each with base_to_hip_xyz + rpy (3-floats each)
    assert set(uj["per_leg"]) == {"FL", "FR", "BL", "BR"}
    for leg in uj["per_leg"]:
        assert len(uj["per_leg"][leg]["base_to_hip_xyz"]) == 3
        assert len(uj["per_leg"][leg]["base_to_hip_rpy"]) == 3
