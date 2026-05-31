from pathlib import Path

import numpy as np
import pytest

from dog_robot_kinematics.kinematics_link import (
    LinkParams, load_link_params, fk_leg, ik_leg)


CFG = (Path(__file__).resolve().parents[2]
       / "dog_robot_description" / "config" / "link_params.yaml")


def _P(leg="FL"):
    return load_link_params(CFG, leg)


def test_load_link_params_per_leg():
    for leg in ("FL", "FR", "BL", "BR"):
        p = load_link_params(CFG, leg)
        assert isinstance(p, LinkParams)
        assert 0.020 < p.L_hh < 0.050   # ~0.038 (full 3D distance)
        assert 0.110 < p.L_th < 0.125
        assert 0.080 < p.L_sh < 0.100
        for R in (p.R_ht, p.R_tk, p.R_kf):
            np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-9)
        for t in (p.t_ht, p.t_tk, p.t_kf):
            assert t.shape == (3,)
        # R_tk is a pure Rz (thigh and knee axes parallel) — IK relies on this.
        np.testing.assert_allclose(p.R_tk[2], np.array([0., 0., 1.]), atol=1e-9)
        np.testing.assert_allclose(p.R_tk[:, 2], np.array([0., 0., 1.]), atol=1e-9)


def test_fk_zero_angles_reproduces_measured_foot():
    # At theta=0 (the CAD rest pose the frames were derived from) fk must
    # reproduce the measured foot-relative-to-hip — exact per leg (no averaging).
    import sys
    sys.path.insert(0, str(CFG.parents[2] / "scripts"))
    import derive_joint_frames as djf
    fr = djf.link_frames_urdf()
    for leg in ("FL", "FR", "BL", "BR"):
        p = load_link_params(CFG, leg)
        foot = fk_leg(p, (0.0, 0.0, 0.0))
        Oh, Rh = fr[f"{leg}_hip_link"]["O"], fr[f"{leg}_hip_link"]["R"]
        Of = fr[f"{leg}_foot_link"]["O"]
        true = Rh.T @ (Of - Oh)
        np.testing.assert_allclose(foot, true, atol=1e-9)


def test_fk_yaw_rotates_foot_in_xy_plane():
    p = _P()
    f0 = fk_leg(p, (0.0, 0.0, 0.0))
    f1 = fk_leg(p, (np.pi / 2, 0.0, 0.0))
    assert np.linalg.norm(f0[:2]) == pytest.approx(np.linalg.norm(f1[:2]), abs=1e-9)
    assert f0[2] == pytest.approx(f1[2], abs=1e-9)


def test_ik_roundtrip_random_targets_all_legs():
    # With hip axis along body+X (abduction), ±0.2 rad covers a generous
    # roll range; combined with the moderate pitch/knee bend below, every
    # sampled theta lies safely in the IK workspace.
    rng = np.random.default_rng(42)
    for leg in ("FL", "FR", "BL", "BR"):
        p = load_link_params(CFG, leg)
        n_pass = 0
        for _ in range(200):
            theta_in = (
                float(rng.uniform(-0.2, 0.2)),
                float(rng.uniform(0.2, 1.0)),
                float(rng.uniform(-1.4, -0.2)),  # knee bent
            )
            foot = fk_leg(p, theta_in)
            try:
                theta_out = ik_leg(p, foot, knee_branch=+1)
            except ValueError:
                continue
            np.testing.assert_allclose(fk_leg(p, theta_out), foot, atol=1e-6)
            n_pass += 1
        assert n_pass > 190, f"{leg}: only {n_pass}/200"


def test_ik_default_branch_recovers_natural_config():
    # Branch +1 recovers the exact joint angles for a natural standing config.
    for leg in ("FL", "FR", "BL", "BR"):
        p = load_link_params(CFG, leg)
        theta_in = (0.1, 0.6, -0.9)
        theta_out = ik_leg(p, fk_leg(p, theta_in), knee_branch=+1)
        np.testing.assert_allclose(theta_out, theta_in, atol=1e-6)


def test_ik_unreachable_raises():
    p = _P()
    far = np.array([1.0, 0.0, 0.0])  # way outside workspace
    with pytest.raises(ValueError):
        ik_leg(p, far)
