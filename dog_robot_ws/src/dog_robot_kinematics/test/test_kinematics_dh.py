import numpy as np
import pytest
from dog_robot_kinematics.kinematics_dh import DHParams, mdh_transform, fk_leg

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

# --- Task 3 IK tests ---
from dog_robot_kinematics.kinematics_dh import ik_leg

JOINT_LIMITS = {
    "hip":   (-0.785, 0.785),
    "thigh": (-1.571, 1.571),
    "knee":  (0.0,    2.617),
}

def test_ik_at_stand_pose_recovers_zero_hip():
    target = np.array([DH.L_hh + DH.L_th + DH.L_sh - 0.012, 0.0, 0.0])
    theta = ik_leg(DH, target, knee_direction=+1)
    assert abs(theta[0]) < 1e-9
    foot_back = fk_leg(DH, theta)
    assert np.allclose(foot_back, target, atol=1e-9)

def test_fk_ik_roundtrip_random():
    rng = np.random.default_rng(seed=42)
    n_ok = 0
    for _ in range(200):
        theta = (
            rng.uniform(*JOINT_LIMITS["hip"]),
            rng.uniform(-0.6, 0.6),
            rng.uniform(0.3, 1.8),
        )
        foot = fk_leg(DH, theta)
        try:
            theta_back = ik_leg(DH, foot, knee_direction=+1)
        except ValueError:
            continue
        foot_again = fk_leg(DH, theta_back)
        assert np.allclose(foot, foot_again, atol=1e-6), (theta, foot, theta_back, foot_again)
        n_ok += 1
    assert n_ok > 150, f"roundtrip succeeded for only {n_ok}/200 samples"

def test_ik_unreachable_raises():
    far = np.array([5.0, 0.0, 0.0])
    with pytest.raises(ValueError):
        ik_leg(DH, far)

# --- Task 4 leg_config tests ---
from dog_robot_kinematics.leg_config import LEGS, get_leg

def test_legs_table_has_4_entries():
    assert len(LEGS) == 4
    assert {L.name for L in LEGS} == {"FL", "FR", "BL", "BR"}

def test_mirror_signs_match_side():
    assert get_leg("FL").mirror == +1
    assert get_leg("FR").mirror == -1
    assert get_leg("BL").mirror == +1
    assert get_leg("BR").mirror == -1


# --- Task 7 d-offset tests ---

def test_dhparams_accepts_d_offsets_with_default_zero():
    """DHParams accepts d_thigh, d_knee, d_foot (default 0 keeps old behaviour)."""
    from dog_robot_kinematics.kinematics_dh import DHParams
    dh0 = DHParams(L_hh=0.025, L_th=0.117, L_sh=0.070)
    assert dh0.d_thigh == 0.0
    assert dh0.d_knee == 0.0
    assert dh0.d_foot == 0.0
    dh1 = DHParams(L_hh=0.025, L_th=0.117, L_sh=0.070,
                   d_thigh=0.025, d_knee=0.041, d_foot=0.019)
    assert dh1.d_thigh == 0.025


def test_fk_leg_with_d_offsets_differs_from_zero_offsets():
    """Non-zero d_thigh / d_knee changes FK output."""
    import numpy as np
    from dog_robot_kinematics.kinematics_dh import DHParams, fk_leg
    dh0 = DHParams(L_hh=0.025, L_th=0.117, L_sh=0.070)
    dh1 = DHParams(L_hh=0.025, L_th=0.117, L_sh=0.070,
                   d_thigh=0.025, d_knee=0.041)
    theta = (0.1, -0.3, 1.0)
    fk0 = fk_leg(dh0, theta)
    fk1 = fk_leg(dh1, theta)
    assert np.linalg.norm(fk0 - fk1) > 0.01  # > 1 cm


# --- Task 8 d-offset IK roundtrip test ---

def test_fk_ik_roundtrip_with_d_offsets():
    """200-iter FK/IK roundtrip with realistic d offsets across all 4 legs."""
    import numpy as np
    from dog_robot_kinematics.kinematics_dh import DHParams, fk_leg, ik_leg
    rng = np.random.default_rng(42)
    dh = DHParams(L_hh=0.02520, L_th=0.10980, L_sh=0.07043,
                  d_thigh=0.02536, d_knee=0.04102)
    fails = 0
    max_err = 0.0
    for _ in range(200):
        theta_in = (
            rng.uniform(-0.6, 0.6),   # hip
            rng.uniform(-1.0, 0.7),   # thigh
            rng.uniform(0.2, 2.2),    # knee
        )
        foot = fk_leg(dh, theta_in)
        try:
            theta_out = ik_leg(dh, foot, knee_direction=+1)
        except ValueError:
            fails += 1
            continue
        foot_back = fk_leg(dh, theta_out)
        err = float(np.linalg.norm(foot - foot_back))
        max_err = max(max_err, err)
    assert fails < 10, f"too many IK failures: {fails}/200"
    assert max_err < 1e-4, f"max roundtrip error {max_err:.6f} m > 0.1 mm"
