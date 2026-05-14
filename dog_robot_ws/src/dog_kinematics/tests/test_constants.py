from dog_kinematics import constants as c


def test_leg_dimensions_match_testik():
    """Constants must match TestIK/4leg.py values."""
    assert c.L1 == 0.0125
    assert c.L2 == 0.04895
    assert abs(c.L3 - 0.109202) < 1e-9
    assert c.L4 == 0.115


def test_body_dimensions():
    assert c.BODY_LENGTH == 0.200
    assert c.BODY_WIDTH == 0.080


def test_leg_names_are_4():
    assert set(c.LEG_NAMES) == {"FL", "FR", "BL", "BR"}


def test_joint_names_are_12():
    assert len(c.JOINT_NAMES) == 12
    for leg in c.LEG_NAMES:
        for joint in ("hip_yaw", "thigh_pitch", "knee_pitch"):
            assert f"{leg}_{joint}" in c.JOINT_NAMES


def test_joint_limits_complete():
    assert all(j in c.JOINT_LIMITS for j in c.JOINT_NAMES)
    fl_hip = c.JOINT_LIMITS["FL_hip_yaw"]
    assert fl_hip["lower"] == -0.785
    assert fl_hip["upper"] == 0.785
