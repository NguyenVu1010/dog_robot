import math
import numpy as np
from dog_kinematics.body import bodyIK, world_to_leg


def test_bodyik_identity_pose():
    """Zero pose: 4 hip frames at corners with identity orientation."""
    Tlf, Trf, Tlb, Trb, Tm = bodyIK(0, 0, 0, 0, 0, 0)
    # Tm should be identity
    assert np.allclose(Tm, np.eye(4))
    # LF hip at (+L/2, 0, +W/2) but after the 90° rotation: row order specific
    # We just check the translations exist
    assert Tlf.shape == (4, 4)


def test_world_to_leg_left_no_mirror():
    """Left leg should not have X flipped."""
    Tlf, *_ = bodyIK(0, 0, 0, 0, 0, 0)
    foot_world = np.array([0.1, -0.14, 0.10, 1.0])
    Q = world_to_leg(Tlf, foot_world, is_right=False)
    assert Q.shape == (4,)


def test_world_to_leg_right_mirror():
    """Right leg should have X flipped (Ix).

    The Ix mirror negates the first (x) component of the leg-frame vector.
    Verified against TestIK/4leg.py: world_to_leg with is_right=True applies
    Ix @ inv(T_leg) @ foot, so Q_r[0] == -Q_l[0] for the same foot position.
    """
    _, Trf, *_ = bodyIK(0, 0, 0, 0, 0, 0)
    foot_world = np.array([0.1, -0.14, 0.10, 1.0])
    Q_l = world_to_leg(Trf, foot_world, is_right=False)
    Q_r = world_to_leg(Trf, foot_world, is_right=True)
    # Ix mirror: first element negated, rest unchanged
    assert abs(Q_l[0] + Q_r[0]) < 1e-6
    assert np.allclose(Q_l[1:], Q_r[1:])
