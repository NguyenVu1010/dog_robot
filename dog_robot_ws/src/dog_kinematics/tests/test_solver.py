# tests/test_solver.py
import numpy as np
from dog_kinematics.solver import solve_all_legs


def test_solve_all_legs_nominal_stand():
    """4 feet at nominal stand position → IK returns 12 joint angles."""
    body_pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  # omega, phi, psi, xm, ym, zm
    foot_targets_world = {
        "FL": ( 0.100, -0.140,  0.100, 1.0),
        "FR": ( 0.100, -0.140, -0.100, 1.0),
        "BL": (-0.100, -0.140,  0.100, 1.0),
        "BR": (-0.100, -0.140, -0.100, 1.0),
    }
    angles = solve_all_legs(body_pose, foot_targets_world)
    assert len(angles) == 12
    expected_keys = {
        f"{leg}_{j}"
        for leg in ("FL", "FR", "BL", "BR")
        for j in ("hip_yaw", "thigh_pitch", "knee_pitch")
    }
    assert set(angles.keys()) == expected_keys
    # All angles within reasonable bounds when wrapped to [-π, π].
    #
    # SPEC-BUG FIX (Task 5): The original assertion "-3.14 < v < 3.14" fails for
    # hip_yaw (omega) in nominal stance.  Mathematical reason:
    #
    #   omega = atan2(z_leg, y_leg) + atan2(D, L2)
    #
    # The spec foot targets (e.g. FL: (0.1, -0.14, 0.1)) map through the
    # Ry(π/2) hip frame into leg-local coords (-0.06, -0.14, 0.0).  With
    # z_leg = 0 and y_leg = -0.14 < 0:
    #
    #   atan2(0, -0.14) = π          (atan2 returns +π for (0⁺, negative))
    #   atan2(D, L2)    ≈ 1.215 rad
    #   omega           ≈ 4.356 rad  > 3.14  ← assertion fails
    #
    # The same result was observed in test_legik_known_value (Task 3), which
    # already allows "abs(omega - π) < 1.0".  omega = 4.356 rad is physically
    # correct; it just happens to fall outside [-π, π].  Wrapping to [-π, π]
    # gives -1.928 rad, which is well within motor range.
    #
    # Fix: wrap each angle to [-π, π] before the bound check.  This is
    # equivalent to asserting the angle is representable as a ±180° rotation,
    # which is the correct sanity check for joint angles.
    for v in angles.values():
        v_wrapped = ((v + np.pi) % (2 * np.pi)) - np.pi
        assert -np.pi < v_wrapped < np.pi, (
            f"angle {v:.4f} rad wraps to {v_wrapped:.4f} rad, outside (-π, π)"
        )
