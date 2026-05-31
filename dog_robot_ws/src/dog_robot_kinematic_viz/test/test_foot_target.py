"""Foot trajectory geometry tests."""
import numpy as np
import pytest

from dog_robot_kinematic_viz.foot_target import (
    FootTargetParams, foot_target_in_hip,
)


REST = np.array([0.03, 0.04, -0.12])
PARAMS = FootTargetParams(stride_per_mps=0.20, swing_height=0.03,
                          stance_phase_ratio=0.5)


def test_zero_velocity_holds_rest():
    for phi in np.linspace(0.0, 0.999, 50):
        p = foot_target_in_hip(REST, phi, (0.0, 0.0), PARAMS)
        # No stride => stance XY stays at rest, swing has z lift only.
        if phi < 0.5:
            np.testing.assert_allclose(p, REST, atol=1e-12)
        else:
            np.testing.assert_allclose(p[:2], REST[:2], atol=1e-12)
            # z >= rest_z during swing (sin >= 0)
            assert p[2] >= REST[2] - 1e-12


def test_stance_start_and_swing_end_match_across_cycle_wrap():
    # phi=0 (stance start) should equal phi -> 1 (swing end)
    p0 = foot_target_in_hip(REST, 0.0, (0.5, 0.0), PARAMS)
    p1 = foot_target_in_hip(REST, 0.99999, (0.5, 0.0), PARAMS)
    # x continuous across the wrap, z back at rest level
    np.testing.assert_allclose(p0[0], p1[0], atol=1e-4)
    np.testing.assert_allclose(p0[2], REST[2], atol=1e-12)
    np.testing.assert_allclose(p1[2], REST[2], atol=1e-4)


def test_stance_swing_seam_continuous():
    # phi=0.5 from stance side vs swing side
    eps = 1e-6
    p_minus = foot_target_in_hip(REST, 0.5 - eps, (0.5, 0.3), PARAMS)
    p_plus = foot_target_in_hip(REST, 0.5 + eps, (0.5, 0.3), PARAMS)
    np.testing.assert_allclose(p_minus, p_plus, atol=1e-4)


def test_swing_apex_lifts_by_swing_height():
    # u = 0.5 in swing -> sin(pi/2) = 1 -> z_lift = swing_height
    phi_apex = 0.5 + 0.5 * (1.0 - 0.5)
    p = foot_target_in_hip(REST, phi_apex, (0.1, 0.0), PARAMS)
    np.testing.assert_allclose(p[2], REST[2] + PARAMS.swing_height, atol=1e-12)


def test_stance_drags_opposite_to_body_velocity():
    # Body moves +x. During stance, foot.x should drop from above rest to below rest.
    v = (1.0, 0.0)
    p_start = foot_target_in_hip(REST, 0.0, v, PARAMS)
    p_mid = foot_target_in_hip(REST, 0.25, v, PARAMS)
    p_end = foot_target_in_hip(REST, 0.499, v, PARAMS)
    assert p_start[0] > p_mid[0] > p_end[0]
    # symmetric around rest
    np.testing.assert_allclose(p_start[0] + p_end[0], 2 * REST[0], atol=1e-3)


def test_swing_resets_forward():
    v = (1.0, 0.0)
    p_swing_start = foot_target_in_hip(REST, 0.501, v, PARAMS)
    p_swing_end = foot_target_in_hip(REST, 0.999, v, PARAMS)
    assert p_swing_end[0] > p_swing_start[0]


def test_stride_scales_linearly_with_velocity():
    v1 = (0.5, 0.0)
    v2 = (1.0, 0.0)
    p1 = foot_target_in_hip(REST, 0.0, v1, PARAMS)
    p2 = foot_target_in_hip(REST, 0.0, v2, PARAMS)
    # Difference from rest should scale 1:2
    d1 = p1[0] - REST[0]
    d2 = p2[0] - REST[0]
    np.testing.assert_allclose(d2, 2.0 * d1, atol=1e-12)


def test_phase_wraps_correctly():
    v = (0.5, 0.0)
    p1 = foot_target_in_hip(REST, 0.3, v, PARAMS)
    p2 = foot_target_in_hip(REST, 1.3, v, PARAMS)   # equivalent
    p3 = foot_target_in_hip(REST, -0.7, v, PARAMS)  # equivalent
    np.testing.assert_allclose(p1, p2, atol=1e-12)
    np.testing.assert_allclose(p1, p3, atol=1e-12)


def test_y_stride_independent_of_x_stride():
    v_x = (1.0, 0.0)
    v_y = (0.0, 1.0)
    p_x = foot_target_in_hip(REST, 0.25, v_x, PARAMS)
    p_y = foot_target_in_hip(REST, 0.25, v_y, PARAMS)
    # x-stride only moves x; y-stride only moves y.
    np.testing.assert_allclose(p_x[1], REST[1], atol=1e-12)
    np.testing.assert_allclose(p_y[0], REST[0], atol=1e-12)
