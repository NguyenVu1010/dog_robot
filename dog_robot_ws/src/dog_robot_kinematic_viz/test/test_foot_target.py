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
    # At v=0 the foot must hold rest position across the entire cycle:
    # stride is zero (sx=sy=0) AND swing lift scales to zero.
    for phi in np.linspace(0.0, 0.999, 50):
        p = foot_target_in_hip(REST, phi, (0.0, 0.0), PARAMS)
        np.testing.assert_allclose(p, REST, atol=1e-12,
                                    err_msg=f"phi={phi}: foot drifted from rest")


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


def test_zero_velocity_no_swing_lift_at_any_phase():
    # Explicit: even at the swing apex (phi=0.75), z must equal REST[2] at v=0.
    p = foot_target_in_hip(REST, 0.75, (0.0, 0.0), PARAMS)
    assert p[2] == pytest.approx(REST[2], abs=1e-12)


def test_swing_lift_scales_linearly_below_activation_speed():
    # At swing apex (u=0.5 -> sin=1), z_lift = swing_height * (|v|/v_act).
    phi_apex = 0.75
    v_act = PARAMS.swing_activation_speed
    # v = 25% of v_act -> 25% of swing_height
    p_25 = foot_target_in_hip(REST, phi_apex, (0.25 * v_act, 0.0), PARAMS)
    # v = 75% of v_act -> 75% of swing_height
    p_75 = foot_target_in_hip(REST, phi_apex, (0.75 * v_act, 0.0), PARAMS)
    lift_25 = p_25[2] - REST[2]
    lift_75 = p_75[2] - REST[2]
    assert lift_25 == pytest.approx(0.25 * PARAMS.swing_height, abs=1e-12)
    assert lift_75 == pytest.approx(0.75 * PARAMS.swing_height, abs=1e-12)


def test_swing_lift_saturates_at_activation_speed():
    # |v| >= swing_activation_speed -> full lift, no further increase.
    phi_apex = 0.75
    p_at = foot_target_in_hip(REST, phi_apex, (PARAMS.swing_activation_speed, 0.0),
                              PARAMS)
    p_2x = foot_target_in_hip(REST, phi_apex, (2.0 * PARAMS.swing_activation_speed, 0.0),
                              PARAMS)
    assert p_at[2] == pytest.approx(REST[2] + PARAMS.swing_height, abs=1e-12)
    assert p_2x[2] == pytest.approx(REST[2] + PARAMS.swing_height, abs=1e-12)


def test_lateral_velocity_also_activates_swing():
    # Symmetric in x and y: pure y velocity must lift the foot the same as pure x.
    phi_apex = 0.75
    p_x = foot_target_in_hip(REST, phi_apex, (PARAMS.swing_activation_speed, 0.0),
                             PARAMS)
    p_y = foot_target_in_hip(REST, phi_apex, (0.0, PARAMS.swing_activation_speed),
                             PARAMS)
    assert p_x[2] == pytest.approx(p_y[2], abs=1e-12)
