"""Foot trajectory geometry tests.

These tests verify the math in body frame (R=identity, body_z=0). Per-leg
geometric integration (R != identity) is covered in test_leg_driver.py.
"""
import numpy as np
import pytest

from dog_robot_kinematic_viz.foot_target import (
    FootTargetParams, foot_target_in_hip,
)


REST = np.array([0.03, 0.04, -0.12])
PARAMS = FootTargetParams(stride_per_mps=0.20, swing_height=0.03,
                          stance_phase_ratio=0.5)
EYE = np.eye(3)
PHI_APEX = PARAMS.stance_phase_ratio + 0.5 * (1.0 - PARAMS.stance_phase_ratio)


def _ft(rest, phi, v_body, body_z=0.0, extra_z=0.0, R=EYE, params=PARAMS):
    """Test helper: call foot_target_in_hip with sensible defaults."""
    return foot_target_in_hip(rest, phi, v_body, body_z, extra_z, R, params)


def test_zero_velocity_holds_rest():
    for phi in np.linspace(0.0, 0.999, 50):
        p = _ft(REST, phi, (0.0, 0.0))
        np.testing.assert_allclose(p, REST, atol=1e-12,
                                    err_msg=f"phi={phi}")


def test_stance_start_and_swing_end_match_across_cycle_wrap():
    p0 = _ft(REST, 0.0, (0.5, 0.0))
    p1 = _ft(REST, 0.99999, (0.5, 0.0))
    # x continuous across the wrap, z back at rest level
    np.testing.assert_allclose(p0[0], p1[0], atol=1e-4)
    np.testing.assert_allclose(p0[2], REST[2], atol=1e-12)
    np.testing.assert_allclose(p1[2], REST[2], atol=1e-4)


def test_stance_swing_seam_continuous():
    eps = 1e-6
    p_minus = _ft(REST, 0.5 - eps, (0.5, 0.3))
    p_plus = _ft(REST, 0.5 + eps, (0.5, 0.3))
    np.testing.assert_allclose(p_minus, p_plus, atol=1e-4)


def test_swing_apex_lifts_by_swing_height():
    p = _ft(REST, PHI_APEX, (0.1, 0.0))
    np.testing.assert_allclose(p[2], REST[2] + PARAMS.swing_height, atol=1e-12)


def test_stance_drags_opposite_to_body_velocity():
    v = (1.0, 0.0)
    p_start = _ft(REST, 0.0, v)
    p_mid = _ft(REST, 0.25, v)
    p_end = _ft(REST, 0.499, v)
    assert p_start[0] > p_mid[0] > p_end[0]
    np.testing.assert_allclose(p_start[0] + p_end[0], 2 * REST[0], atol=1e-3)


def test_swing_resets_forward():
    v = (1.0, 0.0)
    p_swing_start = _ft(REST, 0.501, v)
    p_swing_end = _ft(REST, 0.999, v)
    assert p_swing_end[0] > p_swing_start[0]


def test_stride_scales_linearly_with_velocity():
    v1 = (0.5, 0.0)
    v2 = (1.0, 0.0)
    p1 = _ft(REST, 0.0, v1)
    p2 = _ft(REST, 0.0, v2)
    d1 = p1[0] - REST[0]
    d2 = p2[0] - REST[0]
    np.testing.assert_allclose(d2, 2.0 * d1, atol=1e-12)


def test_phase_wraps_correctly():
    v = (0.5, 0.0)
    p1 = _ft(REST, 0.3, v)
    p2 = _ft(REST, 1.3, v)
    p3 = _ft(REST, -0.7, v)
    np.testing.assert_allclose(p1, p2, atol=1e-12)
    np.testing.assert_allclose(p1, p3, atol=1e-12)


def test_y_stride_independent_of_x_stride():
    v_x = (1.0, 0.0)
    v_y = (0.0, 1.0)
    p_x = _ft(REST, 0.25, v_x)
    p_y = _ft(REST, 0.25, v_y)
    np.testing.assert_allclose(p_x[1], REST[1], atol=1e-12)
    np.testing.assert_allclose(p_y[0], REST[0], atol=1e-12)


def test_zero_velocity_no_swing_lift_at_any_phase():
    p = _ft(REST, PHI_APEX, (0.0, 0.0))
    assert p[2] == pytest.approx(REST[2], abs=1e-12)


def test_swing_lift_scales_linearly_below_activation_speed():
    v_act = PARAMS.swing_activation_speed
    p_25 = _ft(REST, PHI_APEX, (0.25 * v_act, 0.0))
    p_75 = _ft(REST, PHI_APEX, (0.75 * v_act, 0.0))
    assert (p_25[2] - REST[2]) == pytest.approx(0.25 * PARAMS.swing_height, abs=1e-12)
    assert (p_75[2] - REST[2]) == pytest.approx(0.75 * PARAMS.swing_height, abs=1e-12)


def test_swing_lift_saturates_at_activation_speed():
    p_at = _ft(REST, PHI_APEX, (PARAMS.swing_activation_speed, 0.0))
    p_2x = _ft(REST, PHI_APEX, (2.0 * PARAMS.swing_activation_speed, 0.0))
    assert p_at[2] == pytest.approx(REST[2] + PARAMS.swing_height, abs=1e-12)
    assert p_2x[2] == pytest.approx(REST[2] + PARAMS.swing_height, abs=1e-12)


def test_lateral_velocity_also_activates_swing():
    p_x = _ft(REST, PHI_APEX, (PARAMS.swing_activation_speed, 0.0))
    p_y = _ft(REST, PHI_APEX, (0.0, PARAMS.swing_activation_speed))
    assert p_x[2] == pytest.approx(p_y[2], abs=1e-12)


def test_zero_activation_speed_disables_scaling():
    params = FootTargetParams(stride_per_mps=0.20, swing_height=0.03,
                              stance_phase_ratio=0.5,
                              swing_activation_speed=0.0)
    phi_apex = params.stance_phase_ratio + 0.5 * (1.0 - params.stance_phase_ratio)
    p = _ft(REST, phi_apex, (1e-9, 0.0), params=params)
    assert p[2] == pytest.approx(REST[2] + params.swing_height, abs=1e-12)
    p0 = _ft(REST, phi_apex, (0.0, 0.0), params=params)
    assert p0[2] == pytest.approx(REST[2] + params.swing_height, abs=1e-12)


# --- new test for body_z ---

def test_body_z_shifts_foot_negative_in_body_z():
    # body_z=+0.02 should drop the foot by -0.02 in body Z (R=I so body=hip).
    p = _ft(REST, 0.0, (0.0, 0.0), body_z=0.02)
    np.testing.assert_allclose(p[2], REST[2] - 0.02, atol=1e-12)
    np.testing.assert_allclose(p[:2], REST[:2], atol=1e-12)


def test_body_z_composes_with_swing_lift():
    # At swing apex with full velocity: foot z = rest + swing_height - body_z.
    p = _ft(REST, PHI_APEX, (0.10, 0.0), body_z=0.02)
    np.testing.assert_allclose(p[2], REST[2] + PARAMS.swing_height - 0.02, atol=1e-12)


# --- extra_z tests ---

def test_extra_z_zero_matches_baseline():
    # Regression: extra_z=0 (new arg) preserves existing behavior at every phase.
    for phi in (0.0, 0.25, PHI_APEX, 0.75):
        p_default = _ft(REST, phi, (0.10, 0.0))
        p_explicit = _ft(REST, phi, (0.10, 0.0), extra_z=0.0)
        np.testing.assert_allclose(
            p_default, p_explicit, atol=1e-12,
            err_msg=f"phi={phi}: extra_z default != extra_z=0.0")


def test_extra_z_lifts_foot_in_body_z():
    # extra_z=+0.05 should LIFT the foot by +0.05 in body Z (R=I so body=hip).
    p = _ft(REST, 0.0, (0.0, 0.0), extra_z=0.05)
    np.testing.assert_allclose(p[2], REST[2] + 0.05, atol=1e-12)
    np.testing.assert_allclose(p[:2], REST[:2], atol=1e-12)


def test_extra_z_composes_with_body_z():
    # body_z=+0.02 drops foot -0.02, extra_z=+0.05 lifts +0.05 -> net +0.03.
    p = _ft(REST, 0.0, (0.0, 0.0), body_z=0.02, extra_z=0.05)
    np.testing.assert_allclose(p[2], REST[2] + 0.03, atol=1e-12)


def test_extra_z_composes_with_swing_lift():
    # At swing apex with full velocity: foot z = rest + swing_height + extra_z.
    p = _ft(REST, PHI_APEX, (0.10, 0.0), extra_z=0.04)
    np.testing.assert_allclose(
        p[2], REST[2] + PARAMS.swing_height + 0.04, atol=1e-12)
