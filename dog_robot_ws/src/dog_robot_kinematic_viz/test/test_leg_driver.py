"""LegDriver: per-leg roundtrip + joint-limit guarantees over safe directions.

The closed-form ik_leg has a non-convex workspace. Body +/-x is uniformly
reachable for all 4 legs at the gait velocity we expect (<=0.10 m/s). The
tests below pin that operating region; the LegDriver's "hold last on
ValueError" fallback covers the residual cases when a user commands large
lateral velocity.
"""
from pathlib import Path

import numpy as np
import pytest

from dog_robot_kinematics.kinematics_link import load_link_params, fk_leg

from dog_robot_kinematic_viz.leg_geometry import LEG_NAMES, load_leg_geoms
from dog_robot_kinematic_viz.foot_target import FootTargetParams
from dog_robot_kinematic_viz.leg_driver import LegDriver


CFG = Path(__file__).resolve().parents[2] / "dog_robot_description" / "config"
LINK_PARAMS_YAML = CFG / "link_params.yaml"
URDF_JOINTS_YAML = CFG / "urdf_joints.yaml"

# Must mirror leg.xacro's <limit lower=... upper=...>.
JOINT_LIMITS = {
    "hip_roll":     (-0.785, 0.785),
    "thigh_pitch": (-1.571, 1.571),
    "knee_pitch":  (-2.617, 0.5),
}

PARAMS = FootTargetParams(stride_per_mps=0.20, swing_height=0.03,
                          stance_phase_ratio=0.5)


def _make_drivers():
    geoms = load_leg_geoms(URDF_JOINTS_YAML)
    return {
        name: LegDriver(geoms[name],
                        load_link_params(LINK_PARAMS_YAML, name),
                        PARAMS)
        for name in LEG_NAMES
    }


def _assert_within_limits(name, q):
    for joint_name, val in zip(("hip_roll", "thigh_pitch", "knee_pitch"), q):
        lo, hi = JOINT_LIMITS[joint_name]
        assert lo <= val <= hi, f"{name} {joint_name}={val} out of [{lo},{hi}]"


@pytest.mark.parametrize("name", LEG_NAMES)
def test_zero_velocity_stance_phase_recovers_zero_joints(name):
    # Stance with no body velocity => foot stays at CAD rest => q ~ (0,0,0).
    drivers = _make_drivers()
    d = drivers[name]
    for phi in (0.0, 0.1, 0.25, 0.4, 0.499):
        q = d.step((0.0, 0.0), phi)
        np.testing.assert_allclose(
            q, (0.0, 0.0, 0.0), atol=1e-6,
            err_msg=f"{name} @ phi={phi}: q={q}")


@pytest.mark.parametrize("name", LEG_NAMES)
def test_zero_velocity_swing_phase_lifts_foot_within_limits(name):
    # Swing with no body velocity => foot rises in z, XY at rest => q != 0
    # but all within joint limits.
    drivers = _make_drivers()
    d = drivers[name]
    for phi in (0.5, 0.625, 0.75, 0.875, 0.999):
        q = d.step((0.0, 0.0), phi)
        _assert_within_limits(name, q)
        # The hip yaw should still be ~0 since the foot moves only in z.
        assert abs(q[0]) < 1e-3, f"{name} @ phi={phi}: hip_roll={q[0]} drifted"


@pytest.mark.parametrize("name", LEG_NAMES)
def test_forward_velocity_full_cycle_reachable_and_within_limits(name):
    # The primary verification case: body moves +x at the design max velocity.
    # All 30 phase samples in one cycle must produce in-limit joint angles.
    drivers = _make_drivers()
    d = drivers[name]
    for phi in np.linspace(0.0, 1.0, 30, endpoint=False):
        q = d.step((0.10, 0.0), float(phi))
        _assert_within_limits(name, q)


@pytest.mark.parametrize("name", LEG_NAMES)
def test_backward_velocity_full_cycle_reachable(name):
    drivers = _make_drivers()
    d = drivers[name]
    for phi in np.linspace(0.0, 1.0, 30, endpoint=False):
        q = d.step((-0.10, 0.0), float(phi))
        _assert_within_limits(name, q)


@pytest.mark.parametrize("name", LEG_NAMES)
def test_fk_of_step_matches_commanded_target_on_forward_velocity(name):
    # Roundtrip on the reachable forward-velocity gait: FK(LegDriver.step()) ==
    # the foot target the driver computed internally.
    from dog_robot_kinematic_viz.foot_target import foot_target_in_hip
    drivers = _make_drivers()
    d = drivers[name]
    for phi in np.linspace(0.0, 1.0, 30, endpoint=False):
        v = (0.10, 0.0)
        q = d.step(v, float(phi))
        foot_fk = fk_leg(d.link, q)
        v_hip = d.geom.R_base_to_hip.T @ np.array([v[0], v[1], 0.0])
        expected = foot_target_in_hip(
            d.rest_in_hip, float(phi), (v_hip[0], v_hip[1]), PARAMS)
        np.testing.assert_allclose(
            foot_fk, expected, atol=1e-6,
            err_msg=f"{name} phi={phi:.3f}: FK={foot_fk} expected={expected}")


def test_foot_at_rest_is_below_hip_for_all_legs():
    # rest_in_hip is in the hip frame (where hip-Z = body +X after the REP-103
    # convention switch). Convert to body frame and check the foot hangs below
    # the hip (negative body Z).
    drivers = _make_drivers()
    for name, d in drivers.items():
        rest_body = d.geom.R_base_to_hip @ d.rest_in_hip
        assert rest_body[2] < -0.05, (
            f"{name} rest too shallow in body frame: {rest_body}")


def test_unreachable_target_holds_last_joints():
    # Synthesise an unreachable foot target by patching rest to the hip
    # rotation axis (where ik_leg raises). The driver must hold its previous q.
    geoms = load_leg_geoms(URDF_JOINTS_YAML)
    d = LegDriver(geoms["FL"], load_link_params(LINK_PARAMS_YAML, "FL"), PARAMS)
    q1 = d.step((0.0, 0.0), 0.0)
    d.rest_in_hip = np.array([0.0, 0.0, -0.13])    # on yaw axis -> raises
    q2 = d.step((0.0, 0.0), 0.0)
    assert q1 == q2


def test_continuity_across_stance_swing_seam_forward_velocity():
    drivers = _make_drivers()
    eps = 1e-5
    for name, d in drivers.items():
        q_minus = d.step((0.08, 0.0), 0.5 - eps)
        q_plus = d.step((0.08, 0.0), 0.5 + eps)
        np.testing.assert_allclose(
            q_minus, q_plus, atol=1e-3,
            err_msg=f"{name} seam discontinuity at phi=0.5: {q_minus} -> {q_plus}")


def test_continuity_across_cycle_wrap_forward_velocity():
    drivers = _make_drivers()
    eps = 1e-5
    for name, d in drivers.items():
        q_end = d.step((0.08, 0.0), 1.0 - eps)
        q_start = d.step((0.08, 0.0), 0.0)
        np.testing.assert_allclose(
            q_end, q_start, atol=1e-3,
            err_msg=f"{name} wrap discontinuity phi=1->0: {q_end} -> {q_start}")


@pytest.mark.parametrize("name", LEG_NAMES)
def test_zero_velocity_stance_with_body_z_shifts_foot_in_body_z(name):
    # body_z > 0 means the body sits higher relative to the feet, so each
    # foot must be a distance `body_z` LOWER in body Z than at rest.
    drivers = _make_drivers()
    d = drivers[name]
    bz = 0.04
    q = d.step((0.0, 0.0), 0.0, body_z=bz)
    # FK in hip frame, rotate to body frame.
    foot_hip = fk_leg(d.link, q)
    foot_body = d.geom.R_base_to_hip @ foot_hip
    rest_body = d.geom.R_base_to_hip @ d.rest_in_hip
    expected_body = rest_body + np.array([0.0, 0.0, -bz])
    np.testing.assert_allclose(
        foot_body, expected_body, atol=1e-6,
        err_msg=f"{name}: foot_body={foot_body} expected={expected_body}")


@pytest.mark.parametrize("name", LEG_NAMES)
def test_zero_velocity_stance_with_negative_body_z_shifts_foot_up(name):
    drivers = _make_drivers()
    d = drivers[name]
    bz = -0.04
    q = d.step((0.0, 0.0), 0.0, body_z=bz)
    foot_hip = fk_leg(d.link, q)
    foot_body = d.geom.R_base_to_hip @ foot_hip
    rest_body = d.geom.R_base_to_hip @ d.rest_in_hip
    expected_body = rest_body + np.array([0.0, 0.0, -bz])  # = +0.04
    np.testing.assert_allclose(
        foot_body, expected_body, atol=1e-6,
        err_msg=f"{name}: foot_body={foot_body} expected={expected_body}")


@pytest.mark.parametrize("name", LEG_NAMES)
@pytest.mark.parametrize("bz", [+0.04, -0.04])
def test_body_z_extreme_keeps_joints_in_limits_full_cycle(name, bz):
    # Full forward-velocity cycle at the body_z clamp extremes must stay
    # within hardware joint limits.
    drivers = _make_drivers()
    d = drivers[name]
    for phi in np.linspace(0.0, 1.0, 30, endpoint=False):
        q = d.step((0.10, 0.0), float(phi), body_z=bz)
        _assert_within_limits(name, q)


def test_step_body_z_default_matches_zero_explicit_body_z():
    # Backward-compat: calling without body_z must equal body_z=0.0.
    drivers = _make_drivers()
    for name, d in drivers.items():
        # Reset internal _last_joints to avoid cross-call state leak.
        d._last_joints = (0.0, 0.0, 0.0)
        q_default = d.step((0.05, 0.0), 0.25)
        d._last_joints = (0.0, 0.0, 0.0)
        q_explicit = d.step((0.05, 0.0), 0.25, body_z=0.0)
        np.testing.assert_allclose(q_default, q_explicit, atol=1e-12)


@pytest.mark.parametrize("name", LEG_NAMES)
def test_rest_in_hip_not_mutated_by_body_z_step(name):
    # The shift must be per-call: self.rest_in_hip stays at the CAD value.
    drivers = _make_drivers()
    d = drivers[name]
    rest_before = d.rest_in_hip.copy()
    d.step((0.0, 0.0), 0.0, body_z=0.04)
    d.step((0.0, 0.0), 0.0, body_z=-0.04)
    np.testing.assert_array_equal(d.rest_in_hip, rest_before)
