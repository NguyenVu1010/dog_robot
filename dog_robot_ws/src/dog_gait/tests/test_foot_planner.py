import numpy as np
import pytest
from dog_gait.foot_planner import FootPlanner


def test_stance_endpoints():
    """At start of stance (phi=0), foot at +stride/2 forward. At end, -stride/2."""
    fp = FootPlanner(cycle_time=0.4, duty_factor=0.5, step_height=0.05, max_stride=0.1)
    pos_start = fp.foot_position(phase=0.0, vel=(0.1, 0, 0))   # stance start
    pos_end_stance = fp.foot_position(phase=0.499, vel=(0.1, 0, 0))
    assert pos_start[0] > pos_end_stance[0]  # foot moved backward


def test_swing_apex_height():
    """At swing apex (phi=0.75 = stance_end + 0.25 swing time), foot at max height."""
    fp = FootPlanner(cycle_time=0.4, duty_factor=0.5, step_height=0.05, max_stride=0.1)
    pos_swing_apex = fp.foot_position(phase=0.75, vel=(0.1, 0, 0))
    pos_stance = fp.foot_position(phase=0.25, vel=(0.1, 0, 0))
    assert pos_swing_apex[2] > pos_stance[2]  # higher in Z (up)


def test_zero_velocity_static():
    """Zero cmd_vel → foot stays at origin (0, 0, 0) in body frame."""
    fp = FootPlanner(cycle_time=0.4, duty_factor=0.5, step_height=0.05, max_stride=0.1)
    pos = fp.foot_position(phase=0.0, vel=(0, 0, 0))
    assert abs(pos[0]) < 1e-6
    assert abs(pos[1]) < 1e-6
    assert abs(pos[2]) < 1e-6


def test_continuous_at_phase_boundary():
    """Foot trajectory must be continuous at stance→swing transition."""
    fp = FootPlanner(cycle_time=0.4, duty_factor=0.5, step_height=0.05, max_stride=0.1)
    pos_before = fp.foot_position(phase=0.499, vel=(0.1, 0, 0))
    pos_after  = fp.foot_position(phase=0.500, vel=(0.1, 0, 0))
    diff = np.linalg.norm(np.array(pos_before) - np.array(pos_after))
    assert diff < 0.01  # <1cm jump
