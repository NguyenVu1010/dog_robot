import math
import numpy as np
import pytest

from dog_robot_control.gait.gait_config import GaitConfig
from dog_robot_control.gait.trajectory_planner import TrajectoryPlanner


GC = GaitConfig(nominal_height=0.15, stance_duration=0.30, swing_height=0.03,
                stance_depth=0.001, max_linear_velocity_x=0.15,
                max_linear_velocity_y=0.08, max_angular_velocity_z=0.50)


def test_zero_step_length_no_delta():
    tp = TrajectoryPlanner(GC)
    foot = np.array([0.1, 0.0, -0.15])
    out = tp.generate(foot.copy(), step_length=0.0, rotation=0.0,
                      swing_phase=0.0, stance_phase=0.0)
    assert np.allclose(out, foot)


def test_stance_sweep_linear_x():
    tp = TrajectoryPlanner(GC)
    foot0 = np.array([0.0, 0.0, -0.15])
    s = 0.08
    at_start = tp.generate(foot0.copy(), s, 0.0, swing_phase=0.0, stance_phase=0.001)
    at_end   = tp.generate(foot0.copy(), s, 0.0, swing_phase=0.0, stance_phase=0.999)
    assert at_start[0] == pytest.approx( s / 2, abs=1e-3)
    assert at_end[0]   == pytest.approx(-s / 2, abs=1e-3)


def test_swing_peak_lifts_foot():
    tp = TrajectoryPlanner(GC)
    foot0 = np.array([0.0, 0.0, -0.15])
    s = 0.08
    out = tp.generate(foot0.copy(), s, 0.0, swing_phase=0.5, stance_phase=0.0)
    assert out[2] > foot0[2] + 0.005, f"foot did not lift: z={out[2]}"


def test_swing_endpoints_x_match_stance_endpoints():
    tp = TrajectoryPlanner(GC)
    foot0 = np.array([0.0, 0.0, -0.15])
    s = 0.08
    swing_start = tp.generate(foot0.copy(), s, 0.0, swing_phase=0.01, stance_phase=0.0)
    swing_end   = tp.generate(foot0.copy(), s, 0.0, swing_phase=0.99, stance_phase=0.0)
    assert swing_start[0] == pytest.approx(-s / 2, abs=0.02)
    assert swing_end[0]   == pytest.approx( s / 2, abs=0.02)


def test_rotation_steers_delta_into_y():
    tp = TrajectoryPlanner(GC)
    foot0 = np.array([0.0, 0.0, -0.15])
    s = 0.08
    out = tp.generate(foot0.copy(), s, rotation=math.pi/2,
                      swing_phase=0.0, stance_phase=0.001)
    assert out[0] == pytest.approx(0.0, abs=1e-3)
    assert out[1] == pytest.approx(s / 2, abs=1e-3)
