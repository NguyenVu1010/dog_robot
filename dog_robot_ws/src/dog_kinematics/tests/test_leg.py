import math
import pytest
from dog_kinematics.leg import legIK, calcLegPoints, OutOfWorkspace


def test_legik_known_value():
    """Reference value from TestIK with foot at (0, -0.140, 0.100)."""
    # IK input is in leg-frame meters: x=fore-aft, y=up (negative=down), z=lateral
    # Convert from test config: foot at (0, -0.140, 0.10049) [m, IK frame]
    omega, theta, phi, D, G = legIK(0.0, -0.140, 0.10049)
    assert abs(omega - math.pi) < 1.0   # near pi (nominal stance), within ±~60°
    assert abs(theta) < 1.0
    assert 0 < phi < math.pi


def test_legik_roundtrip():
    """IK → FK round-trip: foot position should match input."""
    x_in, y_in, z_in = 0.02, -0.130, 0.08
    omega, theta, phi, D, _ = legIK(x_in, y_in, z_in)
    pts = calcLegPoints(omega, theta, phi, D)
    foot = pts[3]
    assert abs(foot[0] - x_in) < 1e-6
    assert abs(foot[1] - y_in) < 1e-6
    assert abs(foot[2] - z_in) < 1e-6


def test_legik_out_of_workspace_raises():
    """Foot too far → OutOfWorkspace."""
    with pytest.raises(OutOfWorkspace):
        legIK(0.0, -1.0, 0.0)  # 1m down, unreachable
