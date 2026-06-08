"""FootTrail buffer + Marker builder tests.

The buffer logic is pure Python (testable without ROS).
build_marker requires visualization_msgs — skipped when not importable.
"""
import numpy as np
import pytest

from dog_robot_kinematic_viz.foot_trail import (
    FootTrail, LEG_COLORS, build_marker,
)


def test_color_constants_one_per_leg():
    assert set(LEG_COLORS.keys()) == {"FL", "FR", "BL", "BR"}
    for name, (r, g, b) in LEG_COLORS.items():
        for c in (r, g, b):
            assert 0.0 <= c <= 1.0


def test_trail_init_rejects_nonpositive_max_points():
    with pytest.raises(ValueError):
        FootTrail(name="FL", color=(1, 0, 0), max_points=0)
    with pytest.raises(ValueError):
        FootTrail(name="FL", color=(1, 0, 0), max_points=-5)


def test_trail_append_within_capacity():
    t = FootTrail(name="FL", color=LEG_COLORS["FL"], max_points=5)
    for i in range(3):
        t.append([i, 0.0, 0.0])
    assert len(t) == 3
    pts = t.points()
    assert len(pts) == 3
    np.testing.assert_allclose(pts[0], [0, 0, 0])
    np.testing.assert_allclose(pts[2], [2, 0, 0])


def test_trail_rolls_at_max_points():
    t = FootTrail(name="FL", color=LEG_COLORS["FL"], max_points=4)
    for i in range(7):
        t.append([float(i), 0.0, 0.0])
    assert len(t) == 4
    pts = t.points()
    np.testing.assert_allclose(pts[0], [3, 0, 0])
    np.testing.assert_allclose(pts[-1], [6, 0, 0])


def test_clear_empties_buffer():
    t = FootTrail(name="FL", color=LEG_COLORS["FL"], max_points=10)
    for i in range(5):
        t.append([i, i, i])
    t.clear()
    assert len(t) == 0
    assert t.points() == []


def test_append_coerces_to_3vector():
    t = FootTrail(name="FL", color=LEG_COLORS["FL"], max_points=3)
    t.append((1.5, 2.5, 3.5))                 # tuple
    t.append(np.array([4.0, 5.0, 6.0]))       # ndarray
    pts = t.points()
    np.testing.assert_allclose(pts[0], [1.5, 2.5, 3.5])
    np.testing.assert_allclose(pts[1], [4.0, 5.0, 6.0])


def test_append_rejects_wrong_shape():
    t = FootTrail(name="FL", color=LEG_COLORS["FL"], max_points=3)
    with pytest.raises(ValueError):
        t.append([1.0, 2.0])              # wrong length
    with pytest.raises(ValueError):
        t.append([[1, 2, 3], [4, 5, 6]])  # 2D


# --- build_marker tests (require ROS msg packages) ---

vis_msgs = pytest.importorskip("visualization_msgs")
from visualization_msgs.msg import Marker   # noqa: E402


def _stub_stamp():
    """Builtins Time-like stub for tests not running rclpy."""
    from builtin_interfaces.msg import Time
    t = Time()
    t.sec = 0
    t.nanosec = 0
    return t


def test_build_marker_basic_fields():
    t = FootTrail(name="FL", color=LEG_COLORS["FL"], max_points=10)
    for i in range(3):
        t.append([0.1 * i, 0.0, -0.15])
    m = build_marker(t, frame_id="base_link", marker_id=7, stamp=_stub_stamp())
    assert m.header.frame_id == "base_link"
    assert m.ns == "foot_trail"
    assert m.id == 7
    assert m.type == Marker.LINE_STRIP
    assert m.action == Marker.ADD
    assert m.scale.x == pytest.approx(0.005)
    assert m.color.r == pytest.approx(1.0)
    assert m.color.g == pytest.approx(0.0)
    assert m.color.b == pytest.approx(0.0)
    assert m.color.a == pytest.approx(1.0)
    assert m.pose.orientation.w == pytest.approx(1.0)


def test_build_marker_points_match_buffer():
    t = FootTrail(name="FR", color=LEG_COLORS["FR"], max_points=10)
    raw_points = [
        [0.10, -0.05, -0.15],
        [0.12, -0.05, -0.14],
        [0.11, -0.04, -0.15],
    ]
    for p in raw_points:
        t.append(p)
    m = build_marker(t, frame_id="base_link", marker_id=1, stamp=_stub_stamp())
    assert len(m.points) == 3
    for got, want in zip(m.points, raw_points):
        assert got.x == pytest.approx(want[0])
        assert got.y == pytest.approx(want[1])
        assert got.z == pytest.approx(want[2])


def test_build_marker_with_empty_trail():
    t = FootTrail(name="BL", color=LEG_COLORS["BL"], max_points=10)
    m = build_marker(t, frame_id="base_link", marker_id=2, stamp=_stub_stamp())
    assert len(m.points) == 0
    # Even with no points, the marker should still have valid color & type.
    assert m.type == Marker.LINE_STRIP
    assert m.color.b == pytest.approx(1.0)
