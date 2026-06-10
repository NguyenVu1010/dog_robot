"""TeleopKeyboard key handling: r/f drive vz, i/k drive angular.y,
space zeros all 5 axes, /cmd_vel publishes linear.z + angular.y.
Skipped when rclpy is unavailable.
"""
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from geometry_msgs.msg import Twist           # noqa: E402

from dog_robot_kinematic_viz.teleop_keyboard import (   # noqa: E402
    TeleopKeyboard, LIN_STEP, LIN_MAX,
)


@pytest.fixture
def rclpy_ctx():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_r_key_increments_vz(rclpy_ctx):
    node = TeleopKeyboard()
    assert node._vz == 0.0
    assert node.on_key("r") is True
    assert node._vz == pytest.approx(LIN_STEP)
    node.destroy_node()


def test_f_key_decrements_vz(rclpy_ctx):
    node = TeleopKeyboard()
    assert node.on_key("f") is True
    assert node._vz == pytest.approx(-LIN_STEP)
    node.destroy_node()


def test_vz_clamps_to_lin_max(rclpy_ctx):
    node = TeleopKeyboard()
    # Press r enough times to exceed the +LIN_MAX clamp.
    for _ in range(int(LIN_MAX / LIN_STEP) + 5):
        node.on_key("r")
    assert node._vz == pytest.approx(LIN_MAX)
    node.destroy_node()


def test_vz_clamps_to_neg_lin_max(rclpy_ctx):
    node = TeleopKeyboard()
    for _ in range(int(LIN_MAX / LIN_STEP) + 5):
        node.on_key("f")
    assert node._vz == pytest.approx(-LIN_MAX)
    node.destroy_node()


def test_space_zeros_all_five_axes(rclpy_ctx):
    node = TeleopKeyboard()
    node.on_key("w")   # vx > 0
    node.on_key("a")   # vy > 0
    node.on_key("r")   # vz > 0
    node.on_key("i")   # wy > 0   (NEW)
    node.on_key("j")   # wz > 0
    assert node._vx != 0.0
    assert node._vy != 0.0
    assert node._vz != 0.0
    assert node._wy != 0.0
    assert node._wz != 0.0
    node.on_key(" ")
    assert node._vx == 0.0
    assert node._vy == 0.0
    assert node._vz == 0.0
    assert node._wy == 0.0
    assert node._wz == 0.0
    node.destroy_node()


def test_publish_emits_linear_z(rclpy_ctx):
    node = TeleopKeyboard()
    received: list[Twist] = []

    listener = rclpy.create_node("teleop_listener")
    listener.create_subscription(
        Twist, "/cmd_vel", lambda m: received.append(m), 10)

    # Warm up DDS so the subscription is matched before publish.
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.1:
        rclpy.spin_once(node, timeout_sec=0.01)
        rclpy.spin_once(listener, timeout_sec=0.01)

    node.on_key("r")   # vz = +LIN_STEP, also calls publish()

    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.3 and not received:
        rclpy.spin_once(listener, timeout_sec=0.02)
    assert received, "listener did not receive /cmd_vel"
    assert received[-1].linear.z == pytest.approx(LIN_STEP)

    listener.destroy_node()
    node.destroy_node()


def test_q_key_returns_false(rclpy_ctx):
    node = TeleopKeyboard()
    assert node.on_key("q") is False
    node.destroy_node()


# --- i/k -> angular.y (rear-height velocity) ---

def test_i_key_increments_wy(rclpy_ctx):
    node = TeleopKeyboard()
    assert node._wy == 0.0
    assert node.on_key("i") is True
    assert node._wy == pytest.approx(LIN_STEP)
    node.destroy_node()


def test_k_key_decrements_wy(rclpy_ctx):
    node = TeleopKeyboard()
    assert node.on_key("k") is True
    assert node._wy == pytest.approx(-LIN_STEP)
    node.destroy_node()


def test_wy_clamps_to_lin_max(rclpy_ctx):
    node = TeleopKeyboard()
    for _ in range(int(LIN_MAX / LIN_STEP) + 5):
        node.on_key("i")
    assert node._wy == pytest.approx(LIN_MAX)
    node.destroy_node()


def test_wy_clamps_to_neg_lin_max(rclpy_ctx):
    node = TeleopKeyboard()
    for _ in range(int(LIN_MAX / LIN_STEP) + 5):
        node.on_key("k")
    assert node._wy == pytest.approx(-LIN_MAX)
    node.destroy_node()


def test_publish_emits_angular_y(rclpy_ctx):
    node = TeleopKeyboard()
    received: list[Twist] = []

    listener = rclpy.create_node("teleop_wy_listener")
    listener.create_subscription(
        Twist, "/cmd_vel", lambda m: received.append(m), 10)

    # Warm up DDS so the subscription is matched before publish.
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.1:
        rclpy.spin_once(node, timeout_sec=0.01)
        rclpy.spin_once(listener, timeout_sec=0.01)

    node.on_key("i")   # _wy = +LIN_STEP, also calls publish()

    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.3 and not received:
        rclpy.spin_once(listener, timeout_sec=0.02)
    assert received, "listener did not receive /cmd_vel"
    assert received[-1].angular.y == pytest.approx(LIN_STEP)

    listener.destroy_node()
    node.destroy_node()
