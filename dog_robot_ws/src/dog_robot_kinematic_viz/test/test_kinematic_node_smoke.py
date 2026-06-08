"""Smoke test: KinematicNode publishes /joint_states with 12 joints and
reacts to /cmd_vel. Skipped when rclpy is unavailable.
"""
from pathlib import Path
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from geometry_msgs.msg import Twist           # noqa: E402
from rclpy.executors import SingleThreadedExecutor   # noqa: E402
from rclpy.parameter import Parameter         # noqa: E402
from sensor_msgs.msg import JointState        # noqa: E402

from dog_robot_kinematic_viz.kinematic_node import (
    KinematicNode, _all_joint_names,
)


CFG = Path(__file__).resolve().parents[2] / "dog_robot_description" / "config"
LINK_PARAMS_YAML = str(CFG / "link_params.yaml")
URDF_JOINTS_YAML = str(CFG / "urdf_joints.yaml")


def _overrides(**extra):
    base = [
        Parameter("link_params_yaml", value=LINK_PARAMS_YAML),
        Parameter("urdf_joints_yaml", value=URDF_JOINTS_YAML),
        Parameter("publish_rate", value=100.0),
    ]
    for k, v in extra.items():
        base.append(Parameter(k, value=v))
    return base


@pytest.fixture
def rclpy_ctx():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_all_joint_names_canonical_order():
    assert _all_joint_names() == [
        "FL_hip_roll", "FL_thigh_pitch", "FL_knee_pitch",
        "FR_hip_roll", "FR_thigh_pitch", "FR_knee_pitch",
        "BL_hip_roll", "BL_thigh_pitch", "BL_knee_pitch",
        "BR_hip_roll", "BR_thigh_pitch", "BR_knee_pitch",
    ]


def test_node_publishes_12_joints_and_reacts_to_cmd_vel(rclpy_ctx):
    node = KinematicNode(parameter_overrides=_overrides())

    listener = rclpy.create_node("smoke_listener")
    received: list[JointState] = []
    listener.create_subscription(
        JointState, "/joint_states", lambda m: received.append(m), 10)

    publisher = rclpy.create_node("smoke_publisher")
    pub = publisher.create_publisher(Twist, "/cmd_vel", 10)

    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(listener)
    ex.add_node(publisher)

    # Warm-up: collect a few messages at zero velocity.
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.4:
        ex.spin_once(timeout_sec=0.02)
    assert received, "no /joint_states received during warm-up"
    msg = received[-1]
    assert len(msg.name) == 12 and len(msg.position) == 12
    assert msg.name[0] == "FL_hip_roll"
    # Idle = (0,0,0) per leg, swing phases may lift the foot so some joints
    # are non-zero. But the *order of magnitude* should be small.
    assert max(abs(p) for p in msg.position) < 1.0

    # Publish forward velocity; expect joint positions to change visibly.
    snapshot_pre = list(received[-1].position)
    twist = Twist()
    twist.linear.x = 0.10
    pub.publish(twist)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.5:
        ex.spin_once(timeout_sec=0.02)
    snapshot_post = list(received[-1].position)
    delta = max(abs(a - b) for a, b in zip(snapshot_pre, snapshot_post))
    assert delta > 1e-3, "joints did not respond to /cmd_vel"

    node.destroy_node()
    listener.destroy_node()
    publisher.destroy_node()


def test_missing_yaml_paths_raise(rclpy_ctx):
    # Without yaml overrides the node should refuse to start.
    with pytest.raises(RuntimeError, match="link_params_yaml"):
        KinematicNode(parameter_overrides=[
            Parameter("link_params_yaml", value=""),
            Parameter("urdf_joints_yaml", value=""),
        ])


def test_idle_legs_publish_zero_joints(rclpy_ctx):
    # Single-leg mode: only FL drivers; FR/BL/BR should publish idle (0,0,0).
    node = KinematicNode(parameter_overrides=_overrides(
        active_legs=["FL"], idle_joints=[0.0, 0.0, 0.0]))

    listener = rclpy.create_node("idle_listener")
    received: list[JointState] = []
    listener.create_subscription(
        JointState, "/joint_states", lambda m: received.append(m), 10)

    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(listener)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.3:
        ex.spin_once(timeout_sec=0.02)
    assert received
    msg = received[-1]
    # FL = positions[0..3), idle for FR/BL/BR is exactly zero (no swing lift).
    for idx, name in enumerate(msg.name):
        if not name.startswith("FL_"):
            assert msg.position[idx] == 0.0, \
                f"{name} = {msg.position[idx]} (expected idle 0.0)"

    node.destroy_node()
    listener.destroy_node()


def test_linear_z_drives_body_height_state(rclpy_ctx):
    # Publishing linear.z > 0 must move feet DOWN in body Z relative to the
    # zero-input baseline (because body_z > 0 means body rises, feet press
    # further down).  We assert the joint snapshot diverges from baseline.
    # step_freq=0.0 freezes the gait clock so only body_z can move joints.
    node = KinematicNode(parameter_overrides=_overrides(step_freq=0.0))

    listener = rclpy.create_node("body_z_listener")
    received: list[JointState] = []
    listener.create_subscription(
        JointState, "/joint_states", lambda m: received.append(m), 10)

    publisher = rclpy.create_node("body_z_publisher")
    pub = publisher.create_publisher(Twist, "/cmd_vel", 10)

    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(listener)
    ex.add_node(publisher)

    # Baseline: spin at zero input, hold a stance-phase snapshot.
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.4:
        ex.spin_once(timeout_sec=0.02)
    assert received, "no /joint_states received during warm-up"
    snapshot_pre = list(received[-1].position)

    # Drive body up at 0.04 m/s for ~0.6 s -> body_z ~ +0.024 (below the
    # +0.03 default clamp). Republish each tick to defeat DDS pre-match drops.
    twist = Twist()
    twist.linear.z = 0.04
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.6:
        pub.publish(twist)
        ex.spin_once(timeout_sec=0.02)
    snapshot_post = list(received[-1].position)

    delta = max(abs(a - b) for a, b in zip(snapshot_pre, snapshot_post))
    assert delta > 1e-3, "joints did not respond to /cmd_vel.linear.z"

    node.destroy_node()
    listener.destroy_node()
    publisher.destroy_node()


def test_body_z_range_params_passed_to_commander(rclpy_ctx):
    node = KinematicNode(parameter_overrides=_overrides(
        body_z_min=-0.10, body_z_max=+0.10))
    assert node.commander.body_z_min == pytest.approx(-0.10)
    assert node.commander.body_z_max == pytest.approx(+0.10)
    node.destroy_node()
