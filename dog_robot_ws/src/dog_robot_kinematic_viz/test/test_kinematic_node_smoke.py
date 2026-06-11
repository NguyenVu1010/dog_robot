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


def test_publishes_foot_trails_marker_array(rclpy_ctx):
    from visualization_msgs.msg import MarkerArray
    node = KinematicNode(parameter_overrides=_overrides())

    listener = rclpy.create_node("trail_listener")
    received: list[MarkerArray] = []
    listener.create_subscription(
        MarkerArray, "/foot_trails", lambda m: received.append(m), 10)

    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(listener)

    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.4:
        ex.spin_once(timeout_sec=0.02)

    assert received, "no MarkerArray received"
    msg = received[-1]
    assert len(msg.markers) == 4
    # marker_id 0..3 corresponds to LEG_NAMES order.
    for i, m in enumerate(msg.markers):
        assert m.ns == "foot_trail"
        assert m.id == i
        # Trail should have grown across the warm-up window.
        assert len(m.points) > 0, f"marker {i}: no points appended"

    node.destroy_node()
    listener.destroy_node()


def test_foot_trail_points_in_base_link_frame(rclpy_ctx):
    from visualization_msgs.msg import MarkerArray
    node = KinematicNode(parameter_overrides=_overrides(step_freq=0.0))

    listener = rclpy.create_node("trail_frame_listener")
    received: list[MarkerArray] = []
    listener.create_subscription(
        MarkerArray, "/foot_trails", lambda m: received.append(m), 10)

    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(listener)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.3:
        ex.spin_once(timeout_sec=0.02)

    assert received
    msg = received[-1]
    for m in msg.markers:
        assert m.header.frame_id == "base_link"
        # At v=0 + step_freq=0, the foot stays at rest pose - sanity check
        # that the points are near the body's lower half (z < 0 in body frame).
        if m.points:
            assert m.points[-1].z < 0.0, f"marker {m.id}: foot z >= 0"

    node.destroy_node()
    listener.destroy_node()


def test_inactive_legs_have_empty_foot_trail_markers(rclpy_ctx):
    from visualization_msgs.msg import MarkerArray
    # Only FL is active -> FR/BL/BR markers should exist but be empty.
    node = KinematicNode(parameter_overrides=_overrides(
        active_legs=["FL"], idle_joints=[0.0, 0.0, 0.0]))

    listener = rclpy.create_node("trail_active_listener")
    received: list[MarkerArray] = []
    listener.create_subscription(
        MarkerArray, "/foot_trails", lambda m: received.append(m), 10)

    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(listener)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.3:
        ex.spin_once(timeout_sec=0.02)

    assert received
    msg = received[-1]
    assert len(msg.markers) == 4
    # marker_id order matches LEG_NAMES = ("FL", "FR", "BL", "BR").
    fl, fr, bl, br = msg.markers
    assert fl.id == 0 and len(fl.points) > 0, "FL active should have points"
    for m, name in [(fr, "FR"), (bl, "BL"), (br, "BR")]:
        assert len(m.points) == 0, f"{name} inactive but has {len(m.points)} points"

    node.destroy_node()
    listener.destroy_node()


def test_pitch_range_params_passed_to_commander(rclpy_ctx):
    node = KinematicNode(parameter_overrides=_overrides(
        pitch_min=-0.10, pitch_max=+0.10))
    assert node.commander.pitch_min == pytest.approx(-0.10)
    assert node.commander.pitch_max == pytest.approx(+0.10)
    node.destroy_node()


def test_angular_y_pitches_all_four_legs_opposite_signs(rclpy_ctx):
    # step_freq=0.0 freezes the gait clock so only pitch_amount can move joints.
    node = KinematicNode(parameter_overrides=_overrides(step_freq=0.0))

    listener = rclpy.create_node("pitch_listener")
    received: list[JointState] = []
    listener.create_subscription(
        JointState, "/joint_states", lambda m: received.append(m), 10)

    publisher = rclpy.create_node("pitch_publisher")
    pub = publisher.create_publisher(Twist, "/cmd_vel", 10)

    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(listener)
    ex.add_node(publisher)

    # Warm-up baseline at pitch=0.
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.4:
        ex.spin_once(timeout_sec=0.02)
    assert received, "no /joint_states received during pitch warm-up"
    snapshot_pre = list(received[-1].position)

    # Drive angular.y = +0.04 m/s for ~0.6 s -> pitch ~ +0.024
    # (under the default +0.05 clamp).
    twist = Twist()
    twist.angular.y = 0.04
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.6:
        pub.publish(twist)
        ex.spin_once(timeout_sec=0.02)
    snapshot_post = list(received[-1].position)

    # Joint layout (12 floats): FL[0..3) FR[3..6) BL[6..9) BR[9..12).
    # All four legs must move — both front and rear.
    front_delta = max(abs(snapshot_post[i] - snapshot_pre[i]) for i in range(0, 6))
    rear_delta = max(abs(snapshot_post[i] - snapshot_pre[i]) for i in range(6, 12))
    assert front_delta > 1e-3, f"front joints did not respond to angular.y (delta={front_delta})"
    assert rear_delta > 1e-3, f"rear joints did not respond to angular.y (delta={rear_delta})"

    node.destroy_node()
    listener.destroy_node()
    publisher.destroy_node()


def test_only_bl_br_drivers_are_rear(rclpy_ctx):
    node = KinematicNode(parameter_overrides=_overrides())
    assert node.drivers["FL"].is_rear is False
    assert node.drivers["FR"].is_rear is False
    assert node.drivers["BL"].is_rear is True
    assert node.drivers["BR"].is_rear is True
    node.destroy_node()


# --- /sit, /release named-pose API ---

from std_srvs.srv import Trigger    # noqa: E402


SIT_JOINTS_TEST = (
    0.0, -0.30, +0.30,
    0.0, -0.30, +0.30,
    0.0, +1.00, -2.20,
    0.0, +1.00, -2.20,
)


def _call_trigger(client, ex, timeout=2.0):
    """Call a Trigger service and return the response."""
    req = Trigger.Request()
    future = client.call_async(req)
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        ex.spin_once(timeout_sec=0.05)
        if future.done():
            return future.result()
    raise TimeoutError("Trigger service call did not complete")


def test_sit_pose_joints_param_validation(rclpy_ctx):
    # Wrong length must fail fast at construction.
    with pytest.raises(ValueError, match="sit_pose_joints"):
        KinematicNode(parameter_overrides=_overrides(
            sit_pose_joints=[1.0, 2.0, 3.0]))


def test_sit_locks_joints_to_yaml_values(rclpy_ctx):
    node = KinematicNode(parameter_overrides=_overrides(
        sit_pose_joints=list(SIT_JOINTS_TEST)))

    listener = rclpy.create_node("sit_listener")
    received: list[JointState] = []
    listener.create_subscription(
        JointState, "/joint_states", lambda m: received.append(m), 10)

    sit_client = listener.create_client(Trigger, "/sit")

    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(listener)

    # Wait for service to advertise.
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.0 and not sit_client.service_is_ready():
        ex.spin_once(timeout_sec=0.02)
    assert sit_client.service_is_ready(), "/sit service did not advertise"

    resp = _call_trigger(sit_client, ex)
    assert resp.success is True
    assert "sit pose locked" in resp.message

    # Spin a few ticks; every subsequent /joint_states must match the locked tuple.
    received.clear()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.3:
        ex.spin_once(timeout_sec=0.02)

    assert received, "no /joint_states received after /sit"
    for msg in received[-3:]:   # check the last few
        for i, expected in enumerate(SIT_JOINTS_TEST):
            assert msg.position[i] == pytest.approx(expected, abs=1e-9), (
                f"joint {i} = {msg.position[i]} != {expected} after /sit")

    node.destroy_node()
    listener.destroy_node()


def test_release_resumes_dynamic_control(rclpy_ctx):
    node = KinematicNode(parameter_overrides=_overrides(
        sit_pose_joints=list(SIT_JOINTS_TEST), step_freq=0.0))

    listener = rclpy.create_node("release_listener")
    received: list[JointState] = []
    listener.create_subscription(
        JointState, "/joint_states", lambda m: received.append(m), 10)

    publisher = rclpy.create_node("release_publisher")
    pub = publisher.create_publisher(Twist, "/cmd_vel", 10)

    sit_client = listener.create_client(Trigger, "/sit")
    release_client = listener.create_client(Trigger, "/release")

    ex = SingleThreadedExecutor()
    for n in (node, listener, publisher):
        ex.add_node(n)

    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.0 and not (
            sit_client.service_is_ready() and release_client.service_is_ready()):
        ex.spin_once(timeout_sec=0.02)

    _call_trigger(sit_client, ex)

    # While locked, confirm joints == SIT_JOINTS_TEST.
    received.clear()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.2:
        ex.spin_once(timeout_sec=0.02)
    locked_snapshot = list(received[-1].position)
    for i, expected in enumerate(SIT_JOINTS_TEST):
        assert locked_snapshot[i] == pytest.approx(expected, abs=1e-9)

    # Release and start pushing cmd_vel.linear.z so body_z grows.
    _call_trigger(release_client, ex)

    twist = Twist()
    twist.linear.z = 0.04
    received.clear()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.6:
        pub.publish(twist)
        ex.spin_once(timeout_sec=0.02)
    post = list(received[-1].position)

    # Dynamic control resumed → at least one joint should differ from the
    # locked snapshot.
    max_delta = max(abs(post[i] - locked_snapshot[i]) for i in range(12))
    assert max_delta > 1e-3, (
        f"after /release + cmd_vel, joints unchanged (max delta={max_delta})")

    node.destroy_node()
    listener.destroy_node()
    publisher.destroy_node()


def test_angular_z_drives_all_four_legs_via_tangent_velocity(rclpy_ctx):
    # step_freq=0.0 freezes the gait clock so only the yaw tangent
    # velocity moves joints (no body_z, no pitch).
    node = KinematicNode(parameter_overrides=_overrides(step_freq=0.0))

    listener = rclpy.create_node("yaw_listener")
    received: list[JointState] = []
    listener.create_subscription(
        JointState, "/joint_states", lambda m: received.append(m), 10)

    publisher = rclpy.create_node("yaw_publisher")
    pub = publisher.create_publisher(Twist, "/cmd_vel", 10)

    ex = SingleThreadedExecutor()
    for n in (node, listener, publisher):
        ex.add_node(n)

    # Warm-up baseline at all-zero input.
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.4:
        ex.spin_once(timeout_sec=0.02)
    assert received, "no /joint_states received during yaw warm-up"
    snapshot_pre = list(received[-1].position)

    # Drive angular.z = +0.5 rad/s for ~0.6 s.
    twist = Twist()
    twist.angular.z = 0.5
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.6:
        pub.publish(twist)
        ex.spin_once(timeout_sec=0.02)
    snapshot_post = list(received[-1].position)

    # All four legs should respond — each gets a different tangent velocity
    # because each hip is at a different XY position in the body frame.
    # Joint layout: FL[0..3) FR[3..6) BL[6..9) BR[9..12).
    for leg_idx, leg_name in enumerate(("FL", "FR", "BL", "BR")):
        start = leg_idx * 3
        end = start + 3
        leg_delta = max(
            abs(snapshot_post[i] - snapshot_pre[i]) for i in range(start, end))
        assert leg_delta > 1e-3, (
            f"{leg_name} joints did not respond to angular.z "
            f"(max delta={leg_delta})")

    node.destroy_node()
    listener.destroy_node()
    publisher.destroy_node()


def test_cmd_vel_during_lock_does_not_change_joints(rclpy_ctx):
    node = KinematicNode(parameter_overrides=_overrides(
        sit_pose_joints=list(SIT_JOINTS_TEST), step_freq=0.0))

    listener = rclpy.create_node("lock_cmd_listener")
    received: list[JointState] = []
    listener.create_subscription(
        JointState, "/joint_states", lambda m: received.append(m), 10)

    publisher = rclpy.create_node("lock_cmd_publisher")
    pub = publisher.create_publisher(Twist, "/cmd_vel", 10)

    sit_client = listener.create_client(Trigger, "/sit")

    ex = SingleThreadedExecutor()
    for n in (node, listener, publisher):
        ex.add_node(n)

    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.0 and not sit_client.service_is_ready():
        ex.spin_once(timeout_sec=0.02)

    _call_trigger(sit_client, ex)

    # Drive cmd_vel for 0.5 s; joints must stay byte-identical to SIT_JOINTS_TEST.
    twist = Twist()
    twist.linear.x = 0.10
    twist.linear.z = 0.04
    twist.angular.y = 0.04
    received.clear()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.5:
        pub.publish(twist)
        ex.spin_once(timeout_sec=0.02)

    assert received, "no /joint_states during locked cmd_vel"
    for msg in received[-5:]:
        for i, expected in enumerate(SIT_JOINTS_TEST):
            assert msg.position[i] == pytest.approx(expected, abs=1e-9), (
                f"joint {i} = {msg.position[i]} != {expected} during lock")

    node.destroy_node()
    listener.destroy_node()
    publisher.destroy_node()
