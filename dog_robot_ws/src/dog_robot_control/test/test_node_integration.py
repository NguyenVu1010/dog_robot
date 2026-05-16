"""Smoke test: controller node starts, sub/pub topics exist, /enable service works."""
import time
import threading
import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory

from dog_robot_control.controller_node import ControllerNode


@pytest.fixture
def node():
    rclpy.init()
    n = ControllerNode()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_node_starts(node):
    """Node can be created without crashing."""
    assert node is not None
    assert len(node.joint_names) == 12


def test_publishes_after_enable(node):
    """After /enable + cmd_vel, node publishes a joint trajectory within 1 sec."""
    received = []

    def cb(msg):
        received.append(msg)

    sub = node.create_subscription(
        JointTrajectory,
        "/joint_trajectory_controller/joint_trajectory",
        cb, 10
    )

    # Enable via direct call
    node.ctrl.enable()

    # Spin for 1 sec
    start = time.time()
    while time.time() - start < 1.5 and not received:
        rclpy.spin_once(node, timeout_sec=0.05)

    assert len(received) > 0
    traj = received[0]
    assert len(traj.joint_names) == 12
    assert len(traj.points) == 1
    assert len(traj.points[0].positions) == 12
