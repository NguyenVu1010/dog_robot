"""DEPRECATED — superseded by walker_controller (2026-05-24).

walker_controller does everything this node does (stand pose at default
height) plus walking via /cmd_vel. Kept here for backward compat with
stand.launch.py; will be deleted in a future cleanup.
"""
import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from dog_robot_control.kinematics_dh import DHParams, ik_leg
from dog_robot_control.leg_config import LEGS


def Rx(a): c,s=math.cos(a),math.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def Ry(a): c,s=math.cos(a),math.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def Rz(a): c,s=math.cos(a),math.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])


def body_to_hip(point_body, leg):
    r, p, y = leg.base_to_hip_rpy
    R_bh = Rz(y) @ Ry(p) @ Rx(r)
    t_bh = np.array(leg.base_to_hip_xyz)
    return R_bh.T @ (np.asarray(point_body) - t_bh)


class StandController(Node):
    def __init__(self):
        super().__init__("stand_controller")
        self.declare_parameter("dh.L_hh", 0.02553)
        self.declare_parameter("dh.L_th", 0.11725)
        self.declare_parameter("dh.L_sh", 0.07043)
        self.declare_parameter("stand.default_height", 0.18)
        self.declare_parameter("stand.ramp_time", 2.0)
        self.declare_parameter("stand.publish_rate", 50.0)
        self.declare_parameter("stand.knee_direction", 1)
        self.declare_parameter("joint_order", [
            "FL_hip_yaw","FL_thigh_pitch","FL_knee_pitch",
            "FR_hip_yaw","FR_thigh_pitch","FR_knee_pitch",
            "BL_hip_yaw","BL_thigh_pitch","BL_knee_pitch",
            "BR_hip_yaw","BR_thigh_pitch","BR_knee_pitch",
        ])

        self.dh = DHParams(
            L_hh=self.get_parameter("dh.L_hh").value,
            L_th=self.get_parameter("dh.L_th").value,
            L_sh=self.get_parameter("dh.L_sh").value,
        )
        self.height = float(self.get_parameter("stand.default_height").value)
        self.ramp_time = float(self.get_parameter("stand.ramp_time").value)
        self.knee_dir = int(self.get_parameter("stand.knee_direction").value)
        rate = float(self.get_parameter("stand.publish_rate").value)
        self.joint_order = list(self.get_parameter("joint_order").value)

        self.start_angles: Optional[np.ndarray] = None
        self.target_angles: Optional[np.ndarray] = None
        self.ramp_start_t: Optional[float] = None

        self.pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )
        self.sub_js = self.create_subscription(
            JointState, "/joint_states", self._on_js, 10
        )
        self.sub_cmd = self.create_subscription(
            Pose, "/stand_cmd", self._on_cmd, 10
        )
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info("stand_controller up; waiting for /joint_states")

    def _on_js(self, msg: JointState):
        if self.start_angles is not None:
            return
        idx = {n: i for i, n in enumerate(msg.name)}
        try:
            angles = np.array([msg.position[idx[j]] for j in self.joint_order])
        except KeyError as e:
            self.get_logger().warn(f"joint_states missing {e}; will retry")
            return
        self.start_angles = angles
        self._recompute_target()
        self.ramp_start_t = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info("captured start angles; ramping to stand")

    def _on_cmd(self, msg: Pose):
        new_h = float(msg.position.z)
        if not (0.05 < new_h < 0.30):
            self.get_logger().warn(f"ignored stand_cmd height={new_h} (out of bounds)")
            return
        self.height = new_h
        if self.start_angles is None:
            return
        self.start_angles = self._current_command() if self.target_angles is not None else self.start_angles
        self._recompute_target()
        self.ramp_start_t = self.get_clock().now().nanoseconds * 1e-9

    def _recompute_target(self):
        targets = []
        for L in LEGS:
            foot_world = np.array([L.base_to_hip_xyz[0], L.base_to_hip_xyz[1], 0.0])
            foot_body = foot_world - np.array([0.0, 0.0, self.height])
            foot_hip = body_to_hip(foot_body, L)
            try:
                q = ik_leg(self.dh, foot_hip, knee_direction=self.knee_dir)
            except ValueError as e:
                self.get_logger().error(f"IK failed for {L.name}: {e}; height={self.height}")
                return
            targets.extend(q)
        self.target_angles = np.array(targets)

    def _current_command(self) -> np.ndarray:
        if self.target_angles is None or self.start_angles is None or self.ramp_start_t is None:
            return self.start_angles.copy() if self.start_angles is not None else np.zeros(12)
        t = self.get_clock().now().nanoseconds * 1e-9 - self.ramp_start_t
        alpha = float(np.clip(t / self.ramp_time, 0.0, 1.0))
        return (1.0 - alpha) * self.start_angles + alpha * self.target_angles

    def _tick(self):
        if self.start_angles is None or self.target_angles is None:
            return
        q = self._current_command()
        msg = JointTrajectory()
        msg.joint_names = self.joint_order
        pt = JointTrajectoryPoint()
        pt.positions = q.tolist()
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = int(0.1 * 1e9)
        msg.points = [pt]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StandController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
