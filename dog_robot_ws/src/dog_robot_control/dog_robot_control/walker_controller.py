"""Walker controller node.

Subscribes /cmd_vel + /stand_cmd, ticks gait pipeline at publish_rate, calls
DH-IK, publishes JointTrajectory. Subsumes stand_controller: cmd_vel = 0 ->
robot holds stand pose.
"""
import math
from typing import List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from dog_robot_control.kinematics_dh import DHParams, ik_leg
from dog_robot_control.leg_config import LEGS
from dog_robot_control.gait.gait_config import GaitConfig
from dog_robot_control.gait.body_controller import BodyController, BodyPose
from dog_robot_control.gait.leg_controller import LegController, Velocity


def _Rx(a): c, s = math.cos(a), math.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def _Ry(a): c, s = math.cos(a), math.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def _Rz(a): c, s = math.cos(a), math.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])


def _body_to_hip(foot_body_at_hip, leg):
    r, p, y = leg.base_to_hip_rpy
    R_bh = _Rz(y) @ _Ry(p) @ _Rx(r)
    return R_bh.T @ foot_body_at_hip


DEFAULT_JOINT_ORDER = [
    "FL_hip_yaw","FL_thigh_pitch","FL_knee_pitch",
    "FR_hip_yaw","FR_thigh_pitch","FR_knee_pitch",
    "BL_hip_yaw","BL_thigh_pitch","BL_knee_pitch",
    "BR_hip_yaw","BR_thigh_pitch","BR_knee_pitch",
]


class WalkerController(Node):
    def __init__(self):
        super().__init__("walker_controller")
        self.declare_parameter("dh.L_hh", 0.02553)
        self.declare_parameter("dh.L_th", 0.11725)
        self.declare_parameter("dh.L_sh", 0.07043)
        self.declare_parameter("gait.nominal_height", 0.15)
        self.declare_parameter("gait.stance_duration", 0.30)
        self.declare_parameter("gait.swing_height", 0.03)
        self.declare_parameter("gait.stance_depth", 0.001)
        self.declare_parameter("gait.max_linear_velocity_x", 0.15)
        self.declare_parameter("gait.max_linear_velocity_y", 0.08)
        self.declare_parameter("gait.max_angular_velocity_z", 0.50)
        self.declare_parameter("stand.ramp_time", 2.0)
        self.declare_parameter("stand.cmd_vel_timeout", 0.5)
        self.declare_parameter("stand.publish_rate", 50.0)
        self.declare_parameter("stand.knee_direction", 1)
        self.declare_parameter("joint_order", DEFAULT_JOINT_ORDER)

        dh = DHParams(
            L_hh=self.get_parameter("dh.L_hh").value,
            L_th=self.get_parameter("dh.L_th").value,
            L_sh=self.get_parameter("dh.L_sh").value,
        )
        self.gait = GaitConfig(
            nominal_height=self.get_parameter("gait.nominal_height").value,
            stance_duration=self.get_parameter("gait.stance_duration").value,
            swing_height=self.get_parameter("gait.swing_height").value,
            stance_depth=self.get_parameter("gait.stance_depth").value,
            max_linear_velocity_x=self.get_parameter("gait.max_linear_velocity_x").value,
            max_linear_velocity_y=self.get_parameter("gait.max_linear_velocity_y").value,
            max_angular_velocity_z=self.get_parameter("gait.max_angular_velocity_z").value,
        )
        self.dh = dh
        self.knee_dir = int(self.get_parameter("stand.knee_direction").value)
        self.ramp_time = float(self.get_parameter("stand.ramp_time").value)
        self.cmd_timeout = float(self.get_parameter("stand.cmd_vel_timeout").value)
        rate = float(self.get_parameter("stand.publish_rate").value)
        self.joint_order = list(self.get_parameter("joint_order").value)

        self.body_controller = BodyController(LEGS, dh, self.gait)
        self.leg_controller = LegController(LEGS, dh, self.gait)

        self.req_vel = Velocity(0.0, 0.0, 0.0)
        self.req_pose = BodyPose(0, 0, self.gait.nominal_height, 0, 0, 0)
        self.last_cmd_vel_t: Optional[float] = None

        self.start_angles: Optional[np.ndarray] = None
        self.ramp_target: Optional[np.ndarray] = None
        self.ramp_start_t: Optional[float] = None
        self.ramp_done = False

        self.pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )
        self.sub_js = self.create_subscription(
            JointState, "/joint_states", self._on_js, 10
        )
        self.sub_vel = self.create_subscription(
            Twist, "/cmd_vel", self._on_vel, 10
        )
        self.sub_pose = self.create_subscription(
            Pose, "/stand_cmd", self._on_pose, 10
        )
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info("walker_controller up; waiting for /joint_states")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_js(self, msg: JointState):
        if self.start_angles is not None:
            return
        idx = {n: i for i, n in enumerate(msg.name)}
        try:
            self.start_angles = np.array([msg.position[idx[j]] for j in self.joint_order])
        except KeyError as e:
            self.get_logger().warn(f"joint_states missing {e}; will retry")
            return
        self.ramp_target = self._compute_stand_target()
        self.ramp_start_t = self._now()
        self.get_logger().info("captured start angles; ramping to stand")

    def _on_vel(self, msg: Twist):
        self.req_vel = Velocity(msg.linear.x, msg.linear.y, msg.angular.z)
        self.last_cmd_vel_t = self._now()

    def _on_pose(self, msg: Pose):
        new_h = float(msg.position.z)
        if not (0.05 < new_h < 0.30):
            self.get_logger().warn(f"ignored stand_cmd height={new_h}")
            return
        self.req_pose = BodyPose(self.req_pose.x, self.req_pose.y, new_h,
                                 self.req_pose.roll, self.req_pose.pitch,
                                 self.req_pose.yaw)

    def _compute_stand_target(self) -> np.ndarray:
        feet = self.body_controller.pose_command(self.req_pose)
        targets: List[float] = []
        for i, L in enumerate(LEGS):
            foot_h = _body_to_hip(feet[i], L)
            try:
                q = ik_leg(self.dh, foot_h, knee_direction=self.knee_dir)
            except ValueError as e:
                self.get_logger().error(f"IK failed for stand on {L.name}: {e}")
                return self.start_angles.copy() if self.start_angles is not None else np.zeros(12)
            targets.extend(q)
        return np.array(targets)

    def _tick(self):
        if self.start_angles is None or self.ramp_target is None:
            return

        t = self._now()

        if self.last_cmd_vel_t is not None and (t - self.last_cmd_vel_t) > self.cmd_timeout:
            self.req_vel = Velocity(0.0, 0.0, 0.0)

        if not self.ramp_done:
            elapsed = t - self.ramp_start_t
            if elapsed >= self.ramp_time:
                self.ramp_done = True
                q = self.ramp_target
            else:
                alpha = elapsed / self.ramp_time
                q = (1.0 - alpha) * self.start_angles + alpha * self.ramp_target
        else:
            feet = self.body_controller.pose_command(self.req_pose)
            feet = self.leg_controller.velocity_command(feet, self.req_vel, t)
            targets: List[float] = []
            for i, L in enumerate(LEGS):
                foot_h = _body_to_hip(feet[i], L)
                try:
                    angles = ik_leg(self.dh, foot_h, knee_direction=self.knee_dir)
                except ValueError as e:
                    self.get_logger().warn(
                        f"IK fail for {L.name}: {e}", throttle_duration_sec=1.0)
                    return
                targets.extend(angles)
            q = np.array(targets)

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
    node = WalkerController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
