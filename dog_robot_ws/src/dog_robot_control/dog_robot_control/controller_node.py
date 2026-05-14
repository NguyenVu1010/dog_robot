"""Thin ROS2 wrapper around dog_gait.controller.GaitController."""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from rclpy.duration import Duration

from dog_gait.controller import GaitController
from dog_gait.state_machine import State


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")

        # Parameters
        self.declare_parameter("tick_rate", 50.0)
        self.declare_parameter("joint_names", [
            "FL_hip_yaw", "FL_thigh_pitch", "FL_knee_pitch",
            "FR_hip_yaw", "FR_thigh_pitch", "FR_knee_pitch",
            "BL_hip_yaw", "BL_thigh_pitch", "BL_knee_pitch",
            "BR_hip_yaw", "BR_thigh_pitch", "BR_knee_pitch",
        ])
        self.declare_parameter("gait.cycle_time", 0.4)
        self.declare_parameter("gait.duty_factor", 0.5)
        self.declare_parameter("gait.step_height", 0.05)
        self.declare_parameter("gait.max_stride", 0.10)
        self.declare_parameter("cmd_vel.lowpass_alpha", 0.2)

        tick_rate = self.get_parameter("tick_rate").value
        self.joint_names = list(self.get_parameter("joint_names").value)

        # Gait controller
        self.ctrl = GaitController(
            cycle_time=self.get_parameter("gait.cycle_time").value,
            duty_factor=self.get_parameter("gait.duty_factor").value,
            step_height=self.get_parameter("gait.step_height").value,
            max_stride=self.get_parameter("gait.max_stride").value,
        )
        self.alpha = self.get_parameter("cmd_vel.lowpass_alpha").value

        # State
        self.cmd_vel = (0.0, 0.0, 0.0)        # filtered (vx, vy, vyaw)
        self.cmd_vel_raw = (0.0, 0.0, 0.0)    # latest received
        self.body_pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.last_tick = self.get_clock().now()

        # Subscriptions
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Pose, "/body_pose_setpoint", self._on_body_pose, 10)

        # Publications
        self.pub_traj = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)
        self.pub_state = self.create_publisher(String, "/gait_state", 10)

        # Services
        self.create_service(SetBool, "/enable", self._on_enable)
        self.create_service(Trigger, "/reset_gait", self._on_reset)

        # Timer
        self.timer = self.create_timer(1.0 / tick_rate, self._tick)
        self.get_logger().info("Controller node started")

    def _on_cmd_vel(self, msg):
        self.cmd_vel_raw = (msg.linear.x, msg.linear.y, msg.angular.z)

    def _on_body_pose(self, msg):
        q = msg.orientation
        # Convert quaternion to rpy
        siny_cosp = 2*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z)
        psi = math.atan2(siny_cosp, cosy_cosp)
        sinp = 2*(q.w*q.y - q.z*q.x)
        phi = math.asin(max(-1, min(1, sinp)))
        sinr_cosp = 2*(q.w*q.x + q.y*q.z)
        cosr_cosp = 1 - 2*(q.x*q.x + q.y*q.y)
        omega = math.atan2(sinr_cosp, cosr_cosp)
        self.body_pose = (omega, phi, psi, msg.position.x, msg.position.y, msg.position.z)

    def _on_enable(self, req, resp):
        if req.data:
            self.ctrl.enable()
            resp.message = "Enabled (STAND)"
        else:
            self.ctrl.disable()
            resp.message = "Disabled (OFF)"
        resp.success = True
        return resp

    def _on_reset(self, req, resp):
        self.ctrl.phase = 0.0
        resp.success = True
        resp.message = "Phase reset"
        return resp

    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_tick).nanoseconds * 1e-9
        self.last_tick = now
        if dt <= 0 or dt > 1.0:
            dt = 0.02

        # Low-pass filter cmd_vel
        a = self.alpha
        self.cmd_vel = tuple(a*r + (1-a)*f for r, f in zip(self.cmd_vel_raw, self.cmd_vel))

        angles = self.ctrl.tick(self.cmd_vel, self.body_pose, dt)

        # Publish gait state
        sm_state = self.ctrl.sm.state
        self.pub_state.publish(String(data=sm_state.value))

        if angles is None:
            return

        # Publish joint trajectory
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = [angles[name] for name in self.joint_names]
        pt.time_from_start = Duration(seconds=0.1).to_msg()
        traj.points.append(pt)
        self.pub_traj.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
