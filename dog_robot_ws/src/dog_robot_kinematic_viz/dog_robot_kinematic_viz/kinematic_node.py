"""ROS 2 node: /cmd_vel -> BodyCommander -> LegDriver(x N) -> /joint_states.

The static world->base_link TF is published by tf2_ros.static_transform_publisher
from the launch file, not by this node. Inactive legs (those not in
`active_legs`) publish the `idle_joints` triple every tick so RViz still sees
all 12 joints.
"""
from __future__ import annotations
import time
from typing import Dict, List

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState

from dog_robot_kinematics.kinematics_link import load_link_params, fk_leg
from visualization_msgs.msg import MarkerArray

from dog_robot_kinematic_viz.body_commander import BodyCommander
from dog_robot_kinematic_viz.foot_target import FootTargetParams
from dog_robot_kinematic_viz.foot_trail import FootTrail, LEG_COLORS, build_marker
from dog_robot_kinematic_viz.leg_driver import LegDriver
from dog_robot_kinematic_viz.leg_geometry import LEG_NAMES, load_leg_geoms


JOINT_SUFFIXES = ("hip_roll", "thigh_pitch", "knee_pitch")


def _all_joint_names() -> List[str]:
    return [f"{leg}_{s}" for leg in LEG_NAMES for s in JOINT_SUFFIXES]


class KinematicNode(Node):

    def __init__(self, parameter_overrides=None):
        if parameter_overrides is None:
            super().__init__("kinematic_node")
        else:
            super().__init__("kinematic_node",
                             parameter_overrides=parameter_overrides)

        # ----- parameters -----
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("active_legs", list(LEG_NAMES))
        self.declare_parameter("idle_joints", [0.0, 0.0, 0.0])
        self.declare_parameter("link_params_yaml", "")
        self.declare_parameter("urdf_joints_yaml", "")
        self.declare_parameter("step_freq", 1.5)
        self.declare_parameter("stride_per_mps", 0.20)
        self.declare_parameter("swing_height", 0.03)
        self.declare_parameter("stance_phase_ratio", 0.5)
        self.declare_parameter("swing_activation_speed", 0.05)
        self.declare_parameter("body_z_min", -0.03)
        self.declare_parameter("body_z_max", +0.03)
        self.declare_parameter("rear_z_min", -0.05)
        self.declare_parameter("rear_z_max", +0.05)
        self.declare_parameter("foot_trail_max_points", 300)

        publish_rate = float(self.get_parameter("publish_rate").value)
        active = list(self.get_parameter("active_legs").value)
        idle = list(self.get_parameter("idle_joints").value)
        link_yaml = self.get_parameter("link_params_yaml").value
        joints_yaml = self.get_parameter("urdf_joints_yaml").value
        if not link_yaml or not joints_yaml:
            raise RuntimeError(
                "kinematic_node requires link_params_yaml and urdf_joints_yaml "
                "(set by the launch file via PathJoinSubstitution).")

        for name in active:
            if name not in LEG_NAMES:
                raise ValueError(f"active_legs contains unknown leg '{name}'")
        if len(idle) != 3:
            raise ValueError(f"idle_joints must have 3 entries, got {idle}")
        self._idle = tuple(float(x) for x in idle)

        ft_params = FootTargetParams(
            stride_per_mps=float(self.get_parameter("stride_per_mps").value),
            swing_height=float(self.get_parameter("swing_height").value),
            stance_phase_ratio=float(
                self.get_parameter("stance_phase_ratio").value),
            swing_activation_speed=float(
                self.get_parameter("swing_activation_speed").value),
        )

        # ----- runtime state -----
        geoms = load_leg_geoms(joints_yaml)
        self.commander = BodyCommander(
            step_freq=float(self.get_parameter("step_freq").value),
            body_z_min=float(self.get_parameter("body_z_min").value),
            body_z_max=float(self.get_parameter("body_z_max").value),
            rear_z_min=float(self.get_parameter("rear_z_min").value),
            rear_z_max=float(self.get_parameter("rear_z_max").value))
        self.drivers: Dict[str, LegDriver] = {
            name: LegDriver(geoms[name],
                            load_link_params(link_yaml, name),
                            ft_params,
                            is_rear=(name in ("BL", "BR")),
                            logger=self.get_logger())
            for name in active
        }
        self._joint_names = _all_joint_names()

        self._sub = self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self._pub = self.create_publisher(JointState, "/joint_states", 10)

        max_pts = int(self.get_parameter("foot_trail_max_points").value)
        self._trails: Dict[str, FootTrail] = {
            name: FootTrail(name=name, color=LEG_COLORS[name], max_points=max_pts)
            for name in LEG_NAMES
        }
        self._trail_pub = self.create_publisher(
            MarkerArray, "/foot_trails", 10)

        self._t_last = time.monotonic()
        self._timer = self.create_timer(1.0 / publish_rate, self._tick)

        self.get_logger().info(
            f"kinematic_node up: legs={active}, idle={idle}, "
            f"rate={publish_rate} Hz, step_freq={ft_params.stride_per_mps}")

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.commander.on_cmd_vel(
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.y, msg.angular.z)

    def _tick(self) -> None:
        now = time.monotonic()
        dt = now - self._t_last
        self._t_last = now
        self.commander.tick(dt)

        v_xy = self.commander.body_vel_xy()
        bz = self.commander.body_z()
        rz = self.commander.rear_z()
        positions: List[float] = []
        for leg in LEG_NAMES:
            if leg in self.drivers:
                q = self.drivers[leg].step(
                    v_xy, self.commander.phase(leg), bz, rz)
            else:
                q = self._idle
            positions.extend(float(x) for x in q)

        stamp = self.get_clock().now().to_msg()
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = self._joint_names
        msg.position = positions
        self._pub.publish(msg)

        trail_msg = MarkerArray()
        for idx, leg in enumerate(LEG_NAMES):
            if leg in self.drivers:
                d = self.drivers[leg]
                # Reuse the joint positions already computed above.
                q = tuple(positions[3 * idx : 3 * idx + 3])
                foot_hip = fk_leg(d.link, q)
                foot_body = d.geom.base_to_hip_xyz + d.geom.R_base_to_hip @ foot_hip
                self._trails[leg].append(foot_body)
            trail_msg.markers.append(
                build_marker(self._trails[leg], frame_id="base_link",
                             marker_id=idx, stamp=stamp))
        self._trail_pub.publish(trail_msg)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
