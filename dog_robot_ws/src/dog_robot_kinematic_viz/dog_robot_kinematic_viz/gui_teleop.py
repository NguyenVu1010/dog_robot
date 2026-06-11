"""Minimal Tk GUI teleop: 5 sliders → /cmd_vel + 2 buttons → /sit, /release.

Self-contained — no dog_robot_control / external dependency. Designed for the
kinematic verification rig. Sliders publish a Twist at 50 Hz; pose buttons call
the kinematic_node's Trigger services asynchronously and log the response.

Run via:
    ros2 run dog_robot_kinematic_viz gui_teleop
or use the kinematic_gui launch file (full rig + GUI + RViz).
"""
from __future__ import annotations
import tkinter as tk
from tkinter import ttk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import Trigger


PUBLISH_PERIOD_MS = 20   # 50 Hz
SPIN_PERIOD_MS = 20      # 50 Hz spin tick

LIN_MAX = 0.20
ANG_MAX = 0.80
LIN_RES = 0.01
ANG_RES = 0.05


class GuiTeleopNode(Node):
    def __init__(self):
        super().__init__("gui_teleop")
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._sit_client = self.create_client(Trigger, "/sit")
        self._release_client = self.create_client(Trigger, "/release")
        self._twist = Twist()

    def set_axis(self, name: str, value: float) -> None:
        if name == "vx":
            self._twist.linear.x = float(value)
        elif name == "vy":
            self._twist.linear.y = float(value)
        elif name == "vz":
            self._twist.linear.z = float(value)
        elif name == "wy":
            self._twist.angular.y = float(value)
        elif name == "wz":
            self._twist.angular.z = float(value)

    def publish_twist(self) -> None:
        self._pub.publish(self._twist)

    def zero(self) -> None:
        self._twist = Twist()
        self._pub.publish(self._twist)

    def call_sit(self) -> None:
        if not self._sit_client.service_is_ready():
            self.get_logger().warn("/sit service not available")
            return
        future = self._sit_client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f"/sit response: success={f.result().success} "
                f"message='{f.result().message}'"))

    def call_release(self) -> None:
        if not self._release_client.service_is_ready():
            self.get_logger().warn("/release service not available")
            return
        future = self._release_client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f"/release response: success={f.result().success} "
                f"message='{f.result().message}'"))


class TeleopGUI:
    def __init__(self, root: tk.Tk, node: GuiTeleopNode):
        self.root = root
        self.node = node
        root.title("dog_robot_kine teleop GUI")
        root.protocol("WM_DELETE_WINDOW", self._on_close)

        # Slider section.
        self._sliders: dict[str, tk.Scale] = {}
        self._make_slider("Forward / Back  (vx)", "vx",
                          -LIN_MAX, +LIN_MAX, LIN_RES)
        self._make_slider("Strafe          (vy)", "vy",
                          -LIN_MAX, +LIN_MAX, LIN_RES)
        self._make_slider("Body height     (vz)", "vz",
                          -LIN_MAX, +LIN_MAX, LIN_RES)
        self._make_slider("Pitch           (wy)", "wy",
                          -LIN_MAX, +LIN_MAX, LIN_RES)
        self._make_slider("Yaw             (wz)", "wz",
                          -ANG_MAX, +ANG_MAX, ANG_RES)

        zero_btn = ttk.Button(root, text="Zero all axes", command=self._zero)
        zero_btn.pack(fill=tk.X, padx=8, pady=(8, 4))

        ttk.Separator(root, orient=tk.HORIZONTAL).pack(
            fill=tk.X, padx=4, pady=4)

        # Pose buttons.
        pose_frame = ttk.Frame(root)
        pose_frame.pack(fill=tk.X, padx=8, pady=(4, 8))
        sit_btn = ttk.Button(pose_frame, text="Sit",
                             command=self.node.call_sit)
        sit_btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=4)
        release_btn = ttk.Button(pose_frame, text="Release",
                                 command=self.node.call_release)
        release_btn.pack(side=tk.RIGHT, expand=True, fill=tk.X, padx=4)

        # Schedule periodic ROS work.
        self.root.after(PUBLISH_PERIOD_MS, self._publish_tick)
        self.root.after(SPIN_PERIOD_MS, self._spin_tick)

    def _make_slider(self, label: str, axis: str,
                     mn: float, mx: float, res: float) -> None:
        frame = ttk.Frame(self.root)
        frame.pack(fill=tk.X, padx=8, pady=2)
        ttk.Label(frame, text=label, width=22, anchor="w").pack(
            side=tk.LEFT)
        scale = tk.Scale(frame, from_=mn, to=mx, resolution=res,
                         orient=tk.HORIZONTAL, length=200,
                         command=lambda v, a=axis: self.node.set_axis(a, v))
        scale.set(0.0)
        scale.pack(side=tk.RIGHT, fill=tk.X, expand=True)
        self._sliders[axis] = scale

    def _zero(self) -> None:
        for scale in self._sliders.values():
            scale.set(0.0)
        # Setting Scale fires its command which updates self.node._twist,
        # but explicitly zero + publish to be safe.
        self.node.zero()

    def _publish_tick(self) -> None:
        self.node.publish_twist()
        self.root.after(PUBLISH_PERIOD_MS, self._publish_tick)

    def _spin_tick(self) -> None:
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.root.after(SPIN_PERIOD_MS, self._spin_tick)

    def _on_close(self) -> None:
        # Stop the robot before quitting.
        self.node.zero()
        rclpy.spin_once(self.node, timeout_sec=0.05)
        self.root.quit()


def main(args=None):
    rclpy.init(args=args)
    node = GuiTeleopNode()
    root = tk.Tk()
    TeleopGUI(root, node)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
