"""Minimal WASD+JL+RF+IK teleop publisher for /cmd_vel.

Self-contained — no dog_robot_control / external teleop_twist_keyboard
dependency. Reads single keypresses from a TTY in raw mode; designed to be
launched inside a real terminal (e.g. prefix="gnome-terminal --" in the
launch file).

Keys:
    w / s  : linear.x  +/-   (forward / back)
    a / d  : linear.y  +/-   (strafe left / right)
    r / f  : linear.z  +/-   (body up / down — height velocity)
    i / k  : angular.y +/-   (sit / unsit — body pitch velocity)
    j / l  : angular.z +/-   (yaw left / right)
    space  : zero all five axes
    q      : quit
"""
from __future__ import annotations
import sys
import termios
import tty
from select import select

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


LIN_STEP = 0.02     # m/s per keypress
ANG_STEP = 0.10     # rad/s per keypress
LIN_MAX = 0.20
ANG_MAX = 0.80


HELP = """
  Kinematic teleop — keys:
    w/s  forward / back     (linear.x)
    a/d  left / right       (linear.y)
    r/f  body up / down     (linear.z — height velocity)
    i/k  sit / unsit        (angular.y — body pitch velocity)
    j/l  yaw left / right   (angular.z)
    space  zero all
    q      quit
"""


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class TeleopKeyboard(Node):

    def __init__(self):
        super().__init__("teleop_keyboard")
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._vx = 0.0
        self._vy = 0.0
        self._vz = 0.0
        self._wy = 0.0
        self._wz = 0.0

    def publish(self):
        msg = Twist()
        msg.linear.x = self._vx
        msg.linear.y = self._vy
        msg.linear.z = self._vz
        msg.angular.y = self._wy
        msg.angular.z = self._wz
        self._pub.publish(msg)

    def on_key(self, key: str) -> bool:
        """Returns True to keep running, False to quit."""
        if key == "w":
            self._vx = _clamp(self._vx + LIN_STEP, -LIN_MAX, LIN_MAX)
        elif key == "s":
            self._vx = _clamp(self._vx - LIN_STEP, -LIN_MAX, LIN_MAX)
        elif key == "a":
            self._vy = _clamp(self._vy + LIN_STEP, -LIN_MAX, LIN_MAX)
        elif key == "d":
            self._vy = _clamp(self._vy - LIN_STEP, -LIN_MAX, LIN_MAX)
        elif key == "r":
            self._vz = _clamp(self._vz + LIN_STEP, -LIN_MAX, LIN_MAX)
        elif key == "f":
            self._vz = _clamp(self._vz - LIN_STEP, -LIN_MAX, LIN_MAX)
        elif key == "i":
            self._wy = _clamp(self._wy + LIN_STEP, -LIN_MAX, LIN_MAX)
        elif key == "k":
            self._wy = _clamp(self._wy - LIN_STEP, -LIN_MAX, LIN_MAX)
        elif key == "j":
            self._wz = _clamp(self._wz + ANG_STEP, -ANG_MAX, ANG_MAX)
        elif key == "l":
            self._wz = _clamp(self._wz - ANG_STEP, -ANG_MAX, ANG_MAX)
        elif key == " ":
            self._vx = self._vy = self._vz = self._wy = self._wz = 0.0
        elif key in ("q", "\x03"):     # q or Ctrl-C
            return False
        else:
            return True
        self.publish()
        self.get_logger().info(
            f"cmd_vel: linear=({self._vx:+.2f},{self._vy:+.2f},{self._vz:+.2f})  "
            f"angular=({self._wy:+.2f},{self._wz:+.2f})")
        return True


def _read_key(timeout: float = 0.1) -> str:
    """Read one keystroke from stdin (raw mode). Returns '' on timeout."""
    r, _, _ = select([sys.stdin], [], [], timeout)
    if not r:
        return ""
    return sys.stdin.read(1)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    print(HELP)
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while rclpy.ok():
            key = _read_key()
            if key:
                if not node.on_key(key):
                    break
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
