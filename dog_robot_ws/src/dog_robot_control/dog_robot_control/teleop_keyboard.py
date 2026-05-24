"""Keyboard teleop -> /cmd_vel.

Continuous publish at 10 Hz so walker_controller (cmd_vel timeout = 0.5 s)
keeps moving while a key is held. Keys mutate state; space zeros it.
"""
import select
import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

LIN_STEP = 0.02
YAW_STEP = 0.05
LIN_MAX = 0.15
YAW_MAX = 0.50

HELP = """\
Teleop /cmd_vel (continuous at 10 Hz)
  w / s   linear.x  +/- {lstep:.2f}   (cap {lmax:.2f})
  a / d   linear.y  +/- {lstep:.2f}   (cap {lmax:.2f})
  j / l   angular.z +/- {ystep:.2f}   (cap {ymax:.2f})
  space   stop (zero all)
  x       quit
""".format(lstep=LIN_STEP, lmax=LIN_MAX, ystep=YAW_STEP, ymax=YAW_MAX)


def _clamp(v, lim):
    return max(-lim, min(lim, v))


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__("teleop_keyboard")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.alive = True
        self.lock = threading.Lock()
        self.create_timer(0.1, self._publish)

    def _publish(self):
        with self.lock:
            msg = Twist()
            msg.linear.x = self.vx
            msg.linear.y = self.vy
            msg.angular.z = self.wz
        self.pub.publish(msg)

    def on_key(self, k: str) -> bool:
        with self.lock:
            if k == "w":
                self.vx = _clamp(self.vx + LIN_STEP, LIN_MAX)
            elif k == "s":
                self.vx = _clamp(self.vx - LIN_STEP, LIN_MAX)
            elif k == "a":
                self.vy = _clamp(self.vy + LIN_STEP, LIN_MAX)
            elif k == "d":
                self.vy = _clamp(self.vy - LIN_STEP, LIN_MAX)
            elif k == "j":
                self.wz = _clamp(self.wz + YAW_STEP, YAW_MAX)
            elif k == "l":
                self.wz = _clamp(self.wz - YAW_STEP, YAW_MAX)
            elif k == " ":
                self.vx = self.vy = self.wz = 0.0
            elif k == "x" or k == "\x03":
                return False
            sys.stdout.write(
                f"\rvx={self.vx:+.2f}  vy={self.vy:+.2f}  wz={self.wz:+.2f}    "
            )
            sys.stdout.flush()
        return True


def _read_key(fd):
    rlist, _, _ = select.select([fd], [], [], 0.1)
    return sys.stdin.read(1) if rlist else ""


def main():
    rclpy.init()
    node = TeleopKeyboard()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    sys.stdout.write(HELP)
    sys.stdout.flush()

    fd = sys.stdin.fileno()
    if not sys.stdin.isatty():
        # No TTY (background / pipe): publish zeros until SIGINT.
        sys.stdout.write("[teleop] stdin is not a TTY; idling, publishing zeros\n")
        sys.stdout.flush()
        try:
            spin_thread.join()
        except KeyboardInterrupt:
            pass
        node.destroy_node()
        rclpy.shutdown()
        return

    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while True:
            k = _read_key(fd)
            if not k:
                continue
            if not node.on_key(k):
                break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        sys.stdout.write("\n")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
