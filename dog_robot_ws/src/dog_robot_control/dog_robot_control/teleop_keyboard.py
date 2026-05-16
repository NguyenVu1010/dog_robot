"""Keyboard teleop: WASD → /cmd_vel, space = stop, q = quit."""
import sys, termios, tty, select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


KEYS = {
    "w": (0.10,  0.0,  0.0),
    "s": (-0.10, 0.0,  0.0),
    "a": (0.0,   0.10, 0.0),
    "d": (0.0,  -0.10, 0.0),
    "q": (0.0,   0.0,  0.30),
    "e": (0.0,   0.0, -0.30),
    " ": (0.0,   0.0,  0.0),
}


def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        return sys.stdin.read(1) if rlist else ""
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rclpy.init()
    node = Node("teleop_keyboard")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    print("Teleop: w/s = fwd/back, a/d = left/right, q/e = yaw, space = stop, Ctrl-C = exit")
    try:
        while rclpy.ok():
            k = get_key()
            if k in KEYS:
                vx, vy, vyaw = KEYS[k]
                msg = Twist()
                msg.linear.x = vx
                msg.linear.y = vy
                msg.angular.z = vyaw
                pub.publish(msg)
                print(f"\rcmd_vel: ({vx:.2f}, {vy:.2f}, {vyaw:.2f})", end="")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
