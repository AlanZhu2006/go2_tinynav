#!/usr/bin/env python3
import argparse
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP = """
Keyboard teleop publishing geometry_msgs/Twist.

  w/s : forward / backward
  q/e : strafe left / right
  a/d : turn left / right
  space or x : stop
  +/= : increase speed
  -/_ : decrease speed
  h : show help
  Ctrl-C : stop and exit
"""


class KeyboardCmdVel(Node):
    def __init__(self, topic: str, linear_speed: float, angular_speed: float, rate_hz: float,
                 max_linear: float = 1.0, max_angular: float = 2.0):
        self.max_linear = max_linear
        self.max_angular = max_angular
        super().__init__("keyboard_cmd_vel")
        self.pub = self.create_publisher(Twist, topic, 10)
        self.topic = topic
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.current = Twist()
        self.last_key_time = time.monotonic()
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_current)

    def publish_current(self):
        self.pub.publish(self.current)

    def stop(self):
        self.current = Twist()
        for _ in range(5):
            self.pub.publish(self.current)
            time.sleep(0.02)

    def set_key(self, key: str):
        cmd = Twist()

        if key == "w":
            cmd.linear.x = self.linear_speed
        elif key == "s":
            cmd.linear.x = -self.linear_speed
        elif key == "q":
            cmd.linear.y = self.linear_speed
        elif key == "e":
            cmd.linear.y = -self.linear_speed
        elif key == "a":
            cmd.angular.z = self.angular_speed
        elif key == "d":
            cmd.angular.z = -self.angular_speed
        elif key in (" ", "x"):
            pass
        elif key in ("+", "="):
            self.linear_speed = min(self.max_linear, self.linear_speed + 0.05)
            self.angular_speed = min(self.max_angular, self.angular_speed + 0.1)
            self.print_status()
            return
        elif key in ("-", "_"):
            self.linear_speed = max(0.05, self.linear_speed - 0.05)
            self.angular_speed = max(0.1, self.angular_speed - 0.1)
            self.print_status()
            return
        elif key == "h":
            print(HELP)
            self.print_status()
            return
        else:
            return

        self.current = cmd
        self.last_key_time = time.monotonic()
        self.print_status()

    def print_status(self):
        cmd = self.current
        print(
            f"\rtopic={self.topic} "
            f"vx={cmd.linear.x:+.2f} vy={cmd.linear.y:+.2f} wz={cmd.angular.z:+.2f} "
            f"speed=({self.linear_speed:.2f} m/s, {self.angular_speed:.2f} rad/s)   ",
            end="",
            flush=True,
        )


def read_key(timeout_s: float) -> str | None:
    readable, _, _ = select.select([sys.stdin], [], [], timeout_s)
    if not readable:
        return None
    return sys.stdin.read(1)


def main() -> int:
    parser = argparse.ArgumentParser(description="WASDQE keyboard teleop for /cmd_vel.")
    parser.add_argument("--topic", default="/cmd_vel")
    parser.add_argument("--linear-speed", type=float, default=0.20)
    parser.add_argument("--angular-speed", type=float, default=0.45)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--max-linear", type=float, default=1.0, help="HARD cap; +/= cannot exceed")
    parser.add_argument("--max-angular", type=float, default=2.0, help="HARD cap; +/= cannot exceed")
    args = parser.parse_args()

    old_settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = KeyboardCmdVel(args.topic, min(args.linear_speed, args.max_linear), min(args.angular_speed, args.max_angular), args.rate, max_linear=args.max_linear, max_angular=args.max_angular)

    print(HELP)
    node.print_status()
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            key = read_key(0.02)
            if key == "\x03":
                break
            if key is not None:
                node.set_key(key.lower())
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("\nStopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
