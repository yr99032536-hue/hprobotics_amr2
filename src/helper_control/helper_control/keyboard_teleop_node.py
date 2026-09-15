import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


MOVE_BINDINGS = {
    'i': (1.0, 0.0),
    ',': (-1.0, 0.0),
    'j': (0.0, 1.0),
    'l': (0.0, -1.0),
    'u': (1.0, 1.0),
    'o': (1.0, -1.0),
    'm': (-1.0, -1.0),
    '.': (-1.0, 1.0),
    'k': (0.0, 0.0),
}

SPEED_BINDINGS = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('helper_keyboard_teleop')
        self.declare_parameter('cmd_vel_topic', '/control/cmd_vel_smoothed')
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('turn', 1.8)
        self.declare_parameter('repeat_rate', 30.0)
        self.declare_parameter('key_timeout', 0.25)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.speed = float(self.get_parameter('speed').value)
        self.turn = float(self.get_parameter('turn').value)
        self.repeat_rate = float(self.get_parameter('repeat_rate').value)
        self.key_timeout = float(self.get_parameter('key_timeout').value)

        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.last_key_time = self.get_clock().now()
        self.stopped = True

        self.timer = self.create_timer(
            1.0 / max(self.repeat_rate, 1.0),
            self.timer_callback,
        )

        self.get_logger().info(
            f'keyboard teleop publishing to {self.cmd_vel_topic}'
        )
        self.get_logger().info(
            'Use i/j/k/l/u/o/m/,/. to move, q/z/w/x/e/c to scale, Ctrl-C to quit.'
        )

    def handle_key(self, key):
        if key in MOVE_BINDINGS:
            linear, angular = MOVE_BINDINGS[key]
            self.last_linear = linear * self.speed
            self.last_angular = angular * self.turn
            self.last_key_time = self.get_clock().now()
            self.stopped = False
            return

        if key in SPEED_BINDINGS:
            linear_scale, angular_scale = SPEED_BINDINGS[key]
            self.speed *= linear_scale
            self.turn *= angular_scale
            self.get_logger().info(
                f'speed={self.speed:.2f}, turn={self.turn:.2f}'
            )
            return

        if key == '\x03':
            raise KeyboardInterrupt

    def timer_callback(self):
        elapsed = (
            self.get_clock().now() - self.last_key_time
        ).nanoseconds / 1e9

        twist = Twist()
        if elapsed <= self.key_timeout:
            twist.linear.x = self.last_linear
            twist.angular.z = self.last_angular
            self.stopped = False
        elif not self.stopped:
            self.last_linear = 0.0
            self.last_angular = 0.0
            self.stopped = True
            self.publisher.publish(twist)

        if elapsed <= self.key_timeout or not self.stopped:
            self.publisher.publish(twist)


def read_key():
    ready, _, _ = select.select([sys.stdin], [], [], 0.0)
    if not ready:
        return None

    data = sys.stdin.buffer.read(1)
    if not data:
        return None

    # Arrow/function keys arrive as escape sequences. Drain and ignore them.
    if data == b'\x1b':
        while select.select([sys.stdin], [], [], 0.0)[0]:
            sys.stdin.buffer.read(1)
        return None

    try:
        return data.decode('ascii')
    except UnicodeDecodeError:
        return None


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    old_settings = termios.tcgetattr(sys.stdin)

    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            key = read_key()
            if key is not None:
                node.handle_key(key)
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
