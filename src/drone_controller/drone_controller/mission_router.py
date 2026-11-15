import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class MissionRouter(Node):

    def __init__(self):
        super().__init__('mission_command_router')

        self.mode = 'NAV'
        self.disarmed = False

        self.commands = {
            'NAV': None,
            'LANDING': None
        }

        self.command_times = {
            'NAV': 0.0,
            'LANDING': 0.0
        }

        self.enabled = {
            'NAV': None,
            'LANDING': None
        }

        self.enable_times = {
            'NAV': 0.0,
            'LANDING': 0.0
        }

        for mode, name in (
            ('NAV', 'nav'),
            ('LANDING', 'landing')
        ):
            self.create_subscription(
                Twist,
                f'/mission/{name}/cmd_vel',
                lambda msg, m=mode: self.on_cmd(m, msg),
                10
            )

            self.create_subscription(
                Bool,
                f'/mission/{name}/enable',
                lambda msg, m=mode: self.on_enable(m, msg),
                10
            )

        self.create_subscription(
            Bool,
            '/mission/handoff',
            self.on_handoff,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/quadcopter/cmd_vel',
            10
        )

        self.enable_pub = self.create_publisher(
            Bool,
            '/quadcopter/enable',
            10
        )

        self.create_timer(0.1, self.update)

        self.get_logger().info(
            'MISSION ROUTER ACTIVE — A* HAS CONTROL'
        )

    def on_cmd(self, mode, msg):
        self.commands[mode] = msg
        self.command_times[mode] = time.monotonic()

    def on_enable(self, mode, msg):
        self.enabled[mode] = bool(msg.data)
        self.enable_times[mode] = time.monotonic()

        if (
            mode == 'LANDING'
            and self.mode == 'LANDING'
            and not msg.data
            and not self.disarmed
        ):
            self.disarmed = True
            self.get_logger().info(
                'MOTOR SHUTDOWN LATCHED'
            )

    def on_handoff(self, msg):

        if msg.data and self.mode == 'NAV':
            self.mode = 'LANDING'

            self.get_logger().info(
                'A* route complete — landing controller active'
            )

    def update(self):

        if self.disarmed:
            self.cmd_pub.publish(Twist())
            self.enable_pub.publish(Bool(data=False))
            return

        now = time.monotonic()
        mode = self.mode

        cmd = self.commands[mode]
        enabled = self.enabled[mode]

        cmd_fresh = (
            cmd is not None
            and now - self.command_times[mode] < 0.5
        )

        enable_fresh = (
            enabled is not None
            and now - self.enable_times[mode] < 0.5
        )

        if cmd_fresh and enable_fresh:

            self.enable_pub.publish(
                Bool(data=enabled)
            )

            self.cmd_pub.publish(cmd)

        elif mode == 'LANDING' or self.command_times[mode] > 0:

            # Hover during handoff or temporary command loss.
            self.enable_pub.publish(Bool(data=True))
            self.cmd_pub.publish(Twist())


def main():

    rclpy.init()
    node = MissionRouter()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
