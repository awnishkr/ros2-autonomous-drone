import time
import rclpy

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from ai_final_landing import AIFinalLanding


class AIMissionLanding(AIFinalLanding):

    def __init__(self):
        super().__init__()

        self.active = False
        self.pad_xy = (6.0, 6.0)
        self.alignment_timeout = 35.0
        self.max_movement_radius = 1.75

        self.create_subscription(
            Bool,
            '/ai_mission/handoff',
            self.on_handoff,
            10
        )

        self.get_logger().info(
            'AI LANDING READY — WAITING FOR A* HANDOFF'
        )

    def on_handoff(self, msg):

        if not msg.data or self.active:
            return

        if (
            self.position is None
            or time.monotonic() - self.last_odom >= 1.0
        ):
            self.get_logger().error(
                'Handoff rejected: odometry unavailable'
            )
            return

        self.start_position = self.position

        self.alignment_start = None
        self.permanent_hold = False
        self.hold_reason = ''

        self.phase = 'ALIGN'
        self.center_since = None
        self.ground_since = None

        self.contact_count = 0
        self.last_ground_contact = 0.0

        self.active = True

        self.get_logger().info(
            'A* NAVIGATION COMPLETE — AI LANDING ACTIVATED'
        )

    def update(self):

        if self.active:
            super().update()


def main():

    rclpy.init()
    node = AIMissionLanding()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok() and node.active and node.flight_enabled:
            node.cmd_pub.publish(Twist())

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
