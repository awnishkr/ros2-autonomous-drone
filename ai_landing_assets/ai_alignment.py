import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


class AIAlignment(Node):

    def __init__(self):
        super().__init__('ai_alignment')

        self.kp = 0.6
        self.max_speed = 0.15
        self.confidence_threshold = 0.80
        self.timeout = 1.5

        self.target = None
        self.last_update = 0.0

        self.create_subscription(
            Float32MultiArray,
            '/ai_landing/detection',
            self.on_detection,
            10
        )

        self.publisher = self.create_publisher(
            Twist,
            '/ai_landing/cmd_vel_preview',
            10
        )

        self.create_timer(0.5, self.update)

        self.get_logger().info(
            'AI Alignment Controller started'
        )

        self.get_logger().info(
            'DRY RUN ONLY: No drone motor commands'
        )

    def on_detection(self, msg):

        self.last_update = time.monotonic()

        if len(msg.data) != 3:
            self.target = None
            return

        cx, cy, confidence = msg.data

        if (
            confidence < self.confidence_threshold
            or not (0.0 <= cx <= 1.0)
            or not (0.0 <= cy <= 1.0)
        ):
            self.target = None
            return

        self.target = (cx, cy, confidence)

    def update(self):

        cmd = Twist()

        if (
            self.target is None
            or time.monotonic() - self.last_update > self.timeout
        ):
            self.publisher.publish(cmd)

            self.get_logger().info(
                'No valid recent detection - HOLD'
            )
            return

        cx, cy, confidence = self.target

        error_x = cx - 0.5
        error_y = cy - 0.5

        # Calibrated for the fixed, yaw-zero test camera.
        vx = -self.kp * error_y
        vy = -self.kp * error_x

        vx = max(
            -self.max_speed,
            min(self.max_speed, vx)
        )

        vy = max(
            -self.max_speed,
            min(self.max_speed, vy)
        )

        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)

        # No vertical motion during calibration
        cmd.linear.z = 0.0

        self.publisher.publish(cmd)

        self.get_logger().info(
            f'NN confidence={confidence:.2f} | '
            f'Error=({error_x:.3f},{error_y:.3f}) | '
            f'Preview vx={vx:.3f}, vy={vy:.3f}'
        )


def main():
    rclpy.init()

    node = AIAlignment()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
