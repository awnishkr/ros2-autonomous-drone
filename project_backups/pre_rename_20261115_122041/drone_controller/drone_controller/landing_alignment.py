import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32MultiArray


class LandingAlignment(Node):

    def __init__(self):
        super().__init__('vision_landing_controller')

        self.declare_parameter('flight_enabled', False)
        self.flight_enabled = bool(
            self.get_parameter('flight_enabled').value
        )

        self.declare_parameter('horizontal_enabled', False)
        self.horizontal_enabled = bool(
            self.get_parameter('horizontal_enabled').value
        )

        self.target_altitude = 1.5
        self.max_horizontal_speed = 0.12
        self.max_vertical_speed = 0.15

        self.confidence_threshold = 0.80

        self.position = None
        self.start_position = None
        self.yaw = 0.0

        self.last_odom = 0.0
        self.last_detection = 0.0

        self.target = None
        self.alignment_start = None
        self.permanent_hold = False
        self.hold_reason = ''

        self.last_log = 0.0

        self.create_subscription(
            Odometry,
            '/model/quadcopter/odometry',
            self.on_odometry,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Float32MultiArray,
            '/landing/detection',
            self.on_detection,
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

        self.preview_pub = self.create_publisher(
            Twist,
            '/landing/cmd_vel_preview',
            10
        )

        self.create_timer(0.1, self.update)

        self.get_logger().info(
            f'Vision Landing Controller | '
            f'flight_enabled={self.flight_enabled}'
        )

        if not self.flight_enabled:
            self.get_logger().info(
                'DRY RUN: Drone movement disabled'
            )

    @staticmethod
    def limit(value, maximum):
        return max(-maximum, min(maximum, value))

    def on_odometry(self, msg):

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.position = (p.x, p.y, p.z)

        self.yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y*q.y + q.z*q.z)
        )

        self.last_odom = time.monotonic()

        if self.start_position is None:
            self.start_position = self.position

    def on_detection(self, msg):

        if len(msg.data) != 3:
            self.target = None
            return

        cx, cy, confidence = msg.data

        if (
            not all(math.isfinite(v) for v in (cx, cy, confidence))
            or confidence < self.confidence_threshold
            or not (0 <= cx <= 1)
            or not (0 <= cy <= 1)
        ):
            self.target = None
            return

        self.target = (cx, cy, confidence)
        self.last_detection = time.monotonic()

    def update(self):

        now = time.monotonic()
        cmd = Twist()
        status = 'HOLD'

        # Validate odometry.
        odom_valid = (
            self.position is not None
            and now - self.last_odom < 1.0
        )

        if odom_valid:

            x, y, z = self.position

            # Maintain altitude.
            cmd.linear.z = self.limit(
                0.6 * (self.target_altitude - z),
                self.max_vertical_speed
            )

            # Stop movement outside the test region.
            if self.start_position is not None:

                sx, sy, _ = self.start_position

                distance = math.hypot(x - sx, y - sy)

                if distance > 0.85:
                    self.permanent_hold = True
                    self.hold_reason = 'Movement boundary exceeded'

            # Stop alignment if altitude or orientation is unsafe.
            safe_state = (
                0.7 <= z <= 2.3
                and abs(self.yaw) < math.radians(20)
                and not self.permanent_hold
            )

            detection_valid = (
                self.target is not None
                and now - self.last_detection < 2.0
            )

            if safe_state and detection_valid and self.horizontal_enabled:

                cx, cy, confidence = self.target

                error_x = cx - 0.5
                error_y = cy - 0.5

                if self.alignment_start is None:
                    self.alignment_start = now

                # Limit the duration of vision-guided movement.
                if max(abs(error_x), abs(error_y)) < 0.025:
                    self.alignment_start = now
                    status = 'ALIGNED - HOLD'

                elif now - self.alignment_start > 15.0:
                    self.permanent_hold = True
                    self.hold_reason = '15-second alignment timeout'
                    status = 'TIME LIMIT - HOLD'

                else:
                    # Camera-to-body calibration.
                    vx = -0.6 * error_y
                    vy = -0.6 * error_x

                    cmd.linear.x = self.limit(
                        vx, self.max_horizontal_speed
                    )

                    cmd.linear.y = self.limit(
                        vy, self.max_horizontal_speed
                    )

                    status = 'VISION ALIGNING'

            elif not safe_state:
                if self.permanent_hold:
                    reason = self.hold_reason or 'Safety latch activated'
                elif not (0.7 <= self.position[2] <= 2.3):
                    reason = 'Altitude outside permitted range'
                else:
                    reason = 'Yaw outside permitted range'

                status = f'SAFETY HOLD: {reason}'

            elif not detection_valid:
                status = 'NO DETECTION - HOLD'

        else:
            status = 'NO ODOMETRY - HOLD'

        # Publish preview regardless of flight mode.
        self.preview_pub.publish(cmd)

        # Actual flight commands require explicit activation.
        if self.flight_enabled:
            self.enable_pub.publish(Bool(data=True))
            self.cmd_pub.publish(cmd)

        if now - self.last_log >= 2.0:

            self.get_logger().info(
                f'{status} | '
                f'vx={cmd.linear.x:.3f}, '
                f'vy={cmd.linear.y:.3f}, '
                f'vz={cmd.linear.z:.3f}'
            )

            self.last_log = now


def main():

    rclpy.init()
    node = LandingAlignment()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok() and node.flight_enabled:
            node.cmd_pub.publish(Twist())
            time.sleep(0.2)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
