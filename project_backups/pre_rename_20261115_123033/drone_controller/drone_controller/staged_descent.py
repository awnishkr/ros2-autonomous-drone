"""Experimental, altitude-limited vision-guided descent in Gazebo."""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from landing_alignment import LandingAlignment


class AIStagedDescent(LandingAlignment):

    def __init__(self):
        super().__init__()

        self.declare_parameter('descent_enabled', False)
        self.descent_enabled = bool(
            self.get_parameter('descent_enabled').value
        )

        self.center_since = None
        self.descent_started = False
        self.descent_complete = False

        self.stage_altitude = 1.20
        self.stage_stop_altitude = 1.22
        self.stage_max_vz = 0.06

        self.get_logger().info(
            f'STAGED DESCENT | descent_enabled={self.descent_enabled} '
            '| target=1.20m | no touchdown or motor shutdown'
        )

    def update(self):

        now = time.monotonic()
        cmd = Twist()
        status = 'NO ODOMETRY - HOLD'

        odom_valid = (
            self.position is not None
            and now - self.last_odom < 1.0
        )

        if odom_valid:

            x, y, z = self.position
            sx, sy, _ = self.start_position

            distance = math.hypot(x - sx, y - sy)

            if distance > 0.85:
                self.permanent_hold = True
                self.hold_reason = 'Movement boundary exceeded'

            if not 1.10 <= z <= 2.30:
                self.permanent_hold = True
                self.hold_reason = 'Altitude outside staged test limits'

            if abs(self.yaw) >= math.radians(20):
                self.permanent_hold = True
                self.hold_reason = 'Yaw outside staged test limits'

            detection_valid = (
                self.target is not None
                and now - self.last_detection < 2.0
            )

            if self.permanent_hold:

                status = f'SAFETY HOLD: {self.hold_reason}'

            elif self.descent_complete:

                status = 'STAGE COMPLETE - HOLD'

            elif not detection_valid:

                self.center_since = None
                status = 'NO DETECTION - HOLD'

            elif not self.horizontal_enabled:

                self.center_since = None
                status = 'HORIZONTAL DISABLED - HOLD'

            else:

                cx, cy, _ = self.target

                error_x = cx - 0.5
                error_y = cy - 0.5

                aligned = (
                    max(abs(error_x), abs(error_y)) < 0.025
                )

                if self.alignment_start is None:
                    self.alignment_start = now

                if aligned:

                    if self.center_since is None:
                        self.center_since = now

                    self.alignment_start = now
                    status = 'ALIGNED - VERIFYING'

                    if (
                        self.descent_enabled
                        and now - self.center_since >= 2.0
                    ):
                        self.descent_started = True

                    if self.descent_started:

                        if z <= self.stage_stop_altitude:

                            self.descent_complete = True
                            status = 'STAGE COMPLETE - HOLD'

                        else:

                            cmd.linear.z = self.limit(
                                0.6 * (self.stage_altitude - z),
                                self.stage_max_vz
                            )

                            status = 'VISION-GUIDED DESCENT'

                else:

                    self.center_since = None

                    if now - self.alignment_start > 15.0:

                        self.permanent_hold = True
                        self.hold_reason = (
                            '15-second alignment timeout'
                        )
                        status = (
                            f'SAFETY HOLD: {self.hold_reason}'
                        )

                    else:

                        cmd.linear.x = self.limit(
                            -0.6 * error_y,
                            self.max_horizontal_speed
                        )

                        cmd.linear.y = self.limit(
                            -0.6 * error_x,
                            self.max_horizontal_speed
                        )

                        status = (
                            'CORRECTING LANDING ALIGNMENT'
                            if self.descent_started
                            else 'AI ALIGNING'
                        )

            # Maintain 1.5m altitude before descent begins.
            # During descent, missing detection commands zero
            # velocity instead of continuing downward.

            if (
                not self.permanent_hold
                and not self.descent_started
                and not self.descent_complete
            ):

                cmd.linear.z = self.limit(
                    0.6 * (self.target_altitude - z),
                    self.max_vertical_speed
                )

        self.preview_pub.publish(cmd)

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
    node = AIStagedDescent()

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
