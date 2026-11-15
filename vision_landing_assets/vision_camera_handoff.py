import cv2
import rclpy

from std_msgs.msg import Bool
from landing_camera_viewer import LandingCameraView


class VisionCameraHandoff(LandingCameraView):

    def __init__(self):
        super().__init__()

        self.handoff_received = False

        self.create_subscription(
            Bool,
            '/mission/handoff',
            self.on_handoff,
            10
        )

        self.get_logger().info(
            'YOLO11n Camera: Waiting for navigation handoff'
        )

    def on_handoff(self, msg):

        if msg.data and not self.handoff_received:
            self.handoff_received = True

            self.get_logger().info(
                'Navigation complete. Opening vision camera.'
            )


def main():

    rclpy.init()
    node = VisionCameraHandoff()

    window = 'YOLO11n Precision Landing Camera'
    window_open = False

    try:
        while rclpy.ok():

            rclpy.spin_once(
                node,
                timeout_sec=0.03
            )

            if not node.handoff_received:
                continue

            if not window_open:

                cv2.namedWindow(
                    window,
                    cv2.WINDOW_NORMAL
                )

                cv2.resizeWindow(window, 640, 640)

                window_open = True

            cv2.imshow(window, node.draw())

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass

    finally:
        if window_open:
            cv2.destroyAllWindows()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
