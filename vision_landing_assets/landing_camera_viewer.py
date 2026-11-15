import time
import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


class LandingCameraView(Node):

    def __init__(self):
        super().__init__('landing_camera_viewer')

        self.frame = None
        self.detection = None
        self.detection_time = 0.0

        self.create_subscription(
            Image,
            '/quadcopter/landing_camera/image',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Float32MultiArray,
            '/landing/detection',
            self.detection_callback,
            10
        )

        self.get_logger().info(
            'YOLO11n CAMERA VIEWER STARTED'
        )

    def image_callback(self, msg):

        try:
            channels = {
                'rgb8': 3,
                'bgr8': 3,
                'rgba8': 4,
                'bgra8': 4,
                'mono8': 1
            }

            if msg.encoding not in channels:
                return

            count = channels[msg.encoding]

            data = np.frombuffer(msg.data, dtype=np.uint8)

            if count == 1:
                frame = data.reshape(
                    msg.height, msg.step
                )[:, :msg.width]
            else:
                frame = data.reshape(
                    msg.height, msg.step
                )[:, :msg.width * count]

                frame = frame.reshape(
                    msg.height, msg.width, count
                )

            if msg.encoding == 'rgb8':
                frame = cv2.cvtColor(
                    frame, cv2.COLOR_RGB2BGR
                )

            elif msg.encoding == 'rgba8':
                frame = cv2.cvtColor(
                    frame, cv2.COLOR_RGBA2BGR
                )

            elif msg.encoding == 'bgra8':
                frame = cv2.cvtColor(
                    frame, cv2.COLOR_BGRA2BGR
                )

            elif msg.encoding == 'mono8':
                frame = cv2.cvtColor(
                    frame, cv2.COLOR_GRAY2BGR
                )

            self.frame = frame.copy()

        except (ValueError, cv2.error) as error:
            self.get_logger().warning(str(error))

    def detection_callback(self, msg):

        if len(msg.data) != 3:
            return

        cx, cy, confidence = msg.data

        self.detection = (
            float(cx),
            float(cy),
            float(confidence)
        )

        self.detection_time = time.monotonic()

    def draw(self):

        if self.frame is None:
            frame = np.zeros(
                (640, 640, 3), dtype=np.uint8
            )

            cv2.putText(
                frame,
                'WAITING FOR CAMERA...',
                (110, 320),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2
            )

            return frame

        frame = self.frame.copy()

        h, w = frame.shape[:2]

        center_x = w // 2
        center_y = h // 2

        # Camera centre: red crosshair.
        cv2.drawMarker(
            frame,
            (center_x, center_y),
            (0, 0, 255),
            cv2.MARKER_CROSS,
            35,
            2
        )

        status = 'SEARCHING FOR LANDING PAD'
        confidence_text = 'Confidence: --'

        if (
            self.detection is not None
            and time.monotonic() - self.detection_time < 1.5
        ):

            cx, cy, confidence = self.detection

            if (
                confidence >= 0.80
                and 0 <= cx <= 1
                and 0 <= cy <= 1
            ):

                px = int(cx * w)
                py = int(cy * h)

                # Neural network detection: green circle.
                cv2.circle(
                    frame,
                    (px, py),
                    12,
                    (0, 255, 0),
                    3
                )

                cv2.line(
                    frame,
                    (center_x, center_y),
                    (px, py),
                    (0, 255, 255),
                    2
                )

                error_x = cx - 0.5
                error_y = cy - 0.5

                confidence_text = (
                    f'Confidence: {confidence * 100:.1f}%'
                )

                if max(
                    abs(error_x),
                    abs(error_y)
                ) < 0.025:

                    status = 'TARGET ALIGNED'

                else:
                    status = 'VISION ALIGNING'

        # Information panel.
        cv2.rectangle(
            frame,
            (0, 0),
            (w, 110),
            (20, 20, 20),
            -1
        )

        cv2.putText(
            frame,
            'YOLO11n LANDING PAD TRACKER',
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            confidence_text,
            (15, 65),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

        cv2.putText(
            frame,
            status,
            (15, 95),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2
        )

        return frame


def main():

    rclpy.init()

    node = LandingCameraView()

    cv2.namedWindow(
        'YOLO11n Precision Landing Camera',
        cv2.WINDOW_NORMAL
    )

    cv2.resizeWindow(
        'YOLO11n Precision Landing Camera',
        640, 640
    )

    try:
        while rclpy.ok():

            rclpy.spin_once(
                node,
                timeout_sec=0.03
            )

            frame = node.draw()

            cv2.imshow(
                'YOLO11n Precision Landing Camera',
                frame
            )

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass

    finally:
        cv2.destroyAllWindows()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
