import os
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


class LandingDetector(Node):

    def __init__(self):
        super().__init__('yolo11n_pad_detector')

        # Read the model from this checkout for isolated execution.
        project_root = os.environ.get('DRONE_PROJECT_ROOT')
        self.folder = (Path(project_root) / 'vision_landing_assets'
                       if project_root else Path(__file__).resolve().parent)

        self.session = ort.InferenceSession(
            str(self.folder / 'landing_detector.onnx'),
            providers=['CPUExecutionProvider']
        )

        self.input_name = self.session.get_inputs()[0].name
        self.threshold = 0.50
        self.frames = 0

        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/landing/detection',
            10
        )

        self.subscription = self.create_subscription(
            Image,
            '/quadcopter/landing_camera/image',
            self.detect,
            qos_profile_sensor_data
        )

        self.get_logger().info('Pretrained neural network loaded')
        self.get_logger().info('Waiting for live Gazebo images...')

    def detect(self, msg):

        if msg.encoding.lower() not in ('rgb8', 'bgr8'):
            self.get_logger().warning(
                f'Unsupported encoding: {msg.encoding}'
            )
            return

        if msg.step < msg.width * 3:
            self.get_logger().warning('Invalid image stride')
            return

        raw = np.frombuffer(msg.data, dtype=np.uint8)

        frame = raw.reshape(
            msg.height, msg.step
        )[:, :msg.width * 3]

        frame = frame.reshape(
            msg.height, msg.width, 3
        ).copy()

        if msg.encoding.lower() == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        h, w = frame.shape[:2]

        scale = min(640 / w, 640 / h)

        nw = round(w * scale)
        nh = round(h * scale)

        resized = cv2.resize(frame, (nw, nh))

        canvas = np.full(
            (640, 640, 3), 114, dtype=np.uint8
        )

        left = (640 - nw) // 2
        top = (640 - nh) // 2

        canvas[top:top+nh, left:left+nw] = resized

        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)

        tensor = (
            rgb.transpose(2, 0, 1)[None]
            .astype(np.float32) / 255.0
        )

        output = self.session.run(
            None,
            {self.input_name: tensor}
        )[0]

        predictions = output[0].T

        best = predictions[
            np.argmax(predictions[:, 4])
        ]

        x, y, bw, bh, confidence = best

        result = Float32MultiArray()

        if confidence >= self.threshold:

            cx = (float(x) - left) / scale
            cy = (float(y) - top) / scale

            result.data = [
                float(cx / w),
                float(cy / h),
                float(confidence)
            ]

            x1 = int((x - bw / 2 - left) / scale)
            y1 = int((y - bh / 2 - top) / scale)
            x2 = int((x + bw / 2 - left) / scale)
            y2 = int((y + bh / 2 - top) / scale)

            x1 = max(0, min(w - 1, x1))
            y1 = max(0, min(h - 1, y1))
            x2 = max(0, min(w - 1, x2))
            y2 = max(0, min(h - 1, y2))

            cv2.rectangle(
                frame, (x1, y1), (x2, y2),
                (0, 255, 0), 2
            )

            self.get_logger().info(
                f'LANDING PAD DETECTED | '
                f'Confidence: {confidence:.2f} | '
                f'Centre: ({cx/w:.3f}, {cy/h:.3f})'
            )

        else:

            result.data = [-1.0, -1.0, 0.0]

            if self.frames % 5 == 0:
                self.get_logger().info(
                    f'No landing pad | '
                    f'Max confidence: {confidence:.3f}'
                )

        self.publisher.publish(result)

        self.frames += 1

        if self.frames % 5 == 0:
            cv2.imwrite(
                str(self.folder / 'live_detection.png'),
                frame
            )


def main():
    rclpy.init()
    node = LandingDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
