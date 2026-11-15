import time
from pathlib import Path

import rclpy
import numpy as np
import cv2

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraCapture(Node):

    def __init__(self):
        super().__init__('landing_camera_capture')

        self.saved = False

        self.create_subscription(
            Image,
            '/quadcopter/landing_camera/image',
            self.callback,
            qos_profile_sensor_data
        )

    def callback(self, msg):

        if self.saved:
            return

        encoding = msg.encoding.lower()

        channels = {
            'rgb8': 3,
            'bgr8': 3,
            'rgba8': 4,
            'bgra8': 4
        }

        if encoding not in channels:
            self.get_logger().error(
                f'Unsupported image encoding: {encoding}'
            )
            return

        n = channels[encoding]

        raw = np.frombuffer(msg.data, dtype=np.uint8)

        frame = raw.reshape(
            msg.height, msg.step
        )[:, :msg.width * n]

        frame = frame.reshape(
            msg.height, msg.width, n
        ).copy()

        if encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        elif encoding == 'rgba8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

        elif encoding == 'bgra8':
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        output = (
            Path.home()
            / 'drone_landing_assets'
            / 'gazebo_camera.png'
        )

        if not cv2.imwrite(str(output), frame):
            raise RuntimeError('Failed to save camera image')

        print('\n========== CAMERA CAPTURE ==========')
        print('Resolution:', msg.width, 'x', msg.height)
        print('Encoding:', msg.encoding)
        print('Saved:', output)
        print('CAMERA CAPTURE SUCCESSFUL')
        print('====================================')

        self.saved = True


rclpy.init()

node = CameraCapture()

print('Waiting for Gazebo camera image...')

deadline = time.monotonic() + 20

while rclpy.ok() and not node.saved:
    if time.monotonic() > deadline:
        print('ERROR: No camera image received')
        break

    rclpy.spin_once(node, timeout_sec=1.0)

node.destroy_node()
rclpy.shutdown()
