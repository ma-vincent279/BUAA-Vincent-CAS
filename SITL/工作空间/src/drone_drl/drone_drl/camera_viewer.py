#!/usr/bin/env python3
"""独立节点：订阅下视摄像头并显示实时画面，不影响任何现有代码。"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self._cb, 10)
        self.get_logger().info('下视摄像头查看器已启动，按 Q 退出喵~')

    def _cb(self, msg):
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.imshow('Downward Camera (640x480)', img_bgr)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('Q 键按下，退出喵~')
            rclpy.shutdown()


def main():
    rclpy.init()
    node = CameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
