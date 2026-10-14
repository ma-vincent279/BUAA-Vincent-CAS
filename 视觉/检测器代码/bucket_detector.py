#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point

class BucketDetector(Node):
    def __init__(self):
        super().__init__('bucket_detector')
        self.sub = self.create_subscription(
            ModelStates, '/gazebo/model_states', self.callback, 10)
        self.bucket_pub = self.create_publisher(Point, '/bucket/position', 10)
        self.get_logger().info('Bucket detector started (Ground Truth)')

    def callback(self, msg):
        try:
            idx = msg.name.index('red_bucket')
            p = msg.pose[idx].position
            self.bucket_pub.publish(Point(x=p.x, y=p.y, z=p.z))
        except ValueError:
            pass

def main():
    rclpy.init()
    rclpy.spin(BucketDetector())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
