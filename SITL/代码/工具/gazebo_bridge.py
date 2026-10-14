#!/usr/bin/env python3
"""
Gazebo 桥接节点 — 从 /gazebo/model_states 提取桶位姿并发布为 /bucket/position (Point)。
不再依赖 gz model 子进程，避免 headless 模式下的超时堆积问题。
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelStates


class GazeboBridge(Node):
    def __init__(self):
        super().__init__('gazebo_bridge')
        self.pub = self.create_publisher(Point, '/bucket/position', 10)
        self.sub = self.create_subscription(
            ModelStates, '/gazebo/model_states', self._cb_models, 10)
        self.get_logger().info('Gazebo bridge started (using /gazebo/model_states)')

    def _cb_models(self, msg: ModelStates):
        try:
            idx = msg.name.index('red_bucket')
        except ValueError:
            return
        pose = msg.pose[idx]
        p = pose.position
        self.pub.publish(Point(x=p.x, y=p.y, z=p.z))


def main():
    rclpy.init()
    rclpy.spin(GazeboBridge())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
