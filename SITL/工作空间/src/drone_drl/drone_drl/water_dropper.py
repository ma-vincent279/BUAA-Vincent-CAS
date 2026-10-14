#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Point, Pose

class WaterDropper(Node):
    def __init__(self):
        super().__init__('water_dropper')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        self.sub = self.create_subscription(Point, '/drop_water', self.drop_cb, 10)
        self.counter = 0
        self.get_logger().info("Water dropper ready")

    def drop_cb(self, msg):
        # 投弹位置必须有效（非原点，避免误触发）
        if abs(msg.x) < 0.01 and abs(msg.y) < 0.01:
            self.get_logger().warn(f"Ignoring drop at origin ({msg.x:.2f}, {msg.y:.2f})")
            return
        self.drop(msg.x, msg.y, msg.z)

    def drop(self, x, y, z):
        self.counter += 1
        name = f'water_ball_{self.counter}'
        sdf = f'''
        <sdf version="1.6">
          <model name="{name}">
            <pose>{x} {y} {z} 0 0 0</pose>
            <link name="link">
              <inertial><mass>0.1</mass><inertia><ixx>0.001</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.001</iyy><iyz>0</iyz><izz>0.001</izz></inertia></inertial>
              <collision name="collision">
                <geometry><sphere><radius>0.08</radius></sphere></geometry>
              </collision>
              <visual name="visual">
                <geometry><sphere><radius>0.08</radius></sphere></geometry>
                <material>
                  <ambient>0 0 1 1</ambient>
                  <diffuse>0 0 1 1</diffuse>
                </material>
              </visual>
            </link>
          </model>
        </sdf>'''
        req = SpawnEntity.Request()
        req.name = name
        req.xml = sdf
        req.initial_pose = Pose()
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = z
        self.client.call_async(req)
        self.get_logger().info(f"Water ball dropped at ({x:.2f}, {y:.2f}, {z:.2f})")

def main():
    rclpy.init()
    node = WaterDropper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()