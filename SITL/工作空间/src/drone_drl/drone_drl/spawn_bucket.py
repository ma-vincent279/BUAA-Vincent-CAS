#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose

class BucketSpawner(Node):
    def __init__(self):
        super().__init__('bucket_spawner')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')
        self.spawn_bucket()

    def spawn_bucket(self):
        req = SpawnEntity.Request()
        req.name = 'red_bucket'
        req.xml = '''<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="red_bucket">
    <pose>11.2 0.5 0.25 0 0 0</pose>
    <link name="bucket_link">
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.01</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.01</iyy><iyz>0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      <collision name="bucket_collision">
        <geometry>
          <cylinder><radius>0.125</radius><length>0.25</length></cylinder>
        </geometry>
      </collision>
      <visual name="bucket_visual">
        <geometry>
          <cylinder><radius>0.125</radius><length>0.25</length></cylinder>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''
        req.initial_pose = Pose()
        req.initial_pose.position.x = 11.2
        req.initial_pose.position.y = 0.5
        req.initial_pose.position.z = 0.25
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info('Bucket spawned at (11.2, 0.5, 0.25)')
        else:
            self.get_logger().error('Failed to spawn bucket')

def main():
    rclpy.init()
    spawner = BucketSpawner()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
