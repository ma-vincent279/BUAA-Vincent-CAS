#!/usr/bin/env python3
"""
载荷投放节点 — /drop_payload 服务（std_srvs/Trigger）。
从 PX4 位姿推算载荷位置，通过 /spawn_entity 在无人机旁生成独立自由落体模型。
不删除原有嵌套载荷（Gazebo Classic 不支持删除嵌套模型），直接生成新的下落体。
"""

import math
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# 载荷在 Gazebo 机体坐标系中的位置 (x=fwd, y=left, z=up)
BODY_OFFSETS = {
    'payload_l':  (0.0, 0.13, -2.00),
    'payload_r':  (0.0, -0.13, -2.00),
}
# PX4 飞机 Gazebo 生成点 (sitl_run.sh 固定)
SPAWN_GZ_X, SPAWN_GZ_Y, SPAWN_GZ_Z = 1.01, 0.98, 0.83

PAYLOAD_SDF = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <pose>0 0 0 0 0 0</pose>
    <link name="link">
      <inertial>
        <mass>0.55</mass>
        <inertia>
          <ixx>0.002364</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.002364</iyy><iyz>0</iyz>
          <izz>0.000290</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <cylinder><radius>0.0325</radius><length>0.22</length></cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder><radius>0.0325</radius><length>0.22</length></cylinder>
        </geometry>
        <material>
          <ambient>1 0.5 0 1</ambient>
          <diffuse>1 0.5 0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


class PayloadDropper(Node):
    def __init__(self):
        super().__init__('payload_dropper')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE, depth=10)

        self.local_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._cb_local, qos)

        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')

        self._drone_x = self._drone_y = self._drone_z = 0.0
        self._heading = None
        self._spawn_ctr = 0
        self._drop_left = True  # 交替：先左后右

        self.srv = self.create_service(Trigger, '/drop_payload', self._cb_drop)
        self.get_logger().info("Payload dropper ready — /drop_payload (one per call, alternating L/R)")

    def _cb_local(self, m):
        self._drone_x, self._drone_y, self._drone_z = m.x, m.y, m.z
        if math.isfinite(m.heading):
            self._heading = m.heading

    # ========== 坐标转换 ==========

    def _ned_to_gazebo(self, nx, ny, nz):
        return (SPAWN_GZ_X + ny, SPAWN_GZ_Y + nx, SPAWN_GZ_Z - nz)

    def _payload_gazebo_pose(self, bx, by, bz):
        """机体偏移 → Gazebo 世界坐标 (x,y,z, qx,qy,qz,qw)"""
        h = self._heading
        cos_h, sin_h = math.cos(h), math.sin(h)
        # body x=fwd→NED (cos,sin); body y=left→NED (sin,-cos); body z=up→NED(neg)
        ndx = bx * cos_h + by * sin_h
        ndy = bx * sin_h + by * (-cos_h)
        ndz = -bz
        pnx = self._drone_x + ndx
        pny = self._drone_y + ndy
        pnz = self._drone_z + ndz
        gx, gy, gz = self._ned_to_gazebo(pnx, pny, pnz)
        # yaw-only quaternion (heading = yaw in Gazebo, both from North)
        half = h / 2.0
        return gx, gy, gz, 0.0, 0.0, math.sin(half), math.cos(half)

    # ========== 投放 ==========

    def _cb_drop(self, request, response):
        if self._heading is None:
            response.success = False
            response.message = "Drone heading not available yet"
            return response

        if not self.spawn_cli.wait_for_service(timeout_sec=2.0):
            response.success = False
            response.message = "/spawn_entity not available"
            return response

        # 每次只投一枚，左右交替
        key = 'payload_l' if self._drop_left else 'payload_r'
        bx, by, bz = BODY_OFFSETS[key]
        gx, gy, gz, qx, qy, qz, qw = self._payload_gazebo_pose(bx, by, bz)
        name = f"dropped_payload_{self._spawn_ctr}"
        self._spawn_ctr += 1

        req = SpawnEntity.Request()
        req.name = name
        req.xml = PAYLOAD_SDF.format(name=name)
        req.initial_pose = Pose()
        req.initial_pose.position.x = gx
        req.initial_pose.position.y = gy
        req.initial_pose.position.z = gz
        req.initial_pose.orientation.x = qx
        req.initial_pose.orientation.y = qy
        req.initial_pose.orientation.z = qz
        req.initial_pose.orientation.w = qw
        self.spawn_cli.call_async(req)

        self.get_logger().info(
            f"Dropped {key} → {name} ({gx:.2f},{gy:.2f},{gz:.2f})")
        self._drop_left = not self._drop_left

        response.success = True
        response.message = f"Dropped {key}"
        return response


def main():
    rclpy.init()
    rclpy.spin(PayloadDropper())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
