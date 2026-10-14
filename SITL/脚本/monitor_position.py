#!/usr/bin/env python3
"""实时 NED 坐标监视器 — 观察风力扰动、投放质量突变、伺服精度。

用法:
  source ~/SITL/install/setup.bash
  python3 ~/SITL/scripts/monitor_position.py

按 Ctrl-C 退出。
"""

import sys
import signal
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class PositionMonitor(Node):
    def __init__(self):
        super().__init__('position_monitor')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE, depth=10)
        self.sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._cb, qos)
        self._ref = None
        self._count = 0
        self.get_logger().info("=== NED 坐标实时监视器 ===")
        self.get_logger().info("格式: t[s] | x(m) | y(m) | z(m) | Δx | Δy | Δz | vx | vy | vz")
        self.get_logger().info("按 Ctrl-C 退出")
        print(f"{'t':>6s} | {'x(N)':>8s} | {'y(E)':>8s} | {'z(D)':>8s} | "
              f"{'Δx':>7s} | {'Δy':>7s} | {'Δz':>7s} | "
              f"{'vx':>7s} | {'vy':>7s} | {'vz':>7s}", flush=True)
        print("-" * 118, flush=True)

    def _cb(self, m):
        x, y, z = m.x, m.y, m.z
        vx, vy, vz = m.vx, m.vy, m.vz

        if self._ref is None:
            self._ref = (x, y, z)

        rx, ry, rz = self._ref
        dx, dy, dz = x - rx, y - ry, z - rz

        t = self._count * 0.05  # vehicle_local_position 频率 ~50Hz, 但只打印部分
        self._count += 1

        # 打印间隔 (~2Hz, 每 25 帧打印一次)
        if self._count % 25 == 0:
            print(f"{t:6.1f} | {x:8.3f} | {y:8.3f} | {z:8.3f} | "
                  f"{dx:+7.3f} | {dy:+7.3f} | {dz:+7.3f} | "
                  f"{vx:+7.3f} | {vy:+7.3f} | {vz:+7.3f}", flush=True)


def main():
    rclpy.init(args=sys.argv)
    node = PositionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
