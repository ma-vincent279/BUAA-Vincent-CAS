#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition, OffboardControlMode, TrajectorySetpoint, VehicleCommand

class Px4Bridge(Node):
    def __init__(self):
        super().__init__('px4_drl_bridge')
        
        # 为 PX4 传感器数据配置 Best Effort QoS
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # 订阅 PX4 本地位置（话题名以实际 ros2 topic list 为准，可能需要修改）
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',   # ⚠️ 如果实际为 vehicle_local_position_v1，需要改这里
            self.local_pos_callback,
            qos_profile
        )
        
        # 预创建 Offboard 控制发布器（后续 DRL 会用到）
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.traj_sp_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        
        # 控制定时器（占位用，未来在这里写 DRL 控制逻辑）
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Px4Bridge (Python) node started, listening to /fmu/out/vehicle_local_position')
    
    def local_pos_callback(self, msg):
        """接收 PX4 位置信息并打印"""
        self.get_logger().info(
            f'Position: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}',
            throttle_duration_sec=0.5   # 每0.5秒打印一次，不会刷屏
        )
        # TODO: 将位置信息存入状态向量，供 DRL 使用
    
    def control_loop(self):
        """控制循环（未来发布 Offboard 指令）"""
        # 这里暂时留空，后续可以填充 Offboard 控制逻辑
        pass

def main(args=None):
    rclpy.init(args=args)
    node = Px4Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
