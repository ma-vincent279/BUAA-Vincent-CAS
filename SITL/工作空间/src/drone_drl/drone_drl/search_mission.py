#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import (
    TrajectorySetpoint, OffboardControlMode, VehicleCommand,
    VehicleStatus, VehicleLocalPosition, VehicleLandDetected
)
from geometry_msgs.msg import Point
import math

class SearchMission(Node):
    def __init__(self):
        super().__init__('search_mission')
        qos = qos_profile_sensor_data

        # ---------- 发布者 ----------
        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.drop_pub = self.create_publisher(Point, '/drop_water', 10)

        # ---------- 订阅者 ----------
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v4', self.status_cb, qos)
        self.local_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.local_cb, qos)
        self.land_sub = self.create_subscription(VehicleLandDetected, '/fmu/out/vehicle_land_detected', self.land_cb, qos)

        # ---------- 内部状态 ----------
        self.status = None
        self.x = self.y = self.z = 0.0
        self.home_x = self.home_y = self.home_z = None
        self.heading = None          # 锁定的初始航向
        self.landed = False

        # 桶位置（固定）
        self.bucket_x = 5.0
        self.bucket_y = 1.0
        self.bucket_z = 0.5

        # 任务阶段
        self.step = 0
        self.dropped = False
        self.mode_req = self.arm_req = False
        self.forward_dist = 10.0
        self.search_speed = 2.0
        self.takeoff_height = 3.0
        self.drop_altitude_above = 0.5    # 投弹高度（桶上方0.5m）
        self.hover_altitude_above = 3.0   # 悬停高度（桶上方3m）

        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info('Search & drop mission started (bucket at 5,1)')

    # ---------- 回调 ----------
    def status_cb(self, msg): self.status = msg
    def land_cb(self, msg): self.landed = msg.landed

    def local_cb(self, msg):
        self.x, self.y, self.z = msg.x, msg.y, msg.z
        if self.home_x is None:
            self.home_x, self.home_y, self.home_z = self.x, self.y, self.z
            self.heading = msg.heading
            self.get_logger().info(
                f'Home: ({self.home_x:.2f}, {self.home_y:.2f}, {self.home_z:.2f}), '
                f'Initial heading: {math.degrees(self.heading):.1f}°'
            )

    # ---------- 辅助函数 ----------
    def pub_pos(self, x, y, z, yaw=None):
        """发布位置设定点，若不指定yaw则锁定为初始航向"""
        off_mode = OffboardControlMode()
        off_mode.timestamp = self.get_clock().now().nanoseconds // 1000
        off_mode.position = True

        traj = TrajectorySetpoint()
        traj.timestamp = self.get_clock().now().nanoseconds // 1000
        traj.position = [float(x), float(y), float(z)]
        # 默认锁定初始航向，除非明确传入None
        if yaw is None:
            yaw = self.heading
        traj.yaw = float(yaw)
        traj.velocity = [float('nan')] * 3
        traj.acceleration = [float('nan')] * 3

        self.offboard_pub.publish(off_mode)
        self.traj_pub.publish(traj)

    def send_cmd(self, cmd_id, **params):
        cmd = VehicleCommand()
        cmd.timestamp = self.get_clock().now().nanoseconds // 1000
        cmd.command = cmd_id
        cmd.target_system = 1
        cmd.target_component = 1
        for k, v in params.items():
            setattr(cmd, k, v)
        self.cmd_pub.publish(cmd)

    def dist_to(self, x, y, z):
        return math.sqrt((self.x - x)**2 + (self.y - y)**2 + (self.z - z)**2)

    # ---------- 主循环 ----------
    def loop(self):
        if self.home_x is None:
            return

        # 1. 解锁与模式切换
        if self.status and self.status.arming_state != 2:  # 未解锁
            if self.status.nav_state != 14 and not self.mode_req:  # 14 = OFFBOARD
                self.mode_req = True
                self.get_logger().info("Requesting Offboard mode...")
                self.send_cmd(176, param1=1.0, param2=6.0)  # VEHICLE_CMD_DO_SET_MODE
            elif self.status.nav_state == 14 and not self.arm_req:
                self.arm_req = True
                self.get_logger().info("Sending arm command...")
                self.send_cmd(400, param1=1.0)  # COMPONENT_ARM_DISARM
            else:
                # 保持当前点，锁定航向
                self.pub_pos(self.x, self.y, self.z)
            return

        if not self.status or self.status.arming_state != 2:
            return

        # 飞行高度
        target_z = self.home_z - self.takeoff_height  # NED, 上升z变小

        # 2. 阶段 0：起飞
        if self.step == 0:
            self.pub_pos(self.home_x, self.home_y, target_z)
            if self.z <= target_z + 0.1:
                self.step = 1
                self.get_logger().info("Takeoff done, heading to bucket...")

        # 3. 阶段 1：飞向桶正上方 (悬停点)
        elif self.step == 1:
            hover_x, hover_y = self.bucket_x, self.bucket_y
            hover_z = self.bucket_z - self.hover_altitude_above
            self.pub_pos(hover_x, hover_y, hover_z)
            if self.dist_to(hover_x, hover_y, hover_z) < 0.5:
                self.step = 2
                self.hover_start = self.get_clock().now()
                self.get_logger().info("Above bucket, hovering 1 sec...")

        # 4. 阶段 2：悬停 1 秒
        elif self.step == 2:
            self.pub_pos(self.bucket_x, self.bucket_y, self.bucket_z - self.hover_altitude_above)
            if (self.get_clock().now() - self.hover_start).nanoseconds > 1.0e9:
                self.step = 3

        # 5. 阶段 3：下降并投弹
        elif self.step == 3:
            drop_x, drop_y = self.bucket_x, self.bucket_y
            drop_z = self.bucket_z - self.drop_altitude_above
            self.pub_pos(drop_x, drop_y, drop_z)
            if self.dist_to(drop_x, drop_y, drop_z) < 0.3 and not self.dropped:
                self.get_logger().info("Releasing water...")
                self.drop_pub.publish(Point(x=drop_x, y=drop_y, z=drop_z - 0.2))
                self.dropped = True
                self.step = 4

        # 6. 阶段 4：上升回悬停高度
        elif self.step == 4:
            rise_x, rise_y = self.bucket_x, self.bucket_y
            rise_z = self.bucket_z - self.hover_altitude_above
            self.pub_pos(rise_x, rise_y, rise_z)
            if self.dist_to(rise_x, rise_y, rise_z) < 0.3:
                self.step = 5
                self.forward_start = self.get_clock().now()
                self.dist_before_drop = math.sqrt((self.x - self.home_x)**2 + (self.y - self.home_y)**2)
                self.get_logger().info("Rise complete, continuing forward...")

        # 7. 阶段 5：继续前飞 10 米
        elif self.step == 5:
            elapsed = (self.get_clock().now() - self.forward_start).nanoseconds * 1e-9
            dist_from_home = self.dist_before_drop + elapsed * self.search_speed
            if dist_from_home < self.forward_dist:
                # 沿初始航向飞行
                tgt_x = self.home_x + dist_from_home * math.cos(self.heading)
                tgt_y = self.home_y + dist_from_home * math.sin(self.heading)
                self.pub_pos(tgt_x, tgt_y, target_z)
            else:
                self.step = 6
                self.hover2_start = self.get_clock().now()
                self.get_logger().info("Forward flight done, hovering...")

        # 8. 阶段 6：悬停 2 秒
        elif self.step == 6:
            self.pub_pos(self.x, self.y, target_z)
            if (self.get_clock().now() - self.hover2_start).nanoseconds > 2.0e9:
                self.step = 7

        # 9. 阶段 7：降落
        elif self.step == 7:
            if not self.landed:
                if not hasattr(self, 'land_cmd_sent'):
                    self.send_cmd(21)  # NAV_LAND
                    self.land_cmd_sent = True
                    self.get_logger().info("Land command sent")
                self.pub_pos(self.x, self.y, self.home_z)
            else:
                self.send_cmd(400, param1=0.0)  # 上锁
                self.timer.cancel()
                self.get_logger().info("Mission complete, disarmed.")

def main():
    rclpy.init()
    node = SearchMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()