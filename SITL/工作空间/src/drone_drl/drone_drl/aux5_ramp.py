#!/usr/bin/env python3
"""AUX5 PWM 舵机往复测试 — 解锁后 AUX5 在 1000~2000us 间持续往复。

通过 /fmu/in/manual_control_input 发布 ManualControlSetpoint.aux5，
走 RC 输入 → 混控器 → AUX5 引脚路径 (200Hz, 完整通道, SOURCE_RC)。

必须的 PX4 参数配置:
  PWM_AUX_FUNC5 = RC AUX 5     (AUX5 输出功能)
  RC_MAP_AUX5   = Unassigned   (解除 RC 接收机对 aux5 的占用, 让 DDS 数据独占)
"""

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, VehicleStatus, ManualControlSetpoint
from drone_drl.common import QOS_BE, status_topic, send_arm
ARM_DELAY = 2.0
TIMER_PERIOD = 0.005  # 200Hz, 必须快于 rc_update (SBUS~111Hz) 防止被覆盖
OSC_PERIOD = 2.0      # 一个完整往复周期 (s)
PRINT_INTERVAL = 100  # 每 100 次循环打一次 (0.5s)
VAL_MIN = -1.0        # aux5 最小值 → PWM 900us
VAL_MAX = 1.0         # aux5 最大值 → PWM 2100us


class Aux5Ramp(Node):
    def __init__(self):
        super().__init__("aux5_ramp")
        self.status_sub = self.create_subscription(
            VehicleStatus, status_topic(), self._cb_status, QOS_BE)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.manual_pub = self.create_publisher(
            ManualControlSetpoint, "/fmu/in/manual_control_input", 10)

        self.armed = False
        self._start_wall = self.get_clock().now()
        self._last_arm_try = -10.0
        self._osc_start = None
        self._osc_step = 0
        self.get_logger().info("等待解锁...")

    def _cb_status(self, msg: VehicleStatus):
        self.armed = (msg.arming_state == 2)

    def _elapsed(self, t):
        return (self.get_clock().now() - t).nanoseconds / 1e9

    def _triangle_wave(self, t, period, lo, hi):
        phase = (t % period) / period
        if phase < 0.5:
            return lo + (hi - lo) * phase / 0.5
        else:
            return hi - (hi - lo) * (phase - 0.5) / 0.5

    def _loop(self):
        if self._elapsed(self._start_wall) < ARM_DELAY:
            return

        # === 阶段 1: 解锁 ===
        if not self.armed:
            now = self._elapsed(self._start_wall)
            if now - self._last_arm_try > 1.0:
                send_arm(self.cmd_pub, self)
                self._last_arm_try = now
                self.get_logger().info("Arming...")
            return

        # === 阶段 2: 解锁完成，AUX5 往复 ===
        if self._osc_start is None:
            self._osc_start = self.get_clock().now()
            self.get_logger().info(
                f"已解锁! AUX5 往复, 周期 {OSC_PERIOD:.1f}s "
                f"(ManualControlSetpoint.aux5, VAL_MIN={VAL_MIN}, VAL_MAX={VAL_MAX})")

        t = self._elapsed(self._osc_start)
        aux5 = self._triangle_wave(t, OSC_PERIOD, VAL_MIN, VAL_MAX)

        msg = ManualControlSetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.timestamp_sample = msg.timestamp
        msg.valid = True
        msg.data_source = ManualControlSetpoint.SOURCE_RC
        # 所有通道填 0.0 (中位), PX4 不会丢弃完整消息, AUTO_LOITER 下不影响飞行
        msg.roll = 0.0
        msg.pitch = 0.0
        msg.yaw = 0.0
        msg.throttle = -1.0   # 最低油门
        msg.flaps = 0.0
        msg.aux1 = 0.0
        msg.aux2 = 0.0
        msg.aux3 = 0.0
        msg.aux4 = 0.0
        msg.aux5 = aux5       # 只有 aux5 往复
        msg.aux6 = 0.0
        msg.sticks_moving = True
        msg.buttons = 0

        self.manual_pub.publish(msg)

        if self._osc_step % PRINT_INTERVAL == 0:
            direction = "↗" if (t % OSC_PERIOD) < OSC_PERIOD / 2 else "↘"
            self.get_logger().info(f"  aux5={aux5:+.3f} {direction}  t={t:.1f}s")
        self._osc_step += 1

    def run(self):
        self.create_timer(TIMER_PERIOD, self._loop)
        rclpy.spin(self)


def main():
    rclpy.init()
    try:
        Aux5Ramp().run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
