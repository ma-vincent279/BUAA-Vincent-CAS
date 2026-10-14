"""
drone_drl 共享工具模块。

集中管理所有节点重复出现的模式：QoS 配置、PX4 话题名、解锁检测、ARM/OFFBOARD 指令。
sim->实机迁移时，只需修改本文件的 PX4_STATUS_SUFFIX 常量。

fly/ 依据:
  - is_armed() 双源检测: fly/test.py:207,218 (VehicleControlMode.flag_armed 优先)
  - status_topic() _v1: fly/CLAUDE.md line 107
"""

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand

# ============================================================
# QoS
# ============================================================

QOS_BE = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)

# ============================================================
# PX4 话题后缀
#   SITL PX4 固件: vehicle_status_v4
#   实机 PX4 固件: vehicle_status_v1 (fly/ 已验证)
#   vehicle_local_position_v1 两地相同
# ============================================================

PX4_STATUS_SUFFIX = "_v4"
PX4_LOCAL_POS_SUFFIX = "_v1"


def status_topic() -> str:
    """返回 vehicle_status 话题名，含版本后缀。"""
    return f"/fmu/out/vehicle_status{PX4_STATUS_SUFFIX}"


def local_pos_topic() -> str:
    """返回 vehicle_local_position 话题名，含版本后缀。"""
    return f"/fmu/out/vehicle_local_position{PX4_LOCAL_POS_SUFFIX}"


def land_detected_topic() -> str:
    """返回 vehicle_land_detected 话题名。"""
    return "/fmu/out/vehicle_land_detected"


def control_mode_topic() -> str:
    """返回 vehicle_control_mode 话题名（无后缀）。"""
    return "/fmu/out/vehicle_control_mode"


# ============================================================
# 解锁检测 — 双源模式
#   实机 vehicle_status_v1 可能 DDS checksum 不匹配 (fly/ Known Issue #3)
#   VehicleControlMode.flag_armed 更可靠，作为主要来源
# ============================================================

def is_armed(status_msg=None, control_mode_msg=None) -> bool:
    """双源解锁检测。优先 VehicleControlMode.flag_armed，回退 VehicleStatus.arming_state。"""
    if control_mode_msg is not None:
        return bool(control_mode_msg.flag_armed)
    if status_msg is not None:
        return status_msg.arming_state == 2
    return False


# ============================================================
# 指令构造
# ============================================================

def make_cmd(node, command: int, **params) -> VehicleCommand:
    """构造带标准填充字段的 VehicleCommand。"""
    return VehicleCommand(
        timestamp=node.get_clock().now().nanoseconds // 1000,
        command=command,
        target_system=1,
        target_component=1,
        source_system=1,
        source_component=1,
        from_external=True,
        **params,
    )


def send_arm(cmd_pub, node):
    """发送解锁指令 MAV_CMD_COMPONENT_ARM_DISARM。"""
    cmd_pub.publish(make_cmd(node, 400, param1=1.0, param2=2989.0))


def send_offboard(cmd_pub, node):
    """发送 OFFBOARD 模式切换指令 MAV_CMD_DO_SET_MODE。"""
    cmd_pub.publish(make_cmd(node, 176, param1=1.0, param2=6.0))


def send_land(cmd_pub, node):
    """发送自主着陆指令 MAV_CMD_NAV_LAND。"""
    cmd_pub.publish(make_cmd(node, 21))


def send_disarm(cmd_pub, node):
    """发送上锁指令。"""
    cmd_pub.publish(make_cmd(node, 400, param1=0.0))
