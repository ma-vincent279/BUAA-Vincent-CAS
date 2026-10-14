#!/usr/bin/env python3
"""
随机桶位置测试 — 桶在飞机前方 10m 处以 2m 为半径随机摆放，多次测试投放精度。
每次迭代完整重启仿真，收集落点偏差统计。
用法: python3 ~/SITL/scripts/test_drop_random.py [--count N] [--radius R]
"""

import subprocess
import random
import math
import time
import os
import sys
import re
import signal
import argparse
from datetime import datetime

SITL_DIR = "/home/vincent/SITL"
PX4_DIR = "/home/vincent/PX4-Autopilot"
PX4_BUILD = f"{PX4_DIR}/build/px4_sitl_default"
PX4_ROOTFS = f"{PX4_BUILD}/rootfs"
WORLD = f"{SITL_DIR}/worlds/drop_opt.world"

# 标准桶位置 Gazebo 坐标 (对应 NED 0,10)
BUCKET_GZ_X0 = 11.01
BUCKET_GZ_Y0 = 0.98
BUCKET_GZ_Z0 = 0.25
SPAWN_GZ_X = 1.01
SPAWN_GZ_Y = 0.98
SPAWN_GZ_Z = 0.83


def rand_bucket(radius=2.0):
    """在标准 NED(0,10) 周围 radius 米内随机生成桶位置, 返回 (gz_x, gz_y, ned_x, ned_y)."""
    angle = random.uniform(0, 2 * math.pi)
    r = math.sqrt(random.uniform(0, 1)) * radius
    ned_dx = r * math.cos(angle)
    ned_dy = r * math.sin(angle)
    ned_x = 0.0 + ned_dx
    ned_y = 10.0 + ned_dy
    gz_x = SPAWN_GZ_X + ned_y   # Gazebo x = 东 = NED y + spawn
    gz_y = SPAWN_GZ_Y + ned_x   # Gazebo y = 北 = NED x + spawn
    return gz_x, gz_y, ned_x, ned_y


def cleanup():
    """杀死所有残留仿真进程."""
    for port in [11345, 8888]:
        subprocess.run(
            f"for pid in $(lsof -ti:{port} 2>/dev/null); do kill -9 $pid 2>/dev/null; done",
            shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for p in ["px4", "gzserver", "gzclient"]:
        subprocess.run(["pkill", "-9", "-x", p], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for pat in ["MicroXRCEAgent", "sitl_run", "make px4_sitl", "drone_drl"]:
        subprocess.run(["pkill", "-9", "-f", pat], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for f in ["/tmp/px4_lock-0", "/tmp/px4-sock-0"]:
        subprocess.run(["rm", "-f", f], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)


def reset_px4_state():
    """清除 PX4 持久化状态."""
    for f in [f"{PX4_BUILD}/dataman", f"{PX4_ROOTFS}/dataman",
              f"{PX4_ROOTFS}/parameters.bson", f"{PX4_ROOTFS}/parameters_backup.bson"]:
        subprocess.run(["rm", "-f", f], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["rm", "-rf", f"{PX4_ROOTFS}/eeprom"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    # 清理旧日志
    log_dir = f"{PX4_ROOTFS}/log"
    if os.path.isdir(log_dir):
        subprocess.run(["rm", "-rf", f"{log_dir}/*/"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def build():
    """编译 drone_drl 包."""
    result = subprocess.run(
        "source /opt/ros/humble/setup.bash && cd ~/SITL && colcon build --packages-select drone_drl 2>&1 | tail -3",
        shell=True, capture_output=True, text=True, executable='/bin/bash')
    return "Finished" in result.stdout


def run_one_test(gz_x, gz_y, ned_x, ned_y, test_id, timeout=180):
    """运行一次完整仿真测试, 返回 (drop_ned_x, drop_ned_y, drop_ned_z) 或 None."""
    log_file = f"/tmp/drop_random_{test_id}.log"
    print(f"\n{'='*60}")
    print(f"[Test {test_id}] bucket Gazebo({gz_x:.2f},{gz_y:.2f}) NED({ned_x:.2f},{ned_y:.2f}) "
          f"offset=({ned_x:.2f},{ned_y-10:.2f})m")
    print(f"{'='*60}")

    # 1. 清理 + 重置
    cleanup()
    reset_px4_state()

    # 2. 编译
    if not build():
        print(f"[Test {test_id}] BUILD FAILED, skipping")
        return None

    # 3. 启动 Agent
    agent_proc = subprocess.Popen(
        ["MicroXRCEAgent", "udp4", "-p", "8888"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)

    # 4. 启动 PX4 + Gazebo
    env = os.environ.copy()
    env["GAZEBO_MODEL_DATABASE_URI"] = ""
    env["PX4_PARAM_CBRK_SUPPLY_CHK"] = "894281"
    env["PX4_PARAM_CBRK_IO_SAFETY"] = "22027"
    env["PX4_PARAM_CBRK_USB_CHK"] = "197848"
    env["PX4_PARAM_COM_OF_LOSS_T"] = "10.0"
    env["PX4_PARAM_COM_LOW_BAT_ACT"] = "0"
    env["PX4_SITL_WORLD"] = WORLD

    px4_proc = subprocess.Popen(
        ["make", "px4_sitl", "gazebo-classic_iris_mono_cam_down"],
        cwd=PX4_DIR, env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f"  Waiting for PX4+Gazebo init (30s)...")
    time.sleep(30)

    # 检查关键进程
    if subprocess.run(["pgrep", "-x", "gzserver"], stdout=subprocess.DEVNULL).returncode != 0:
        print(f"[Test {test_id}] gzserver failed to start")
        kill_all(agent_proc, px4_proc)
        return None
    if subprocess.run(["pgrep", "-x", "px4"], stdout=subprocess.DEVNULL).returncode != 0:
        print(f"[Test {test_id}] px4 failed to start")
        kill_all(agent_proc, px4_proc)
        return None

    # 5. 等待 DDS
    print(f"  Waiting for DDS...")
    dds_ready = False
    for _ in range(30):
        result = subprocess.run(
            "source /opt/ros/humble/setup.bash && source ~/SITL/install/setup.bash && "
            "ros2 topic list 2>/dev/null | grep -q /fmu/out/vehicle_status_v4",
            shell=True, executable='/bin/bash')
        if result.returncode == 0:
            dds_ready = True
            break
        time.sleep(1)
    if not dds_ready:
        print(f"[Test {test_id}] DDS timeout")
        kill_all(agent_proc, px4_proc)
        return None
    print(f"  DDS ready")

    # 6. 启动辅助节点
    source_cmd = "source /opt/ros/humble/setup.bash && source ~/SITL/install/setup.bash"
    detector_proc = subprocess.Popen(
        f"{source_cmd} && ros2 run drone_drl bucket_vision_detector --ros-args "
        f"-p bucket1_gz_x:={gz_x} -p bucket1_gz_y:={gz_y} -p bucket1_gz_z:={BUCKET_GZ_Z0} "
        f"-p bucket2_gz_x:={gz_x} -p bucket2_gz_y:={gz_y} -p bucket2_gz_z:={BUCKET_GZ_Z0}",
        shell=True, executable='/bin/bash',
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1)

    # 7. 启动任务
    print(f"  Starting mission...")
    with open(log_file, 'w') as lf:
        mission_proc = subprocess.Popen(
            f"{source_cmd} && ros2 run drone_drl drop_opt_mission",
            shell=True, executable='/bin/bash',
            stdout=lf, stderr=subprocess.STDOUT)

        # 等待 DROP 事件或超时
        drop_pos = None
        start = time.time()
        while time.time() - start < timeout:
            if mission_proc.poll() is not None:
                break
            try:
                with open(log_file, 'r') as f:
                    content = f.read()
                # 匹配 DROP 位置
                m = re.search(r'>>> DROP!.*?at NED\(([\d.-]+),([\d.-]+),([\d.-]+)\)', content)
                if m:
                    drop_pos = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
                    print(f"  DROP detected at NED({drop_pos[0]:.3f},{drop_pos[1]:.3f},{drop_pos[2]:.3f})")
                    break
            except Exception:
                pass
            time.sleep(0.5)

        # 等待任务自行结束 (CLIMB → RETURN → LAND) 或再等 60s
        print(f"  Waiting for mission to finish...")
        try:
            mission_proc.wait(timeout=60)
        except subprocess.TimeoutExpired:
            print(f"  Mission timeout, killing...")
            mission_proc.kill()

    # 8. 清理
    kill_all(agent_proc, px4_proc, detector_proc)
    cleanup()
    reset_px4_state()

    if drop_pos is None:
        print(f"[Test {test_id}] NO DROP detected!")
        return None

    # 计算偏差
    dist = math.hypot(drop_pos[0] - ned_x, drop_pos[1] - ned_y)
    print(f"[Test {test_id}] RESULT: drop=({drop_pos[0]:.3f},{drop_pos[1]:.3f}) "
          f"bucket=({ned_x:.3f},{ned_y:.3f}) dist={dist:.3f}m")
    return drop_pos


def kill_all(*procs):
    for p in procs:
        try:
            p.kill()
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(description="Random bucket drop test")
    parser.add_argument("--count", type=int, default=6, help="Number of test iterations")
    parser.add_argument("--radius", type=float, default=2.0, help="Random radius in meters")
    args = parser.parse_args()

    print(f"=== Random Bucket Drop Test ===")
    print(f"  iterations: {args.count}")
    print(f"  radius: {args.radius}m")
    print(f"  start: {datetime.now().strftime('%H:%M:%S')}")
    sys.stdout.flush()

    results = []
    for i in range(args.count):
        gz_x, gz_y, ned_x, ned_y = rand_bucket(args.radius)
        drop = run_one_test(gz_x, gz_y, ned_x, ned_y, i + 1, timeout=180)
        if drop:
            dist = math.hypot(drop[0] - ned_x, drop[1] - ned_y)
            results.append({
                'test': i + 1,
                'bucket_ned': (ned_x, ned_y),
                'drop_ned': (drop[0], drop[1]),
                'dist': dist,
            })
        print(f"  Progress: {len(results)}/{i+1} completed")
        sys.stdout.flush()

    # ============================================================
    # 汇总
    # ============================================================
    print(f"\n{'='*60}")
    print(f"=== SUMMARY ===")
    print(f"{'='*60}")
    if not results:
        print("No successful tests!")
        return

    print(f"{'Test':<6} {'Bucket NED':<18} {'Drop NED':<18} {'Dist(m)':<10}")
    print("-" * 52)
    dists = []
    for r in results:
        bx, by = r['bucket_ned']
        dx, dy = r['drop_ned']
        d = r['dist']
        dists.append(d)
        print(f"#{r['test']:<5} ({bx:6.2f},{by:6.2f})      ({dx:6.2f},{dy:6.2f})      {d:.3f}")

    dists.sort()
    mean_d = sum(dists) / len(dists)
    median_d = dists[len(dists) // 2]
    print(f"\n  Count:     {len(results)}")
    print(f"  Mean:      {mean_d:.3f}m")
    print(f"  Median:    {median_d:.3f}m")
    print(f"  Min:       {dists[0]:.3f}m")
    print(f"  Max:       {dists[-1]:.3f}m")
    print(f"  ≤0.30m:    {sum(1 for d in dists if d <= 0.30)}/{len(dists)} ({100*sum(1 for d in dists if d <= 0.30)/len(dists):.0f}%)")
    print(f"  ≤0.50m:    {sum(1 for d in dists if d <= 0.50)}/{len(dists)} ({100*sum(1 for d in dists if d <= 0.50)/len(dists):.0f}%)")
    print(f"  ≤0.75m:    {sum(1 for d in dists if d <= 0.75)}/{len(dists)} ({100*sum(1 for d in dists if d <= 0.75)/len(dists):.0f}%)")
    print(f"  ≤1.00m:    {sum(1 for d in dists if d <= 1.00)}/{len(dists)} ({100*sum(1 for d in dists if d <= 1.00)/len(dists):.0f}%)")
    print(f"\n  end: {datetime.now().strftime('%H:%M:%S')}")


if __name__ == '__main__':
    main()
