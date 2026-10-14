#!/usr/bin/env python3
"""
投放精度自动优化器。
自动运行仿真 → 提取结果 → 分析日志 → 调参 → 重复。
停止条件: 连续 3 次得分 ≥ 8 分 或 迭代 ≥ 20 轮。
"""

import subprocess
import re
import sys
import os
import time
import json
from datetime import datetime

MISSION_FILE = "/home/vincent/SITL/src/drone_drl/drone_drl/drop_opt_mission.py"
LOG_FILE = "/tmp/drop_opt_output.log"
TEST_SCRIPT = "/home/vincent/SITL/scripts/test_drop.sh"

# 可调参数及其范围
PARAM_RANGES = {
    'KP_H':         (0.0005, 0.02,   'PID Kp (m/s per pixel)'),
    'KD_H':         (0.0001, 0.01,   'PID Kd'),
    'DROP_HEIGHT':  (1.0,    2.8,    '投放高度 (m)'),
    'DESCEND_SPEED':(0.2,    1.0,    '下降速度 (m/s)'),
    'TRACK_ARRIVE_PX': (15.0, 80.0,  'TRACK 像素收敛阈值'),
    'HOVER_ARRIVE_PX': (8.0,  40.0,  'HOVER 像素收敛阈值'),
    'STABILIZE_TIME':  (1.0,  5.0,   '稳定时间 (s)'),
    'MAX_HORIZ_VEL':   (0.4,  2.0,   '最大水平速度 (m/s)'),
}

INITIAL_PARAMS = {
    'KP_H': 0.004, 'KD_H': 0.002, 'DROP_HEIGHT': 2.0,
    'DESCEND_SPEED': 0.5, 'TRACK_ARRIVE_PX': 40.0,
    'HOVER_ARRIVE_PX': 20.0, 'STABILIZE_TIME': 2.0,
    'MAX_HORIZ_VEL': 1.5,
}


def read_current_params():
    """从 mission 文件中读取当前参数值"""
    params = {}
    with open(MISSION_FILE, 'r') as f:
        content = f.read()
    for name in INITIAL_PARAMS:
        m = re.search(rf'^{name}\s*=\s*([\d.]+)', content, re.MULTILINE)
        if m:
            params[name] = float(m.group(1))
        else:
            params[name] = INITIAL_PARAMS[name]
    return params


def write_params(params):
    """将参数写入 mission 文件"""
    with open(MISSION_FILE, 'r') as f:
        content = f.read()

    for name, val in params.items():
        if name == 'DROP_HEIGHT':
            val_str = f"{val:.1f}"
        elif name in ('TRACK_ARRIVE_PX', 'HOVER_ARRIVE_PX'):
            val_str = f"{val:.1f}"
        elif name == 'STABILIZE_TIME':
            val_str = f"{val:.1f}"
        elif name == 'MAX_HORIZ_VEL':
            val_str = f"{val:.1f}"
        else:
            val_str = f"{val:.6f}".rstrip('0').rstrip('.')
            if val_str == '':
                val_str = '0'
        content = re.sub(
            rf'^{name}\s*=\s*[\d.]+',
            f'{name} = {val_str}',
            content, flags=re.MULTILINE)
    with open(MISSION_FILE, 'w') as f:
        f.write(content)


def run_simulation(timeout=300):
    """运行一次仿真，返回日志内容"""
    print(f"\n{'='*60}")
    print(f"  启动仿真...")
    print(f"{'='*60}")
    sys.stdout.flush()

    try:
        result = subprocess.run(
            ['bash', TEST_SCRIPT],
            capture_output=True, text=True, timeout=timeout,
            cwd='/home/vincent/SITL'
        )
        output = result.stdout + result.stderr
        # 也读取日志文件
        if os.path.exists(LOG_FILE):
            with open(LOG_FILE, 'r') as f:
                output += f.read()
        return output
    except subprocess.TimeoutExpired:
        print("  WARNING: 仿真超时，强制终止...")
        # 清理
        subprocess.run(['pkill', '-9', '-f', 'gzserver'], capture_output=True)
        subprocess.run(['pkill', '-9', '-f', 'px4'], capture_output=True)
        subprocess.run(['pkill', '-9', '-f', 'MicroXRCEAgent'], capture_output=True)
        time.sleep(3)
        if os.path.exists(LOG_FILE):
            with open(LOG_FILE, 'r') as f:
                return f.read()
        return ""


def extract_result(output):
    """从输出中提取距离和得分"""
    # 找裁判输出: "距离: X.XXXXm, 得分: X"
    patterns = [
        r'距离:\s*([\d.]+)m,\s*得分:\s*(\d+)',
        r'dist=([\d.]+)m?\s*→.*?(\d+)分',
        r'距离:\s*([\d.]+)m',
    ]
    for pat in patterns:
        m = re.search(pat, output)
        if m:
            dist = float(m.group(1))
            score = int(m.group(2)) if m.lastindex >= 2 else score_from_dist(dist)
            return dist, score
    return None, None


def score_from_dist(d):
    if d <= 0.05: return 10
    elif d <= 0.15: return 8
    elif d <= 0.30: return 6
    elif d <= 0.50: return 4
    elif d <= 0.75: return 2
    elif d <= 1.00: return 1
    else: return 0


def analyze_convergence(output):
    """分析日志中的收敛曲线，返回诊断信息"""
    issues = []

    # 提取 TRACK/DESCEND/HOVER 中的 px_err
    track_errs = []
    for m in re.finditer(r'\[TRACK\]\s+px_err=\(([-\d.]+),([-\d.]+)\)\s+total=([\d.]+)px', output):
        track_errs.append(float(m.group(3)))

    descend_errs = []
    for m in re.finditer(r'\[DESCEND\]\s+px_err=\(([-\d.]+),([-\d.]+)\)\s+total=([\d.]+)px', output):
        descend_errs.append(float(m.group(3)))

    hover_errs = []
    for m in re.finditer(r'\[HOVER\]\s+px_err=\(([-\d.]+),([-\d.]+)\)\s+total=([\d.]+)px', output):
        hover_errs.append(float(m.group(3)))

    # 诊断 1: 收敛速度
    if len(track_errs) >= 5:
        first_5 = track_errs[:5]
        if all(e > 100 for e in first_5):
            issues.append("slow_converge")
        if len(track_errs) >= 10:
            mid = len(track_errs) // 2
            if max(track_errs[:mid]) > 200 and max(track_errs[mid:]) > 100:
                issues.append("oscillation")

    # 诊断 2: 下降时像素偏差增大
    if len(track_errs) >= 3 and len(descend_errs) >= 3:
        avg_track_end = sum(track_errs[-3:]) / 3
        avg_desc = sum(descend_errs) / len(descend_errs)
        if avg_desc > avg_track_end * 1.5:
            issues.append("descend_divergence")

    # 诊断 3: HOVER 阶段无法收敛到阈值
    if len(hover_errs) >= 5:
        if all(e > 20 for e in hover_errs):
            issues.append("hover_stuck")

    # 诊断 4: 收敛很快但投不准
    if len(track_errs) >= 5 and track_errs[-1] < 20:
        issues.append("converged_but_inaccurate")

    return issues


def suggest_params(current_params, issues, dist, score, iteration):
    """根据诊断结果建议新参数"""
    params = dict(current_params)
    print(f"\n  --- 迭代 {iteration} 诊断 ---")
    print(f"  当前参数: KP={params['KP_H']:.5f} KD={params['KD_H']:.5f} "
          f"DROP={params['DROP_HEIGHT']:.1f}m DESC_SPD={params['DESCEND_SPEED']:.2f}m/s "
          f"HOVER_PX={params['HOVER_ARRIVE_PX']:.0f} STAB={params['STABILIZE_TIME']:.1f}s")

    if issues:
        for iss in issues:
            print(f"  问题: {iss}")
    else:
        print(f"  无明显问题")

    # === 调参策略 ===
    # 基础扰动幅度随迭代递减
    base_factor = max(0.3, 1.0 - iteration * 0.05)

    if score is not None and score >= 8:
        # 已很好，微调
        print("  >> 得分 ≥8，微调参数")
        if dist is not None and dist < 0.05:
            return params  # 已完美
        # 微调: 降低投放高度、减小 KP
        if params['DROP_HEIGHT'] > 1.2:
            params['DROP_HEIGHT'] = max(1.0, params['DROP_HEIGHT'] - 0.1)
        params['KP_H'] *= 0.95
        params['KD_H'] *= 0.95
        params['STABILIZE_TIME'] = min(5.0, params['STABILIZE_TIME'] + 0.3)
        return params

    # 根据问题类型调参
    changes = []
    factor = base_factor * (1.0 + 0.2 * iteration)  # 后期加大变动

    if 'oscillation' in issues or 'descend_divergence' in issues:
        # 振荡或下降发散 → 降低增益
        factor = min(0.5, factor)
        params['KP_H'] *= (1.0 - factor)
        params['KD_H'] *= (1.0 - factor)
        changes.append(f"KP↓, KD↓ (factor={factor:.2f})")
    elif 'slow_converge' in issues:
        # 收敛太慢 → 增大 KP
        params['KP_H'] = min(PARAM_RANGES['KP_H'][1], params['KP_H'] * (1.0 + factor))
        changes.append(f"KP↑ (factor={factor:.2f})")
    elif 'hover_stuck' in issues:
        # 低空无法收敛 → 放宽阈值或降低高度
        if params['HOVER_ARRIVE_PX'] < 35:
            params['HOVER_ARRIVE_PX'] += 5
            changes.append(f"HOVER_PX↑ → {params['HOVER_ARRIVE_PX']:.0f}")
        params['DESCEND_SPEED'] = max(0.2, params['DESCEND_SPEED'] - 0.1)
        changes.append(f"DESC_SPD↓ → {params['DESCEND_SPEED']:.2f}")
    elif 'converged_but_inaccurate' in issues:
        # 收敛了但投不准 → 降低投放高度，增加稳定时间
        params['DROP_HEIGHT'] = max(1.0, params['DROP_HEIGHT'] - 0.2)
        params['STABILIZE_TIME'] = min(5.0, params['STABILIZE_TIME'] + 0.5)
        params['HOVER_ARRIVE_PX'] = max(8.0, params['HOVER_ARRIVE_PX'] - 3)
        changes.append(f"DROP↓ STAB↑ HOVER_PX↓")

    if not changes:
        # 无明确诊断 → 根据距离方向性调整
        if dist is not None:
            if dist > 1.0:
                # 差很多 → 大变动
                if iteration % 2 == 0:
                    params['KP_H'] *= 1.5
                    changes.append("KP×1.5 (大幅)")
                else:
                    params['DROP_HEIGHT'] = max(1.0, params['DROP_HEIGHT'] - 0.3)
                    changes.append(f"DROP↓ → {params['DROP_HEIGHT']:.1f}m")
            elif dist > 0.5:
                if iteration % 2 == 0:
                    params['KP_H'] *= 0.7
                    params['KD_H'] *= 0.7
                    changes.append("KP↓ KD↓")
                else:
                    params['DROP_HEIGHT'] = max(1.0, params['DROP_HEIGHT'] - 0.2)
                    params['STABILIZE_TIME'] = min(5.0, params['STABILIZE_TIME'] + 0.5)
                    changes.append("DROP↓ STAB↑")
            else:
                # dist < 0.5，微调
                params['DROP_HEIGHT'] = max(1.0, params['DROP_HEIGHT'] - 0.1)
                params['KP_H'] *= 0.95
                changes.append("微调 DROP↓ KP↓")
        else:
            # 无距离数据，随机尝试
            if iteration % 3 == 0:
                params['KP_H'] = max(0.001, params['KP_H'] * 0.5)
                changes.append("KP↓ 50%")
            elif iteration % 3 == 1:
                params['KP_H'] = min(0.02, params['KP_H'] * 2.0)
                changes.append("KP↑ 2x")
            else:
                params['DROP_HEIGHT'] = max(1.0, params['DROP_HEIGHT'] - 0.5)
                changes.append(f"DROP↓ → {params['DROP_HEIGHT']:.1f}m")

    # 边界检查
    for name, (lo, hi, _) in PARAM_RANGES.items():
        if name in params:
            params[name] = max(lo, min(hi, params[name]))

    print(f"  调整: {', '.join(changes)}" if changes else "  无需调整")
    return params


def main():
    print("=" * 60)
    print("  投放精度自动优化器")
    print(f"  启动时间: {datetime.now().strftime('%H:%M:%S')}")
    print(f"  停止条件: 连续 3 次 ≥ 8 分 或 ≥ 20 轮")
    print("=" * 60)

    # 恢复初始参数
    print("\n>>> 设置初始参数")
    write_params(INITIAL_PARAMS)

    history = []          # [(iter, dist, score, params)]
    best = (None, 0, None)  # (dist, score, params)
    consecutive_good = 0
    max_iterations = 20
    stale_count = 0
    best_score_ever = 0

    for iteration in range(1, max_iterations + 1):
        current_params = read_current_params()
        print(f"\n{'#'*60}")
        print(f"# 迭代 {iteration}/{max_iterations}")
        print(f"{'#'*60}")

        # 运行仿真
        output = run_simulation(timeout=300)
        dist, score = extract_result(output)

        if dist is None:
            print("  ERROR: 无法从日志提取距离/得分！")
            # 打印最后 50 行帮助调试
            lines = output.strip().split('\n')
            print("  --- 日志最后 50 行 ---")
            for line in lines[-50:]:
                print(f"    {line}")
            dist = 99.0
            score = 0

        history.append((iteration, dist, score, dict(current_params)))

        print(f"\n  >>> 迭代 {iteration} 结果: 距离={dist:.4f}m, 得分={score}")

        # 更新最佳
        if score > best_score_ever or (score == best_score_ever and dist < (best[0] or 999)):
            best = (dist, score, dict(current_params))
            best_score_ever = score
            stale_count = 0
            print(f"  ★ 新最佳！")
        else:
            stale_count += 1

        # 检查停止条件
        if score >= 8:
            consecutive_good += 1
            print(f"  ✓ 连续高分: {consecutive_good}/3")
        else:
            consecutive_good = 0

        if consecutive_good >= 3:
            print(f"\n  >>> 连续 3 次 ≥8 分，优化提前结束！")
            break

        if stale_count >= 5 and iteration > 5:
            print(f"\n  >>> 连续 {stale_count} 轮未提升，尝试大幅变动...")
            current_params = read_current_params()
            # 大幅变动
            if stale_count % 2 == 0:
                current_params['KP_H'] = max(0.001, current_params['KP_H'] * 0.3)
                current_params['KD_H'] = max(0.0001, current_params['KD_H'] * 0.3)
                print(f"  策略: KP/KD 大幅降低 → KP={current_params['KP_H']:.5f}")
            else:
                current_params['DROP_HEIGHT'] = max(1.0, current_params['DROP_HEIGHT'] - 0.5)
                current_params['KP_H'] = min(0.02, current_params['KP_H'] * 2.0)
                print(f"  策略: DROP↓ + KP↑ → DROP={current_params['DROP_HEIGHT']:.1f} KP={current_params['KP_H']:.5f}")
            # 边界检查
            for name, (lo, hi, _) in PARAM_RANGES.items():
                if name in current_params:
                    current_params[name] = max(lo, min(hi, current_params[name]))
            write_params(current_params)
            stale_count = 0  # 重置以允许尝试
            continue

        # 分析 & 调参
        issues = analyze_convergence(output)
        new_params = suggest_params(current_params, issues, dist, score, iteration)
        write_params(new_params)

        # 进度保存
        if iteration % 3 == 0:
            print(f"\n  --- 进度: {iteration}/{max_iterations} 迭代完成 ---")

    # ================================================================
    # 最终报告
    # ================================================================
    print(f"\n\n{'='*60}")
    print(f"  优化完成！")
    print(f"{'='*60}")

    print(f"\n  所有迭代得分:")
    print(f"  {'Iter':>5s} {'距离(m)':>10s} {'得分':>5s} {'KP':>10s} {'KD':>10s} {'DROP(m)':>8s}")
    print(f"  {'-'*55}")
    for it, d, s, p in history:
        print(f"  {it:5d} {d:10.4f} {s:5d} {p['KP_H']:10.5f} {p['KD_H']:10.5f} {p['DROP_HEIGHT']:8.1f}")

    print(f"\n  最佳结果:")
    print(f"    距离: {best[0]:.4f}m, 得分: {best[1]}")
    print(f"    参数: KP={best[2]['KP_H']:.5f}, KD={best[2]['KD_H']:.5f}, "
          f"DROP={best[2]['DROP_HEIGHT']:.1f}m, DESC_SPD={best[2]['DESCEND_SPEED']:.2f}m/s")
    print(f"    TRACK_PX={best[2]['TRACK_ARRIVE_PX']:.0f}, HOVER_PX={best[2]['HOVER_ARRIVE_PX']:.0f}, "
          f"STAB={best[2]['STABILIZE_TIME']:.1f}s, MAX_V={best[2]['MAX_HORIZ_VEL']:.1f}")

    # 保存到文件
    report = {
        'history': [(it, d, s) for it, d, s, _ in history],
        'best': {'dist': best[0], 'score': best[1], 'params': best[2]},
        'all_iterations': [{'iter': it, 'dist': d, 'score': s, 'params': p}
                          for it, d, s, p in history],
    }
    report_file = '/tmp/drop_opt_report.json'
    with open(report_file, 'w') as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    print(f"\n  报告已保存到 {report_file}")


if __name__ == '__main__':
    main()
