#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cuadc_judge.py — 按 2026 CUADC 规则计分的投放裁判 (2026-10-11 新建)

与 final_judge.py 的区别 (为什么不改 final_judge: 其他项目仍依赖它)
--------------------------------------------------------------------
| | final_judge.py (旧) | cuadc_judge.py (本文件) |
|---|---|---|
| 计分 | 距离分带 10/8/6/4/2/1/0 (自定) | **规则原样**: 落桶内 = 该桶 A 区分值 (500/300/100); 距桶心 ≤0.5m = B 区 50 分; 否则 0 |
| 桶径 | 单一 `bucket_radius=0.125` 通用 | **每个桶各自的 radius** (7.5/10/12.5cm) |
| 真值来源 | 命令行手抄 `bucketN_gz_*` ×3 + 裁判 ×3 | **`worlds/<name>.truth.json` 单一真值源** (与 gen_world_cuadc.py 配对的产物) |
| 落点输入 | `/drop/impact_point` (**绝对** NED, 需与 home≈0 的假设共存) | `/drop/impact_rel` (**相对 home** NED, 与真值同系, 不依赖 home≈0) |

计分规则 (RULE_MAPPING 原文):
    drop_1: 直径 0.15m, A 区 500 分
    drop_2: 直径 0.20m, A 区 300 分
    drop_3: 直径 0.25m, A 区 100 分
    B 区: 距桶心 0.5m 内, 50 分

两投上限: 取直径最小的两个桶 ⇒ 500 + 300 = **800 分**。

用法:
    ros2 run drone_drl cuadc_judge --ros-args -p truth_file:=worlds/k.truth.json
"""
import json
import math
import pathlib

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class CuadcJudge(Node):

    def __init__(self):
        super().__init__('cuadc_judge')

        self.declare_parameter('truth_file', 'worlds/k.truth.json')
        self.declare_parameter('impact_topic', '/drop/impact_rel')
        self.declare_parameter('match_max_dist', 3.0)   # 落点与桶的最大匹配距离
        self.declare_parameter('allow_reuse', False)    # 比赛每桶只投一次
        self.declare_parameter('summary_file', '')      # 非空 ⇒ 每投后写 JSON

        tf = self.get_parameter('truth_file').value
        p = pathlib.Path(tf)
        if not p.is_absolute():
            # 允许相对仓库根 (脚本里传 worlds/k.truth.json)
            for cand in (pathlib.Path.cwd() / tf,
                         pathlib.Path(__file__).resolve().parent.parent.parent / tf):
                if cand.exists():
                    p = cand
                    break
        if not p.exists():
            raise RuntimeError(f"truth_file 不存在: {tf}")
        truth = json.loads(p.read_text(encoding='utf-8'))

        self._match_max = float(self.get_parameter('match_max_dist').value)
        self._allow_reuse = bool(self.get_parameter('allow_reuse').value)
        self._summary_file = self.get_parameter('summary_file').value

        sc = truth['scoring']
        self._b_radius = float(sc['b_zone_radius'])
        self._b_score = int(sc['b_zone_score'])
        self._buckets = [
            {'id': b['id'], 'x': float(b['ned_x']), 'y': float(b['ned_y']),
             'radius': float(b['radius']), 'diameter': float(b['diameter']),
             'a_score': int(b['a_zone_score'])}
            for b in truth['buckets']
        ]
        self._seed = truth.get('seed')
        self._scored = {}          # bucket_id -> (dist, score)  或 drop_idx -> (...)
        self._drop_count = 0
        self._records = []

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(
            Point, self.get_parameter('impact_topic').value, self._cb_impact, qos)

        self.get_logger().info(
            f"CUADC judge ready: seed={self._seed} {len(self._buckets)} buckets | "
            f"规则: d<=桶半径 → A区(该桶分值) / d<={self._b_radius}m → B区{self._b_score}分 / 否则 0 | "
            f"两投上限={sum(b['a_score'] for b in sorted(self._buckets, key=lambda b: b['diameter'])[:2])}分")
        for b in self._buckets:
            self.get_logger().info(
                f"  bucket {b['id']}: NED({b['x']:+.3f},{b['y']:+.3f}) "
                f"直径{b['diameter'] * 100:.0f}cm r={b['radius'] * 100:.1f}cm A区={b['a_score']}分")

    # ------------------------------------------------------------------
    def _cb_impact(self, msg):
        self._drop_count += 1
        x, y = float(msg.x), float(msg.y)

        best, best_d = None, float('inf')
        for b in self._buckets:
            if not self._allow_reuse and b['id'] in self._scored:
                continue
            d = math.hypot(x - b['x'], y - b['y'])
            if d < best_d:
                best, best_d = b, d

        if best is None:
            self.get_logger().warn(
                f"DROP #{self._drop_count}: 没有未评分桶可用 (漏投/桶不够)")
            self._record(self._drop_count, x, y, None, best_d, 0, 'no_bucket')
            return

        if best_d > self._match_max:
            self.get_logger().warn(
                f"DROP #{self._drop_count}: 落点 NED({x:+.3f},{y:+.3f}) 距最近未评分桶 "
                f"{best['id']} {best_d:.3f}m > {self._match_max}m ⇒ invalid 0 分")
            self._record(self._drop_count, x, y, best['id'], best_d, 0, 'invalid')
            return

        if best_d <= best['radius']:
            zone, score = 'A', best['a_score']
        elif best_d <= self._b_radius:
            zone, score = 'B', self._b_score
        else:
            zone, score = 'outside', 0

        self._scored[best['id']] = (best_d, score)
        self.get_logger().info(
            f"DROP #{self._drop_count}: 落点 NED({x:+.3f},{y:+.3f}) → 桶 {best['id']} "
            f"(直径{best['diameter'] * 100:.0f}cm) 距离 {best_d * 100:.1f}cm ⇒ {zone} 区 {score} 分")
        self._record(self._drop_count, x, y, best['id'], best_d, score, zone)
        self._summary()

    # ------------------------------------------------------------------
    def _record(self, idx, x, y, bucket, dist, score, zone):
        self._records.append({
            'drop': idx, 'impact_ned': [round(x, 4), round(y, 4)],
            'bucket': bucket, 'dist_m': None if dist == float('inf') else round(dist, 4),
            'zone': zone, 'score': score,
        })
        self._write_summary()

    def _summary(self):
        total = sum(r['score'] for r in self._records)
        n = len(self._records)
        pts = [r['dist_m'] for r in self._records if r['dist_m'] is not None]
        avg = sum(pts) / len(pts) if pts else float('nan')
        self.get_logger().info(
            f"  Cumulative: {n} 投, avg_dist={avg:.4f}m, total_score={total} / 800")

    def _write_summary(self):
        if not self._summary_file:
            return
        total = sum(r['score'] for r in self._records)
        payload = {
            'seed': self._seed, 'drops': self._records,
            'total_score': total, 'max_score': 800,
            'n_drops': len(self._records),
            'hit_a': sum(1 for r in self._records if r['zone'] == 'A'),
            'hit_b': sum(1 for r in self._records if r['zone'] == 'B'),
        }
        try:
            pathlib.Path(self._summary_file).write_text(
                json.dumps(payload, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        except OSError as exc:      # 写不进去不该打断任务
            self.get_logger().warn(f"summary 写入失败: {exc}")


def main():
    rclpy.init()
    node = CuadcJudge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._summary()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
