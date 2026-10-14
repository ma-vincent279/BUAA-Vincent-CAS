#!/usr/bin/env python3
# ================================================================
# 2026-09-20 选桶健壮性修复 (项目 e, SITL+fly)
# ----------------------------------------------------------------
# 诊断依据 (e SITL 一轮, TRK-DBG/TRK-NEW 日志):
#   图像边缘存在"残缺框"(部分出画, 只剩 3~5px 的窄条, 如 box(cx=636,cy=308,w=5,h=23)
#   或 box(cx=176,cy=356,w=17,h=5)); 其中心不是桶的投影中心 → 反投影偏 0.2~3m
#   (同一桶的完整框给 NED(0.12,33.88), 残框给 NED(-1.46,31.47)) → 偏出 >1.5m 就会
#   自己建一个追踪器, 候选池里多出一个"就在真桶附近"的幽灵; 选桶只按"离中线最近"取前 2,
#   于是两个名额可能被"真桶 + 它的残框幽灵"吃掉 (与 9-20 两轮 d 的现象一致:
#   两候选相距 1.7m、都离 B3 最近, 第二投打到 B3 → 0 分).
# 修法 (不依赖桶间距, 实机两桶只有 1.66m 也适用):
#   ① 剔除残缺框: 最小边 < max(8px, 本帧最小边中位数×0.5) 的框不进入估计/追踪器池
#   ② 选桶加"共同可见"校验: 两个被选目标必须由画面里**两个不同的框**支撑
#      (追踪器各记录自己的 roi_center, 两框中心像素相距 <25px 视为同一个框)
# 用法: python3 scripts/fix_selection_robustness_e_20260920.py [--check]
# ================================================================
import ast
import os
import sys

ROOT = '/home/vincent/SITL'
TARGETS = [f'{ROOT}/src/drone_drl/drone_drl/e.py', f'{ROOT}/fly/src/fly/fly/e.py']
CHECK_ONLY = '--check' in sys.argv

OLD_FILTER = """        # 2026-09-12: 同帧重复框合并 (防同一个桶被喂出两个追踪器 → 两个目标其实是同一个桶)
        valid = self._dedup_dets(valid)
        self._bucket_ests = valid"""
NEW_FILTER = """        # 2026-09-12: 同帧重复框合并 (防同一个桶被喂出两个追踪器 → 两个目标其实是同一个桶)
        valid = self._dedup_dets(valid)
        # 2026-09-20: 剔除"残缺框"(部分出画/贴边) —— 其中心不是桶的投影中心, 反投影可偏 0.2~3m,
        #   偏出 1.5m 就会自建追踪器 → 候选池多出"真桶附近的幽灵" → 可能占满前两名.
        if len(valid) >= 2:
            sides = sorted(min(r.width, r.height) for _x, _y, r, _d, _cx, _cy in valid)
            _thr = max(8.0, 0.5 * sides[len(sides) // 2])
        else:
            _thr = 8.0
        _kept = [v for v in valid if min(v[2].width, v[2].height) >= _thr]
        if len(_kept) != len(valid):
            self.get_logger().info(
                f"  DET-FILTER 剔除残缺框 {len(valid) - len(_kept)}/{len(valid)} "
                f"(阈值{_thr:.0f}px, 本帧最小边="
                f"{[min(v[2].width, v[2].height) for v in valid]})")
        valid = _kept
        self._bucket_ests = valid"""

OLD_SEL = """            selected = [(t["ned"][0], t["ned"][1]) for t in confirmed[:2]]"""
NEW_SEL = """            # 2026-09-20: 两个目标必须由画面里"两个不同的框"支撑 (防真桶+残框幽灵占满两名).
            #   追踪器记有 roi_center(最近一次匹配框中心); 两框中心相距 <25px 视为同一个框.
            picked = []
            for _t in confirmed:
                if all(math.hypot(_t["roi_center"][0] - _p["roi_center"][0],
                                  _t["roi_center"][1] - _p["roi_center"][1]) >= 25.0
                       for _p in picked):
                    picked.append(_t)
                if len(picked) == 2:
                    break
            if len(picked) < 2:
                # 没有两个"由不同框支撑"的已确认目标: 继续悬停搜索, 不用同一桶的两个候选占位
                self.get_logger().info(
                    f"  SELECT-HOLD 已确认 {len(confirmed)} 个但只有 {len(picked)} 个由不同框支撑, 继续搜索")
                return
            selected = [(t["ned"][0], t["ned"][1]) for t in picked]"""

OLD_LAT = """                for t in confirmed[:2]) if self._heading0 is not None else \"\""""
NEW_LAT = """                for t in picked) if self._heading0 is not None else \"\""""

plan = []
for p in TARGETS:
    s = open(p, encoding='utf-8').read()
    assert 'DET-FILTER' not in s, f'{p} 已打过补丁'
    for old, new, tag in ((OLD_FILTER, NEW_FILTER, 'filter'), (OLD_SEL, NEW_SEL, 'select'), (OLD_LAT, NEW_LAT, 'lat')):
        n = s.count(old)
        assert n == 1, f'[{os.path.basename(p)}/{tag}] 期望 1 次, 实际 {n} 次:\n{old[:120]}'
        s = s.replace(old, new, 1)
    ast.parse(s)
    plan.append((p, s))

for p, s in plan:
    if CHECK_ONLY:
        print('  [check ok]', os.path.relpath(p, ROOT))
    else:
        open(p, 'w', encoding='utf-8').write(s)
        print('  [written] ', os.path.relpath(p, ROOT))
print('OK' + (' (未写盘)' if CHECK_ONLY else ''))
