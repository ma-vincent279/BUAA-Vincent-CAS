#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""i 的两处试飞改动 (2026-10-09): ①横向补偿往右挪 3cm ②停留 5s→2s

用户 2026-10-09 17:22 实飞 (i, 航向 212°) 两投**都偏左**, 要求"先改 3cm 试试 + 缩短停留", 不行就回退。

改动 (只动 i 的两棵树; j/k/l 暂不动):
  P1  PAYLOAD_OFFSET_BODY_Y_B1  +0.18 → +0.15   (落点右移 3cm: C = r − 100·Y ⇒ Y 减小 = 落点右移)
  P2  PAYLOAD_OFFSET_BODY_Y_B2   0.00 → −0.03   (同上)
  P3  I_DWELL_S                  5.0 → 2.0      (停留: 让 dev 少跑 3s)

为什么这两条 (数据):
  · 17:22 那趟 dev 横向 = **+15.6 / +13.1cm(右)** ⇒ 落点必偏左(实测"偏左" ✓); 内部一切正常
    (到位 3.8/6.4cm, pred 2.1/5.8cm, 冻结散布 2cm) ⇒ 误差来自"冻结之后参照又跑了"。
  · 17 投统计: 常数项 C = 落点+dev = +8.4cm(右) σ5.9  ⇒ 落点均值 = C − dev = **+1.5cm**(基本居中);
    横向平移 3cm ⇒ 均值 ∓1.5cm, 桶口 12.5cm 下命中率 **58.3% → 58.3% (Δ −0.0pt)** ⇒ 本身不赚不亏, 属"低风险试飞"。
  · **停留才是大头**: 实测"到位后残差"曲线 (8 投): +0s≈30cm → +1s≈13cm → **+2s≈4~5cm (最小)** → +3s≈8cm
    → +4s≈6cm (在 4~19cm 间晃) ⇒ 早投不损位置精度, 却少暴露 3s 的参照漂移
    (各趟 dev 在停留期以 ~3~6cm/s 增长 ⇒ 省 8~15cm) ⇒ 预期这才是命中率的实际提升项。
  · 释放仍是 AND 门: `停留≥2s ∧ pred<9cm ∧ vxy<0.15` + 20s 必投兜底 ⇒ 缩短停留不会产生"不投"路径。

用法:
  python3 scripts/patch_i_comp3_dwell2_20261009.py --check
  python3 scripts/patch_i_comp3_dwell2_20261009.py
回退 (一行): bash scripts/rollback_i_comp3_dwell2_20261009.sh     (或见脚本末尾打印的 cp 命令)
"""
import datetime
import os
import re
import shutil
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ST = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
CHECK = '--check' in sys.argv
TARGETS = [os.path.join(ROOT, t, 'i.py') for t in ('src/drone_drl/drone_drl', 'fly/src/fly/fly')]

PATCHES = [
    (r'^PAYLOAD_OFFSET_BODY_Y_B1 = \+0\.18[^\n]*',
     'PAYLOAD_OFFSET_BODY_Y_B1 = +0.15    # 2026-10-09 试飞 (原 +0.18): 落点右移 3cm —— 17:22 两投偏左',
     'P1 B1 补偿 0.18→0.15', 1),
    (r'^PAYLOAD_OFFSET_BODY_Y_B2 = 0\.00[^\n]*',
     'PAYLOAD_OFFSET_BODY_Y_B2 = -0.03    # 2026-10-09 试飞 (原 0.00): 落点右移 3cm (minimax: k=1→−3.0 / k=2→+3.0cm)',
     'P2 B2 补偿 0.00→-0.03', 1),
    (r'^I_DWELL_S\s*=\s*5\.0[^\n]*',
     'I_DWELL_S        = 2.0     # 2026-10-09 试飞 (原 5.0): 到位后 +2s 残差最小~4cm, 少暴露 dev 3s ⇒ 省 8~15cm',
     'P3 停留 5→2s', 1),
]


def apply(txt, name, check_only):
    for rx, rep, desc, n in PATCHES:
        hits = re.findall(rx, txt, re.M)
        if len(hits) != n:
            print(f'  ✗ {name}: {desc} 命中 {len(hits)} 次 (应为 {n})')
            return None
        if not check_only:
            txt = re.sub(rx, rep, txt, count=n, flags=re.M)
    return txt


def done(txt):
    return (re.search(r'^PAYLOAD_OFFSET_BODY_Y_B1 = \+0\.15', txt, re.M)
            and re.search(r'^PAYLOAD_OFFSET_BODY_Y_B2 = -0\.03', txt, re.M)
            and re.search(r'^I_DWELL_S\s*=\s*2\.0', txt, re.M))


def main():
    print('=== patch_i_comp3_dwell2_20261009 (i: 补偿 +3cm 右 / 停留 2s) ===')
    todo, skipped, bad = [], [], []
    for p in TARGETS:
        if not os.path.exists(p):
            continue
        t = open(p, encoding='utf-8').read()
        rel = os.path.relpath(p, ROOT)
        if done(t):
            skipped.append(rel)
            continue
        if apply(t, os.path.basename(p), True) is None:
            bad.append(rel)
        else:
            todo.append((rel, t))
    print(f'\n待改 {len(todo)} / 已符合 {len(skipped)} / 锚点失败 {len(bad)}')
    if bad:
        print('锚点校验失败 ⇒ 不落盘: ' + ', '.join(bad))
        return 1
    if not todo:
        print('无需改动')
        return 0
    if CHECK:
        print('--check: 只校验, 未落盘')
        return 0
    backs = []
    for rel, t in todo:
        p = os.path.join(ROOT, rel)
        bak = f'{p}.bak.{ST}_comp3dwell2'
        shutil.copy2(p, bak)
        open(p, 'w', encoding='utf-8').write(apply(t, os.path.basename(p), False))
        backs.append((rel, os.path.basename(bak)))
        print(f'  ✓ {rel} (备份 {os.path.basename(bak)})')
    # 回退脚本
    rb = os.path.join(ROOT, 'scripts', 'rollback_i_comp3_dwell2_20261009.sh')
    with open(rb, 'w', encoding='utf-8') as f:
        f.write('#!/bin/bash\n# 回退 2026-10-09 的 i 试飞改动 (补偿 3cm + 停留 2s)\nset -e\ncd "$(dirname "$0")/.."\n')
        for rel, b in backs:
            f.write(f'cp {os.path.join(os.path.dirname(rel), b)} {rel}\n')
        f.write('echo "已回退 i 的补偿/停留 (需重新部署到板载: bash scripts/deploy_i_20261009.sh)"\n')
    os.chmod(rb, 0o755)
    print(f'\n完成。回退: bash scripts/rollback_i_comp3_dwell2_20261009.sh')
    print('部署: 需 scp 到板载 + colcon build + 校验 (见下一步)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
