#!/usr/bin/env python3
# ================================================================
# 2026-09-24 项目 f 判据离线验收 (真代码路径)
#   把 f.py 里"两级释放判据"整段原文抽出来 exec 成一个函数, 用合成残差序列驱动,
#   验证: Tier1 优选 / Tier2 保底(10s 窗口) / 不触发(退兜底) / 低谷夹下限防噪声 / 准静止门控
#   用法: python3 scripts/test_f_gate_logic_20260924.py
# ================================================================
import math
import re
import sys
import textwrap

ROOT = '/home/vincent/SITL'
FILES = {'SITL': 'src/drone_drl/drone_drl/f.py', 'fly': 'fly/src/fly/fly/f.py'}
BLOCK_RE = re.compile(r'( *# ===== 2026-09-24 f: 两级释放判据 \(方案 B\) =====.*?)\n *if not self\._drop_triggered',
                      re.S)
CONST_RE = re.compile(r'^(DROP_[A-Z_]*(?:M|LATE|EARLY|BAND|FLOOR|VMAX|WINDOW))\s*=\s*([0-9.]+)', re.M)

FAIL = []


def check(ok, msg):
    print(('  ✓ ' if ok else '  ✗ ') + msg)
    if not ok:
        FAIL.append(msg)


def load_gate(rel):
    src = open(f'{ROOT}/{rel}', encoding='utf-8').read()
    m = BLOCK_RE.search(src)
    assert m, f'{rel}: 未找到两级释放判据块'
    block = textwrap.dedent(m.group(1))
    consts = {k: float(v) for k, v in CONST_RE.findall(src)}
    name = rel.split('/')[0]
    ns = {'math': math, **consts}
    exec(f'def gate(self, conv_m, sim_t, release_valid):\n{textwrap.indent(block, "    ")}\n    return early_ok, tier', ns)
    return ns['gate'], consts


class Fake:
    def __init__(self):
        self._drop_conv_min = None
        self._drop_tier = None
        self._drop_converge_start = None
        self.vx = self.vy = 0.0
        self.t = 0.0
        self.samples = []          # (t, conv_m)

    def _step_elapsed(self):
        return self.t


def run(gate, seq, dt=0.25, vxy=0.05, stop_on_release=True):
    """seq: [(t, conv_cm, vxy|None), ...]; 返回 (release_t, tier, conv_cm) 或 (None, None, None)."""
    f = Fake()
    for (t, conv_cm, v) in seq:
        f.t = t
        f.vx = (v if v is not None else vxy)
        f.vy = 0.0
        ok, tier = gate(f, conv_cm / 100.0, t, True)
        f.samples.append((t, conv_cm))
        if ok and stop_on_release:
            return t, tier, conv_cm
    return None, None, None


def seq_from(fn, t_end, dt=0.25):
    t, out = 0.0, []
    while t <= t_end + 1e-9:
        out.append((round(t, 3), fn(t), None))
        t += dt
    return out


def main():
    for name, rel in FILES.items():
        print(f'===== {name} ({rel})')
        gate, consts = load_gate(rel)
        print(f'  常量: T1={consts["DROP_ARRIVE_M"]}m T2={consts["DROP_ARRIVE_M_LATE"]}m '
              f'confirm={consts["DROP_CONVERGE_EARLY"]}/{consts["DROP_CONVERGE_LATE"]}s '
              f'band={consts["DROP_CONV_MIN_BAND"]} floor={consts["DROP_CONV_MIN_FLOOR"]} '
              f'vmax={consts["DROP_ARRIVE_VMAX"]} window={consts["DROP_PREFER_WINDOW"]}s')

        # A: 3s 处穿过 3cm 低谷 -> Tier1 应在 ~3+0.5s 触发
        tA = run(gate, seq_from(lambda t: 20 - 5.5 * t + 0.6 * math.sin(t * 3) if t < 3 else 3.0, 8.0))
        check(tA[1] == 1 and tA[0] is not None and 3.0 <= tA[0] <= 3.8,
              f'A 低谷穿越 -> Tier1 @{tA[0]}s 残差{tA[2]:.2f}cm (期望 Tier1, 3.0~3.8s)')

        # B: 全程 6~9cm (够不到 5cm) -> 前 10s 不触发, Tier2 应在 10s+1.0s 后触发
        seqB = seq_from(lambda t: 7.5 + 1.5 * math.sin(t * 1.2), 20.0)
        f = Fake()
        rel_t, rel_tier, rel_c = None, None, None
        for (t, c, _) in seqB:
            f.t = t
            ok, tier = gate(f, c / 100.0, t, True)
            if ok:
                rel_t, rel_tier, rel_c = t, tier, c
                break
        check(rel_tier == 2 and rel_t >= 11.0, f'B 地板 6~9cm -> Tier2 @{rel_t}s 残差{rel_c:.2f}cm (期望 Tier2 且 >=11s)')

        # C: 全程 15~25cm -> 段内不触发 (退 35s 兜底)
        tC = run(gate, seq_from(lambda t: 20 + 5 * math.sin(t), 30.0))
        check(tC[0] is None, f'C 地板 15~25cm -> 段内不触发 (期望 None, 实得 {tC[0]})')

        # D: 单帧噪声 0.5cm 后稳定在 5cm -> 因最小值下限夹 2cm, Tier1 要求 <=4cm, 故不应误投
        seqD = [(0.0, 30.0, None), (0.25, 0.5, None)] + [(0.5 + 0.25 * i, 5.0, None) for i in range(40)]
        f = Fake()
        d_tier = None
        for (t, c, _) in seqD:
            f.t = t
            ok, tier = gate(f, c / 100.0, t, True)
            if ok and tier == 1:
                d_tier = tier
                break
        check(d_tier is None, 'D 噪声单帧 0.5cm + 稳定 5cm -> 不误投 Tier1 (下限夹 2cm 生效)')

        # E: 残差 3cm 但水平速度 0.2m/s -> Tier1 被准静止门控挡住; 速度降到 0.08 后立即触发
        f = Fake()
        e_res = None
        for i in range(60):
            t = 0.25 * i
            f.t = t
            f.vx = 0.20 if t < 4.0 else 0.08
            f.vy = 0.0
            ok, tier = gate(f, 0.03, t, True)
            if ok:
                e_res = (t, tier)
                break
        check(e_res is not None and e_res[1] == 1 and e_res[0] >= 4.0,
              f'E vxy=0.20 被挡 -> 降到 0.08 后 Tier1 @{e_res[0] if e_res else None}s (期望 >=4.0s)')

    print()
    if FAIL:
        print(f'失败 {len(FAIL)} 项:')
        for m in FAIL:
            print('   -', m)
        sys.exit(1)
    print('全部通过 ✓ (两份 f.py 的真实判据代码)')


if __name__ == '__main__':
    main()
