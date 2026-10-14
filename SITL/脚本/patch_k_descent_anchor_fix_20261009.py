#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
patch_k_descent_anchor_fix_20261009.py — 下降锚点兜底分支必须**存住** (只 k 两份)

问题 (验收 test_k_alt_5m 的 B2b 抓到): 上一步把限速改成时间锚定后, 兜底分支写成
    _z_anchor = getattr(self, '_step_entry_z', None)
    if _z_anchor is None:
        _z_anchor = self.z                      # ← 每拍取"上一拍的命令"
若 `_step_entry_z` 缺失, 则每拍的锚点是**上一拍命令本身** ⇒ 增量 rate*elapsed 逐拍累积
⇒ 平方增长 (13 拍就冲到目标 = 失控式下降) ✗✗  实机虽然 `_enter_step` 一定会设
`_step_entry_z`, 但"兜底分支比主路径更危险"是不能接受的。
修法: 兜底时**锚定一次并存住** (`self._step_entry_z = _z_anchor = self.z`) ⇒ 行为与主路径一致。

用法: python3 scripts/patch_k_descent_anchor_fix_20261009.py [--check]
"""
import argparse, datetime, hashlib, pathlib, shutil, sys
REPO = pathlib.Path(__file__).resolve().parent.parent
FILES = [REPO / "src/drone_drl/drone_drl/k.py", REPO / "fly/src/fly/fly/k.py"]
TS = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
BAK_SUFFIX = f"bak.{TS}_anchor"

A_ANCHOR = ("            if _z_anchor is None:\n"
            "                _z_anchor = self.z\n")
A_REPLACE = ("            if _z_anchor is None:\n"
             "                # ⚠ 必须**存住**锚点: 若每拍取 self.z (上一拍命令), 增量会逐拍累积\n"
             "                #   ⇒ 平方增长/失控下降 (验收 B2b 实测 13 拍冲到目标)\n"
             "                self._step_entry_z = _z_anchor = self.z\n")
MARK = "self._step_entry_z = _z_anchor = self.z"


def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()[:16]


def patch_one(path, check):
    src = path.read_text(encoding="utf-8")
    if MARK in src:
        return "SKIP (已打过, 幂等)"
    if src.count(A_ANCHOR) != 1:
        print(f"  ✗ {path}: 锚点命中 {src.count(A_ANCHOR)} 次 ⇒ 中止", file=sys.stderr); sys.exit(2)
    out = src.replace(A_ANCHOR, A_REPLACE, 1)
    try:
        compile(out, str(path), "exec")
    except SyntaxError as e:
        print(f"  ✗ {path}: 语法错误 {e} ⇒ 中止", file=sys.stderr); sys.exit(3)
    if check:
        return f"CHECK-OK (将改 {len(out) - len(src):+d} 字节)"
    bak = path.with_name(path.name + "." + BAK_SUFFIX)
    shutil.copy2(path, bak); path.write_text(out, encoding="utf-8")
    return f"OK  {sha(path)}  (备份 {bak.name})"


ap = argparse.ArgumentParser(); ap.add_argument("--check", action="store_true"); a = ap.parse_args()
print(f"=== patch_k_descent_anchor_fix_20261009 {'(CHECK)' if a.check else ''} ===")
for f in FILES:
    print(f"  {f.relative_to(REPO)}\n    before {sha(f)} → {patch_one(f, a.check)}")
print("=== done ===")
