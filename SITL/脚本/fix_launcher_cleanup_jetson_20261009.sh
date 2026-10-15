#!/bin/bash
# ================================================================
# 修 Jetson 板载启动脚本的**清理不彻底**问题 (2026-10-09)
# ----------------------------------------------------------------
# 症状 (用户现场反馈): 每次终止脚本后 X11 上都留着一个视觉窗口; 实测还留着
#   PID 5190  /home/nvidia/fly/install/fly/lib/fly/see   (PPID=1, 孤儿)
#   PID 5124/5125 MicroXRCEAgent serial --dev /dev/ttyTHS1
# 根因: 脚本里 `ros2 run fly see &` / `sg dialout -c "MicroXRCEAgent ..." &` / `ros2 run fly k &`
#   拿到的是**包装进程**(ros2 run 是 python 包装器, sg 是 sh -c 包装器)的 PID,
#   真节点是它们的子进程 ⇒ `kill "$SEE_PID"` 只杀包装器, 子进程变孤儿继续活着:
#     · see 孤儿 ⇒ **OpenCV 窗口不关 + /dev/video0 被占**(下次起飞前必须手动 pkill)
#     · MicroXRCEAgent 孤儿 ⇒ 串口被占
# 修法 (幂等, 自动备份):
#   ① 新增 "0/3 清理上次残留" —— 脚本一启动就按【可执行文件路径】清掉上次孤儿
#   ② cleanup() 在 kill PID 之后再按路径 pkill (先 TERM 再 -9), 保证窗口/串口真的释放
# 注意: 只改板载 ~/fly/{i,k,l}.sh (仓库里的同名文件是 12 行占位脚本, 不动)
#
# 用法: bash scripts/fix_launcher_cleanup_jetson_20261009.sh
# ================================================================
set -e

JETSON="nvidia@192.168.98.136"
export SSHPASS="${SSHPASS:-nvidia}"
SSOC="StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR"
SSH="sshpass -e ssh -o $SSOC $JETSON"

echo "=== 0. 先清掉此刻的孤儿 (并展示改前状态) ==="
# ⚠ 必须把清理写成**板载脚本文件**再执行: 如果把 pkill 的 pattern 直接写在 ssh 的远程命令行里,
#   那个 pattern 会出现在远端 shell 自己的 cmdline 上 ⇒ pkill 把自己(以及 ssh 会话)一起杀掉
#   (2026-10-09 实测: exit 255, 只打印了 4 行)。
$SSH "cat > /tmp/resid_clean.sh <<'EOS'
#!/bin/bash
pats=(\"lib/fly/see\" \"lib/fly/k\" \"lib/fly/l\" \"MicroXRCEAgent serial\")
echo \"  改前残留:\"
ps -eo pid,ppid,cmd | grep -E \"lib/fly/see|lib/fly/k|lib/fly/l|MicroXRCEAgent serial\" | grep -v grep | sed \"s/^/    /\" || echo \"    (无)\"
for p in \"\${pats[@]}\"; do pkill -f \"\$p\" 2>/dev/null || true; done
sleep 1
for p in \"\${pats[@]}\"; do pkill -9 -f \"\$p\" 2>/dev/null || true; done
sleep 1
echo \"  清理后:\"
ps -eo pid,ppid,cmd | grep -E \"lib/fly/see|lib/fly/k|lib/fly/l|MicroXRCEAgent serial\" | grep -v grep | sed \"s/^/    /\" || echo \"    ✓ 无残留\"
echo \"  /dev/video0 占用: \$(fuser /dev/video0 2>/dev/null || echo 无)\"
EOS
bash /tmp/resid_clean.sh"

echo
echo "=== 1. 给 ~/fly/{i,k,l}.sh 打补丁 (备份 + 幂等) ==="
$SSH "python3 - <<'PY'
import datetime, pathlib, shutil, re, sys

TS = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
BLOCK_MARK = '# ---- 0. 清理上次残留'

# ① 启动前清理残留 (插在 '检查环境' 之后 / 环境检查前后均可, 这里插在 DDS 启动之前)
ANCHOR_HEAD = '''# ---- 1. 启动 DDS Agent ----'''
NEW_HEAD = '''# ---- 0. 清理上次残留 (ros2 run / sg 的包装进程被杀后, 真节点会变孤儿) ----
#   2026-10-09 实测: 上一趟结束后残留 .../lib/fly/see (占 /dev/video0 + 留 OpenCV 窗口)
#   与 MicroXRCEAgent ⇒ 这里按【可执行文件路径】先清一遍, 免去每次起飞前手动 pkill。
echo \"\"
echo \"=== 0/3 清理上次残留 ===\"
RESID=0
for pat in \"lib/fly/see\" \"lib/fly/k\" \"lib/fly/l\"; do
    if pgrep -f \"\$pat\" >/dev/null 2>&1; then
        pkill -f \"\$pat\" 2>/dev/null || true
        echo \"  已清理: \$pat\"
        RESID=1
    fi
done
sleep 1
for pat in \"lib/fly/see\" \"lib/fly/k\" \"lib/fly/l\"; do
    pkill -9 -f \"\$pat\" 2>/dev/null || true
done
[[ \"\$RESID\" = 0 ]] && echo \"  ✓ 无残留\"

# ---- 1. 启动 DDS Agent ----'''

# ② cleanup(): 在 kill PID 之后按路径补刀
#    锚点用三者共有的那行 (i.sh 用 I_PID / k.sh 用 K_PID ⇒ 不能用含 PID 变量的行做锚点)
OLD_CLEAN = '''    echo \"=== 清理完成 ===\"'''
NEW_CLEAN = '''    # 2026-10-09: 上面杀的都是【包装进程】(ros2 run / sg); 真节点是它们的子进程, 会变孤儿
    #   ⇒ 按可执行路径补刀 (see 孤儿会占 /dev/video0 并留一个 OpenCV 窗口)
    for pat in \"lib/fly/see\" \"lib/fly/k\" \"lib/fly/l\" \"MicroXRCEAgent serial\"; do
        pkill -f \"\$pat\" 2>/dev/null && echo \"  已清理 \$pat\"
    done
    sleep 1
    for pat in \"lib/fly/see\" \"lib/fly/k\" \"lib/fly/l\" \"MicroXRCEAgent serial\"; do
        pkill -9 -f \"\$pat\" 2>/dev/null || true
    done
    echo \"=== 清理完成 ===\"'''

changed = []
for letter in ('i', 'k', 'l'):
    p = pathlib.Path('/home/nvidia/fly/%s.sh' % letter)
    if not p.exists():
        print('  · %s.sh 不存在, 跳过' % p.name)
        continue
    t = p.read_text(encoding='utf-8')
    todo = []
    if BLOCK_MARK not in t:
        assert t.count(ANCHOR_HEAD) == 1, '%s: DDS 锚点命中 %d 次' % (p.name, t.count(ANCHOR_HEAD))
        todo.append(('启动前清理残留', ANCHOR_HEAD, NEW_HEAD))
    if '2026-10-09: 上面杀的都是' not in t:
        assert t.count(OLD_CLEAN) == 1, '%s: cleanup 锚点命中 %d 次' % (p.name, t.count(OLD_CLEAN))
        todo.append(('cleanup 补刀', OLD_CLEAN, NEW_CLEAN))
    if not todo:
        print('  · %s.sh 已修过, 跳过' % p.name)
        continue
    shutil.copy2(p, '%s.bak.%s_cleanup' % (p, TS))
    t2 = t
    for name, a, b in todo:
        t2 = t2.replace(a, b, 1)
        print('  ✓ %s.sh: %s' % (p.name, name))
    p.write_text(t2, encoding='utf-8')
    changed.append(p.name)

print('  本次改动: %s' % (changed if changed else '无'))
PY"

echo
echo "=== 2. 语法检查 + 差异审计 ==="
$SSH 'for f in ~/fly/i.sh ~/fly/k.sh ~/fly/l.sh; do
        [ -f "$f" ] || continue
        bash -n "$f" && echo "  ✓ $(basename $f) 语法 OK"
      done
      echo "  ── k.sh 的清理段 ──"
      sed -n "/0\/3 清理上次残留/,/1\/3 启动 DDS/p" ~/fly/k.sh | head -18 | sed "s/^/    /"
      echo "  ── cleanup() 的补刀段 ──"
      sed -n "/lib\/fly\/see\" \"lib\/fly\/k/,+6p" ~/fly/k.sh | head -8 | sed "s/^/    /"'

echo
echo "=== 3. 注意 ==="
echo "  · k/l.sh 的 cleanup 修复是靠 i.sh 派生的; 三者已分别打过补丁 ⇒ 幂等"
echo "  · 想不要那个视觉窗口: ./k.sh 前手动跑 ros2 run fly see --ros-args -p visualize:=false"
echo "    (或改 k.sh 里那行 ros2 run fly see &)"
