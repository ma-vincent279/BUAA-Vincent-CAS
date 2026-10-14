#!/bin/bash
# ================================================================
# 磁盘清理脚本 — 根分区安全清理
# 用法: bash ~/SITL/scripts/clean_disk.sh
# ================================================================
set -e

echo "=== 磁盘清理开始 ==="
echo "清理前:"
df -h / /home
echo ""

# ---- 1. Claude Code 会话临时文件 ----
echo "--- 1. Claude Code 临时文件 ---"
rm -rf /tmp/claude-1000/ 2>/dev/null && echo "  /tmp/claude-1000 已清理" || echo "  (不存在或已清理)"

# ---- 2. apt 缓存 ----
echo "--- 2. apt 缓存 ---"
du -sh /var/cache/apt 2>/dev/null || true
sudo apt clean && echo "  apt cache 已清理"

# ---- 3. 旧内核 ----
echo "--- 3. 旧内核 ---"
CURRENT_KERNEL=$(uname -r)
echo "  当前运行内核: $CURRENT_KERNEL"

# 列出待删除的旧内核
OLD_IMAGES=$(dpkg -l 2>/dev/null | awk '/^ii.*linux-image-[0-9]/ && $2 !~ /'"$CURRENT_KERNEL"'/ {print $2}')
OLD_HEADERS=$(dpkg -l 2>/dev/null | awk '/^ii.*linux-headers-[0-9]/ && $2 !~ /'"$CURRENT_KERNEL"'/ {print $2}')
OLD_MODULES=$(dpkg -l 2>/dev/null | awk '/^ii.*linux-modules-[0-9]/ && $2 !~ /'"$CURRENT_KERNEL"'/ {print $2}')
OLD_EXTRA=$(dpkg -l 2>/dev/null | awk '/^ii.*linux-modules-extra-[0-9]/ && $2 !~ /'"$CURRENT_KERNEL"'/ {print $2}')

TO_REMOVE="$OLD_IMAGES $OLD_HEADERS $OLD_MODULES $OLD_EXTRA"
if [ -z "$(echo $TO_REMOVE | tr -d ' ')" ]; then
    echo "  没有旧内核需要删除"
else
    echo "  待删除: $TO_REMOVE"
    sudo apt purge -y $TO_REMOVE && echo "  旧内核已删除" || echo "  删除失败，跳过"
fi

# ---- 4. 残留依赖 ----
echo "--- 4. 残留依赖 ---"
sudo apt autoremove --purge -y && echo "  残留包已清理"

# ---- 5. snap 旧版本 ----
echo "--- 5. snap 旧版本 ---"
DISABLED=$(snap list --all 2>/dev/null | awk '/disabled/{print $1, $3}')
if [ -z "$DISABLED" ]; then
    echo "  没有需要清理的 snap 旧版本"
else
    echo "$DISABLED" | while read name rev; do
        sudo snap remove "$name" --revision="$rev" && echo "  已删除 snap $name rev $rev"
    done
fi

# ---- 6. systemd 日志 ----
echo "--- 6. journal 日志 ---"
sudo journalctl --vacuum-size=100M 2>&1 | tail -1

# ---- 7. pip 缓存 (user, 不需要 sudo) ----
echo "--- 7. pip 缓存 ---"
rm -rf ~/.cache/pip 2>/dev/null && echo "  pip 缓存已清理" || echo "  (无 pip 缓存)"

# ---- 结果 ----
echo ""
echo "=== 清理完成 ==="
echo "清理后:"
df -h / /home
