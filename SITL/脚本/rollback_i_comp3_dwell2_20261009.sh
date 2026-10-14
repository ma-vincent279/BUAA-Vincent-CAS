#!/bin/bash
# 回退 2026-10-09 的 i 试飞改动 (补偿 3cm + 停留 2s)
set -e
cd "$(dirname "$0")/.."
cp src/drone_drl/drone_drl/i.py.bak.20261009_173950_comp3dwell2 src/drone_drl/drone_drl/i.py
cp fly/src/fly/fly/i.py.bak.20261009_173950_comp3dwell2 fly/src/fly/fly/i.py
echo "已回退 i 的补偿/停留 (需重新部署到板载: bash scripts/deploy_i_20261009.sh)"
