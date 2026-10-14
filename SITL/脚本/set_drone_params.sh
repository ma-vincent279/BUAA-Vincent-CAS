#!/bin/bash
# ===========================================================================
# 调整 SITL Gazebo Iris 模型物理参数，使仿真更接近实机动力学
# ===========================================================================
# 用法:
#   bash scripts/set_drone_params.sh mass=5.3 prop_inch=15     # 全自动换算
#   bash scripts/set_drone_params.sh mass=2.0 tc_up=0.02       # 手动覆盖
#   bash scripts/set_drone_params.sh --list                     # 查看当前值
#   bash scripts/set_drone_params.sh --restore                  # 恢复到 Iris 默认
#   bash scripts/set_drone_params.sh --defaults                 # 查看 Iris 默认
#
# 参数:
#   mass         机身质量 (kg), Iris 默认 1.5
#   prop_inch    桨尺寸 (英寸), Iris 默认 10. 自动换算 motor_k/drag_coef
#   wheelbase_mm 轴距 (mm), Iris 默认 350. 自动修正转动惯量
#   tc_up        电机加速时间常数 (s), Iris 默认 0.0125
#   tc_down      电机减速时间常数 (s), Iris 默认 0.025
#   motor_k      推力常数 (N·s²), Iris 默认 5.84e-06
#   max_rpm      电机最大转速 (rad/s), Iris 默认 1100
#   drag_coef    旋翼阻力系数, Iris 默认 0.000175
# ===========================================================================

set -e

IRIS_JINJA=~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf.jinja
IRIS_SDF=~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf
BACKUP_DIR=~/SITL/backups/iris_params

# ---- Iris 默认值 ----
DEF_MASS="1.5"
DEF_PROP="10"
DEF_WHEELBASE="350"
DEF_TC_UP="0.0125"
DEF_TC_DOWN="0.025"
DEF_MOTOR_K="5.84e-06"
DEF_MAX_RPM="1100"
DEF_DRAG="0.000175"
DEF_IXX="0.029125"
DEF_IYY="0.029125"
DEF_IZZ="0.055225"
DEF_ROTOR_MASS="0.005"
DEF_ROTOR_RADIUS="0.128"

# ---- 解析参数 ----
ACTION="apply"
NEW_MASS=""; NEW_PROP=""; NEW_WHEELBASE=""
NEW_TC_UP=""; NEW_TC_DOWN=""; NEW_MOTOR_K=""; NEW_MAX_RPM=""; NEW_DRAG=""

for arg in "$@"; do
    case "$arg" in
        --list)      ACTION="list" ;;
        --restore)   ACTION="restore" ;;
        --defaults)  ACTION="defaults" ;;
        mass=*)      NEW_MASS="${arg#mass=}" ;;
        prop_inch=*) NEW_PROP="${arg#prop_inch=}" ;;
        wheelbase_mm=*) NEW_WHEELBASE="${arg#wheelbase_mm=}" ;;
        tc_up=*)     NEW_TC_UP="${arg#tc_up=}" ;;
        tc_down=*)   NEW_TC_DOWN="${arg#tc_down=}" ;;
        motor_k=*)   NEW_MOTOR_K="${arg#motor_k=}" ;;
        max_rpm=*)   NEW_MAX_RPM="${arg#max_rpm=}" ;;
        drag_coef=*) NEW_DRAG="${arg#drag_coef=}" ;;
        *)           echo "未知参数: $arg"; exit 1 ;;
    esac
done

# ---- 工具函数 ----
_get() {
    if [ ! -f "$IRIS_JINJA" ]; then echo "错误: 找不到 $IRIS_JINJA"; exit 1; fi
    local mass=$(grep -oP '<mass>\K[0-9.e+\-]+(?=</mass>)' "$IRIS_JINJA" | head -1)
    local tc_up=$(grep -oP '<timeConstantUp>\K[0-9.e+\-]+(?=</timeConstantUp>)' "$IRIS_JINJA" | head -1)
    local tc_down=$(grep -oP '<timeConstantDown>\K[0-9.e+\-]+(?=</timeConstantDown>)' "$IRIS_JINJA" | head -1)
    local motor_k=$(grep -oP '<motorConstant>\K[0-9.e+\-]+(?=</motorConstant>)' "$IRIS_JINJA" | head -1)
    local drag=$(grep -oP '<rotorDragCoefficient>\K[0-9.e+\-]+(?=</rotorDragCoefficient>)' "$IRIS_JINJA" | head -1)
    local max_rpm=$(grep -oP '<maxRotVelocity>\K[0-9.e+\-]+(?=</maxRotVelocity>)' "$IRIS_JINJA" | head -1)
    local ixx=$(grep -oP '<ixx>\K[0-9.e+\-]+(?=</ixx>)' "$IRIS_JINJA" | head -1)
    local iyy=$(grep -oP '<iyy>\K[0-9.e+\-]+(?=</iyy>)' "$IRIS_JINJA" | head -1)
    local izz=$(grep -oP '<izz>\K[0-9.e+\-]+(?=</izz>)' "$IRIS_JINJA" | head -1)
    echo "$mass $tc_up $tc_down $motor_k $drag $max_rpm $ixx $iyy $izz"
}

_backup() {
    mkdir -p "$BACKUP_DIR"
    local ts=$(date +%Y%m%d_%H%M%S)
    cp "$IRIS_JINJA" "$BACKUP_DIR/iris.sdf.jinja.bak.$ts"
    cp "$IRIS_SDF"   "$BACKUP_DIR/iris.sdf.bak.$ts"
    echo "$BACKUP_DIR/iris.sdf.jinja.bak.$ts"
}

_sed_both() {
    # 同时对 jinja 模板和生成的 sdf 执行 sed
    local expr="$1"
    sed -i "$expr" "$IRIS_JINJA"
    sed -i "$expr" "$IRIS_SDF"
}

# ---- 动作: list / defaults / restore ----
case "$ACTION" in
    list)
        read m tc_u tc_d mk dr mxr ix iy iz <<< $(_get)
        echo "当前 Iris 模型物理参数:"
        echo "  mass      = $m kg"
        echo "  ixx/iyy/izz = $ix / $iy / $iz  kg·m²"
        echo "  tc_up     = $tc_u s"
        echo "  tc_down   = $tc_d s"
        echo "  motor_k   = $mk N·s²"
        echo "  max_rpm   = $mxr rad/s"
        echo "  drag_coef = $dr"
        hr=$(python3 -c "m=float($m); k=float($mk); g=9.81; w=(m*g/(4*k))**0.5; print(f'{w:.0f}')")
        hr_rpm=$(python3 -c "print(f'{float($hr)*60/6.28318:.0f}')")
        echo "  悬停转速 ≈ $hr rad/s ($hr_rpm RPM)"
        exit 0
        ;;
    defaults)
        echo "Iris 默认: mass=$DEF_MASS prop=${DEF_PROP}\" wheelbase=${DEF_WHEELBASE}mm"
        echo "  tc_up=$DEF_TC_UP tc_down=$DEF_TC_DOWN motor_k=$DEF_MOTOR_K max_rpm=$DEF_MAX_RPM drag_coef=$DEF_DRAG"
        echo "  ixx=$DEF_IXX iyy=$DEF_IYY izz=$DEF_IZZ"
        exit 0
        ;;
    restore)
        _backup > /dev/null
        # mass 只恢复第一个 (base_link)
        python3 << PYEOF
import re
for fpath in ["$IRIS_JINJA", "$IRIS_SDF"]:
    import os
    if not os.path.exists(fpath): continue
    with open(fpath) as f: content = f.read()
    def replace_nth(pat, repl, text, n):
        ms = list(re.finditer(pat, text))
        if n >= len(ms): return text
        m = ms[n]
        return text[:m.start()] + repl + text[m.end():]
    content = replace_nth(r'<mass>[0-9.e+\-]+</mass>', f'<mass>$DEF_MASS</mass>', content, 0)
    for pat, repl in [
        (r'<timeConstantUp>[0-9.e+\-]+</timeConstantUp>', f'<timeConstantUp>$DEF_TC_UP</timeConstantUp>'),
        (r'<timeConstantDown>[0-9.e+\-]+</timeConstantDown>', f'<timeConstantDown>$DEF_TC_DOWN</timeConstantDown>'),
        (r'<motorConstant>[0-9.e+\-]+</motorConstant>', f'<motorConstant>$DEF_MOTOR_K</motorConstant>'),
        (r'<maxRotVelocity>[0-9.e+\-]+</maxRotVelocity>', f'<maxRotVelocity>$DEF_MAX_RPM</maxRotVelocity>'),
        (r'<rotorDragCoefficient>[0-9.e+\-]+</rotorDragCoefficient>', f'<rotorDragCoefficient>$DEF_DRAG</rotorDragCoefficient>'),
    ]:
        content = re.sub(pat, repl, content)
    # inertia: 只替换第 0 个 (base_link), IMU 和旋翼保持不变
    for pat, repl in [
        (r'<ixx>[0-9.e+\-]+</ixx>', f'<ixx>$DEF_IXX</ixx>'),
        (r'<iyy>[0-9.e+\-]+</iyy>', f'<iyy>$DEF_IYY</iyy>'),
        (r'<izz>[0-9.e+\-]+</izz>', f'<izz>$DEF_IZZ</izz>'),
    ]:
        content = replace_nth(pat, repl, content, 0)
    with open(fpath, "w") as f: f.write(content)
PYEOF
        echo "已恢复为 Iris 默认参数"
        exit 0
        ;;
esac

# ---- 应用参数 ----
read cur_m cur_tu cur_td cur_mk cur_dr cur_mr cur_ix cur_iy cur_iz <<< $(_get)

# 自动换算: 桨尺寸 → motor_k, drag_coef, max_rpm, tc
prop=${NEW_PROP:-$DEF_PROP}
prop_ratio=$(python3 -c "print(f'{float($prop)/float($DEF_PROP):.4f}')")

# motor_k ∝ diameter⁴ (动量理论: 推力 ∝ 桨盘面积·诱导速度² ∝ D⁴)
auto_motor_k=$(python3 -c "print(f'{float($DEF_MOTOR_K) * float($prop_ratio)**4:.6e}')")

# drag_coef ∝ 桨面积 ∝ diameter²
auto_drag=$(python3 -c "print(f'{float($DEF_DRAG) * float($prop_ratio)**2:.6f}')")

# max_rpm: 大桨安全转速更低 (经验: 15寸≈7000RPM≈733rad/s), Iris 10寸≈10500RPM≈1100rad/s
# 粗略: max_rpm ∝ 1/diameter (保持桨尖速度近似)
auto_max_rpm=$(python3 -c "print(f'{int(float($DEF_MAX_RPM) / float($prop_ratio))}')")

# 时间常数: 大桨惯量大, ∝ diameter³ (质量∝D² × 半径²∝D² → 惯量∝D⁴, 但电机力矩也∝D)
auto_tc_up=$(python3 -c "print(f'{float($DEF_TC_UP) * float($prop_ratio)**2:.4f}')")
auto_tc_down=$(python3 -c "print(f'{float($DEF_TC_DOWN) * float($prop_ratio)**2:.4f}')")

# 使用用户指定值或自动换算值
mass=${NEW_MASS:-$cur_m}
tc_up=${NEW_TC_UP:-$auto_tc_up}
tc_down=${NEW_TC_DOWN:-$auto_tc_down}
motor_k=${NEW_MOTOR_K:-$auto_motor_k}
max_rpm=${NEW_MAX_RPM:-$auto_max_rpm}
drag_coef=${NEW_DRAG:-$auto_drag}

# 转动惯量: I ∝ mass · arm_radius²
# arm_radius ∝ wheelbase
wheelbase=${NEW_WHEELBASE:-$DEF_WHEELBASE}
scale=$(python3 -c "
mr=float($mass)/float($cur_m)
wr=float($wheelbase)/float($DEF_WHEELBASE)
print(f'{mr * wr**2:.6f}')")
ixx=$(python3 -c "print(f'{float($cur_ix) * $scale:.6f}')")
iyy=$(python3 -c "print(f'{float($cur_iy) * $scale:.6f}')")
izz=$(python3 -c "print(f'{float($cur_iz) * $scale:.6f}')")

# 备份
bak=$(_backup)
echo "已备份到: $(basename $bak)"

# 写入 — mass 只改第一个出现 (base_link), 用 Python 精确定位
python3 << PYEOF
import re

for fpath in ["$IRIS_JINJA", "$IRIS_SDF"]:
    import os
    if not os.path.exists(fpath):
        continue
    with open(fpath, "r") as f:
        content = f.read()

    def replace_nth(pattern, repl, text, n):
        matches = list(re.finditer(pattern, text))
        if n >= len(matches): return text
        m = matches[n]
        return text[:m.start()] + repl + text[m.end():]

    # mass: 只替换第 0 个 (base_link), IMU 和旋翼保持不变
    content = replace_nth(r'<mass>[0-9.e+\-]+</mass>', f'<mass>$mass</mass>', content, 0)
    # 其他标签: 替换所有出现
    for pat, repl in [
        (r'<timeConstantUp>[0-9.e+\-]+</timeConstantUp>', f'<timeConstantUp>$tc_up</timeConstantUp>'),
        (r'<timeConstantDown>[0-9.e+\-]+</timeConstantDown>', f'<timeConstantDown>$tc_down</timeConstantDown>'),
        (r'<motorConstant>[0-9.e+\-]+</motorConstant>', f'<motorConstant>$motor_k</motorConstant>'),
        (r'<maxRotVelocity>[0-9.e+\-]+</maxRotVelocity>', f'<maxRotVelocity>$max_rpm</maxRotVelocity>'),
        (r'<rotorDragCoefficient>[0-9.e+\-]+</rotorDragCoefficient>', f'<rotorDragCoefficient>$drag_coef</rotorDragCoefficient>'),
    ]:
        content = re.sub(pat, repl, content)
    # inertia: 只替换第 0 个 (base_link), IMU 和旋翼保持不变
    for pat, repl in [
        (r'<ixx>[0-9.e+\-]+</ixx>', f'<ixx>$ixx</ixx>'),
        (r'<iyy>[0-9.e+\-]+</iyy>', f'<iyy>$iyy</iyy>'),
        (r'<izz>[0-9.e+\-]+</izz>', f'<izz>$izz</izz>'),
    ]:
        content = replace_nth(pat, repl, content, 0)

    with open(fpath, "w") as f:
        f.write(content)
PYEOF

# 显示
echo ""
echo "=== 更新后 Iris 物理参数 ==="
printf "  %-12s %-16s → %s\n" mass       "$cur_m kg"          "$mass kg"
printf "  %-12s %-16s → %s\n" prop       "${DEF_PROP}英寸"     "${prop}英寸"
printf "  %-12s %-16s → %s\n" wheelbase  "${DEF_WHEELBASE}mm"  "${wheelbase}mm"
echo   "  --- 自动换算 (基于桨尺寸) ---"
printf "  %-12s %-16s → %s\n" motor_k    "$cur_mk"            "$motor_k"
printf "  %-12s %-16s → %s\n" max_rpm    "$cur_mr rad/s"      "$max_rpm rad/s"
printf "  %-12s %-16s → %s\n" tc_up      "$cur_tu s"           "$tc_up s"
printf "  %-12s %-16s → %s\n" tc_down    "$cur_td s"           "$tc_down s"
printf "  %-12s %-16s → %s\n" drag_coef  "$cur_dr"             "$drag_coef"
echo   "  --- 转动惯量 (质量 × 轴距²) ---"
printf "  %-12s %-16s → %s\n" ixx        "$cur_ix"            "$ixx"
printf "  %-12s %-16s → %s\n" iyy        "$cur_iy"            "$iyy"
printf "  %-12s %-16s → %s\n" izz        "$cur_iz"            "$izz"

hr=$(python3 -c "m=float($mass); k=float($motor_k); g=9.81; w=(m*g/(4*k))**0.5; print(f'{w:.0f}')")
hr_rpm=$(python3 -c "print(f'{float($hr)*60/6.28318:.0f}')")
max_rpm_pct=$(python3 -c "print(f'{float($hr)/float($max_rpm)*100:.0f}')")
tw_max=$(python3 -c "print(f'{4*float($motor_k)*float($max_rpm)**2/(float($mass)*9.81):.1f}')")
echo ""
echo "  悬停转速 ≈ $hr rad/s ($hr_rpm RPM)  (最大转速的 ${max_rpm_pct}%)"
echo "  最大推重比 ≈ ${tw_max}:1"
echo ""
echo "  ⚠ 修改后需重新编译: cd ~/PX4-Autopilot && make px4_sitl gazebo-classic"
echo "  恢复默认: bash ~/SITL/scripts/set_drone_params.sh --restore"
