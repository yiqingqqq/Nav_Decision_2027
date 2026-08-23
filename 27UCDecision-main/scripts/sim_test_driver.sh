#!/usr/bin/env bash
# RMUC 2026 哨兵决策系统 — 条件转换与巡航策略转换仿真测试
#
# 测试内容:
#   1) 三套巡航预设: defense → balanced → aggressive
#   2) 条件转换: 低血量→补给, 受击→撤退, 导航失败→fallback
#
# 用法:
#   bash scripts/sim_test_driver.sh [-i|--interactive]
#
# 日志:
#   /tmp/bt_<preset>.log  — BT 节点日志（关键）
#   /tmp/sim_<preset>.log — 仿真信号节点日志

set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
LOG_DIR="/tmp"

R='\033[91m'; G='\033[92m'; Y='\033[93m'; B='\033[94m'; C='\033[96m'; M='\033[95m'; BOLD='\033[1m'; NC='\033[0m'

banner() { echo -e "\n${BOLD}${C}══════════════════════════════════════════════════════════════${NC}"; echo -e "${BOLD}${C}  $1${NC}"; echo -e "${BOLD}${C}══════════════════════════════════════════════════════════════${NC}"; }
phase()  { echo -e "\n${BOLD}${G}>>> $1${NC}"; }
waitkey() { if [ "${INTERACTIVE:-0}" = 1 ]; then read -p "$(echo -e ${BOLD}按回车继续...${NC})"; fi; }
signal() { echo -e "${BOLD}${Y}  [信号] $1${NC}"; }

INTERACTIVE=0
[[ "${1:-}" =~ ^-i ]] && INTERACTIVE=1

# Source
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

cleanup() {
    pkill -f "rm_behavior_tree" 2>/dev/null || true
    pkill -f "sim_signal_node"  2>/dev/null || true
    rm -f /tmp/sim_game_run /tmp/sim_game_stop /tmp/sim_hp_low \
          /tmp/sim_hp_mid /tmp/sim_hp_high /tmp/sim_nav_fail
}
trap cleanup EXIT

run_preset() {
    local preset="$1" desc="$2"
    local bt_log="$LOG_DIR/bt_${preset}.log"
    local sim_log="$LOG_DIR/sim_${preset}.log"

    cleanup
    banner "策略: ${preset} — ${desc}"

    # ── 1. Start mock environment ──
    phase "[0] 启动仿真信号节点 (mock Nav2 + TF + 游戏信号)"
    /usr/bin/python3.10 "$SCRIPT_DIR/sim_signal_node.py" > "$sim_log" 2>&1 &
    SIM_PID=$!
    echo "  PID=$SIM_PID, log: $sim_log"
    sleep 2

    # ── 2. Start BT node with this preset ──
    phase "[0] 启动 BT (preset=$preset)"
    ros2 launch rm_behavior_tree rm_behavior_tree.launch.py \
        style:=rmuc_2026.xml preset:="$preset" alliance:=red use_sim_time:=True \
        > "$bt_log" 2>&1 &
    BT_PID=$!
    echo "  PID=$BT_PID, log: $bt_log"
    sleep 3

    # Verify processes alive
    kill -0 $SIM_PID 2>/dev/null || { echo -e "${R}[FATAL] sim_signal_node died${NC}"; return 1; }
    kill -0 $BT_PID  2>/dev/null || { echo -e "${R}[FATAL] rm_behavior_tree died${NC}"; return 1; }

    # ──────────────────────────────────────────────────────────
    # Phase A: Normal patrol
    # ──────────────────────────────────────────────────────────
    phase "[A] 比赛开始: GameProgress=4, HP=500"
    signal "touch /tmp/sim_game_run"
    touch /tmp/sim_game_run
    waitkey && sleep 5

    phase "[A] 继续观察巡航 (再等 5 秒, 看 patrol 循环)"
    waitkey && sleep 5

    # ──────────────────────────────────────────────────────────
    # Phase B: Low HP → Supply
    # ──────────────────────────────────────────────────────────
    phase "[B] 低血量触发: HP → 70 (< hp_retreat=110)"
    signal "touch /tmp/sim_hp_low"
    touch /tmp/sim_hp_low
    echo -e "  ${Y}期望: BT 中断巡航 → 向 supply_waypoint 导航${NC}"
    waitkey && sleep 6

    # ──────────────────────────────────────────────────────────
    # Phase C: Partial recovery (hysteresis)
    # ──────────────────────────────────────────────────────────
    phase "[C] 部分恢复: HP → 200 (仍 < hp_recover=400, 滞回中)"
    signal "touch /tmp/sim_hp_mid"
    touch /tmp/sim_hp_mid
    echo -e "  ${Y}期望: 仍保持 Supply (不会返回巡航)${NC}"
    waitkey && sleep 5

    # ──────────────────────────────────────────────────────────
    # Phase D: Full recovery → resume patrol
    # ──────────────────────────────────────────────────────────
    phase "[D] 完全恢复: HP → 600 (> hp_recover=400)"
    signal "touch /tmp/sim_hp_high"
    touch /tmp/sim_hp_high
    echo -e "  ${Y}期望: 恢复巡航, 继续 patrol route${NC}"
    waitkey && sleep 6

    # ──────────────────────────────────────────────────────────
    # Phase E: Navigation failure → fallback
    # ──────────────────────────────────────────────────────────
    phase "[E] 导航失败: 模拟 Nav2 ABORT"
    signal "touch /tmp/sim_nav_fail"
    touch /tmp/sim_nav_fail
    echo -e "  ${Y}期望: BT 重试 2 次 → fallback_waypoint${NC}"
    waitkey && sleep 8

    # ──────────────────────────────────────────────────────────
    # Phase F: Resume
    # ──────────────────────────────────────────────────────────
    phase "[F] 恢复正常巡航"
    echo -e "  ${Y}期望: 继续 patrol route 循环${NC}"
    waitkey && sleep 6

    # ── Stop ──
    phase "[✓] ${preset} 测试完成"
    kill $SIM_PID 2>/dev/null || true
    kill $BT_PID  2>/dev/null || true
    cleanup
    sleep 1

    # Print last 15 lines of BT log
    echo -e "\n${BOLD}${C}BT 日志尾段 (${bt_log}):${NC}"
    tail -20 "$bt_log" 2>/dev/null || echo "(empty)"
    echo ""
}

# ── Main ──
echo -e "${BOLD}${M}RMUC 2026 哨兵决策 — 条件转换 + 三预设巡航仿真测试${NC}"
echo -e "日志目录: $LOG_DIR\n"

for entry in "defense|基地/前哨防守巡航" "balanced|中场控制巡航 (默认)" "aggressive|进攻区前压巡航"; do
    IFS='|' read -r preset desc <<< "$entry"
    run_preset "$preset" "$desc"
done

banner "全部测试完成!"
echo -e "${G}日志文件:${NC}"
echo "  $LOG_DIR/bt_defense.log"
echo "  $LOG_DIR/bt_balanced.log"
echo "  $LOG_DIR/bt_aggressive.log"
echo -e "\n${BOLD}比较三个日志中的 route_index / waypoint 序列即可看出预设差异${NC}"
