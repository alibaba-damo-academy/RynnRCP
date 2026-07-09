#!/bin/bash
# ============================================================
# RynnRCP 仿真一键启动脚本
# ============================================================
# 用法:
#   bash start_rcp.sh --config so101    # SO101 (6 DOF, 2 cameras)
#   bash start_rcp.sh --config rm75     # RM75 (9 DOF, 5 cameras)
#   bash start_rcp.sh                   # 默认 so101
#
# 前置: 仿真已运行, 已执行过 setup_sim.sh
# Ctrl+C 停止所有进程
# ============================================================

set -e

# 清除仿真传入的 PYTHONPATH，避免加载容器内旧代码
unset PYTHONPATH

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
VENV="$REPO_ROOT/venv"
CONFIG_DIR="$SCRIPT_DIR/rynnrcp_robot_sim/config"

# 解析参数
ROBOT_CONFIG="so101"
while [[ $# -gt 0 ]]; do
    case $1 in
        --config) ROBOT_CONFIG="$2"; shift 2 ;;
        *) shift ;;
    esac
done

SERVER_CONFIG="$CONFIG_DIR/sim_server_${ROBOT_CONFIG}.yaml"
RYNNBOT_CONFIG="$CONFIG_DIR/sim_rynnbot_app.yaml"
ACTION_BRIDGE="$SCRIPT_DIR/action_bridge.py"

if [ ! -f "$SERVER_CONFIG" ]; then
    echo "[ERROR] 配置文件不存在: $SERVER_CONFIG"
    echo "  可用配置: $(ls $CONFIG_DIR/sim_server_*.yaml 2>/dev/null | sed 's/.*sim_server_//;s/.yaml//' | tr '\n' ' ')"
    exit 1
fi

if [ ! -d "$VENV" ]; then
    echo "[ERROR] venv 不存在，请先运行: bash setup_sim.sh"
    exit 1
fi

source "$VENV/bin/activate"

# 清理旧进程
pkill -f "rynnrcp-server.*sim" 2>/dev/null || true
pkill -f "rynnrcp-rynnbot-app.*sim" 2>/dev/null || true
pkill -f "action_bridge.py" 2>/dev/null || true
sleep 0.5

PIDS=()

cleanup() {
    echo ""
    echo "[RCP] 停止所有进程..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    sleep 0.5
    for pid in "${PIDS[@]}"; do
        kill -9 "$pid" 2>/dev/null || true
    done
    echo "[RCP] 已停止"
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "[RCP] 配置: $ROBOT_CONFIG ($SERVER_CONFIG)"

# 1. rynnrcp-server
echo "[RCP] 启动 rynnrcp-server..."
rynnrcp-server --config "$SERVER_CONFIG" &
PIDS+=($!)
sleep 3
kill -0 "${PIDS[0]}" 2>/dev/null || { echo "[ERROR] rynnrcp-server 启动失败"; exit 1; }
echo "[RCP] rynnrcp-server OK (PID ${PIDS[0]})"

# 2. rynnrcp-rynnbot-app
echo "[RCP] 启动 rynnrcp-rynnbot-app..."
rynnrcp-rynnbot-app --config "$RYNNBOT_CONFIG" --server-config "$SERVER_CONFIG" &
PIDS+=($!)
sleep 1
if kill -0 "${PIDS[1]}" 2>/dev/null; then
    echo "[RCP] rynnrcp-rynnbot-app OK (PID ${PIDS[1]})"
else
    echo "[WARNING] rynnbot-app 启动失败，请检查凭据: $RYNNBOT_CONFIG"
fi

# 3. action_bridge
echo "[RCP] 启动 action_bridge..."
python "$ACTION_BRIDGE" --robot-id "${ROBOT_CONFIG}_sim" &
PIDS+=($!)
sleep 3
kill -0 "${PIDS[2]}" 2>/dev/null && echo "[RCP] action_bridge OK (PID ${PIDS[2]})"

echo ""
echo "============================================================"
echo "  RynnRCP 已启动 [$ROBOT_CONFIG] | Ctrl+C 停止"
echo "============================================================"
echo ""

wait -n "${PIDS[@]}" 2>/dev/null
cleanup
