#!/bin/bash
# ============================================================
# RynnRCP 仿真一键启动脚本
# ============================================================
# 用法:
#   bash start_rcp.sh --config lerobot_so101_sim_v1
#   bash start_rcp.sh --config lerobot_so101_sim_v2
#   bash start_rcp.sh --config lerobot_so101_dual_sim_v1
#   bash start_rcp.sh --config lerobot_so101_dual_sim_v2
#   bash start_rcp.sh --config aero_hand_sim_v1
#   bash start_rcp.sh --config aero_hand_dual_sim_v1
#   bash start_rcp.sh --config aero_hand_dual_sim_v2
#   bash start_rcp.sh --config rm75_rmg24_sim_v1
#   bash start_rcp.sh --config rm75_rmg24_sim_v2
#   bash start_rcp.sh --config franka_r3_sim_v1
#   bash start_rcp.sh --config franka_r3_sim_v2
#   bash start_rcp.sh                 # 默认 lerobot_so101_sim_v1
#
# 前置: 仿真已运行, 已执行过 setup_sim.sh
# Ctrl+C 停止所有进程
# ============================================================

set -e

# 清除仿真传入的 PYTHONPATH，避免加载容器内旧代码
unset PYTHONPATH

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
VENV="$SCRIPT_DIR/venv"
if [ ! -d "$VENV" ]; then
    VENV="$REPO_ROOT/venv"
fi
CONFIG_DIR="$SCRIPT_DIR/rynnrcp_robot_sim/config"

# 解析参数
ROBOT_CONFIG="lerobot_so101_sim_v1"
while [[ $# -gt 0 ]]; do
    case $1 in
        --config) ROBOT_CONFIG="$2"; shift 2 ;;
        *) shift ;;
    esac
done

SERVER_CONFIG="$CONFIG_DIR/sim_server_${ROBOT_CONFIG}.yaml"
RYNNBOT_CONFIG="$CONFIG_DIR/sim_rynnbot_app.yaml"
ACTION_BRIDGE="$SCRIPT_DIR/action_bridge.py"
ROBOT_ID="$ROBOT_CONFIG"
ACTION_BRIDGE_MAPPINGS=("robot:action.robot.joint_position")

if [[ "$ROBOT_CONFIG" == lerobot_so101_dual_sim_v* ]]; then
    ACTION_BRIDGE_MAPPINGS=(
        "left_robot+right_robot:action.robot.joint_position"
    )
fi

if [ ! -f "$SERVER_CONFIG" ]; then
    echo "[ERROR] 配置文件不存在: $SERVER_CONFIG"
    echo "  可用配置: $(ls $CONFIG_DIR/sim_server_*.yaml 2>/dev/null | sed 's/.*sim_server_//;s/.yaml//' | tr '\n' ' ')"
    exit 1
fi

if [ ! -d "$VENV" ]; then
    echo "[ERROR] venv 不存在，请先运行: bash setup_sim.sh"
    echo "[ERROR] checked: $SCRIPT_DIR/venv and $REPO_ROOT/venv"
    exit 1
fi

source "$VENV/bin/activate"

# 清理旧进程
pkill -f "rynnrcp-server.*sim" 2>/dev/null || true
pkill -f "rynnrcp-rynnbot-app.*sim" 2>/dev/null || true
pkill -f "action_bridge.py" 2>/dev/null || true
sleep 0.2

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

wait_for_robot_id() {
    local robot_id="$1"
    local timeout_s="${2:-10}"
    echo "[RCP] 等待 robot_id 可发现: $robot_id"
    # 使用单次 Python 进程 + local_registry 轮询，避免每次循环重复 import 和 mDNS 等待
    if python - "$robot_id" "$timeout_s" <<'PY' >/dev/null 2>&1
import sys
import time
from rynnrcp.interface.client import ClientInterface
from rynnrcp.interface.protocol_client import connect_to_server

robot_id = sys.argv[1]
timeout_s = float(sys.argv[2])
deadline = time.time() + timeout_s
# 仅使用本地注册表发现，避免 mDNS 超时；server 默认会写入 local_registry
interface = ClientInterface.with_defaults(local_registry=True, mdns=False)
while time.time() < deadline:
    try:
        client = connect_to_server(
            robot_id=robot_id,
            interface=interface,
            timeout_s=0.2,
            request_timeout_ms=200,
        )
        client.close()
        sys.exit(0)
    except Exception:
        time.sleep(0.1)
sys.exit(1)
PY
    then
        echo "[RCP] robot_id 已可发现: $robot_id"
        return 0
    fi
    echo "[ERROR] 等待 robot_id 超时: $robot_id"
    return 1
}

echo "[RCP] 配置: $ROBOT_CONFIG ($SERVER_CONFIG)"

# 1. rynnrcp-server
echo "[RCP] 启动 rynnrcp-server..."
rynnrcp-server --config "$SERVER_CONFIG" &
PIDS+=($!)
sleep 0.2
kill -0 "${PIDS[0]}" 2>/dev/null || { echo "[ERROR] rynnrcp-server 启动失败"; exit 1; }
echo "[RCP] rynnrcp-server OK (PID ${PIDS[0]})"
wait_for_robot_id "$ROBOT_ID" 10

# 2. rynnrcp-rynnbot-app
echo "[RCP] 启动 rynnrcp-rynnbot-app..."
rynnrcp-rynnbot-app --config "$RYNNBOT_CONFIG" --server-config "$SERVER_CONFIG" &
PIDS+=($!)
sleep 0.2
if kill -0 "${PIDS[1]}" 2>/dev/null; then
    echo "[RCP] rynnrcp-rynnbot-app OK (PID ${PIDS[1]})"
else
    echo "[WARNING] rynnbot-app 启动失败，请检查凭据: $RYNNBOT_CONFIG"
fi

# 3. action_bridge
if [[ "${SKIP_ACTION_BRIDGE:-0}" == "1" ]]; then
    echo "[RCP] 跳过 action_bridge (SKIP_ACTION_BRIDGE=1)"
else
    ACTION_BRIDGE_ARGS=(--robot-id "$ROBOT_ID" --port "${PORT:-8080}")
    for mapping in "${ACTION_BRIDGE_MAPPINGS[@]}"; do
        ACTION_BRIDGE_ARGS+=(--mapping "$mapping")
    done

    echo "[RCP] 启动 action_bridge..."
    python "$ACTION_BRIDGE" "${ACTION_BRIDGE_ARGS[@]}" &
    PIDS+=($!)
    sleep 0.2
    kill -0 "${PIDS[-1]}" 2>/dev/null && echo "[RCP] action_bridge OK (PID ${PIDS[-1]})"
fi

echo ""
echo "============================================================"
echo "  RynnRCP 已启动 [$ROBOT_CONFIG] | Ctrl+C 停止"
echo "============================================================"
echo ""

wait -n "${PIDS[@]}" 2>/dev/null
cleanup
