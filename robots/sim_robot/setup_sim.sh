#!/bin/bash
# 安装 RynnRCP 仿真机器人包
# 用法: cd robots/sim_robot && bash setup_sim.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "[Sim Robot] Setting up from: $REPO_ROOT"

if [ -z "$VIRTUAL_ENV" ]; then
    echo "[Sim Robot] Creating Python virtual environment..."
    # 优先使用 Isaac Sim Python（容器内），否则用系统 python3
    if [ -x "/isaac-sim/kit/python/bin/python3" ]; then
        PYTHON_BIN="/isaac-sim/kit/python/bin/python3"
    else
        PYTHON_BIN="python3"
    fi
    $PYTHON_BIN -m venv "$REPO_ROOT/venv"
    source "$REPO_ROOT/venv/bin/activate"
    python -m pip install --upgrade pip setuptools wheel
else
    echo "[Sim Robot] Using active venv: $VIRTUAL_ENV"
fi

echo "[Sim Robot] Installing RynnRCP runtime and apps..."
python -m pip install -e "$REPO_ROOT" \
    -e "$REPO_ROOT/apps/common" \
    -e "$REPO_ROOT/apps/teleop" \
    -e "$REPO_ROOT/apps/mcp" \
    -e "$REPO_ROOT/apps/rynnbot"

echo "[Sim Robot] Installing sim robot package..."
python -m pip install -e "$SCRIPT_DIR"

echo ""
echo "[Sim Robot] Setup complete!"
echo ""
echo "启动方式:"
echo "  bash start_rcp.sh --config so101    # SO101 (6 DOF)"
echo "  bash start_rcp.sh --config rm75     # RM75 (9 DOF)"
echo ""
