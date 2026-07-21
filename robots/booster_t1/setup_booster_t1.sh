#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
VENV_DIR="$SCRIPT_DIR/venv"
PYTHON_BIN="${PYTHON:-}"
PIP_INDEX_URL="${PIP_INDEX_URL:-}"
DEFAULT_PIP_INDEX_URLS=("https://pypi.tuna.tsinghua.edu.cn/simple" "https://mirrors.aliyun.com/pypi/simple" "")
BOOSTER_SDK_VERSION="${BOOSTER_SDK_VERSION:-1.5.6}"
PYTHON_CMD=()
RECREATE=0

usage() {
  cat <<'EOF'
Usage: bash setup_booster_t1.sh [options]

Create a robot-local RynnRCP environment for Booster T1 high-level and low-level control.
This script installs the Booster Python SDK version used by RynnRCP and creates
the venv with system site packages enabled for robot-side runtime libraries.

Options:
  --python PATH        Python interpreter to use.
  --venv PATH          Virtual environment path. Defaults to venv.
  --booster-sdk-version VERSION
                       Booster Python SDK version. Defaults to 1.5.6.
  --pip-index-url URL  Python package index. When omitted, tries Tsinghua, Aliyun, then pip default.
  --recreate           Remove and recreate the virtual environment.
  -h, --help           Show this help.

After setup:
  source venv/bin/activate
  rynnrcp-server --config rynnrcp_robot_booster_t1/config/t1_high_server.yaml
  Debug UI: open the address printed by each Server.
  rynnrcp-protocol-debug --config rynnrcp_robot_booster_t1/config/t1_high_server.yaml
  rynnrcp-mcp-app --server-config rynnrcp_robot_booster_t1/config/t1_high_server.yaml
  rynnrcp-rynnbot-app --config rynnrcp_robot_booster_t1/config/booster_t1_rynnbot_app.yaml --server-config rynnrcp_robot_booster_t1/config/t1_high_server.yaml
EOF
}

fail() { printf 'Error: %s\n' "$*" >&2; exit 1; }
log() { printf '%s\n' "$*"; }

pip_install() {
  local index
  local indexes=()
  if [[ -n "$PIP_INDEX_URL" ]]; then
    indexes=("$PIP_INDEX_URL")
  else
    indexes=("${DEFAULT_PIP_INDEX_URLS[@]}")
  fi
  for index in "${indexes[@]}"; do
    if [[ -n "$index" ]]; then
      log "pip install via $index: $*"
      "$VENV_PYTHON" -m pip install --index-url "$index" "$@" && return 0
    else
      log "pip install via pip default index: $*"
      "$VENV_PYTHON" -m pip install "$@" && return 0
    fi
  done
  return 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --python)
      [[ $# -ge 2 ]] || fail "--python requires a path"
      PYTHON_BIN="$2"
      shift 2
      ;;
    --venv)
      [[ $# -ge 2 ]] || fail "--venv requires a path"
      VENV_DIR="$2"
      shift 2
      ;;
    --booster-sdk-version)
      [[ $# -ge 2 ]] || fail "--booster-sdk-version requires a version"
      BOOSTER_SDK_VERSION="$2"
      shift 2
      ;;
    --pip-index-url)
      [[ $# -ge 2 ]] || fail "--pip-index-url requires a URL"
      PIP_INDEX_URL="$2"
      shift 2
      ;;
    --recreate)
      RECREATE=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      fail "unknown option: $1"
      ;;
  esac
done

cd "$REPO_ROOT"
[[ -f "$REPO_ROOT/pyproject.toml" ]] || fail "repository root not found: $REPO_ROOT"

if [[ -n "$PYTHON_BIN" ]]; then
  PYTHON_CMD=("$PYTHON_BIN")
elif command -v python3 >/dev/null 2>&1; then
  PYTHON_CMD=("$(command -v python3)")
elif command -v python >/dev/null 2>&1; then
  PYTHON_CMD=("$(command -v python)")
fi

[[ "${#PYTHON_CMD[@]}" -gt 0 ]] || fail "Python was not found"
[[ -x "${PYTHON_CMD[0]}" ]] || fail "Python interpreter is not executable: ${PYTHON_CMD[0]}"

if [[ "$RECREATE" -eq 1 && -d "$VENV_DIR" ]]; then
  rm -rf "$VENV_DIR"
fi

if [[ ! -d "$VENV_DIR" ]]; then
  "${PYTHON_CMD[@]}" -m venv --system-site-packages "$VENV_DIR"
fi

if [[ -x "$VENV_DIR/bin/python" ]]; then
  VENV_PYTHON="$VENV_DIR/bin/python"
  VENV_BIN="$VENV_DIR/bin"
elif [[ -x "$VENV_DIR/Scripts/python.exe" ]]; then
  VENV_PYTHON="$VENV_DIR/Scripts/python.exe"
  VENV_BIN="$VENV_DIR/Scripts"
else
  fail "virtual environment Python not found under $VENV_DIR"
fi

log "Repository: $REPO_ROOT"
log "Python:     $("${PYTHON_CMD[@]}" -c 'import sys; print(f"{sys.executable} ({sys.version.split()[0]})")')"
log "Venv:       $VENV_DIR"
log "Pip indexes: ${PIP_INDEX_URL:-Tsinghua -> Aliyun -> pip default}"
log "Booster SDK: booster_robotics_sdk_python==$BOOSTER_SDK_VERSION"

pip_install --upgrade pip setuptools wheel
pip_install "booster_robotics_sdk_python==$BOOSTER_SDK_VERSION"
pip_install \
  -e "$REPO_ROOT" \
  -e "$REPO_ROOT/apps/protocol_debug" \
  -e "$REPO_ROOT/apps/common" \
  -e "$REPO_ROOT/apps/mcp" \
  -e "$REPO_ROOT/apps/rynnbot" \
  -e "$REPO_ROOT/robots/booster_t1"

"$VENV_PYTHON" - <<'PY'
import importlib.metadata
import booster_robotics_sdk_python as sdk
from rynnrcp_robot_booster_t1.controller import BoosterT1HighController
import rynnrcp_app_mcp
import rynnrcp_app_protocol_debug
import rynnrcp_app_rynnbot

print("Booster SDK import OK:", getattr(sdk, "__file__", "<builtin>"))
print("Booster SDK version:", importlib.metadata.version("booster_robotics_sdk_python"))
print("RobotMode:", [x for x in dir(sdk.RobotMode) if not x.startswith("_")])
print("RCP controller import OK:", BoosterT1HighController.__name__)
print("MCP app import OK:", rynnrcp_app_mcp.__name__)
print("Protocol Debug app import OK:", rynnrcp_app_protocol_debug.__name__)
print("RynnBot app import OK:", rynnrcp_app_rynnbot.__name__)
PY

log "Booster T1 setup completed."
log "Activate the environment:"
VENV_DISPLAY="$VENV_DIR"
[[ "$VENV_DIR" == "$SCRIPT_DIR/venv" ]] && VENV_DISPLAY="venv"
if [[ "$VENV_BIN" == *"/Scripts" ]]; then
  log "  source $VENV_DISPLAY/Scripts/activate"
else
  log "  source $VENV_DISPLAY/bin/activate"
fi
log "Next steps:"
log "  rynnrcp-server --config rynnrcp_robot_booster_t1/config/t1_high_server.yaml"
log "  rynnrcp-server --config rynnrcp_robot_booster_t1/config/t1_low_server.yaml"
log "  Debug UI: open the address printed by each Server."
log "  rynnrcp-protocol-debug --config rynnrcp_robot_booster_t1/config/t1_high_server.yaml"
log "  rynnrcp-mcp-app --server-config rynnrcp_robot_booster_t1/config/t1_high_server.yaml"
log "  rynnrcp-rynnbot-app --config rynnrcp_robot_booster_t1/config/booster_t1_rynnbot_app.yaml --server-config rynnrcp_robot_booster_t1/config/t1_high_server.yaml"
