#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
VENV_DIR="$SCRIPT_DIR/venv"
PYTHON_BIN="${PYTHON:-}"
PIP_INDEX_URL="${PIP_INDEX_URL:-}"
DEFAULT_PIP_INDEX_URLS=("https://pypi.tuna.tsinghua.edu.cn/simple" "https://mirrors.aliyun.com/pypi/simple" "")
PYTHON_CMD=()
RECREATE=0

usage() {
  cat <<'EOF'
Usage: bash setup_lekiwi.sh [options]

Options:
  --python PATH          Python 3.10 interpreter.
  --venv PATH            Virtual environment path (default: venv).
  --pip-index-url URL    Python package index. When omitted, tries Tsinghua, Aliyun, then pip default.
  --recreate             Recreate the virtual environment.
  -h, --help             Show this help.

After setup:
  rynnrcp-lekiwi-configure-web
  rynnrcp-server --config rynnrcp_robot_lekiwi/config/lekiwi_server.yaml
  rynnrcp-server --config rynnrcp_robot_lekiwi/config/lekiwi_leader_server.yaml
  Debug UI: open the address printed by each Server.
  rynnrcp-protocol-debug --config rynnrcp_robot_lekiwi/config/lekiwi_server.yaml
  rynnrcp-mcp-app --server-config rynnrcp_robot_lekiwi/config/lekiwi_server.yaml
  rynnrcp-rynnbot-app --config rynnrcp_robot_lekiwi/config/lekiwi_rynnbot_app.yaml --server-config rynnrcp_robot_lekiwi/config/lekiwi_server.yaml
  rynnrcp-teleop-app
EOF
}

fail() {
  printf 'Error: %s\n' "$*" >&2
  exit 1
}

log() {
  printf '%s\n' "$*"
}

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

is_python_310() {
  "$@" -c 'import sys; raise SystemExit(0 if sys.version_info[:2] == (3, 10) else 1)' >/dev/null 2>&1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --python) [[ $# -ge 2 ]] || fail "--python requires a path"; PYTHON_BIN="$2"; shift 2 ;;
    --venv) [[ $# -ge 2 ]] || fail "--venv requires a path"; VENV_DIR="$2"; shift 2 ;;
    --pip-index-url) [[ $# -ge 2 ]] || fail "--pip-index-url requires a URL"; PIP_INDEX_URL="$2"; shift 2 ;;
    --recreate) RECREATE=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) fail "unknown option: $1" ;;
  esac
done

if [[ -n "$PYTHON_BIN" ]]; then
  PYTHON_CMD=("$PYTHON_BIN")
elif command -v python3.10 >/dev/null 2>&1 && is_python_310 "$(command -v python3.10)"; then
  PYTHON_CMD=("$(command -v python3.10)")
elif command -v python >/dev/null 2>&1 && is_python_310 "$(command -v python)"; then
  PYTHON_CMD=("$(command -v python)")
elif command -v python3 >/dev/null 2>&1 && is_python_310 "$(command -v python3)"; then
  PYTHON_CMD=("$(command -v python3)")
elif command -v py >/dev/null 2>&1 && is_python_310 "$(command -v py)" -3.10; then
  PYTHON_CMD=("$(command -v py)" -3.10)
fi

[[ "${#PYTHON_CMD[@]}" -gt 0 ]] || fail "Python 3.10 was not found"

cd "$REPO_ROOT"
[[ -f pyproject.toml ]] || fail "repository root not found: $REPO_ROOT"
[[ -d robots/lerobot_lekiwi ]] || fail "robots/lerobot_lekiwi not found"

if [[ "$RECREATE" -eq 1 && -d "$VENV_DIR" ]]; then
  rm -rf "$VENV_DIR"
fi
if [[ ! -d "$VENV_DIR" ]]; then
  "${PYTHON_CMD[@]}" -m venv "$VENV_DIR"
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

pip_install --upgrade pip setuptools wheel
pip_install \
  -e "$REPO_ROOT/rynnkit" \
  -e "$REPO_ROOT" \
  -e "$REPO_ROOT/apps/protocol_debug" \
  -e "$REPO_ROOT/apps/common" \
  -e "$REPO_ROOT/apps/mcp" \
  -e "$REPO_ROOT/apps/rynnbot" \
  -e "$REPO_ROOT/apps/teleop" \
  -e "$REPO_ROOT/robots/lerobot_lekiwi"

"$VENV_PYTHON" - <<'PY'
import rynnrcp
import rynnkit
import lerobot_lekiwi
import rynnrcp_app_protocol_debug
import rynnrcp_robot_lekiwi
import pynput
import scservo_sdk
print("LeKiwi imports OK")
PY

printf '\nLeKiwi setup completed.\n'
VENV_DISPLAY="$VENV_DIR"
[[ "$VENV_DIR" == "$SCRIPT_DIR/venv" ]] && VENV_DISPLAY="venv"
if [[ "$VENV_BIN" == *"/Scripts" ]]; then
  printf 'Activate: source %s/Scripts/activate\n' "$VENV_DISPLAY"
else
  printf 'Activate: source %s/bin/activate\n' "$VENV_DISPLAY"
fi
printf 'Configure and calibrate: rynnrcp-lekiwi-configure-web\n'
printf 'Start: rynnrcp-server --config rynnrcp_robot_lekiwi/config/lekiwi_server.yaml\n'
printf 'Leader start: rynnrcp-server --config rynnrcp_robot_lekiwi/config/lekiwi_leader_server.yaml\n'
printf 'Debug UI: open the address printed by each Server.\n'
printf 'Protocol Debug: rynnrcp-protocol-debug --config rynnrcp_robot_lekiwi/config/lekiwi_server.yaml\n'
printf 'MCP: rynnrcp-mcp-app --server-config rynnrcp_robot_lekiwi/config/lekiwi_server.yaml\n'
printf 'RynnBot: rynnrcp-rynnbot-app --config rynnrcp_robot_lekiwi/config/lekiwi_rynnbot_app.yaml --server-config rynnrcp_robot_lekiwi/config/lekiwi_server.yaml\n'
printf 'Teleop: rynnrcp-teleop-app\n'
