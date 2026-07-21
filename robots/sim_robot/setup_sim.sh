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
Usage: bash setup_sim.sh [options]

Set up a Sim Robot-local virtual environment and install RynnRCP as a local library.

Options:
  --python PATH        Python interpreter to use.
  --venv PATH          Virtual environment path. Defaults to venv.
  --pip-index-url URL  Python package index. When omitted, tries Tsinghua, Aliyun, then pip default.
  --recreate           Remove and recreate the virtual environment.
  -h, --help           Show this help.

After setup:
  source venv/bin/activate
  bash start_rcp.sh --config so101
  Debug UI: open the address printed by each Server.
  rynnrcp-protocol-debug --config rynnrcp_robot_sim/config/sim_server_so101.yaml
  rynnrcp-mcp-app --server-config rynnrcp_robot_sim/config/sim_server_so101.yaml
  rynnrcp-rynnbot-app --config rynnrcp_robot_sim/config/sim_rynnbot_app.yaml --server-config rynnrcp_robot_sim/config/sim_server_so101.yaml
  rynnrcp-teleop-app
EOF
}

log() { printf '%s\n' "$*"; }
fail() { printf 'Error: %s\n' "$*" >&2; exit 1; }

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
elif [[ -x "/isaac-sim/kit/python/bin/python3" ]]; then
  PYTHON_CMD=("/isaac-sim/kit/python/bin/python3")
elif command -v python3 >/dev/null 2>&1; then
  PYTHON_CMD=("$(command -v python3)")
elif command -v python >/dev/null 2>&1; then
  PYTHON_CMD=("$(command -v python)")
fi

[[ "${#PYTHON_CMD[@]}" -gt 0 ]] || fail "Python was not found"
[[ -x "${PYTHON_CMD[0]}" ]] || fail "Python interpreter is not executable: ${PYTHON_CMD[0]}"

log "Repository: $REPO_ROOT"
log "Python:     $("${PYTHON_CMD[@]}" -c 'import sys; print(f"{sys.executable} ({sys.version.split()[0]})")')"
log "Venv:       $VENV_DIR"
log "Pip indexes: ${PIP_INDEX_URL:-Tsinghua -> Aliyun -> pip default}"

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
  -e "$REPO_ROOT/robots/sim_robot"

"$VENV_PYTHON" - <<'PY'
import rynnrcp
import rynnrcp_app_common
import rynnrcp_app_mcp
import rynnrcp_app_protocol_debug
import rynnrcp_app_rynnbot
import rynnrcp_app_teleop
import rynnrcp_robot_sim
print("Imports OK")
PY

log "Sim Robot setup completed."
log "Activate the environment:"
VENV_DISPLAY="$VENV_DIR"
[[ "$VENV_DIR" == "$SCRIPT_DIR/venv" ]] && VENV_DISPLAY="venv"
if [[ "$VENV_BIN" == *"/Scripts" ]]; then
  log "  source $VENV_DISPLAY/Scripts/activate"
else
  log "  source $VENV_DISPLAY/bin/activate"
fi
log "Next steps:"
log "  bash start_rcp.sh --config so101"
log "  bash start_rcp.sh --config rm75"
log "  Debug UI: open the address printed by each Server."
log "  rynnrcp-protocol-debug --config rynnrcp_robot_sim/config/sim_server_so101.yaml"
log "  rynnrcp-mcp-app --server-config rynnrcp_robot_sim/config/sim_server_so101.yaml"
log "  rynnrcp-rynnbot-app --config rynnrcp_robot_sim/config/sim_rynnbot_app.yaml --server-config rynnrcp_robot_sim/config/sim_server_so101.yaml"
log "  rynnrcp-teleop-app"
