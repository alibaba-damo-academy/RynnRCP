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
Usage: bash setup_so101.sh [options]

Create the SO101 virtual environment and install RynnRCP, its apps, and the
single-arm/dual-arm robot package from this checkout.

Options:
  --python PATH     Python 3.10 interpreter to use.
  --venv PATH       Virtual environment path. Defaults to venv.
  --pip-index-url URL
                   Python package index. When omitted, tries Tsinghua, Aliyun, then pip default.
  --recreate        Remove and recreate the virtual environment.
  -h, --help        Show this help.

After setup:
  macOS/Linux:      source venv/bin/activate
  Windows Git Bash: source venv/Scripts/activate
  Windows PowerShell: .\venv\Scripts\Activate.ps1
  rynnrcp-so101-configure
EOF
}

log() {
  printf '%s\n' "$*"
}

fail() {
  printf 'Error: %s\n' "$*" >&2
  exit 1
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

python_is_310() {
  "$@" - <<'PY' >/dev/null 2>&1
import sys
raise SystemExit(0 if sys.version_info[:2] == (3, 10) else 1)
PY
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
[[ -d "$REPO_ROOT/apps/common" ]] || fail "apps/common not found"
[[ -d "$REPO_ROOT/robots/lerobot_so101" ]] || fail "robots/lerobot_so101 not found"

if [[ -n "$PYTHON_BIN" ]]; then
  PYTHON_CMD=("$PYTHON_BIN")
elif command -v python3.10 >/dev/null 2>&1 && python_is_310 "$(command -v python3.10)"; then
  PYTHON_CMD=("$(command -v python3.10)")
elif command -v python >/dev/null 2>&1 && python_is_310 "$(command -v python)"; then
  PYTHON_CMD=("$(command -v python)")
elif command -v python3 >/dev/null 2>&1 && python_is_310 "$(command -v python3)"; then
  PYTHON_CMD=("$(command -v python3)")
elif command -v py >/dev/null 2>&1 && python_is_310 "$(command -v py)" -3.10; then
  PYTHON_CMD=("$(command -v py)" -3.10)
fi

[[ "${#PYTHON_CMD[@]}" -gt 0 ]] || fail "Python 3.10 was not found"
[[ -x "${PYTHON_CMD[0]}" ]] || fail "Python interpreter is not executable: ${PYTHON_CMD[0]}"

python_is_310 "${PYTHON_CMD[@]}" || fail "Python 3.10 is required: ${PYTHON_CMD[*]}"

log "Repository: $REPO_ROOT"
log "Python:     $("${PYTHON_CMD[@]}" -c 'import sys; print(f"{sys.executable} ({sys.version.split()[0]})")')"
log "Venv:       $VENV_DIR"
log "Pip indexes: ${PIP_INDEX_URL:-Tsinghua -> Aliyun -> pip default}"

if [[ "$RECREATE" -eq 1 && -d "$VENV_DIR" ]]; then
  log "Removing existing virtual environment..."
  rm -rf "$VENV_DIR"
fi

if [[ ! -d "$VENV_DIR" ]]; then
  log "Creating virtual environment..."
  "${PYTHON_CMD[@]}" -m venv "$VENV_DIR"
else
  log "Using existing virtual environment."
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

log "Upgrading installer tools..."
pip_install --upgrade pip setuptools wheel

log "Installing RynnRCP as a local library and official apps..."
pip_install \
  -e "$REPO_ROOT/rynnkit" \
  -e "$REPO_ROOT" \
  -e "$REPO_ROOT/apps/protocol_debug" \
  -e "$REPO_ROOT/apps/common" \
  -e "$REPO_ROOT/apps/mcp" \
  -e "$REPO_ROOT/apps/rynnbot" \
  -e "$REPO_ROOT/apps/teleop"

log "Installing SO101 robot package..."
pip_install -e "$REPO_ROOT/robots/lerobot_so101"

log "Verifying imports..."
"$VENV_PYTHON" - <<'PY'
import rynnrcp
import rynnrcp_app_common
import rynnrcp_app_mcp
import rynnrcp_app_protocol_debug
import rynnrcp_app_rynnbot
import rynnrcp_app_teleop
import rynnrcp_robot_so101
from rynnrcp_robot_so101.controller import SO101BimanualController

assert SO101BimanualController.n_dof == 12
assert SO101BimanualController.task_keys == [
    "observation.state",
    "observation.images.front",
    "observation.images.left_wrist",
    "observation.images.right_wrist",
    "action",
]
print("Imports OK")
PY

log ""
log "SO101 setup completed."
log ""
log "Activate the environment:"
VENV_DISPLAY="$VENV_DIR"
[[ "$VENV_DIR" == "$SCRIPT_DIR/venv" ]] && VENV_DISPLAY="venv"
if [[ "$VENV_BIN" == *"/Scripts" ]]; then
  log "  Git Bash:  source $VENV_DISPLAY/Scripts/activate"
  log "  PowerShell: & \"$VENV_DISPLAY\\Scripts\\Activate.ps1\""
else
  log "  source $VENV_DISPLAY/bin/activate"
fi
log ""
log "Configure the robot:"
log "  rynnrcp-so101-configure"
log ""
log "Start a follower Server:"
log "  Single: rynnrcp-server --config rynnrcp_robot_so101/config/so101_follower_server.yaml"
log "  Dual:   rynnrcp-server --config rynnrcp_robot_so101/config/so101_bimanual_follower_server.yaml"
log ""
log "See README.md or README.zh-CN.md for leader, Teleop, MCP, and RynnBot commands."
