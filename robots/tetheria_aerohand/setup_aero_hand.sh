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
Usage: bash setup_aero_hand.sh [options]

Set up an Aero Hand-local virtual environment with RynnRCP, official apps, and camera-gesture support.

Options:
  --python PATH     Python interpreter to use.
  --venv PATH       Virtual environment path. Defaults to venv.
  --pip-index-url URL
                   Python package index. When omitted, tries Tsinghua, Aliyun, then pip default.
  --recreate        Remove and recreate the virtual environment.
  -h, --help        Show this help.

After setup, activate the environment and configure the robot:
  source venv/bin/activate
  rynnrcp-aero-hand-configure

Start a single-hand or dual-hand target Server:
  rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml
  rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml
  Debug UI: open the address printed by each Server.

Inspect a target Server:
  rynnrcp-protocol-debug --config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml
  rynnrcp-protocol-debug --config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml
  rynnrcp-mcp-app --server-config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml
  rynnrcp-mcp-app --server-config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml

Connect a target Server to RynnBot:
  rynnrcp-rynnbot-app --config rynnrcp_robot_aero_hand/config/aero_hand_single_rynnbot_app.yaml --server-config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml
  rynnrcp-rynnbot-app --config rynnrcp_robot_aero_hand/config/aero_hand_dual_rynnbot_app.yaml --server-config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml

Local single-hand gesture Teleop (three terminals):
  rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_single_hand_master_server.yaml
  rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml
  rynnrcp-teleop-app

Local dual-hand gesture Teleop (three terminals):
  rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_dual_hand_master_server.yaml
  rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml
  rynnrcp-teleop-app

RynnBot single-hand camera gesture controller for a simulated target (start Server before App):
  rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_single_hand_master_server.yaml
  rynnrcp-rynnbot-app --config rynnrcp_robot_aero_hand/config/aero_hand_single_hand_master_rynnbot_app.yaml --server-config rynnrcp_robot_aero_hand/config/aero_hand_single_hand_master_server.yaml

RynnBot dual-hand camera gesture controller for a simulated target (start Server before App):
  rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_dual_hand_master_server.yaml
  rynnrcp-rynnbot-app --config rynnrcp_robot_aero_hand/config/aero_hand_dual_hand_master_rynnbot_app.yaml --server-config rynnrcp_robot_aero_hand/config/aero_hand_dual_hand_master_server.yaml
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
[[ -d "$REPO_ROOT/robots/tetheria_aerohand" ]] || fail "robots/tetheria_aerohand not found"

if [[ -n "$PYTHON_BIN" ]]; then
  PYTHON_CMD=("$PYTHON_BIN")
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
  -e "$REPO_ROOT" \
  -e "$REPO_ROOT/apps/protocol_debug" \
  -e "$REPO_ROOT/apps/common" \
  -e "$REPO_ROOT/apps/mcp" \
  -e "$REPO_ROOT/apps/rynnbot" \
  -e "$REPO_ROOT/apps/teleop" \
  -e "$REPO_ROOT/robots/tetheria_aerohand"

log "Verifying imports..."
"$VENV_PYTHON" - <<'PY'
import rynnrcp
import rynnrcp_app_common
import rynnrcp_app_mcp
import rynnrcp_app_protocol_debug
import rynnrcp_app_rynnbot
import rynnrcp_app_teleop
import rynnrcp_robot_aero_hand
print("Imports OK")
PY

log ""
log "Aero Hand setup completed."
log ""
log "Activate the environment:"
VENV_DISPLAY="$VENV_DIR"
[[ "$VENV_DIR" == "$SCRIPT_DIR/venv" ]] && VENV_DISPLAY="venv"
if [[ "$VENV_BIN" == *"/Scripts" ]]; then
  log "  source $VENV_DISPLAY/Scripts/activate"
else
  log "  source $VENV_DISPLAY/bin/activate"
fi
log ""
log "Next steps:"
log "  rynnrcp-aero-hand-configure"
log ""
log "Start a single-hand or dual-hand target Server:"
log "  rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml"
log "  rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml"
log "  Debug UI: open the address printed by each Server."
log ""
log "Inspect a target Server:"
log "  rynnrcp-protocol-debug --config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml"
log "  rynnrcp-protocol-debug --config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml"
log "  rynnrcp-mcp-app --server-config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml"
log "  rynnrcp-mcp-app --server-config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml"
log ""
log "Connect a target Server to RynnBot:"
log "  rynnrcp-rynnbot-app --config rynnrcp_robot_aero_hand/config/aero_hand_single_rynnbot_app.yaml --server-config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml"
log "  rynnrcp-rynnbot-app --config rynnrcp_robot_aero_hand/config/aero_hand_dual_rynnbot_app.yaml --server-config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml"
log ""
log "Local single-hand gesture Teleop (three terminals):"
log "  rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_single_hand_master_server.yaml"
log "  rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml"
log "  rynnrcp-teleop-app"
log ""
log "Local dual-hand gesture Teleop (three terminals):"
log "  rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_dual_hand_master_server.yaml"
log "  rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_dual_server.yaml"
log "  rynnrcp-teleop-app"
log ""
log "RynnBot single-hand camera gesture controller for a simulated target (start Server before App):"
log "  rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_single_hand_master_server.yaml"
log "  rynnrcp-rynnbot-app --config rynnrcp_robot_aero_hand/config/aero_hand_single_hand_master_rynnbot_app.yaml --server-config rynnrcp_robot_aero_hand/config/aero_hand_single_hand_master_server.yaml"
log ""
log "RynnBot dual-hand camera gesture controller for a simulated target (start Server before App):"
log "  rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_dual_hand_master_server.yaml"
log "  rynnrcp-rynnbot-app --config rynnrcp_robot_aero_hand/config/aero_hand_dual_hand_master_rynnbot_app.yaml --server-config rynnrcp_robot_aero_hand/config/aero_hand_dual_hand_master_server.yaml"
