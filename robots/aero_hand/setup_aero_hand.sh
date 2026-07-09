#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
VENV_DIR="$SCRIPT_DIR/venv"
PYTHON_BIN="${PYTHON:-}"
PIP_INDEX_URL="${PIP_INDEX_URL:-https://mirrors.aliyun.com/pypi/simple/}"
PYTHON_CMD=()
RECREATE=0

usage() {
  cat <<'EOF'
Usage: robots/aero_hand/setup_aero_hand.sh [options]

Set up an Aero Hand-local virtual environment and install RynnRCP as a local library.

Options:
  --python PATH     Python interpreter to use.
  --venv PATH       Virtual environment path. Defaults to robots/aero_hand/venv.
  --pip-index-url URL
                   Python package index. Defaults to Aliyun PyPI mirror.
  --recreate        Remove and recreate the virtual environment.
  -h, --help        Show this help.

After setup:
  source venv/bin/activate
  rynnrcp-aero-hand-configure
  rynnrcp-server --config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml
  rynnrcp-rynnbot-app --config rynnrcp_robot_aero_hand/config/aero_hand_single_rynnbot_app.yaml --server-config rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml
EOF
}

log() {
  printf '%s\n' "$*"
}

fail() {
  printf 'Error: %s\n' "$*" >&2
  exit 1
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
[[ -d "$REPO_ROOT/robots/aero_hand" ]] || fail "robots/aero_hand not found"

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
log "Pip index:  $PIP_INDEX_URL"

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
"$VENV_PYTHON" -m pip install --index-url "$PIP_INDEX_URL" --upgrade pip setuptools wheel

log "Installing RynnRCP as a local library and official apps..."
"$VENV_PYTHON" -m pip install --index-url "$PIP_INDEX_URL" \
  -e "$REPO_ROOT" \
  -e "$REPO_ROOT/apps/common" \
  -e "$REPO_ROOT/apps/mcp" \
  -e "$REPO_ROOT/apps/rynnbot" \
  -e "$REPO_ROOT/apps/teleop" \
  -e "$REPO_ROOT/robots/aero_hand"

log "Verifying imports..."
"$VENV_PYTHON" - <<'PY'
import rynnrcp
import rynnrcp_app_common
import rynnrcp_app_mcp
import rynnrcp_app_rynnbot
import rynnrcp_app_teleop
import rynnrcp_robot_aero_hand
print("Imports OK")
PY

log ""
log "Aero Hand setup completed."
log ""
log "Activate the environment:"
if [[ "$VENV_BIN" == *"/Scripts" ]]; then
  log "  source $VENV_DIR/Scripts/activate"
else
  log "  source $VENV_DIR/bin/activate"
fi
log ""
log "Next steps:"
log "  rynnrcp-aero-hand-configure"
log "  rynnrcp-server --config robots/aero_hand/rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml"
log "  rynnrcp-rynnbot-app --config robots/aero_hand/rynnrcp_robot_aero_hand/config/aero_hand_single_rynnbot_app.yaml --server-config robots/aero_hand/rynnrcp_robot_aero_hand/config/aero_hand_single_server.yaml"
