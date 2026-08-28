#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
VENV_DIR="$SCRIPT_DIR/venv"
SDK_ROOT="${ASTRIBOT_SDK_ROOT:-/home/astribot/astribot_sdk_aarch64}"
PYTHON_BIN="${PYTHON:-python3}"

usage() {
  cat <<'EOF'
Usage: bash setup_astribot_s1.sh [--sdk-root PATH] [--venv PATH] [--python PATH]

Create the robot-local RynnRCP environment for Astribot S1. The SDK defaults to
/home/astribot/astribot_sdk_aarch64. Use --sdk-root to select another installed
SDK directory containing env.sh.

After setup, source the SDK environment before activating this venv and starting
the server. See README.zh-CN.md for the complete sequence.
EOF
}

fail() { printf 'Error: %s\n' "$*" >&2; exit 1; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sdk-root) [[ $# -ge 2 ]] || fail "--sdk-root requires a path"; SDK_ROOT="$2"; shift 2 ;;
    --venv) [[ $# -ge 2 ]] || fail "--venv requires a path"; VENV_DIR="$2"; shift 2 ;;
    --python) [[ $# -ge 2 ]] || fail "--python requires a path"; PYTHON_BIN="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) fail "unknown option: $1" ;;
  esac
done

SDK_ROOT="$(cd "$SDK_ROOT" && pwd)"
[[ -f "$SDK_ROOT/env.sh" ]] || fail "Astribot env.sh was not found under $SDK_ROOT"
if [[ "$PYTHON_BIN" != */* ]]; then
  PYTHON_BIN="$(command -v "$PYTHON_BIN" || true)"
fi
[[ -x "$PYTHON_BIN" ]] || fail "Python is not executable: $PYTHON_BIN"

# shellcheck disable=SC1090
set +u
source "$SDK_ROOT/env.sh"
set -u

if [[ ! -d "$VENV_DIR" ]]; then
  "$PYTHON_BIN" -m venv --system-site-packages "$VENV_DIR"
fi
VENV_PYTHON="$VENV_DIR/bin/python"
[[ -x "$VENV_PYTHON" ]] || fail "venv Python was not found: $VENV_PYTHON"

"$VENV_PYTHON" -m pip install --upgrade pip "setuptools<80" wheel
"$VENV_PYTHON" -m pip install \
  -e "$REPO_ROOT" \
  -e "$REPO_ROOT/apps/common" \
  -e "$REPO_ROOT/apps/rynnbot" \
  -e "$SCRIPT_DIR"

"$VENV_PYTHON" - <<'PY'
from astribot_sdk.core.astribot_api.astribot_client import Astribot
import rynnrcp_app_rynnbot
from rynnrcp_robot_astribot_s1.controller import AstribotS1Controller

print("Astribot SDK import OK:", Astribot.__name__)
print("RynnBot App import OK:", rynnrcp_app_rynnbot.__name__)
print("RynnRCP controller import OK:", AstribotS1Controller.__name__)
PY

printf '%s\n' "Astribot S1 setup completed."
printf '%s\n' "Next: source $SDK_ROOT/env.sh"
printf '%s\n' "Then: source $VENV_DIR/bin/activate"
printf '%s\n' "Configure: rynnrcp-astribot-s1-configure"
printf '%s\n' "Start: rynnrcp-server --config $SCRIPT_DIR/rynnrcp_robot_astribot_s1/config/astribot_s1_server.yaml"
printf '%s\n' "RynnBot: rynnrcp-rynnbot-app --config $SCRIPT_DIR/rynnrcp_robot_astribot_s1/config/astribot_s1_rynnbot_app.yaml --server-config $SCRIPT_DIR/rynnrcp_robot_astribot_s1/config/astribot_s1_server.yaml"
