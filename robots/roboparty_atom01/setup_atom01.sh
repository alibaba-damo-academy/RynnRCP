#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
VENV_DIR="$SCRIPT_DIR/venv"
PYTHON_BIN="${PYTHON:-}"
PIP_INDEX_URL="${PIP_INDEX_URL:-}"
DEFAULT_PIP_INDEX_URLS=("https://pypi.tuna.tsinghua.edu.cn/simple" "https://mirrors.aliyun.com/pypi/simple" "")
RECREATE=0
SKIP_APT=0
PYTHON_CMD=()

usage() {
  cat <<'EOF'
Usage: bash setup_atom01.sh [options]

Set up an Atom01-local virtual environment, install RynnRCP, and build atom01_py.

Options:
  --python PATH     Python interpreter to use.
  --venv PATH       Virtual environment path. Defaults to venv.
  --pip-index-url URL
                   Python package index. When omitted, tries Tsinghua, Aliyun, then pip default.
  --recreate        Remove and recreate the virtual environment.
  --skip-apt        Do not attempt apt-get install of C++ build dependencies.
  -h, --help        Show this help.

After setup:
  source venv/bin/activate
  rynnrcp-atom01-configure
  rynnrcp-server --config rynnrcp_robot_atom01/config/atom01_server.yaml
  Debug UI: open the address printed by the Server.
  rynnrcp-protocol-debug --config rynnrcp_robot_atom01/config/atom01_server.yaml
  rynnrcp-mcp-app --server-config rynnrcp_robot_atom01/config/atom01_server.yaml
  rynnrcp-rynnbot-app --config rynnrcp_robot_atom01/config/atom01_rynnbot_app.yaml --server-config rynnrcp_robot_atom01/config/atom01_server.yaml
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

jobs_count() {
  if command -v nproc >/dev/null 2>&1; then
    nproc
  elif command -v sysctl >/dev/null 2>&1; then
    sysctl -n hw.ncpu
  else
    printf '4\n'
  fi
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
    --skip-apt)
      SKIP_APT=1
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
[[ -d "$REPO_ROOT/apps/rynnbot" ]] || fail "apps/rynnbot not found"
[[ -d "$REPO_ROOT/robots/roboparty_atom01" ]] || fail "robots/roboparty_atom01 not found"

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

if [[ "$SKIP_APT" -eq 0 && "$(uname -s)" == "Linux" && -x /usr/bin/apt-get ]]; then
  log "Installing C++ build dependencies with apt-get..."
  SUDO=()
  if [[ "${EUID:-$(id -u)}" -ne 0 ]]; then
    command -v sudo >/dev/null 2>&1 || fail "sudo is required for apt-get; rerun with --skip-apt to skip"
    SUDO=(sudo)
  fi
  "${SUDO[@]}" apt-get update
  "${SUDO[@]}" apt-get install -y \
    build-essential cmake pkg-config \
    libeigen3-dev libspdlog-dev libfmt-dev libyaml-cpp-dev libboost-all-dev
else
  log "Skipping apt-get dependency install."
fi

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

log "Installing RynnRCP, RynnBot app, and Atom01 package..."
pip_install \
  -e "$REPO_ROOT" \
  -e "$REPO_ROOT/apps/protocol_debug" \
  -e "$REPO_ROOT/apps/common" \
  -e "$REPO_ROOT/apps/mcp" \
  -e "$REPO_ROOT/apps/rynnbot" \
  -e "$REPO_ROOT/robots/roboparty_atom01"

ATOM_CONTROL_DIR="$REPO_ROOT/robots/roboparty_atom01/rynnrcp_robot_atom01/atom_control"
BUILD_DIR="$ATOM_CONTROL_DIR/build"
PYBIND11_DIR="$("$VENV_PYTHON" -m pybind11 --cmakedir)"

log "Building atom01_py from packaged atom_control..."
rm -rf "$BUILD_DIR"
cmake -S "$ATOM_CONTROL_DIR" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$VENV_PYTHON" \
  -Dpybind11_DIR="$PYBIND11_DIR"
cmake --build "$BUILD_DIR" --parallel "$(jobs_count)"
cmake --install "$BUILD_DIR"

log "Verifying imports..."
"$VENV_PYTHON" - <<'PY'
import atom01_py
import rynnrcp
import rynnrcp_app_common
import rynnrcp_app_mcp
import rynnrcp_app_protocol_debug
import rynnrcp_app_rynnbot
import rynnrcp_robot_atom01
print("Imports OK")
PY

log ""
log "Atom01 setup completed."
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
log "  rynnrcp-atom01-configure"
log "  rynnrcp-server --config rynnrcp_robot_atom01/config/atom01_server.yaml"
log "  Debug UI: open the address printed by the Server."
log "  rynnrcp-protocol-debug --config rynnrcp_robot_atom01/config/atom01_server.yaml"
log "  rynnrcp-mcp-app --server-config rynnrcp_robot_atom01/config/atom01_server.yaml"
log "  rynnrcp-rynnbot-app --config rynnrcp_robot_atom01/config/atom01_rynnbot_app.yaml --server-config rynnrcp_robot_atom01/config/atom01_server.yaml"
