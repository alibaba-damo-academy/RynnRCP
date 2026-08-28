#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
VENV_DIR="$SCRIPT_DIR/venv"
DEPS_DIR="$SCRIPT_DIR/.deps"
LIBFRANKA_SOURCE="$DEPS_DIR/libfranka-src"
LIBFRANKA_BUILD="$DEPS_DIR/libfranka-build"
LIBFRANKA_INSTALL="$DEPS_DIR/libfranka-install"
RUCKIG_SOURCE="$DEPS_DIR/ruckig-src"
NATIVE_BUILD="$DEPS_DIR/native-build"
LIBFRANKA_VERSION="${LIBFRANKA_VERSION:-0.13.3}"
RUCKIG_VERSION="${RUCKIG_VERSION:-0.15.3}"
PYTHON_BIN="${PYTHON:-}"
PIP_INDEX_URL="${PIP_INDEX_URL:-}"
DEFAULT_PIP_INDEX_URLS=("https://pypi.tuna.tsinghua.edu.cn/simple" "https://mirrors.aliyun.com/pypi/simple" "")
PYTHON_CMD=()
RECREATE=0
SKIP_APT=0

usage() {
  cat <<'EOF'
Usage: bash setup_franka_fr3.sh [options]

Set up a Franka FR3-local virtual environment, build official libfranka with
Ruckig online trajectory generation, and install the RynnRCP robot package.

Options:
  --libfranka-version VERSION  Official libfranka tag (default: 0.13.3).
  --ruckig-version VERSION     Ruckig tag (default: 0.15.3).
  --python PATH                Python 3.10-3.12 interpreter.
  --venv PATH                  Virtual environment path. Defaults to venv.
  --pip-index-url URL
                               Python package index. When omitted, tries Tsinghua, Aliyun, then pip default.
  --recreate                   Recreate the virtual environment and local build dependencies.
  --skip-apt                   Skip apt-get installation of native build dependencies.
  -h, --help                   Show this help.

After setup:
  source venv/bin/activate
  rynnrcp-franka-fr3-configure
  rynnrcp-server --config rynnrcp_robot_franka_fr3/config/franka_fr3_server.yaml
  Debug UI: open the address printed by the Server.
  rynnrcp-protocol-debug --config rynnrcp_robot_franka_fr3/config/franka_fr3_server.yaml
  rynnrcp-teleop-app
  rynnrcp-mcp-app --server-config rynnrcp_robot_franka_fr3/config/franka_fr3_server.yaml
  rynnrcp-rynnbot-app --config rynnrcp_robot_franka_fr3/config/franka_fr3_rynnbot_app.yaml --server-config rynnrcp_robot_franka_fr3/config/franka_fr3_server.yaml
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

jobs_count() {
  if command -v nproc >/dev/null 2>&1; then
    nproc
  else
    printf '4\n'
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --libfranka-version)
      [[ $# -ge 2 ]] || fail "--libfranka-version requires a version"
      LIBFRANKA_VERSION="$2"
      shift 2
      ;;
    --python)
      [[ $# -ge 2 ]] || fail "--python requires a path"
      PYTHON_BIN="$2"
      shift 2
      ;;
    --ruckig-version)
      [[ $# -ge 2 ]] || fail "--ruckig-version requires a version"
      RUCKIG_VERSION="$2"
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
[[ -d "$REPO_ROOT/robots/franka_fr3" ]] || fail "robots/franka_fr3 not found"
[[ "$(uname -s)" == "Linux" ]] || fail "Franka FCI setup requires a supported Ubuntu host"
[[ "$LIBFRANKA_VERSION" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] || fail "invalid libfranka version"
[[ "$RUCKIG_VERSION" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] || fail "invalid Ruckig version"

if [[ "$SKIP_APT" -eq 0 && -x /usr/bin/apt-get ]]; then
  log "Installing native build dependencies with apt-get..."
  SUDO=()
  if [[ "${EUID:-$(id -u)}" -ne 0 ]]; then
    command -v sudo >/dev/null 2>&1 || fail "sudo is required for apt-get; rerun with --skip-apt to skip"
    SUDO=(sudo)
  fi
  "${SUDO[@]}" apt-get update
  "${SUDO[@]}" apt-get install -y \
    build-essential cmake git libeigen3-dev libpoco-dev python3-dev
else
  log "Skipping apt-get dependency installation."
fi

if [[ -n "$PYTHON_BIN" ]]; then
  PYTHON_CMD=("$PYTHON_BIN")
elif command -v python3 >/dev/null 2>&1; then
  PYTHON_CMD=("$(command -v python3)")
else
  fail "Python 3 was not found"
fi
[[ -x "${PYTHON_CMD[0]}" ]] || fail "Python interpreter is not executable: ${PYTHON_CMD[0]}"
"${PYTHON_CMD[@]}" - <<'PY'
import sys
if not ((3, 10) <= sys.version_info[:2] < (3, 13)):
    raise SystemExit(f"Python 3.10-3.12 is required, found {sys.version.split()[0]}")
PY

log "Repository: $REPO_ROOT"
log "Python:     $("${PYTHON_CMD[@]}" -c 'import sys; print(f"{sys.executable} ({sys.version.split()[0]})")')"
log "Venv:       $VENV_DIR"
log "Pip indexes: ${PIP_INDEX_URL:-Tsinghua -> Aliyun -> pip default}"

if [[ "$RECREATE" -eq 1 ]]; then
  rm -rf "$VENV_DIR"
  rm -rf "$LIBFRANKA_SOURCE" "$LIBFRANKA_BUILD" "$LIBFRANKA_INSTALL" \
    "$RUCKIG_SOURCE" "$NATIVE_BUILD"
fi
mkdir -p "$DEPS_DIR"

if [[ ! -d "$LIBFRANKA_SOURCE/.git" ]]; then
  log "Cloning official libfranka $LIBFRANKA_VERSION..."
  git clone --branch "$LIBFRANKA_VERSION" --depth 1 --recurse-submodules \
    https://github.com/frankarobotics/libfranka.git "$LIBFRANKA_SOURCE"
else
  installed_tag="$(git -C "$LIBFRANKA_SOURCE" describe --tags --exact-match 2>/dev/null || true)"
  [[ "$installed_tag" == "$LIBFRANKA_VERSION" ]] || fail \
    "existing libfranka is ${installed_tag:-not on a tag}; use --recreate"
  git -C "$LIBFRANKA_SOURCE" submodule update --init --recursive
fi

if [[ ! -d "$RUCKIG_SOURCE/.git" ]]; then
  log "Cloning Ruckig $RUCKIG_VERSION..."
  git clone --branch "v$RUCKIG_VERSION" --depth 1 \
    https://github.com/pantor/ruckig.git "$RUCKIG_SOURCE"
else
  installed_tag="$(git -C "$RUCKIG_SOURCE" describe --tags --exact-match 2>/dev/null || true)"
  [[ "$installed_tag" == "v$RUCKIG_VERSION" ]] || fail \
    "existing Ruckig is ${installed_tag:-not on a tag}; use --recreate"
fi

log "Building and installing official libfranka..."
cmake -S "$LIBFRANKA_SOURCE" -B "$LIBFRANKA_BUILD" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_INSTALL_PREFIX="$LIBFRANKA_INSTALL" \
  -DBUILD_TESTS=OFF \
  -DBUILD_EXAMPLES=ON
cmake --build "$LIBFRANKA_BUILD" --parallel "$(jobs_count)"
cmake --install "$LIBFRANKA_BUILD"

if [[ ! -d "$VENV_DIR" ]]; then
  log "Creating virtual environment..."
  "${PYTHON_CMD[@]}" -m venv "$VENV_DIR"
else
  log "Using existing virtual environment."
fi
VENV_PYTHON="$VENV_DIR/bin/python"
[[ -x "$VENV_PYTHON" ]] || fail "virtual environment Python was not found under $VENV_DIR"

log "Installing Python build tools..."
pip_install --upgrade pip setuptools wheel "pybind11>=2.9,<3"
PYBIND11_CMAKE_DIR="$("$VENV_PYTHON" -m pybind11 --cmakedir)"

log "Building the RynnRCP native libfranka adapter..."
cmake -S "$SCRIPT_DIR/native" -B "$NATIVE_BUILD" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$LIBFRANKA_INSTALL;$PYBIND11_CMAKE_DIR" \
  -DFranka_DIR="$LIBFRANKA_INSTALL/lib/cmake/Franka" \
  -Dpybind11_DIR="$PYBIND11_CMAKE_DIR" \
  -DRUCKIG_SOURCE_DIR="$RUCKIG_SOURCE" \
  -DPython3_EXECUTABLE="$VENV_PYTHON" \
  -DFRANKA_RCP_OUTPUT_DIR="$SCRIPT_DIR/rynnrcp_robot_franka_fr3" \
  -DFRANKA_RCP_LIB_DIR="$LIBFRANKA_INSTALL/lib"
cmake --build "$NATIVE_BUILD" --parallel "$(jobs_count)"

log "Installing RynnRCP and the Franka package..."
pip_install \
  -e "$REPO_ROOT/rynnkit[realsense]" \
  -e "$REPO_ROOT" \
  -e "$REPO_ROOT/apps/common" \
  -e "$REPO_ROOT/apps/protocol_debug" \
  -e "$REPO_ROOT/apps/mcp" \
  -e "$REPO_ROOT/apps/rynnbot" \
  -e "$REPO_ROOT/apps/teleop" \
  -e "$REPO_ROOT/robots/meta_quest3" \
  -e "$SCRIPT_DIR"

log "Verifying imports..."
"$VENV_PYTHON" - <<'PY'
from rynnrcp_robot_franka_fr3 import _franka_native
from rynnrcp_robot_franka_fr3.controller import FrankaController
assert _franka_native.__file__
assert FrankaController.n_dof == 7
print("Imports OK")
PY

if ! uname -a | grep -Eqi 'PREEMPT_RT|PREEMPT RT'; then
  log "Warning: PREEMPT_RT was not detected. Read-only checks are supported,"
  log "but install a Franka-supported real-time kernel before production motion control."
fi

log ""
log "Franka FR3 setup completed."
log ""
log "Activate the environment:"
VENV_DISPLAY="$VENV_DIR"
[[ "$VENV_DIR" == "$SCRIPT_DIR/venv" ]] && VENV_DISPLAY="venv"
log "  source $VENV_DISPLAY/bin/activate"
log ""
log "Next steps:"
log "  rynnrcp-franka-fr3-configure"
log "  rynnrcp-server --config rynnrcp_robot_franka_fr3/config/franka_fr3_server.yaml"
log "  rynnrcp-server --config rynnrcp_robot_meta_quest3/config/meta_quest3_franka_fr3_right_server.yaml"
log "  Debug UI: open the address printed by the Server."
log "  rynnrcp-protocol-debug --config rynnrcp_robot_franka_fr3/config/franka_fr3_server.yaml"
log "  rynnrcp-teleop-app"
log "  rynnrcp-mcp-app --server-config rynnrcp_robot_franka_fr3/config/franka_fr3_server.yaml"
log "  rynnrcp-rynnbot-app --config rynnrcp_robot_franka_fr3/config/franka_fr3_rynnbot_app.yaml --server-config rynnrcp_robot_franka_fr3/config/franka_fr3_server.yaml"
