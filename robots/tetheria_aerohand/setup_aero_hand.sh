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

log "Installing MediaPipe with headless OpenCV..."
pip_install \
  "absl-py" \
  "attrs>=19.1.0" \
  "flatbuffers>=2.0" \
  "jax" \
  "jaxlib" \
  "matplotlib" \
  "numpy<2" \
  "opencv-python-headless>=4.9" \
  "protobuf>=4.25.3,<5" \
  "sentencepiece" \
  "sounddevice>=0.4.4"
# MediaPipe 0.10.18 is the latest release with a Linux ARM64 wheel. Its
# metadata requires GUI-enabled opencv-contrib-python, so install the wheel
# without dependencies after providing the headless runtime above.
pip_install --no-deps "mediapipe==0.10.18"

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

# --- NPU gesture backends (platform-detected) -------------------------------
# Detects the NPU platform and builds the matching zero-copy C++ backend:
#   Moore Threads E300/M1000 (libmtnnrt + mtc) -> src_mtnn/ (aero_hand_mtnn)
#   Rockchip RK3566/RK3588   (rknn wheel)      -> src/      (aero_hand_rga)
# Both replicate the MediaPipe hand cascade on-NPU. Skipped on non-aarch64
# hosts; set AERO_HAND_SKIP_NPU=1 to opt out entirely.
RKNN_WHEEL="$REPO_ROOT/rknn_toolkit_lite2-2.3.2-cp310-cp310-manylinux_2_17_aarch64.manylinux2014_aarch64.whl"
MTNN_WHEEL="${MTNN_WHEEL:-}"
RKNN_RT_MIN="2.3.0"
RKNN_RT_URL="https://gitee.com/alibaba-damo-academy/rknn-toolkit2/raw/v2.3.0/rknpu2/runtime/Linux/librknn_api/aarch64/librknnrt.so"
RKNN_RT_PROJECT="$REPO_ROOT/models/rknn_runtime/librknnrt.so"

# Print the "X.Y.Z" version embedded in a librknnrt.so (empty when unknown).
rknnrt_version_of() {
  strings "$1" 2>/dev/null | grep -oEm1 'librknnrt version: [0-9]+\.[0-9]+(\.[0-9]+)?' \
    | grep -oE '[0-9]+\.[0-9]+(\.[0-9]+)?' || true
}

# version_ge A B -> success when A >= B (numeric, dot separated).
version_ge() {
  [[ "$1" == "$2" ]] && return 0
  [[ "$(printf '%s\n%s\n' "$1" "$2" | sort -V | head -n1)" == "$2" ]]
}

# The rknnlite Python backend dlopens librknnrt.so at runtime; container-v6
# models need >= 2.3. Never touch the system copy: fetch into the repo-root
# models/rknn_runtime/ (the C++ build and the Python glue prefer it there).
ensure_rknn_runtime() {
  local sys_ver
  sys_ver="$(rknnrt_version_of /usr/lib/librknnrt.so)"
  if [[ -n "$sys_ver" ]] && version_ge "$sys_ver" "$RKNN_RT_MIN"; then
    log "System librknnrt.so is $sys_ver (>= $RKNN_RT_MIN): OK."
    return 0
  fi
  if [[ -f "$RKNN_RT_PROJECT" ]]; then
    log "System librknnrt.so is ${sys_ver:-missing} (< $RKNN_RT_MIN);" \
        "using project runtime: $RKNN_RT_PROJECT"
    return 0
  fi
  log "System librknnrt.so is ${sys_ver:-missing} (< $RKNN_RT_MIN)."
  log "Fetching librknnrt.so v$RKNN_RT_MIN into $RKNN_RT_PROJECT ..."
  mkdir -p "$(dirname "$RKNN_RT_PROJECT")"
  if command -v curl >/dev/null 2>&1; then
    curl -fSL --retry 3 -o "$RKNN_RT_PROJECT.part" "$RKNN_RT_URL"
  elif command -v wget >/dev/null 2>&1; then
    wget -q -O "$RKNN_RT_PROJECT.part" "$RKNN_RT_URL"
  else
    log "WARNING: neither curl nor wget found; cannot fetch librknnrt.so."
    return 1
  fi || { rm -f "$RKNN_RT_PROJECT.part"; return 1; }
  # Raw endpoints can answer HTTP 200 with an HTML error page; require ELF.
  if ! head -c 16 "$RKNN_RT_PROJECT.part" | grep -q 'ELF'; then
    log "WARNING: downloaded librknnrt.so is not an ELF binary; discarding."
    rm -f "$RKNN_RT_PROJECT.part"
    return 1
  fi
  mv "$RKNN_RT_PROJECT.part" "$RKNN_RT_PROJECT"
  log "Project RKNN runtime ready: $RKNN_RT_PROJECT (system path untouched)."
}

# Platform detection: prefer the device-tree compatible string (definitive
# hardware id), falling back to the NPU runtime library presence. Checking the
# device-tree avoids false positives from wheel files bundled at repo root.
board_compatible() {
  tr '\0' '\n' < /proc/device-tree/compatible 2>/dev/null | tr 'A-Z' 'a-z'
}
is_moore_threads() {
  board_compatible | grep -qE 'm1000|mthreads|moore' || [[ -f /usr/lib/libmtnnrt.so ]]
}
is_rockchip() {
  board_compatible | grep -qE 'rockchip|rk3[0-9]{3}' || [[ -f /usr/lib/librknnrt.so ]]
}
is_jetson() {
  [[ -f /etc/nv_tegra_release ]]
}

if [[ "${AERO_HAND_SKIP_NPU:-0}" == "1" ]]; then
  log "Skipping NPU backend setup (AERO_HAND_SKIP_NPU=1)."
elif [[ "$(uname -sm)" != "Linux aarch64" ]]; then
  log "Non-aarch64 host: skipping NPU backend setup."
elif is_moore_threads; then
  # ----- Moore Threads E300 / M1000 (MTNN) -----
  log "Detected Moore Threads NPU (libmtnnrt/mtc): building MTNN gesture backend..."
  log "Fetching MTNN hand models from the Rynn model zoo..."
  "$VENV_PYTHON" "$SCRIPT_DIR/scripts/fetch_aero_hand_models.py" --platform mtnn_e300 \
    || log "WARNING: model download failed; the backend will retry at runtime."
  PY_TAG="$("$VENV_PYTHON" -c 'import sys; print(f"cp{sys.version_info[0]}{sys.version_info[1]}")')"
  pip_install "pybind11" "cmake"
  if [[ -n "$MTNN_WHEEL" && -f "$MTNN_WHEEL" ]]; then
    log "Installing Moore Threads MTNN python runtime (mtnn_api)..."
    pip_install "$MTNN_WHEEL"
  else
    log "note: MTNN_WHEEL not set; the Python backend 'mediapipe_lite_mtnn' needs mtnn_api."
    log "      The C++ backend 'mediapipe_lite_mtnn_zero' only needs libmtnnrt.so (present)."
  fi
  log "Building the MTNN zero-copy gesture backend..."
  bash "$SCRIPT_DIR/src_mtnn/build.sh" "$VENV_PYTHON"
  log "Verifying MTNN backend import..."
  "$VENV_PYTHON" - <<'PY'
import importlib

from rynnrcp_robot_aero_hand.accelerators import aero_hand_mtnn  # noqa: F401
from rynnrcp_robot_aero_hand.platform_detect import recommended_backend

_, module = recommended_backend("mtnn_e300")
importlib.import_module(module)
print("MTNN backend module OK (libmtnnrt resolved)")
PY
elif is_rockchip; then
  # ----- Rockchip RK3566 / RK3588 (RKNN) -----
  log "Detected Rockchip NPU: building RGA + RKNN gesture backend..."
  log "Fetching RKNN hand models from the Rynn model zoo..."
  # No --platform flag: auto-detection picks rk3588 vs rk3566 here.
  "$VENV_PYTHON" "$SCRIPT_DIR/scripts/fetch_aero_hand_models.py" \
    || log "WARNING: model download failed; the backend will retry at runtime."
  ensure_rknn_runtime \
    || log "WARNING: librknnrt >= $RKNN_RT_MIN unavailable; the rknn backends will fail at runtime."
  RKNN_PY_OK=0
  if [[ -f "$RKNN_WHEEL" ]]; then
    PY_TAG="$("$VENV_PYTHON" -c 'import sys; print(f"cp{sys.version_info[0]}{sys.version_info[1]}")')"
    if [[ "$PY_TAG" != "cp310" ]]; then
      log "warning: bundled rknn wheel targets cp310 but venv is $PY_TAG; trying pip index."
    elif pip_install "$RKNN_WHEEL"; then
      RKNN_PY_OK=1
    fi
  else
    log "rknn-toolkit-lite2 wheel not found at repo root: $RKNN_WHEEL"
  fi
  if [[ "$RKNN_PY_OK" -eq 0 ]]; then
    log "Installing rknn-toolkit-lite2 >= $RKNN_RT_MIN from the pip index..."
    pip_install "rknn-toolkit-lite2>=$RKNN_RT_MIN" && RKNN_PY_OK=1 \
      || log "WARNING: rknn-toolkit-lite2 >= $RKNN_RT_MIN install failed."
  fi
  if [[ "$RKNN_PY_OK" -eq 1 ]]; then
    log "Installing build tools..."
    pip_install "pybind11" "cmake"
    log "Building the RGA + RKNN zero-copy gesture backend..."
    bash "$SCRIPT_DIR/src/build.sh" "$VENV_PYTHON"
    log "Verifying NPU backend import..."
    "$VENV_PYTHON" - <<'PY'
from rynnrcp_robot_aero_hand.accelerators import aero_hand_rga  # noqa: F401
print("NPU backend module OK (bundled librknnrt resolved via $ORIGIN)")
PY
    if [[ -f "$RKNN_RT_PROJECT" ]]; then
      log "note: 'mediapipe_lite_rknn' preloads the project runtime"
      log "      $RKNN_RT_PROJECT; the system librknnrt.so is left untouched."
    fi
  else
    log "Skipping the RGA + RKNN backend build (rknn-toolkit-lite2 unavailable)."
  fi
elif is_jetson; then
  # ----- NVIDIA Jetson (ORIN NX etc.): TensorRT engines are device-local -----
  log "Detected Jetson (TensorRT): prebuilding mediapipe_lite_jetson engines..."
  log "Fetching Jetson models (ONNX + prebuilt ORIN NX engines) from the Rynn model zoo..."
  "$VENV_PYTHON" "$SCRIPT_DIR/scripts/fetch_aero_hand_models.py" --platform jetson \
    || log "WARNING: model download failed; the backend will retry at runtime."
  JETSON_PY=""
  for cand in "$VENV_PYTHON" python3; do
    if "$cand" -c "import tensorrt, cv2" 2>/dev/null; then
      JETSON_PY="$cand"
      break
    fi
  done
  if [[ -z "$JETSON_PY" ]]; then
    log "note: no python with tensorrt+cv2 found; skipping engine prebuild."
    log "      The backend builds engines lazily on first use, or falls back"
    log "      to onnxruntime. To prebuild: python3 scripts/build_jetson_engines.py"
  else
    if ! "$JETSON_PY" "$SCRIPT_DIR/scripts/build_jetson_engines.py"; then
      log "WARNING: TensorRT engine build failed; mediapipe_lite_jetson will fall"
      log "         back to onnxruntime (install the Jetson onnxruntime-gpu wheel"
      log "         for GPU acceleration)."
    fi
  fi
else
  log "No supported NPU platform detected (neither Moore Threads nor Rockchip)."
  log "  Using the CPU MediaPipe baseline backend."
fi

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
