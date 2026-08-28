#!/usr/bin/env bash
# Build the aero_hand_rga pybind module (RGA + RKNN zero-copy pipeline).
#
# Usage: ./build.sh [/path/to/python]
# Python resolution order: argument > active virtualenv > python3 on PATH.
# The chosen environment must provide pybind11 and cmake:
#   pip install pybind11 cmake
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"

if [[ $# -ge 1 ]]; then
  PYTHON="$1"
elif [[ -n "${VIRTUAL_ENV:-}" && -x "$VIRTUAL_ENV/bin/python" ]]; then
  PYTHON="$VIRTUAL_ENV/bin/python"
else
  PYTHON="$(command -v python3)"
fi
echo "using python: $PYTHON"

if ! "$PYTHON" -c "import pybind11" >/dev/null 2>&1; then
  echo "error: pybind11 not found in this Python environment." >&2
  echo "       Run: $PYTHON -m pip install pybind11 cmake" >&2
  exit 1
fi

CMAKE_BIN="$("$PYTHON" -c 'import cmake, os; print(os.path.join(cmake.CMAKE_BIN_DIR, "cmake"))' 2>/dev/null || command -v cmake || true)"
if [[ -z "$CMAKE_BIN" ]]; then
  echo "error: cmake not found. Run: $PYTHON -m pip install cmake" >&2
  exit 1
fi
PYBIND11_DIR="$("$PYTHON" -m pybind11 --cmakedir)"

# Drop stale cache so a different interpreter is always picked up cleanly.
rm -rf "$HERE/build/CMakeCache.txt" "$HERE/build/CMakeFiles"

"$CMAKE_BIN" -S "$HERE" -B "$HERE/build" \
  -Dpybind11_DIR="$PYBIND11_DIR" \
  -DPython_EXECUTABLE="$PYTHON" \
  -DPYTHON_EXECUTABLE="$PYTHON"
"$CMAKE_BIN" --build "$HERE/build" -j"$(nproc)"

echo "built: $(ls "$HERE/../rynnrcp_robot_aero_hand/accelerators/"aero_hand_rga*.so)"
