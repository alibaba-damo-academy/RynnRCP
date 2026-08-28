#!/usr/bin/env bash
set -euo pipefail

ROBOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${ROBOT_DIR}/../.." && pwd)"
VENV_DIR="${ROBOT_DIR}/venv"
VENV_PYTHON="${VENV_DIR}/bin/python"
VENV_ACTIVATE="${VENV_DIR}/bin/activate"
INSTALL_FULL=false

if [[ "${1:-}" == "--full" ]]; then
    INSTALL_FULL=true
elif [[ $# -ne 0 ]]; then
    echo "Usage: bash setup_meta_quest3.sh [--full]" >&2
    exit 2
fi

if [[ -d "${VENV_DIR}" ]] && {
    [[ ! -f "${VENV_ACTIVATE}" ]] ||
        ! grep -Fq "${VENV_DIR}" "${VENV_ACTIVATE}"
}; then
    echo "The project directory moved. Rebuilding ${VENV_DIR}."
    python3 -m venv --clear "${VENV_DIR}"
fi

if [[ ! -x "${VENV_PYTHON}" ]]; then
    python3 -m venv "${VENV_DIR}"
fi

if ! "${VENV_PYTHON}" -c "import setuptools.build_meta, wheel" >/dev/null 2>&1; then
    "${VENV_PYTHON}" -m pip install \
        --index-url "${RYNNRCP_PYPI_INDEX_URL:-https://pypi.org/simple}" \
        "setuptools>=68.0" wheel
fi

"${VENV_PYTHON}" -m pip install \
    --no-build-isolation --no-deps -e "${REPO_ROOT}"
"${VENV_PYTHON}" -m pip install \
    --no-build-isolation -e "${ROBOT_DIR}"

if [[ "${INSTALL_FULL}" == true ]]; then
    "${VENV_PYTHON}" -m pip install \
        --no-build-isolation -e "${REPO_ROOT}"
    "${VENV_PYTHON}" -m pip install \
        --no-build-isolation -e "${REPO_ROOT}/apps/common"
    "${VENV_PYTHON}" -m pip install \
        --no-build-isolation -e "${REPO_ROOT}/apps/teleop"
    echo "Meta Quest 3 Server and Web simulator are ready."
else
    echo "Meta Quest 3 Web simulator is ready."
    echo "For the Server and Teleop apps, rerun: bash setup_meta_quest3.sh --full"
fi

echo "Run: source ${ROBOT_DIR}/venv/bin/activate"
echo "Web simulator: rynnrcp-meta-quest3-sim --config rynnrcp_robot_meta_quest3/config/sim/franka_fr3_right.yaml"
