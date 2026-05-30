#!/usr/bin/env bash
set -euo pipefail

VENV_DIR="${VENV_DIR:-$HOME/venvs/ros2_jazzy_robot}"
PYTHON_BIN="${PYTHON_BIN:-python3}"

echo "Creating ROS-aware venv at: ${VENV_DIR}"
"${PYTHON_BIN}" -m venv "${VENV_DIR}" --system-site-packages

# shellcheck source=/dev/null
source "${VENV_DIR}/bin/activate"
python -m pip install --upgrade pip setuptools wheel

echo
echo "Venv ready."
echo "Activate with:"
echo "  source ${VENV_DIR}/bin/activate"
