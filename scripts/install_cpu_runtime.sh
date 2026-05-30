#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${VENV_DIR:-$HOME/venvs/ros2_jazzy_robot_cpu}"

VENV_DIR="${VENV_DIR}" "${ROOT_DIR}/scripts/create_venv.sh"

# shellcheck source=/dev/null
source "${VENV_DIR}/bin/activate"

python -m pip install --upgrade pip setuptools wheel
python -m pip install -r "${ROOT_DIR}/requirements/runtime-robot.txt"

echo
echo "CPU runtime dependencies installed into: ${VENV_DIR}"
echo "No apt packages were installed by this script."
