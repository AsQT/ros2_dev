#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${VENV_DIR:-$HOME/venvs/ros2_jazzy_train_gpu}"
TORCH_CUDA_INDEX="${TORCH_CUDA_INDEX:-https://download.pytorch.org/whl/cu128}"

VENV_DIR="${VENV_DIR}" "${ROOT_DIR}/scripts/create_venv.sh"

# shellcheck source=/dev/null
source "${VENV_DIR}/bin/activate"

python -m pip install --upgrade pip setuptools wheel
python -m pip install torch torchvision torchaudio --index-url "${TORCH_CUDA_INDEX}"
python -m pip install -r "${ROOT_DIR}/requirements/yolo-gpu.txt"
python -m pip install -r "${ROOT_DIR}/requirements/rl.txt"

echo
echo "GPU training dependencies installed into: ${VENV_DIR}"
echo "PyTorch CUDA wheel index: ${TORCH_CUDA_INDEX}"
echo "No apt packages were installed by this script."
