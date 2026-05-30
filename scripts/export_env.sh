#!/usr/bin/env bash
# Source this file: source scripts/export_env.sh [cpu|gpu]

set -euo pipefail

PROFILE="${1:-cpu}"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

case "${PROFILE}" in
  cpu)
    VENV_DIR="${VENV_DIR:-$HOME/venvs/ros2_jazzy_robot_cpu}"
    ;;
  gpu)
    VENV_DIR="${VENV_DIR:-$HOME/venvs/ros2_jazzy_train_gpu}"
    ;;
  *)
    echo "Usage: source scripts/export_env.sh [cpu|gpu]"
    return 2 2>/dev/null || exit 2
    ;;
esac

if [ -f /opt/ros/jazzy/setup.bash ]; then
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash
else
  echo "Warning: /opt/ros/jazzy/setup.bash not found"
fi

if [ -f "${ROOT_DIR}/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "${ROOT_DIR}/install/setup.bash"
fi

if [ -f "${VENV_DIR}/bin/activate" ]; then
  # shellcheck source=/dev/null
  source "${VENV_DIR}/bin/activate"
else
  echo "Warning: venv not found at ${VENV_DIR}"
fi

export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export PYTHONUNBUFFERED=1

echo "Environment profile: ${PROFILE}"
echo "Workspace root     : ${ROOT_DIR}"
echo "Venv               : ${VENV_DIR}"
echo "ROS_DISTRO         : ${ROS_DISTRO:-unset}"
