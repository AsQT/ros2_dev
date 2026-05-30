# Environment Setup

This workspace targets ROS 2 Jazzy on Ubuntu 24.04, with two separate Python
profiles:

- GPU training machine: YOLO training, RL/SAC experiments, CUDA PyTorch.
- CPU robot runtime machine: ROS 2 nodes, camera, YOLO inference without CUDA.

The scripts do not run `sudo apt install` and do not install into system Python.
They create virtual environments with `--system-site-packages` so ROS Python
packages installed by apt remain visible inside the venv.

## Dependency Files

Python requirements:

- `requirements/base.txt`: shared numerical/helper packages.
- `requirements/yolo-cpu.txt`: CPU YOLO and ONNX Runtime.
- `requirements/yolo-gpu.txt`: GPU YOLO and ONNX Runtime GPU, excluding PyTorch.
- `requirements/rl.txt`: Gymnasium, Stable-Baselines3, TensorBoard.
- `requirements/dev.txt`: Python development tools.
- `requirements/runtime-robot.txt`: CPU robot runtime bundle.

Ubuntu/ROS package lists:

- `apt/apt-base.txt`: base Ubuntu packages.
- `apt/apt-ros-jazzy.txt`: ROS Jazzy packages inferred from this workspace.
- `apt/apt-dev.txt`: development and lint/test tooling.

Prefer `rosdep` for ROS dependencies when possible:

```bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
```

Review the files in `apt/` before installing anything manually.

## CPU Robot Runtime

```bash
cd ~/ros2
VENV_DIR=~/venvs/ros2_jazzy_robot_cpu scripts/install_cpu_runtime.sh
source scripts/export_env.sh cpu
python scripts/check_env.py
colcon build --symlink-install
```

This profile is intended for the robot computer. Use YOLO with `device:=cpu` and
avoid CUDA-only packages.

Example:

```bash
ros2 launch robot_vision_pipeline yolo_detect.launch.py device:=cpu
```

## GPU Training Machine

```bash
cd ~/ros2
VENV_DIR=~/venvs/ros2_jazzy_train_gpu scripts/install_gpu_train.sh
source scripts/export_env.sh gpu
python scripts/check_env.py
```

By default the GPU script installs PyTorch from:

```bash
https://download.pytorch.org/whl/cu128
```

Override it when your NVIDIA driver/CUDA target requires a different PyTorch
wheel channel:

```bash
TORCH_CUDA_INDEX=https://download.pytorch.org/whl/cu126 \
  VENV_DIR=~/venvs/ros2_jazzy_train_gpu \
  scripts/install_gpu_train.sh
```

Run SAC experiments after sourcing the GPU profile:

```bash
cd ~/ros2/rl_ros_sac/Single_Agent
python train_sac.py
```

## Manual Venv Creation

For a custom profile:

```bash
VENV_DIR=~/venvs/my_ros_env scripts/create_venv.sh
source ~/venvs/my_ros_env/bin/activate
python -m pip install -r requirements/runtime-robot.txt
```

## Safe Checks

These commands are read-only:

```bash
python3 --version
colcon list
python scripts/check_env.py
python -m pip check
```

## Notes Requiring Confirmation

- The exact PyTorch CUDA wheel channel depends on the GPU driver. The script
  defaults to `cu128`; confirm this before using the GPU setup.
- The current SAC freeze file in `rl_ros_sac/Single_Agent/requirements_rl_ros.txt`
  contains CUDA 13 / PyTorch 2.12 packages. Keep it as a historical snapshot
  until the target training machine is confirmed.
- OpenVINO is not included by default. Add it only if the robot CPU runtime will
  use OpenVINO-exported YOLO models.
- `ros-jazzy-astra-camera-msgs` may be unavailable from apt if the workspace's
  local `src/astra_camera_msgs` package is the intended provider.
