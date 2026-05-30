# AGENTS.md

## Project context

This repository is a ROS 2 Jazzy robot project for a 6-DOF robot arm with vision, YOLO detection, MoveIt 2 planning, ros2_control hardware interface, and reinforcement learning experiments.

The project may include:

* ROS 2 packages
* Python vision nodes
* YOLO detection/inference
* RL/SAC training code
* MoveIt 2 action servers
* robot hardware interface
* camera calibration and ArUco/YOLO pipelines

## Environment rules

Do not install packages directly unless the user explicitly asks.

Do not run destructive commands.

Do not run:

* sudo apt install
* sudo apt remove
* rm -rf
* pip install into system Python

Safe commands are allowed:

* python --version
* python -m pip list
* python -m pip freeze
* python -m pip check
* ros2 --version
* colcon list
* grep/find commands for code inspection

## Dependency management policy

Separate dependencies into:

1. Python pip dependencies:

   * requirements/base.txt
   * requirements/yolo-cpu.txt
   * requirements/yolo-gpu.txt
   * requirements/rl.txt
   * requirements/runtime-robot.txt
   * requirements/dev.txt

2. Ubuntu/ROS apt dependencies:

   * apt/apt-base.txt
   * apt/apt-ros-jazzy.txt
   * apt/apt-dev.txt

3. ROS package dependencies:

   * package.xml
   * rosdep

4. Documentation:

   * docs/ENVIRONMENT_SETUP.md

## GPU and CPU split

Training machine:

* Has NVIDIA GPU.
* May use CUDA, PyTorch CUDA, onnxruntime-gpu.
* Used for YOLO training, RL training, simulation-heavy workflows.

Robot runtime machine:

* May not have discrete GPU.
* Must not depend on CUDA.
* Should use CPU inference, ONNX Runtime CPU, or OpenVINO when appropriate.
* Should run ROS 2, camera nodes, detection nodes, TF conversion, and action clients.

## ROS 2 notes

ROS 2 version target: Jazzy on Ubuntu 24.04.

Because ROS 2 Python packages are often installed through apt, virtual environments should usually be created with:

python3 -m venv ~/venvs/ros_env --system-site-packages

When adding ROS dependencies, prefer package.xml and rosdep where possible.

## Expected deliverables for environment packaging tasks

When asked to package dependencies, create or update:

* requirements/*.txt
* apt/*.txt
* scripts/create_venv.sh
* scripts/install_cpu_runtime.sh
* scripts/install_gpu_train.sh
* scripts/export_env.sh
* scripts/check_env.py
* docs/ENVIRONMENT_SETUP.md

Always summarize what changed and what the user should run next.
