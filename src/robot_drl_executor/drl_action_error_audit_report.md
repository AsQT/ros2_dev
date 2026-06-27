# DRL Action Error Audit Report

Date: 2026-06-27
Workspace: `/home/minhquang/ros2_dev/src`

## 1. DRL/RL actions found

- `/move_pose_rl`
  - Type: `robot_task_manager/action/MovePoseRl`
  - Server: `robot_task_manager/src/move_pose_rl_server.cpp`
  - Launch: `robot_task_manager/launch/task_servers.launch.py`, `robot_task_manager/launch/task_servers_sim.launch.py`
  - Backend services used: `/drl/plan`, `/drl/clear_trajectory`, `/drl/execute_forward`, `/drl/get_execution_status`
- `/drl_pickplace`
  - Type: `robot_task_manager/action/DrlPickPlace`
  - Server: `robot_task_manager/src/drl_pickplace_server.cpp`
  - Launch: `robot_task_manager/launch/task_servers.launch.py`, `robot_task_manager/launch/task_servers_sim.launch.py`, `robot_task_manager/launch/drl_pick_place_random_test.launch.py`
  - Backend services used through the same `drl_unified_planner_node`

## 2. DRL launch files found

- `robot_task_manager/launch/task_servers.launch.py`
- `robot_task_manager/launch/task_servers_sim.launch.py`
- `robot_drl/launch/drl_mock_hw.launch.py`
- `robot_drl/launch/drl_gazebo.launch.py`
- `robot_drl/launch/rl_sim_rviz.launch.py`
- `robot_drl/launch/mock_drl.launch.py`
- `robot_drl/launch/mock_drl_rviz.launch.py`
- `robot_drl/launch/main.launch.py`
- `robot_drl/launch/drl_unified_planner.launch.py`
- `robot_bringup/launch/drl_test.launch.py`
- `robot_bringup/launch/rl_pick_place_box_gazebo_demo.launch.py`

All files above launch `robot_drl/drl_unified_planner_node`.

## 3. Nodes needing RL Python libraries

- `robot_drl/drl_unified_planner_node`
  - Imports `robot_drl.drl_planner_core`, which loads the DRL model through `robot_drl/model_loader.py`.
  - `robot_drl/model_loader.py` imports `numpy`, `gymnasium.spaces`, and `stable_baselines3` (`DDPG`, `SAC`, `TD3`). `stable_baselines3` pulls in `torch`.
- `robot_drl/mock_environment_node`
  - Does not need RL model libraries. It needs ROS messages for mock vision data.
- `robot_task_manager` action servers and `robot_drl_executor_node`
  - C++ nodes; they must not be prefixed with the RL Python virtualenv.

## 4. Nodes currently running with default Python

The launch files listed in section 2 started `drl_unified_planner_node` without a `prefix`. That means the node ran through whichever Python interpreter the installed console script resolved to in the active shell environment. This violates the task requirement that RL-library nodes run through:

```bash
/home/minhquang/venvs/ros_rl/bin/python3
```

## 5. Virtualenv check

Command:

```bash
/home/minhquang/venvs/ros_rl/bin/python3 --version
/home/minhquang/venvs/ros_rl/bin/python3 -c "import torch; import stable_baselines3; import gymnasium; import numpy"
```

Result:

- Python: `3.12.3`
- `torch`: OK
- `stable_baselines3`: OK
- `gymnasium`: OK
- `numpy`: OK

ROS import check with `/opt/ros/jazzy/setup.bash` and workspace setup sourced:

- `rclpy`: OK
- `action_msgs`: OK
- `geometry_msgs`: OK

Default `python3` in this machine currently also imports the RL libraries, so a `ModuleNotFoundError` is not reproducible in this exact shell. The launch configuration is still wrong because it does not pin the required venv and can fail on a clean terminal, service environment, or another machine.

## 6. Runtime/import errors found

- `drl_unified_planner_node` launch entries do not pin the RL virtualenv. This is the main environment error.
- `robot_drl/robot_drl/drl_unified_planner_node.py` imports `Box` from `robot_vision_pipeline.msg`, but the current message package is `robot_vision_pipeline_msgs.msg`.
- `robot_drl/robot_drl/mock_environment_node.py` imports `Box` and `BoxDetection` from `robot_vision_pipeline.msg`, but those message definitions are in `robot_vision_pipeline_msgs.msg`.
- Direct import check:

```text
from robot_vision_pipeline.msg import Box, BoxDetection -> ImportError
from robot_vision_pipeline_msgs.msg import Box, BoxDetection -> OK
```

The message import issue can break vision/mock DRL flows even if the RL model libraries are available.

## 7. Action server/client and service checks

Expected action servers:

- `/move_pose_rl`: `robot_task_manager/action/MovePoseRl`
- `/drl_pickplace`: `robot_task_manager/action/DrlPickPlace`

Expected DRL backend services:

- `/drl_unified_planner_node/set_parameters`
- `/drl/plan`
- `/drl/replan`
- `/drl/clear_trajectory`
- `/drl/execute_forward`
- `/drl/get_execution_status`

Expected executor service:

- `/move_cartesian_pose_sequence`
  - Type: `robot_task_executor_msgs/srv/MoveCartesianPoseSequence`
  - Provided by `robot_drl_executor/robot_drl_executor_node.cpp`

The `robot_drl_executor` package intentionally still depends on `robot_task_executor_msgs` because the service type remains there.

## 8. Migration leftovers

- `robot_drl_executor` correctly provides the DRL-specific `/move_cartesian_pose_sequence` service.
- Some docs and interface type names still mention `robot_task_executor` or `robot_task_executor_msgs`; the message package reference is expected.
- No fix should delete `robot_task_executor`. Legacy task executor code remains available.

## 9. Proposed fix

- Add `prefix="/home/minhquang/venvs/ros_rl/bin/python3"` only to `launch_ros.actions.Node` entries that run `robot_drl/drl_unified_planner_node`.
- Do not prefix C++ nodes (`robot_task_manager` action servers, `robot_drl_executor_node`) or non-RL Python nodes.
- Update DRL vision message imports from `robot_vision_pipeline.msg` to `robot_vision_pipeline_msgs.msg`.
- Add the missing `robot_vision_pipeline_msgs` dependency to `robot_drl/package.xml`.
- Build selected packages and run a mock-hardware DRL action flow:
  - `colcon build --symlink-install --packages-select robot_drl_executor robot_drl robot_task_manager robot_bringup`
  - `ros2 action list -t`
  - `ros2 action send_goal /move_pose_rl robot_task_manager/action/MovePoseRl ... execute=false`
