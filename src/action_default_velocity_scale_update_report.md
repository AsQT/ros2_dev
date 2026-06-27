# Action default velocity scale update report

## 1. Files changed

- `robot_task_manager/src/task_manager_client.cpp`
- `robot_task_manager/src/repeatability_test_server.cpp`
- `robot_task_manager/src/drl_pickplace_server.cpp`
- `robot_task_manager/src/move_gripper_server.cpp`
- `robot_task_manager/include/robot_task_manager/moveit_executor.hpp`
- `robot_task_manager/include/robot_task_manager/gripper_executor.hpp`
- `robot_task_manager/scripts/repeatability_test_client.py`
- `robot_task_manager/scripts/drl_pick_place_random_test_client.py`
- `robot_task_manager/launch/repeatability_test_client.launch.py`
- `robot_task_manager/launch/task_servers.launch.py`
- `robot_task_manager/launch/task_servers_sim.launch.py`
- `robot_task_manager/Call_action.md`
- `robot_task_manager/README.md`
- `robot_task_manager/README_old.md`
- `robot_task_manager/move_pose_rl_action_report.md`
- `robot_task_manager/checker_board_current_flow_report.md`
- `robot_task_manager/pickplace_start_state_fix_report.md`
- `robot_gui/src/task_action_controller.cpp`
- `robot_gui/ui/robot_gui.ui`
- `robot_gui/task_action_gui_report.md`
- `robot_gui/move_pose_rl_gui_backend_report.md`
- `robot_task_executor/src/task_executor_node.cpp`
- `robot_task_executor/launch/task_executor.launch.py`
- `robot_task_executor/config/task_executor.yaml`
- `robot_task_executor/README.md`
- `robot_task_executor/README_old.md`
- `robot_drl/launch/drl_mock_hw.launch.py`
- `robot_drl/launch/drl_gazebo.launch.py`

## 2. Actions, clients, and launches updated to default 0.1

- `/move_to_pose`
  - `task_manager_client`
  - `MoveItExecutor::moveToPose()` default argument
  - `Call_action.md` and README examples/defaults
- `/move_to_pose_cartesian`
  - `task_manager_client`
  - `MoveItExecutor::moveToPoseCartesian()` default argument
  - `Call_action.md` and README examples/defaults
- `/move_checker_board`
  - `task_manager_client`
  - `MoveItExecutor::checkerBoard()` velocity default argument
  - `Call_action.md` examples/defaults
- `/pickplace`
  - `task_manager_client`
  - `Call_action.md` and README examples/defaults
- `/repeatability_test`
  - `repeatability_test_server` `fast_velocity_scale`
  - `task_servers.launch.py`
  - `task_servers_sim.launch.py`
  - `repeatability_test_client.py`
  - `repeatability_test_client.launch.py`
  - `Call_action.md` and README examples/defaults
- `/move_pose_rl`
  - `task_manager_client`
  - GUI default input `txtVelocityScale`
  - `Call_action.md` and reports/examples
- `/drl_pickplace`
  - `drl_pickplace_server` internal `cartesian_velocity_scale`
  - `drl_pick_place_random_test_client.py` `start_velocity_scale`
- `/move_gripper`
  - `move_gripper_server` internal `velocity_scale` parameter
  - `GripperExecutor::moveToOpening()` velocity default argument
- `robot_task_executor`
  - `max_velocity_scaling_factor` default in node, launch, and config
- `robot_drl`
  - `max_velocity_scaling_factor` default in mock hardware and gazebo launch files

## 3. Old values replaced

- `0.7`
  - `fast_velocity_scale` -> `0.1`
- `0.5`
  - MoveToPose/Cartesian/PickPlace/MovePoseRL docs and examples -> `0.1`
  - MoveGripper `velocity_scale` default -> `0.1`
  - `robot_task_executor` and `robot_drl` max velocity scaling defaults -> `0.1`
- `0.35`
  - DRL random test `start_velocity_scale` -> `0.1`
- `0.3`
  - DRL PickPlace `cartesian_velocity_scale` -> `0.1`
  - MoveItExecutor velocity default arguments -> `0.1`
  - task executor config max velocity scaling -> `0.1`
- `0.25`
  - Repeatability client/launch/docs `velocity_scale` -> `0.1`
- `0.2`
  - `task_manager_client` action goal velocity defaults -> `0.1`
  - GUI `txtVelocityScale` -> `0.1`
- `0.15`
  - GUI repeatability velocity default -> `0.1`
  - Repeatability CLI examples -> `0.1`

## 4. fast_velocity_scale

Changed: yes.

- `repeatability_test_server` default `fast_velocity_scale`: `0.7` -> `0.1`
- `task_servers.launch.py`: `0.7` -> `0.1`
- `task_servers_sim.launch.py`: `0.7` -> `0.1`

## 5. Gripper velocity_scale

Changed: yes.

- `move_gripper_server` parameter default `velocity_scale`: `0.5` -> `0.1`
- `GripperExecutor::moveToOpening()` velocity default argument: `0.5` -> `0.1`

## 6. acceleration_scale

Changed: no.

Acceleration defaults were intentionally left unchanged because the request only requires velocity defaults and says acceleration is not mandatory.

## 7. Build result

Command:

```bash
cd ~/ros2_dev
colcon build --packages-select robot_task_manager robot_gui robot_drl robot_task_executor --symlink-install
```

Result:

- Passed.
- `robot_task_executor` emitted existing deprecation warnings for `create_service(..., rmw_qos_profile_t, ...)`.
- `robot_task_manager` emitted existing `rosidl_target_interfaces()` deprecation warnings.

## 8. Grep check result

Command:

```bash
rg -n "velocity_scale.*0\\.5|velocity_scale.*0\\.3|velocity_scale.*0\\.25|velocity_scale.*0\\.2|start_velocity_scale.*0\\.35|cartesian_velocity_scale.*0\\.3|fast_velocity_scale.*0\\.7|max_velocity_scaling_factor.*0\\.5|max_velocity_scaling_factor.*0\\.3" robot_task_manager robot_gui robot_task_executor robot_drl
```

Remaining result:

```text
robot_task_manager/src/pickplace_server.cpp:88:        goal->velocity_scale > 0.2)
robot_task_manager/src/pickplace_server.cpp:90:      RCLCPP_WARN(get_logger(), "Reject PickPlace goal: velocity_scale must be in (0, 0.2]");
```

This is validation, not a default value. It was left unchanged because the task says not to change validation or lock velocity.

## 9. Launch and client test

Launch command:

```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_task_manager task_servers.launch.py enable_drl_backend:=false
```

Result:

- Launch started the task action servers, including `/move_to_pose`, `/move_to_pose_cartesian`, `/move_checker_board`, `/move_gripper`, `/pickplace`, `/drl_pickplace`, `/move_pose_rl`, and `/repeatability_test`.
- `ros2 action list` showed the expected actions.
- During this environment test, `ros2 action info /move_to_pose -t` showed duplicate `/move_to_pose_server` action servers, so CLI/client goal tests timed out or could receive mixed responses.
- Stopping the launch with Ctrl-C caused MoveIt/action-client shutdown exceptions in some nodes; this happened during shutdown and is not related to the velocity default changes.

Client/CLI checks:

- `task_manager_client --ros-args -p task_name:=move_to_pose -p execute:=false` was attempted with timeout; it did not complete in this environment because of the duplicate action server condition.
- CLI `/move_to_pose` goal with `velocity_scale: 0.1` was attempted; discovery/action response did not complete before timeout in this environment.
- Static code-path check confirms the defaults sent by `task_manager_client` are now `0.1`.

## 10. Confirmations

- Only default velocity values were changed.
- Velocity is not hard-coded to `0.1`; users can still override goal or launch parameters.
- Action interfaces were not changed.
- Action names and server names were not changed.
- Planning/execution logic was not changed.
- Existing validation logic was not changed.
