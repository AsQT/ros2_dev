# Call Action Documentation Update Report

## Files changed

- `robot_task_manager/Call_action.md`
- `call_action_doc_update_report.md`

## Action definitions checked

Verified with:

```bash
cd ~/ros2_dev
source install/setup.bash
ros2 interface show robot_task_manager/action/GoHome
ros2 interface show robot_task_manager/action/MoveToPose
ros2 interface show robot_task_manager/action/MoveToPoseCartesian
ros2 interface show robot_task_manager/action/CheckerBoard
ros2 interface show robot_task_manager/action/MoveGripper
ros2 interface show robot_task_manager/action/PickPlace
ros2 interface show robot_task_manager/action/DrlPickPlace
ros2 interface show robot_task_manager/action/RepeatabilityTest
```

All eight action goals include `bool execute`. `RepeatabilityTest` includes `AXIS_X=0`, `AXIS_Y=1`, and `AXIS_Z=2`.

## Updated actions

`Call_action.md` now documents `execute` for:

- `/gohome`
- `/move_to_pose`
- `/move_to_pose_cartesian`
- `/move_checker_board`
- `/move_gripper`
- `/pickplace`
- `/drl_pickplace`
- `/repeatability_test`

The document explains:

- `execute: true`: plan and execute, preserving existing behavior.
- `execute: false`: planning/dry-run only, no robot or gripper movement.
- Client and launch defaults use `execute=true` unless the user overrides the parameter.

## CLI examples updated

All `ros2 action send_goal` examples in `robot_task_manager/Call_action.md` now include `execute`.

Added or verified plan-only examples for:

- `/move_to_pose`
- `/move_to_pose_cartesian`
- `/pickplace`
- `/drl_pickplace`
- `/repeatability_test`

## RepeatabilityTest updates

The `/repeatability_test` section now documents:

- `AXIS_Z=2`
- X/Y/Z measurement pose rules
- `axis` validation for `0`, `1`, `2`
- `execute` goal field
- `velocity_scale` as the slow measurement segment speed
- `fast_velocity_scale` server parameter default `0.7`
- Updated sequence using `fast_velocity_scale` for non-measurement moves
- Launch examples for Z-axis `execute:=true` and `execute:=false`

## Client and launch docs checked

Documentation was matched against:

- `task_manager_client.cpp`
- `repeatability_test_client.py`
- `repeatability_test_client.launch.py`
- `drl_pick_place_box_demo_client.py`
- `drl_pick_place_random_test_client.py`
- `drl_pick_place_random_test.launch.py`
- `task_servers.launch.py`
- `task_servers_sim.launch.py`

Notable corrections:

- `task_manager_client` documents `execute` parameter default `true`.
- `checker_board` client entry now correctly maps to `/move_checker_board`.
- DRL random test launch documents `execute:=true/false`.

## Build and checks

Build:

```bash
cd ~/ros2_dev
colcon build --packages-select robot_task_manager
source install/setup.bash
```

Result: build succeeded.

Documentation checks:

```bash
grep -n "ros2 action send_goal" -A 5 src/robot_task_manager/Call_action.md
grep -n "execute" src/robot_task_manager/Call_action.md
```

Result: every action CLI example includes `execute`; the document contains `AXIS_Z` and `fast_velocity_scale`.

## Confirmation

`robot_task_manager/Call_action.md` is aligned with the current action definitions, server/client defaults, and launch parameters inspected in the repository. No action/server/topic names were changed.
