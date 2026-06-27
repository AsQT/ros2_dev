# RL Pick-Place Gazebo Rework Audit Report

Date: 2026-06-27
Workspace: `/home/minhquang/ros2_dev/src`

## 1. Reports read before editing

- `robot_drl_executor/drl_action_fix_report.md`
- `robot_drl_executor/drl_action_error_audit_report.md`
- `robot_drl_executor/robot_drl_executor_migration_report.md`
- `robot_gazebo/GAZEBO_MIGRATION_REPORT.md`
- `robot_task_manager/move_pose_rl_action_report.md`
- `robot_task_manager/task_servers_launch_fix_report.md`

## 2. Current launch used for the demo

The current Gazebo RL pick-place entrypoint is:

```text
robot_bringup/launch/rl_pick_place_box_gazebo_demo.launch.py
```

It includes:

- `robot_bringup/launch/sim.launch.py` with `spawn_demo_woods=false`
- `robot_drl_executor/launch/robot_drl_executor.launch.py`
- `robot_drl/drl_unified_planner_node` with prefix `/home/minhquang/venvs/ros_rl/bin/python3`
- `robot_gazebo/gazebo/spawn_pick_box.py`
- `robot_task_manager/scripts/drl_pick_place_box_demo_client.py`

## 3. Nodes, actions, and services in the flow

- Action server: `/drl_pickplace`
  - Type: `robot_task_manager/action/DrlPickPlace`
  - Source: `robot_task_manager/src/drl_pickplace_server.cpp`
- DRL backend node: `/drl_unified_planner_node`
  - Services: `/drl/plan`, `/drl/clear_trajectory`, `/drl/execute_forward`, `/drl/get_execution_status`
  - Parameter service: `/drl_unified_planner_node/set_parameters`
- DRL executor node:
  - Service: `/move_cartesian_pose_sequence`
  - Type: `robot_task_executor_msgs/srv/MoveCartesianPoseSequence`
- MoveIt planning scene:
  - Services: `/apply_planning_scene`, `/get_planning_scene`

## 4. Current wood/box spawn behavior

Current demo behavior:

- Spawns one entity named `pick_box` from `worlds/pick_box_3cm/pick_box_3cm.sdf`.
- Publishes sim object info as `visualization_msgs/Marker` on `/sim/pick_box_info`.
- The demo client subscribes to `/sim/pick_box_info` and uses that pose as `target_pick`.
- Legacy random/demo woods are disabled in this launch by passing `spawn_demo_woods=false`.

Other available Gazebo object assets:

- `robot_gazebo/worlds/wood_block/wood_model.sdf`
- `robot_gazebo/worlds/box/box_model.sdf`
- `robot_gazebo/worlds/pick_box_3cm/pick_box_3cm.sdf`

## 5. Mismatch with the requested convention

The current launch uses `box` as the pick object:

```text
pick_box -> /sim/pick_box_info -> target_pick
```

This violates the required convention:

```text
wood = pick_object
box  = obstacle
```

The current demo also does not create a distinct `obstacle_box` or feed a box collision object into the planning scene for the DRL planner. Camera/YOLO is not used by this launch, which is already correct for the new simulation-ground-truth requirement.

## 6. Plan before editing

1. Keep `/drl_pickplace`, DRL model, reward, observation, and action space unchanged.
2. Keep the existing launch name so the documented command still works.
3. Add a Gazebo spawner dedicated to this demo:
   - entity `pick_wood`
   - entity `obstacle_box`
   - ground-truth marker `/sim/pick_wood_info`
   - ground-truth marker `/sim/obstacle_box_info`
4. Add a demo client that:
   - reads wood pose from `/sim/pick_wood_info`
   - reads box pose and size from `/sim/obstacle_box_info`
   - applies `obstacle_box` to MoveIt PlanningScene
   - sends `/drl_pickplace` with `target_pick` from the wood pose
5. Update `robot_bringup/launch/rl_pick_place_box_gazebo_demo.launch.py` to use the new spawner/client and preserve the DRL planner venv prefix.
6. Build the requested packages and run a Gazebo plan-only smoke test first. Execute mode remains available through a launch argument, but plan-only is safer for this rework.
