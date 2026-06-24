# DRL Pick Place Test Report

Date: 2026-06-17
Workspace: `/home/minhquang/ros2_dev`

## Action

New action: `robot_task_manager/action/DrlPickPlace`

Goal:
- `geometry_msgs/PoseStamped target_pick`
- `geometry_msgs/PoseStamped target_place`
- `float64 gripper_close_width_m`

Result:
- `bool success`
- `string message`
- `string failed_stage`

Feedback:
- `string current_stage`
- `float32 progress`
- `geometry_msgs/PoseStamped current_pose`

## Execution Chain

1. Validate/transform goal and wait for sub-action/services.
2. Open gripper with `/move_gripper` at `0.05 m`.
3. Use DRL planner/services to move to pre-pick.
4. Use `/move_to_pose_cartesian` to descend to pick.
5. Close gripper with `/move_gripper` at requested width, default `0.028 m`.
6. Use `/move_to_pose_cartesian` to lift.
7. Use DRL planner/services to move to target place.
8. Open gripper with `/move_gripper` at `0.05 m`.

## Verification Commands

Build:

```bash
colcon build --symlink-install --packages-select robot_drl robot_task_manager
```

Interface:

```bash
source install/setup.bash
ros2 interface show robot_task_manager/action/DrlPickPlace
```

Random test:

```bash
source install/setup.bash
ros2 launch robot_task_manager drl_pick_place_random_test.launch.py number_of_trials:=20 random_seed:=0
```

Manual goal:

```bash
source install/setup.bash
ros2 action send_goal /drl_pickplace robot_task_manager/action/DrlPickPlace "{target_pick: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.4147, y: -0.0553, z: 0.0841}, orientation: {x: 0.7071068, y: 0.7071068, z: 0.0, w: 0.0}}}, target_place: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.3030, y: 0.0752, z: 0.1713}, orientation: {x: 0.7071068, y: 0.7071068, z: 0.0, w: 0.0}}}, gripper_close_width_m: 0.028}"
```

## Results

Random test result:

```text
Total trials: 20
Passed: 20
Failed: 0
Success rate: 100.0%
Average execution time: 34.76s
Failed seeds: []
Failed stages: []
```

Manual action result:

```text
success: true
message: DrlPickPlace completed successfully
failed_stage: ''
Goal finished with status: SUCCEEDED
```

Controller check:

```text
gripper_controller      active
arm_controller          active
joint_state_broadcaster active
```

## Important Fixes

- Added `DrlPickPlace` action, C++ action server, random test client, and launch file.
- Connected long motions to the existing DRL planner services and short vertical motions to `MoveToPoseCartesian`.
- Added PlanningScene obstacle spawning in the random client, with randomized start, obstacle pose/size/orientation, pick, and place.
- Refreshed DRL dynamic parameters before each plan so server/client parameter updates affect planning.
- Allowed near-converged DRL rollouts only when the exact appended target waypoint still passes obstacle and MoveIt IK/collision validation.
- Added DRL plan retry and a separate trajectory endpoint tolerance while preserving final pose tolerance checks.

## Changed Files

Created:
- `robot_task_manager/action/DrlPickPlace.action`
- `robot_task_manager/src/drl_pickplace_server.cpp`
- `robot_task_manager/scripts/drl_pick_place_random_test_client.py`
- `robot_task_manager/launch/drl_pick_place_random_test.launch.py`
- `robot_task_manager/DRL_PICK_PLACE_TEST_REPORT.md`

Modified:
- `robot_task_manager/CMakeLists.txt`
- `robot_task_manager/package.xml`
- `robot_task_manager/launch/task_servers.launch.py`
- `robot_task_manager/launch/task_servers_sim.launch.py`
- `robot_drl/robot_drl/drl_planner_node_base.py`
- `robot_drl/robot_drl/drl_unified_planner_node.py`

## Configurable Parameters

Action server:
- `position_tolerance_m`
- `orientation_tolerance_rad`
- `sub_action_timeout_sec`
- `drl_timeout_sec`
- `drl_trajectory_endpoint_tolerance_m`
- `drl_plan_attempts`
- `gripper_open_width_m`
- `gripper_default_close_width_m`
- `pick_approach_height_m`
- `cartesian_velocity_scale`
- `planning_frame`
- `ee_link`

Random test client:
- `number_of_trials`
- `random_seed`
- `gripper_close_width_m`
- `workspace_min`
- `workspace_max`
- `start_min`
- `start_max`
- `obstacle_size_min`
- `obstacle_size_max`
- `obstacle_path_fraction_min`
- `obstacle_path_fraction_max`
- `obstacle_lateral_min_m`
- `obstacle_lateral_max_m`
- `target_obstacle_clearance_m`
