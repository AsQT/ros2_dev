# Call Action trong `robot_task_manager`

Tài liệu này gom lại các action trong package `robot_task_manager`, tên action server, kiểu action, goal/result/feedback, tham số mặc định của node server và các cách gọi nhanh bằng CLI hoặc client có sẵn.

## Chạy action servers

```bash
cd ~/ros2_dev
colcon build --packages-select robot_task_manager
source install/setup.bash
```

Mock/real stack trong `task_servers.launch.py`:

```bash
ros2 launch robot_task_manager task_servers.launch.py
```

Simulation trong `task_servers_sim.launch.py`:

```bash
ros2 launch robot_task_manager task_servers_sim.launch.py
```

Backend DRL:

- Hai launch trên mặc định chạy thêm `robot_drl/drl_unified_planner_node` để cung cấp `/drl_unified_planner_node/set_parameters`, `/drl/plan`, `/drl/clear_trajectory`, `/drl/execute_forward`, `/drl/get_execution_status` và topic `/drl/forward_trajectory_poses`.
- Nếu backend DRL đã được chạy ở launch khác, dùng `enable_drl_backend:=false` để tránh trùng node:

```bash
ros2 launch robot_task_manager task_servers.launch.py enable_drl_backend:=false
ros2 launch robot_task_manager task_servers_sim.launch.py enable_drl_backend:=false
```

Kiểm tra backend:

```bash
ros2 service list | grep drl
ros2 service list | grep drl_unified_planner_node
```

Trong `task_servers.launch.py`, backend DRL mặc định tắt phụ thuộc PlanningScene/IK service ngoài launch để `/move_pose_rl execute=false` có thể plan ngay với task servers/mock GUI.

Lưu ý:

- `task_servers.launch.py` và `task_servers_sim.launch.py` đều chạy `gohome_server`.
- `task_servers_sim.launch.py` có chạy đủ các server, gồm cả `gohome_server`.
- `CheckerBoard` server tạo action tên `/move_checker_board`.

## Tổng quan action

| Action name | Type | Server executable | Node name mặc định | Ghi chú |
|---|---|---|---|---|
| `/gohome` | `robot_task_manager/action/GoHome` | `gohome_server` | `gohome_action_server` | MoveIt named target về home |
| `/move_to_pose` | `robot_task_manager/action/MoveToPose` | `move_to_pose_server` | `move_to_pose_action_server` | Plan MoveIt tới pose |
| `/move_to_pose_cartesian` | `robot_task_manager/action/MoveToPoseCartesian` | `move_pose_cartesian_server` | `move_to_pose_cartesian_action_server` | Cartesian tới pose |
| `/move_checker_board` | `robot_task_manager/action/CheckerBoard` | `checker_board_server` | `move_checker_board_action_server` | Chạy checker-board scan |
| `/move_gripper` | `robot_task_manager/action/MoveGripper` | `move_gripper_server` | `move_gripper_action_server` | Mở/đóng gripper theo width |
| `/pickplace` | `robot_task_manager/action/PickPlace` | `pickplace_server` | `pickplace_action_server` | Composite pick-place qua MoveToPose, Cartesian, Gripper |
| `/drl_pickplace` | `robot_task_manager/action/DrlPickPlace` | `drl_pickplace_server` | `drl_pickplace_action_server` | Pick-place có DRL planner |
| `/move_pose_rl` | `robot_task_manager/action/MovePoseRl` | `move_pose_rl_server` | `move_pose_rl_action_server` | DRL plan/execute tới target pose |
| `/repeatability_test` | `robot_task_manager/action/RepeatabilityTest` | `repeatability_test_server` | `repeatability_test_action_server` | Test repeatability theo X/Y/Z |

Tất cả action có field goal `execute`.

- `execute: true`: giữ hành vi cũ, planning rồi execute.
- `execute: false`: planning/validate plan thật nhưng bỏ qua execution; result message sẽ ghi rõ `execution skipped`.
- Các client/launch mẫu mặc định dùng `execute=true` để giữ behavior cũ, trừ khi người dùng truyền tham số khác.

## Default parameter theo launch

`task_servers.launch.py` truyền chung:

| Nhóm | Parameter | Default |
|---|---|---|
| Arm servers | `planning_group` | `arm` |
| Arm servers | `home_target` | `home` |
| Arm servers | `base_frame` | `world` |
| Gripper server | `planning_group` | `gripper` |
| Gripper server | `base_frame` | `link_6` |
| DRL PickPlace thêm | `planning_frame` | `base_link` |
| DRL PickPlace thêm | `ee_link` | `tcp_link` |
| DRL PickPlace thêm | `planner_node_name` | `/drl_unified_planner_node` |
| MovePoseRl thêm | `planning_frame` | `base_link` |
| MovePoseRl thêm | `ee_link` | `tcp_link` |
| MovePoseRl thêm | `planner_node_name` | `/drl_unified_planner_node` |

`task_servers_sim.launch.py` giống trên nhưng thêm `use_sim_time: true`.

## `/gohome`

Goal:

```yaml
start: bool
execute: bool
```

Result:

```yaml
success: bool
message: string
```

Feedback:

```yaml
current_step: string
progress: float32
```

Server parameters:

| Parameter | Default |
|---|---|
| `planning_group` | `arm` |
| `home_target` | `home` |
| `base_frame` | `world` |

Rule:

- `start: false`: action fail rõ ràng với message `GoHome rejected because start=false`.
- `execute: true`: plan và execute về named target `home`.
- `execute: false`: chỉ plan về `home`, không execute, không làm robot di chuyển.
- Trước khi plan, server lấy current robot state từ `/joint_states`; nếu timeout thì không plan từ zero/default state.

CLI execute:

```bash
ros2 action send_goal /gohome robot_task_manager/action/GoHome \
  "{start: true, execute: true}" --feedback
```

CLI plan-only:

```bash
ros2 action send_goal /gohome robot_task_manager/action/GoHome \
  "{start: true, execute: false}" --feedback
```

Chạy bằng `task_manager_client`:

```bash
ros2 run robot_task_manager task_manager_client --ros-args -p task_name:=gohome
```

Default goal trong `task_manager_client`: `start=true`, `execute=true`.

`task_manager_client` có parameter `execute`, default `true`.

## `/move_to_pose`

Goal:

```yaml
target_pose: geometry_msgs/Pose
velocity_scale: float64
execute: bool
```

Result:

```yaml
success: bool
message: string
```

Feedback:

```yaml
stage: string
progress: float32
```

Server parameters:

| Parameter | Default |
|---|---|
| `planning_group` | `arm` |
| `base_frame` | `world` |

Rule:

- `velocity_scale` phải nằm trong `(0, 1]`.
- Server gọi MoveIt với acceleration scale cố định `0.3` và timeout nội bộ `5.0`.

CLI:

```bash
ros2 action send_goal /move_to_pose robot_task_manager/action/MoveToPose \
  "{target_pose: {position: {x: 0.40, y: 0.10, z: 0.35}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}, velocity_scale: 0.1, execute: true}" \
  --feedback
```

Plan only:

```bash
ros2 action send_goal /move_to_pose robot_task_manager/action/MoveToPose \
  "{target_pose: {position: {x: 0.40, y: 0.10, z: 0.35}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}, velocity_scale: 0.1, execute: false}" \
  --feedback
```

Chạy bằng `task_manager_client`:

```bash
ros2 run robot_task_manager task_manager_client --ros-args -p task_name:=move_to_pose
```

Default goal trong `task_manager_client`:

| Field | Default |
|---|---|
| `target_pose.position` | `{x: 0.40, y: 0.10, z: 0.35}` |
| `target_pose.orientation` | `{x: 1.0, y: 1.0, z: 0.0, w: 0.0}` |
| `velocity_scale` | `0.5` |
| `execute` | `true` |

## `/move_to_pose_cartesian`

Goal:

```yaml
target_pose: geometry_msgs/Pose
velocity_scale: float64
execute: bool
```

Result:

```yaml
success: bool
message: string
```

Feedback:

```yaml
stage: string
progress: float32
```

Server parameters:

| Parameter | Default |
|---|---|
| `planning_group` | `arm` |
| `base_frame` | `world` |

Rule:

- `velocity_scale` phải nằm trong `(0, 1]`.
- Server gọi Cartesian MoveIt với acceleration scale cố định `0.3` và timeout nội bộ `5.0`.

CLI:

```bash
ros2 action send_goal /move_to_pose_cartesian robot_task_manager/action/MoveToPoseCartesian \
  "{target_pose: {position: {x: 0.40, y: 0.10, z: 0.35}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}, velocity_scale: 0.5, execute: true}" \
  --feedback
```

Plan only:

```bash
ros2 action send_goal /move_to_pose_cartesian robot_task_manager/action/MoveToPoseCartesian \
  "{target_pose: {position: {x: 0.40, y: 0.10, z: 0.35}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}, velocity_scale: 0.5, execute: false}" \
  --feedback
```

Chạy bằng `task_manager_client`:

```bash
ros2 run robot_task_manager task_manager_client --ros-args -p task_name:=move_to_pose_cartesian
```

Default goal trong `task_manager_client`:

| Field | Default |
|---|---|
| `target_pose.position` | `{x: 0.40, y: 0.10, z: 0.35}` |
| `target_pose.orientation` | `{x: 1.0, y: 1.0, z: 0.0, w: 0.0}` |
| `velocity_scale` | `0.5` |
| `execute` | `true` |

## `/move_checker_board`

Goal:

```yaml
step: float64
velocity_scale: float64
execute: bool
```

Result:

```yaml
success: bool
message: string
```

Feedback:

```yaml
stage: string
progress: float32
```

Server parameters:

| Parameter | Default |
|---|---|
| `planning_group` | `arm` |
| `base_frame` | `world` |

Rule:

- `velocity_scale` phải nằm trong `(0, 1]`.
- Server gọi `executor_->checkerBoard(step, ..., velocity_scale, 0.3, 5.0)`.

CLI:

```bash
ros2 action send_goal /move_checker_board robot_task_manager/action/CheckerBoard \
  "{step: 0.10, velocity_scale: 0.5, execute: true}" --feedback
```

`task_manager_client` có nhánh `task_name:=checker_board` và connect tới `/move_checker_board`.

Default goal trong `task_manager_client`:

| Field | Default |
|---|---|
| `step` | `0.40` |
| `velocity_scale` | `0.5` |
| `execute` | `true` |

## `/move_gripper`

Goal:

```yaml
position: float64
execute: bool
```

Result:

```yaml
success: bool
message: string
```

Feedback:

```yaml
progress: float32
stage: string
```

Server parameters:

| Parameter | Default |
|---|---|
| `planning_group` | `gripper` |
| `base_frame` | `link_6` |
| `velocity_scale` | `0.5` |
| `acceleration_scale` | `0.5` |
| `gripper_max_open` | `0.05` |
| `gripper_min_open` | `0.0` |

Rule:

- `position` phải finite và `>= 0.0`.
- `velocity_scale` và `acceleration_scale` parameter phải nằm trong `(0, 1]`.
- Range thật của `position` được `GripperExecutor` kiểm tra bằng `gripper_min_open` và `gripper_max_open`.

CLI:

```bash
ros2 action send_goal /move_gripper robot_task_manager/action/MoveGripper \
  "{position: 0.03, execute: true}" --feedback
```

Chạy bằng `task_manager_client`:

```bash
ros2 run robot_task_manager task_manager_client --ros-args -p task_name:=move_gripper
```

Default goal trong `task_manager_client`:

| Field | Default |
|---|---|
| `position` | `0.03` |
| `execute` | `true` |

## `/pickplace`

Goal:

```yaml
pose_pick: geometry_msgs/Pose
pose_place: geometry_msgs/Pose
gripper: float64
velocity_scale: float64
execute: bool
```

Result:

```yaml
success: bool
message: string
```

Feedback:

```yaml
stage: string
progress: float32
```

Server parameters:

| Parameter | Default |
|---|---|
| `approach_height` | `0.10` |
| `open_gripper_position` | `0.048` |
| `server_wait_timeout_s` | `5.0` |
| `action_result_timeout_s` | `90.0` |

Rule:

- `velocity_scale` phải finite và nằm trong `(0, 1]`.
- `gripper` phải finite và `>= 0.0`.
- Server phụ thuộc `/move_gripper`, `/move_to_pose`, `/move_to_pose_cartesian`.

Sequence chính:

1. Open gripper tới `open_gripper_position`.
2. MoveToPose tới `pose_pick.z + approach_height`.
3. Cartesian xuống `pose_pick`.
4. Close gripper tới `goal.gripper`.
5. Wait settle `1000 ms`.
6. MoveToPose trực tiếp tới `pose_place.z + approach_height`.
7. Cartesian xuống `pose_place`.
8. Open gripper release.

CLI:

```bash
ros2 action send_goal /pickplace robot_task_manager/action/PickPlace \
  "{pose_pick: {position: {x: 0.40, y: 0.10, z: 0.25}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}, pose_place: {position: {x: 0.30, y: 0.00, z: 0.25}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}, gripper: 0.01, velocity_scale: 0.5, execute: true}" \
  --feedback
```

Plan only:

```bash
ros2 action send_goal /pickplace robot_task_manager/action/PickPlace \
  "{pose_pick: {position: {x: 0.40, y: 0.10, z: 0.25}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}, pose_place: {position: {x: 0.30, y: 0.00, z: 0.25}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}, gripper: 0.01, velocity_scale: 0.5, execute: false}" \
  --feedback
```

Chạy bằng `task_manager_client`:

```bash
ros2 run robot_task_manager task_manager_client --ros-args -p task_name:=pickplace
```

Default goal trong `task_manager_client`:

| Field | Default |
|---|---|
| `pose_pick.position` | `{x: 0.40, y: 0.10, z: 0.25}` |
| `pose_pick.orientation` | `{x: 1.0, y: 1.0, z: 0.0, w: 0.0}` |
| `pose_place.position` | `{x: 0.30, y: 0.00, z: 0.25}` |
| `pose_place.orientation` | `{x: 1.0, y: 1.0, z: 0.0, w: 0.0}` |
| `gripper` | `0.01` |
| `velocity_scale` | `0.5` |
| `execute` | `true` |

## `/drl_pickplace`

Goal:

```yaml
target_pick: geometry_msgs/PoseStamped
target_place: geometry_msgs/PoseStamped
gripper_close_width_m: float64
execute: bool
```

Result:

```yaml
success: bool
message: string
failed_stage: string
```

Feedback:

```yaml
current_stage: string
progress: float32
current_pose: geometry_msgs/PoseStamped
```

Server parameters:

| Parameter | Default |
|---|---|
| `planning_frame` | `base_link` |
| `ee_link` | `tcp_link` |
| `position_tolerance_m` | `0.01` |
| `orientation_tolerance_rad` | `0.10` |
| `sub_action_timeout_sec` | `60.0` |
| `drl_timeout_sec` | `120.0` |
| `drl_trajectory_endpoint_tolerance_m` | `0.015` |
| `drl_plan_attempts` | `3` |
| `gripper_open_width_m` | `0.05` |
| `gripper_default_close_width_m` | `0.028` |
| `pick_approach_height_m` | `0.05` |
| `cartesian_velocity_scale` | `0.3` |
| `tf_timeout_sec` | `2.0` |
| `planner_node_name` | `/drl_unified_planner_node` |

Rule:

- `target_pick.pose` và `target_place.pose` phải finite.
- Nếu `target_pick.header.frame_id` hoặc `target_place.header.frame_id` rỗng, server xem như `planning_frame`.
- Nếu frame khác `planning_frame`, server transform qua TF.
- `gripper_close_width_m <= 0` hoặc không finite sẽ dùng `gripper_default_close_width_m`.
- Close width được clamp về `[0.0, gripper_open_width_m]`.
- Server phụ thuộc `/move_gripper`, `/move_to_pose_cartesian`, service `${planner_node_name}/set_parameters`, `/drl/plan`, `/drl/clear_trajectory`, `/drl/execute_forward`, `/drl/get_execution_status`, topic `/drl/forward_trajectory_poses`.

Sequence chính:

1. Validate và transform goal.
2. Open gripper.
3. DRL plan/execute tới pre-pick (`target_pick.z + pick_approach_height_m`).
4. Cartesian xuống pick.
5. Close gripper.
6. Cartesian lift về pre-pick.
7. DRL plan/execute tới place.
8. Open gripper tại place.

CLI:

```bash
ros2 action send_goal /drl_pickplace robot_task_manager/action/DrlPickPlace \
  "{target_pick: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.40, y: 0.05, z: 0.08}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}}, target_place: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.34, y: -0.10, z: 0.08}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}}, gripper_close_width_m: 0.028, execute: true}" \
  --feedback
```

Plan only:

```bash
ros2 action send_goal /drl_pickplace robot_task_manager/action/DrlPickPlace \
  "{target_pick: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.40, y: 0.05, z: 0.08}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}}, target_place: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.34, y: -0.10, z: 0.08}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}}, gripper_close_width_m: 0.028, execute: false}" \
  --feedback
```

Client box demo:

```bash
ros2 run robot_task_manager drl_pick_place_box_demo_client.py
```

Default parameter của `drl_pick_place_box_demo_client.py`:

| Parameter | Default |
|---|---|
| `object_info_topic` | `/sim/pick_box_info` |
| `action_name` | `drl_pickplace` |
| `frame_id` | `base_link` |
| `place_xyz` | `[0.34, -0.10, 0.035]` |
| `pick_z_offset_m` | `0.0` |
| `place_z_offset_m` | `0.0` |
| `min_pick_z_m` | `0.025` |
| `gripper_close_width_m` | `0.025` |
| `execute` | `true` |
| `object_timeout_sec` | `60.0` |
| `action_server_timeout_sec` | `120.0` |
| `goal_timeout_sec` | `420.0` |

Random test launch:

```bash
ros2 launch robot_task_manager drl_pick_place_random_test.launch.py \
  number_of_trials:=20 random_seed:=0 gripper_close_width_m:=0.028 execute:=true
```

Với Gazebo:

```bash
ros2 launch robot_task_manager drl_pick_place_random_test.launch.py use_gazebo:=true execute:=true
```

Plan only:

```bash
ros2 launch robot_task_manager drl_pick_place_random_test.launch.py \
  number_of_trials:=20 random_seed:=0 gripper_close_width_m:=0.028 execute:=false
```

Default parameter của `drl_pick_place_random_test_client.py`:

| Parameter | Default |
|---|---|
| `number_of_trials` | `20` |
| `random_seed` | `0` |
| `gripper_close_width_m` | `0.028` |
| `action_name` | `drl_pickplace` |
| `frame_id` | `base_link` |
| `workspace_min` | `[0.30, -0.12, 0.08]` |
| `workspace_max` | `[0.48, 0.12, 0.18]` |
| `start_min` | `[0.34, -0.08, 0.20]` |
| `start_max` | `[0.41, 0.08, 0.27]` |
| `min_pick_place_distance_m` | `0.08` |
| `goal_timeout_sec` | `300.0` |
| `setup_timeout_sec` | `60.0` |
| `action_server_timeout_sec` | `30.0` |
| `wait_for_joint_states` | `false` |
| `joint_states_timeout_sec` | `30.0` |
| `start_velocity_scale` | `0.35` |
| `execute` | `true` |
| `obstacle_id` | `drl_pick_place_random_obstacle` |
| `obstacle_size_min` | `[0.018, 0.018, 0.018]` |
| `obstacle_size_max` | `[0.040, 0.040, 0.045]` |
| `obstacle_path_fraction_min` | `0.35` |
| `obstacle_path_fraction_max` | `0.75` |
| `obstacle_lateral_min_m` | `0.090` |
| `obstacle_lateral_max_m` | `0.130` |
| `obstacle_endpoint_clearance_m` | `0.090` |
| `target_obstacle_clearance_m` | `0.060` |
| `obstacle_roll_pitch_max_rad` | `0.20` |
| `obstacle_yaw_min_rad` | `-pi` |
| `obstacle_yaw_max_rad` | `pi` |

Gazebo branch trong launch override thêm:

| Parameter | Gazebo override |
|---|---|
| `workspace_min` | `[0.32, -0.10, 0.10]` |
| `workspace_max` | `[0.46, 0.10, 0.18]` |
| `start_min` | `[0.35, -0.06, 0.22]` |
| `start_max` | `[0.40, 0.06, 0.27]` |
| `goal_timeout_sec` | `420.0` |
| `setup_timeout_sec` | `90.0` |
| `action_server_timeout_sec` | `120.0` |
| `wait_for_joint_states` | `true` |
| `joint_states_timeout_sec` | `90.0` |

## `/repeatability_test`

Goal:

```yaml
AXIS_X: 0
AXIS_Y: 1
AXIS_Z: 2
retract_pose: geometry_msgs/PoseStamped
disturb_pose_1: geometry_msgs/PoseStamped
disturb_pose_2: geometry_msgs/PoseStamped
axis: uint8
meas_offset: float64
repeat_count: int32
velocity_scale: float64
execute: bool
```

Result:

```yaml
success: bool
message: string
completed_count: int32
```

Feedback:

```yaml
current_index: int32
current_step: string
```

Server parameters:

| Parameter | Default |
|---|---|
| `server_wait_timeout_s` | `5.0` |
| `action_result_timeout_s` | `120.0` |
| `measurement_settle_time_s` | `2.0` |
| `fast_velocity_scale` | `0.7` |

Rule:

- `repeat_count > 0`.
- `axis` chỉ nhận `0` (`AXIS_X`), `1` (`AXIS_Y`) hoặc `2` (`AXIS_Z`).
- `meas_offset` phải finite và khác `0.0`.
- `velocity_scale` phải finite và nằm trong `(0, 1]`.
- `execute` quyết định chạy thật hay chỉ planning/dry-run.
- `velocity_scale` chỉ dùng cho đoạn đo chậm từ `retract_pose` tới `meas_pose`; các đoạn còn lại dùng parameter server `fast_velocity_scale` default `0.7`.
- Các pose phải finite và quaternion norm lớn hơn `1e-12`.
- Server phụ thuộc `/move_to_pose`, `/move_to_pose_cartesian`.

Sequence:

1. MoveToPose tới `retract_pose` với `fast_velocity_scale`.
2. Mỗi loop:
   - Cartesian tới `meas_pose` với `goal.velocity_scale`.
   - Wait `measurement_settle_time_s`.
   - Cartesian quay về `retract_pose` với `fast_velocity_scale`.
   - MoveToPose tới `disturb_pose_1` với `fast_velocity_scale`.
   - MoveToPose tới `disturb_pose_2` với `fast_velocity_scale`.
   - MoveToPose quay về `retract_pose` với `fast_velocity_scale`.

`meas_pose` được tính từ `retract_pose`:

- `axis=0`: `meas_pose.x = retract_pose.x + meas_offset`.
- `axis=1`: `meas_pose.y = retract_pose.y + meas_offset`.
- `axis=2`: `meas_pose.z = retract_pose.z + meas_offset`.

CLI:

```bash
ros2 action send_goal /repeatability_test robot_task_manager/action/RepeatabilityTest \
  "{retract_pose: {header: {frame_id: 'world'}, pose: {position: {x: 0.40, y: 0.00, z: 0.18}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}}, disturb_pose_1: {header: {frame_id: 'world'}, pose: {position: {x: 0.35, y: -0.08, z: 0.18}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}}, disturb_pose_2: {header: {frame_id: 'world'}, pose: {position: {x: 0.45, y: 0.08, z: 0.18}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}}, axis: 0, meas_offset: 0.02, repeat_count: 3, velocity_scale: 0.25, execute: true}" \
  --feedback
```

Plan only:

```bash
ros2 action send_goal /repeatability_test robot_task_manager/action/RepeatabilityTest \
  "{retract_pose: {header: {frame_id: 'world'}, pose: {position: {x: 0.40, y: 0.00, z: 0.18}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}}, disturb_pose_1: {header: {frame_id: 'world'}, pose: {position: {x: 0.35, y: -0.08, z: 0.18}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}}, disturb_pose_2: {header: {frame_id: 'world'}, pose: {position: {x: 0.45, y: 0.08, z: 0.18}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}}, axis: 2, meas_offset: 0.02, repeat_count: 3, velocity_scale: 0.25, execute: false}" \
  --feedback
```

Client launch:

```bash
ros2 launch robot_task_manager repeatability_test_client.launch.py \
  axis:=0 repeat_count:=3 meas_offset:=0.02 velocity_scale:=0.25 execute:=true frame_id:=world
```

Z-axis chạy thật:

```bash
ros2 launch robot_task_manager repeatability_test_client.launch.py \
  axis:=2 repeat_count:=3 meas_offset:=-0.02 velocity_scale:=0.15 execute:=true frame_id:=world
```

Z-axis plan only:

```bash
ros2 launch robot_task_manager repeatability_test_client.launch.py \
  axis:=2 repeat_count:=3 meas_offset:=-0.02 velocity_scale:=0.15 execute:=false frame_id:=world
```

Default parameter của `repeatability_test_client.py` và launch:

| Parameter | Default |
|---|---|
| `action_name` | `repeatability_test` |
| `axis` | `0` |
| `repeat_count` | `3` |
| `meas_offset` | `0.02` |
| `velocity_scale` | `0.25` |
| `execute` | `true` |
| `frame_id` | `world` |
| `goal_timeout_sec` | `600.0` |

Default goal pose trong `repeatability_test_client.py`:

| Field | Default |
|---|---|
| `retract_pose.position` | `{x: 0.40, y: 0.00, z: 0.18}` |
| `disturb_pose_1.position` | `{x: 0.35, y: -0.08, z: 0.18}` |
| `disturb_pose_2.position` | `{x: 0.45, y: 0.08, z: 0.18}` |
| orientation của cả 3 pose | `{x: 1.0, y: 1.0, z: 0.0, w: 0.0}` |

## `/move_pose_rl`

Action mới dùng DRL planner để plan/execute từ TCP hiện tại tới target pose.

| Thuộc tính | Giá trị |
|---|---|
| Action name | `/move_pose_rl` |
| Type | `robot_task_manager/action/MovePoseRl` |
| Server executable | `move_pose_rl_server` |
| Node name | `move_pose_rl_action_server` |

Goal:

```yaml
target_pose: geometry_msgs/Pose
velocity_scale: float64
execute: bool
```

Result:

```yaml
success: bool
message: string
failed_stage: string
```

Feedback:

```yaml
current_stage: string
progress: float32
current_pose: geometry_msgs/PoseStamped
```

Server parameters:

| Parameter | Default |
|---|---|
| `planning_frame` | `base_link` |
| `ee_link` | `tcp_link` |
| `position_tolerance_m` | `0.01` |
| `orientation_tolerance_rad` | `0.10` |
| `drl_timeout_sec` | `120.0` |
| `drl_trajectory_endpoint_tolerance_m` | `0.015` |
| `drl_plan_attempts` | `3` |
| `tf_timeout_sec` | `2.0` |
| `sub_action_timeout_sec` | `60.0` |
| `planner_node_name` | `/drl_unified_planner_node` |

Rule validation:

- `velocity_scale` phải finite và nằm trong `(0, 1]`.
- `target_pose` phải finite.
- Quaternion của `target_pose.orientation` phải hợp lệ, norm > `1e-12`.
- Server phải nhận được `/joint_states` trước khi plan; nếu không có sẽ abort tại `get_current_pose`.
- Server phải lấy được TF current TCP từ `planning_frame <- ee_link`; nếu không có sẽ abort tại `get_current_pose`.
- DRL planner hiện nhận target qua parameter `manual_default_target` dạng `{x, y, z}`, nên endpoint/final check là position-only. Quaternion vẫn được validate để giữ interface đồng bộ với `/move_to_pose`.

Sequence:

1. Validate goal.
2. Chờ `/joint_states` và lấy current TCP pose qua TF.
3. Set DRL planner params: `manual_default_target`, `preposition_before_plan=false`, `update_start_tcp_from_tf_before_plan=true`, `auto_execute_after_plan=false`.
4. Gọi `/drl/clear_trajectory`.
5. Gọi `/drl/plan`.
6. Chờ trajectory mới trên `/drl/forward_trajectory_poses`.
7. Kiểm tra final waypoint gần target theo `drl_trajectory_endpoint_tolerance_m`.
8. Nếu `execute=false`: succeed, không gọi `/drl/execute_forward`.
9. Nếu `execute=true`: gọi `/drl/execute_forward`, poll `/drl/get_execution_status`, rồi kiểm tra final TCP position.

Plan only:

```bash
ros2 action send_goal /move_pose_rl robot_task_manager/action/MovePoseRl \
  "{target_pose: {position: {x: 0.40, y: 0.10, z: 0.35}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, velocity_scale: 0.5, execute: false}" \
  --feedback
```

Execute:

```bash
ros2 action send_goal /move_pose_rl robot_task_manager/action/MovePoseRl \
  "{target_pose: {position: {x: 0.40, y: 0.10, z: 0.35}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, velocity_scale: 0.5, execute: true}" \
  --feedback
```

Chạy bằng `task_manager_client`:

```bash
ros2 run robot_task_manager task_manager_client --ros-args \
  -p task_name:=move_pose_rl
```

Plan-only qua client:

```bash
ros2 run robot_task_manager task_manager_client --ros-args \
  -p task_name:=move_pose_rl \
  -p execute:=false
```

Default goal trong `task_manager_client`:

| Field | Default |
|---|---|
| `target_pose.position` | `{x: 0.40, y: 0.10, z: 0.35}` |
| `target_pose.orientation` | `{x: 0.0, y: 0.0, z: 0.0, w: 1.0}` |
| `velocity_scale` | `0.5` |
| `execute` | `true` |

Có thể override position trong client bằng `target_x`, `target_y`, `target_z`.

## `task_manager_client`

Client C++ mẫu:

```bash
ros2 run robot_task_manager task_manager_client --ros-args -p task_name:=<task>
```

`task_name` default là `gohome`.

Các giá trị đang có:

| `task_name` | Action client gọi |
|---|---|
| `gohome` | `/gohome` |
| `move_to_pose` | `/move_to_pose` |
| `move_to_pose_cartesian` | `/move_to_pose_cartesian` |
| `move_pose_rl` | `/move_pose_rl` |
| `checker_board` | `/move_checker_board` |
| `move_gripper` | `/move_gripper` |
| `pickplace` | `/pickplace` |

Client này có parameter `execute`, default `true`; tất cả goal trong client dùng giá trị parameter này.

`task_manager_client` hiện chưa có nhánh cho `/drl_pickplace` và `/repeatability_test`; hai action này có Python client riêng hoặc dùng `ros2 action send_goal`.
