# MovePoseRl action report

## 1. File da them/sua

Them:

- `robot_task_manager/action/MovePoseRl.action`
- `robot_task_manager/src/move_pose_rl_server.cpp`
- `robot_task_manager/move_pose_rl_action_report.md`

Sua:

- `robot_task_manager/CMakeLists.txt`
- `robot_task_manager/launch/task_servers.launch.py`
- `robot_task_manager/launch/task_servers_sim.launch.py`
- `robot_task_manager/src/task_manager_client.cpp`
- `robot_task_manager/Call_action.md`

## 2. Interface action `MovePoseRl`

```action
geometry_msgs/Pose target_pose
float64 velocity_scale
bool execute
---
bool success
string message
string failed_stage
---
string current_stage
float32 progress
geometry_msgs/PoseStamped current_pose
```

## 3. Action name, executable, node name

- Action name: `/move_pose_rl`
- Type: `robot_task_manager/action/MovePoseRl`
- Executable: `move_pose_rl_server`
- Node name: `move_pose_rl_action_server`

## 4. DRL logic tai su dung tu `/drl_pickplace`

Server moi tai su dung cung co che DRL core:

- Lay current TCP pose bang TF `planning_frame <- ee_link`.
- Goi `${planner_node_name}/set_parameters`.
- Set `manual_default_target`, `preposition_before_plan=false`, `update_start_tcp_from_tf_before_plan=true`, `auto_execute_after_plan=false`.
- Goi `/drl/clear_trajectory`.
- Goi `/drl/plan`.
- Cho trajectory tren `/drl/forward_trajectory_poses`.
- Goi `/drl/execute_forward` chi khi `execute=true`.
- Poll `/drl/get_execution_status` khi execute.

Khong sua behavior cua `drl_pickplace_server`.

## 5. Flow `execute=true`

1. Validate goal.
2. Wait `/joint_states`.
3. Lay current TCP pose qua TF.
4. Wait DRL services.
5. Set DRL target va plan.
6. Check final waypoint position gan target.
7. Goi `/drl/execute_forward`.
8. Poll `/drl/get_execution_status`.
9. Check final TCP position.
10. Result success neu tat ca pass.

## 6. Flow `execute=false`

1. Validate goal.
2. Wait `/joint_states`.
3. Lay current TCP pose qua TF.
4. Wait DRL plan/clear services.
5. Set DRL target va plan.
6. Check final waypoint position gan target.
7. Success voi message `DRL plan succeeded; execution skipped because execute=false`.
8. Khong goi `/drl/execute_forward`.

## 7. Xu ly khi thieu TF/current pose/current state

- Neu goal non-finite, quaternion invalid, hoac `velocity_scale` ngoai `(0, 1]`: abort stage `validate_goal`.
- Neu khong nhan `/joint_states`: abort stage `get_current_pose` voi message `Refusing to plan from zero/default state`.
- Neu khong lay duoc TF current TCP: abort stage `get_current_pose`.
- Neu DRL service thieu hoac planner fail: abort stage `drl_plan` hoac `endpoint_check`.

DRL planner hien chi nhan target position qua `manual_default_target`, nen orientation duoc validate nhung khong enforce o endpoint/final check.

## 8. Build

```bash
cd ~/ros2_dev
colcon build --packages-select robot_task_manager robot_drl --symlink-install
```

Ket qua:

- `robot_task_manager`: build thanh cong.
- `robot_drl`: build thanh cong.

## 9. Test da chay

Interface:

```bash
ros2 interface show robot_task_manager/action/MovePoseRl
```

Ket qua: interface hien dung cac field goal/result/feedback.

Action list:

```bash
ros2 launch robot_task_manager task_servers.launch.py
ros2 action list | rg 'move_pose_rl'
```

Ket qua: `/move_pose_rl` xuat hien.

Fail-safe khi chua co `/joint_states`:

```bash
ros2 action send_goal /move_pose_rl robot_task_manager/action/MovePoseRl \
  "{target_pose: {position: {x: 0.40, y: 0.10, z: 0.35}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, velocity_scale: 0.1, execute: false}" \
  --feedback
```

Ket qua: abort tai `get_current_pose`, message:

```text
Failed to receive /joint_states before DRL planning. Refusing to plan from zero/default state.
```

Plan-only voi DRL mock hardware:

```bash
ros2 launch robot_drl drl_mock_hw.launch.py \
  auto_plan_on_start:=false \
  auto_execute_after_plan:=false \
  manual_prompt_on_start:=false \
  input_mode:=manual \
  target_x:=0.40 target_y:=0.10 target_z:=0.25

ros2 launch robot_task_manager task_servers.launch.py

ros2 action send_goal /move_pose_rl robot_task_manager/action/MovePoseRl \
  "{target_pose: {position: {x: 0.375, y: 0.0, z: 0.25}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, velocity_scale: 0.1, execute: false}" \
  --feedback
```

Ket qua:

```text
success: true
message: DRL plan succeeded; execution skipped because execute=false
```

Execute voi DRL mock hardware:

```bash
ros2 action send_goal /move_pose_rl robot_task_manager/action/MovePoseRl \
  "{target_pose: {position: {x: 0.375, y: 0.0, z: 0.25}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, velocity_scale: 0.1, execute: true}" \
  --feedback
```

Ket qua:

```text
success: true
message: MovePoseRl completed successfully
```

Final TCP pose trong feedback gan target:

```text
x: 0.374952
y: -0.000020
z: 0.249947
```

`task_manager_client`:

```bash
ros2 run robot_task_manager task_manager_client --ros-args \
  -p task_name:=move_pose_rl \
  -p execute:=false \
  -p target_x:=0.375 \
  -p target_y:=0.0 \
  -p target_z:=0.25
```

Ket qua:

```text
success: true | failed_stage:  | message: DRL plan succeeded; execution skipped because execute=false
```

Ghi chu test: `robot_drl drl_mock_hw.launch.py` co `mock_environment_node` fail do import `Box` tu `robot_vision_pipeline.msg`, nhung `drl_unified_planner_node` van chay duoc trong `input_mode:=manual` va cac service `/drl/*` hoat dong.

## 10. Xac nhan khong doi behavior action khac

Khong sua code server cua:

- `/move_to_pose`
- `/move_to_pose_cartesian`
- `/pickplace`
- `/drl_pickplace`
- `/repeatability_test`

Chi them action/interface/server moi, launch entry moi, client branch moi va tai lieu.
