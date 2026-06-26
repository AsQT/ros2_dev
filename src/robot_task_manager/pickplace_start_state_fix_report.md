# PickPlace start state fix report

## 1. Nguyen nhan

`/pickplace` la composite action goi cac action con `/move_gripper`,
`/move_to_pose`, va `/move_to_pose_cartesian`.

Nguyen nhan chinh lam RViz co the nhay ve joint zero nam o tang executor dung
chung:

- `MoveItExecutor::moveToPose()` van goi `setStartStateToCurrentState()` truc
  tiep truoc khi `plan()`.
- `MoveItExecutor::moveToPoseCartesian()` van goi
  `setStartStateToCurrentState()` truc tiep truoc khi
  `computeCartesianPath()`.
- Neu current state monitor chua co state hop le tu `/joint_states`, MoveIt co
  nguy co lap ke hoach tu state mac dinh/zero.
- `GripperExecutor` cung lay `getCurrentJointValues()` ma chua bat buoc lay
  current state hop le truoc planning.

`pickplace_server` khong tu publish `DisplayTrajectory` va khong tao
`trajectory_start`. Loi den tu action con khi planning.

## 2. File da sua

- `robot_task_manager/src/moveit_executor.cpp`
- `robot_task_manager/include/robot_task_manager/gripper_executor.hpp`
- `robot_task_manager/src/gripper_executor.cpp`
- `robot_task_manager/src/pickplace_server.cpp`
- `robot_task_manager/pickplace_start_state_fix_report.md`

## 3. Sua o tang dung chung

Co. `MoveItExecutor` da co helper `getCurrentStateForPlanning()`, va sau thay
doi helper nay duoc dung truoc cac duong planning chinh:

- `goNamedTarget()`
- `moveToPose()`
- `moveToPoseCartesian()`
- `checkerBoard()`

Them helper tuong tu cho `GripperExecutor` de `/move_gripper` cung khong plan tu
state mac dinh.

## 4. Cach lay current state truoc planning

Executor goi:

```cpp
move_group_->startStateMonitor();
auto current_state = move_group_->getCurrentState(timeout_sec);
move_group_->setStartState(*current_state);
```

Neu `current_state == nullptr`, action fail truoc khi plan.

## 5. Xu ly khi khong co `/joint_states`

Message loi dung chung:

```text
Failed to get current robot state from /joint_states. Refusing to plan from zero/default state.
```

Khong fallback ve zero/default state, va khong tiep tuc `plan()` hay
`computeCartesianPath()` khi current state unavailable.

## 6. PickPlace plan-only

`/pickplace execute=false` khong execute robot motion va khong execute gripper
motion. Tuy nhien staged composite plan-only chua duoc ho tro dung nghia vi cac
segment sau can bat dau tu terminal state cua segment truoc, trong khi robot
khong di chuyen khi `execute=false`.

Vi vay server khong bao success gia. Sau khi validate duoc open gripper
plan-only va move-to-pre-pick plan-only, server abort ro:

```text
cartesian_to_pick: PickPlace plan-only staged composite planning is not fully supported without executing intermediate segments; refusing to report fake success.
```

## 7. Ket qua test

Build:

```bash
cd /home/minhquang/ros2_dev
source install/setup.bash
colcon build --packages-select robot_task_manager robot_task_executor --symlink-install
```

Ket qua: thanh cong. Co warning colcon overlay va warning CMake cu ve
`rosidl_target_interfaces()`, khong phai loi compile.

Launch mock stack:

```bash
ros2 launch robot_moveit moveit_gui.launch.py use_mock:=true gui_delay:=1.0 initial_page:=1
ros2 launch robot_task_manager task_servers.launch.py
```

Kiem tra `/joint_states`:

```bash
ros2 topic echo /joint_states --once
```

Ket qua: co joint state tu mock hardware.

### `/move_to_pose execute=false`

Command:

```bash
ros2 action send_goal /move_to_pose robot_task_manager/action/MoveToPose \
  "{target_pose: {position: {x: 0.40, y: 0.10, z: 0.35}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}, velocity_scale: 0.3, execute: false}" \
  --feedback
```

Ket qua: `SUCCEEDED`, message
`MoveToPose planning success; execution skipped`.

### `/move_to_pose_cartesian execute=false`

Command:

```bash
ros2 action send_goal /move_to_pose_cartesian robot_task_manager/action/MoveToPoseCartesian \
  "{target_pose: {position: {x: 0.40, y: 0.10, z: 0.30}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}, velocity_scale: 0.2, execute: false}" \
  --feedback
```

Ket qua: `SUCCEEDED`, message
`MoveToPoseCartesian planning success; execution skipped`.

### `/pickplace execute=false`

Command:

```bash
ros2 action send_goal /pickplace robot_task_manager/action/PickPlace \
  "{pose_pick: {position: {x: 0.40, y: 0.10, z: 0.25}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}, pose_place: {position: {x: 0.30, y: 0.00, z: 0.25}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}, gripper: 0.01, velocity_scale: 0.3, execute: false}" \
  --feedback
```

Ket qua: `ABORTED` co chu dich, message:

```text
cartesian_to_pick: PickPlace plan-only staged composite planning is not fully supported without executing intermediate segments; refusing to report fake success.
```

Robot/gripper khong execute motion.

### `/pickplace execute=true`

Command:

```bash
ros2 action send_goal /pickplace robot_task_manager/action/PickPlace \
  "{pose_pick: {position: {x: 0.40, y: 0.10, z: 0.25}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}, pose_place: {position: {x: 0.30, y: 0.00, z: 0.25}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}, gripper: 0.01, velocity_scale: 0.3, execute: true}" \
  --feedback
```

Ket qua: `SUCCEEDED`, feedback di qua day du flow:

- `Open gripper`
- `Move to pick approach`
- `Cartesian down to pick`
- `Close gripper`
- `Move directly to place approach`
- `Cartesian down to place`
- `Open gripper to release`
- `Fast PickPlace completed`

Message: `Fast PickPlace completed successfully`.

### GUI Plan / Execute

GUI node duoc chay trong launch mock:

```text
robot_gui_node
```

PickPlace GUI mapping da co san trong `TaskActionController`:

- `btnPickPlacePlan` gui `/pickplace` voi `execute=false`
- `btnPickPlaceStart` gui `/pickplace` voi `execute=true`

Chua thuc hien click GUI tu dong trong luot nay. CLI da test truc tiep cung
action `/pickplace` va cung backend server ma GUI goi.

## 8. Xac nhan

- Khong fallback ve zero/default state trong `MoveItExecutor` va
  `GripperExecutor`.
- Khong doi action name.
- Khong doi topic/action server name.
- Khong doi logic gripper ngoai viec gripper planning cung bat buoc current
  state va van respect `execute=false`.
- Khong thay doi GUI action mapping.
- Khong sua bang cach an marker; start state duoc sua o goc planning.

