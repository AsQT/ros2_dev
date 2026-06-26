# CheckerBoard start state fix report

## Nguyen nhan robot ve joint zero

`/move_checker_board` truoc day chi goi `setStartStateToCurrentState()` roi lap tuc tinh Cartesian path. Neu current state monitor chua co du lieu `/joint_states` du tin cay, MoveIt co the dung state mac dinh/zero khi tao request/path visualization, lam RViz hien robot nhay ve joint zero truoc khi planning.

Khong thay code cua CheckerBoard tao `DisplayTrajectory.trajectory_start` rieng. Visualization hien tai chi publish marker trajectory line bang `moveit_visual_tools`, vi vay diem can sua nam o start state cua MoveGroup truoc khi `computeCartesianPath()`.

## File da sua

- `robot_task_manager/include/robot_task_manager/moveit_executor.hpp`
- `robot_task_manager/src/moveit_executor.cpp`

## Cach lay current state truoc khi planning

Da them helper noi bo `MoveItExecutor::getCurrentStateForPlanning(timeout_sec, error_msg)`:

- Goi `move_group_->startStateMonitor()`.
- Goi `move_group_->getCurrentState(timeout_sec)` voi timeout 2 giay cho CheckerBoard.
- Chi khi lay duoc state moi goi `move_group_->setStartState(*current_state)`.
- `checkerBoard()` dung helper nay truoc khi lay current pose va truoc khi `computeCartesianPath()`.

## Xu ly khi khong co `/joint_states`

Neu khong lay duoc current state, action dung ngay va tra loi:

```text
Failed to get current robot state from /joint_states. Refusing to plan from zero/default state.
```

Nhu vay CheckerBoard khong tiep tuc planning tu zero/default state.

## Ket qua build

Lenh:

```bash
cd ~/ros2_dev
colcon build --packages-select robot_task_manager robot_task_executor --symlink-install
```

Ket qua: build thanh cong. Co canh bao CMake deprecation cu cua `rosidl_target_interfaces()`, khong lien quan den thay doi nay.

## Ket qua kiem tra `/joint_states`

Da chay stack mock/MoveIt va task servers:

```bash
ros2 launch robot_moveit moveit_gui.launch.py use_mock:=true gui_delay:=1.0 initial_page:=1
ros2 launch robot_task_manager task_servers.launch.py
ros2 topic echo /joint_states --once
```

`/joint_states` co du lieu cho `joint_1` den `joint_6` va gripper joints.

## Test CLI `execute=false`

Lenh theo yeu cau:

```bash
ros2 action send_goal /move_checker_board robot_task_manager/action/CheckerBoard \
  "{step: 0.1, velocity_scale: 0.1, execute: false}" --feedback
```

Ket qua: action duoc accept, lay duoc current state, sau do abort do Cartesian path khong dat nguong:

```text
success: false
message: Cartesian path planning failed, fraction = 0.302632
```

Ket qua nay xac nhan action khong fail vi timeout current state/`joint_states`; loi con lai la kha nang lap Cartesian path voi `step=0.1` tai pose mock hien tai.

Kiem tra bo sung voi duong nho hon:

```bash
ros2 action send_goal /move_checker_board robot_task_manager/action/CheckerBoard \
  "{step: 0.01, velocity_scale: 0.1, execute: false}" --feedback
```

Ket qua:

```text
success: true
message: CheckerBoard planning success; execution skipped
```

## Test CLI `execute=true`

Lenh theo yeu cau:

```bash
ros2 action send_goal /move_checker_board robot_task_manager/action/CheckerBoard \
  "{step: 0.1, velocity_scale: 0.1, execute: true}" --feedback
```

Ket qua: action duoc accept, lay duoc current state, sau do abort o buoc planning voi cung ly do:

```text
success: false
message: Cartesian path planning failed, fraction = 0.302632
```

Kiem tra bo sung:

```bash
ros2 action send_goal /move_checker_board robot_task_manager/action/CheckerBoard \
  "{step: 0.01, velocity_scale: 0.1, execute: true}" --feedback
```

Server log cho thay `CheckerBoard()` tra `ok=true` va MoveIt accept execute request. Client CLI bi timeout 60 giay truoc khi nhan result cuoi, nhung `/joint_states` sau do da thay doi khoi vi tri ban dau, xac nhan mock robot da bat dau di chuyen theo trajectory.

Log MoveIt lien quan:

```text
Computed Cartesian path with 29 points (followed 100.000000% of requested trajectory)
Starting trajectory execution ...
arm_controller started execution
Goal request accepted!
```

## Test GUI

Da launch `robot_moveit moveit_gui.launch.py`; log GUI/RViz bao:

```text
RViz inputs: robot_description available: yes, robot_description_semantic available: yes, joint_states available: yes
```

Trong phien CLI nay chua tu dong hoa thao tac bam nut CheckerBoard tren GUI. Duong goi action tu GUI van di vao cung action `/move_checker_board`, va fix nam trong server-side `MoveItExecutor::checkerBoard()`, nen ca GUI va CLI deu dung chung guard current state moi.

## Xac nhan pham vi thay doi

- Khong doi action name `/move_checker_board`.
- Khong doi action type `robot_task_manager/action/CheckerBoard`.
- Khong doi topic name.
- Khong doi layout GUI.
- Khong sua logic cac action khac nhu `/move_to_pose`, `/move_to_pose_cartesian`, `/pickplace`.
