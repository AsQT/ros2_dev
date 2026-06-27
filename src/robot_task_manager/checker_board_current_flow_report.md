# Bao cao cach chay CheckerBoard action hien tai

Ngay tao: 2026-06-27

## 1. Tom tat nhanh

CheckerBoard hien tai la ROS 2 action `/move_checker_board`, type
`robot_task_manager/action/CheckerBoard`, server executable la
`checker_board_server`.

Server nhan goal gom:

```text
float64 step
float64 velocity_scale
bool execute
```

Ket qua tra ve:

```text
bool success
string message
```

Feedback tra ve:

```text
string stage
float32 progress
```

Muc dich cua action la lay TCP pose hien tai lam tam, tao mot luoi 3x3 tren mat
phang XY, sau do tinh Cartesian path theo kieu zig-zag. Moi diem dich se co 2
waypoint: mot diem travel duoc nang Z len `step / 2.0`, va mot diem drop o dung
Z goc cua target.

## 2. File va thanh phan lien quan

| Thanh phan | File | Vai tro |
| --- | --- | --- |
| Action definition | `robot_task_manager/action/CheckerBoard.action` | Khai bao goal/result/feedback |
| Action server | `robot_task_manager/src/move_checker_board_server.cpp` | Tao action `/move_checker_board`, validate goal, goi executor |
| Executor | `robot_task_manager/src/moveit_executor.cpp` | Ham `MoveItExecutor::checkerBoard()` tao waypoint, plan va execute |
| Header executor | `robot_task_manager/include/robot_task_manager/moveit_executor.hpp` | Khai bao API `checkerBoard()` |
| Build target | `robot_task_manager/CMakeLists.txt` | Build executable `checker_board_server` |
| Launch | `robot_task_manager/launch/task_servers.launch.py` va `task_servers_sim.launch.py` | Start node `checker_board_server` |
| Client tong | `robot_task_manager/src/task_manager_client.cpp` | Nhan `task_name:=checker_board` va gui goal mac dinh |

## 3. Action server chay nhu the nao

`CheckerBoardActionServer` duoc tao voi node name mac dinh
`move_checker_board_action_server`.

Khi khoi tao:

1. Doc parameter `planning_group`, mac dinh `arm`.
2. Doc parameter `base_frame`, mac dinh `world`.
3. Tao action server ten `move_checker_board`.
4. Goi `initialize_moveit()` de tao `MoveItExecutor` va khoi tao MoveIt.

Khi nhan goal:

1. Server chi validate `velocity_scale`.
2. Dieu kien hop le: `0.0 < velocity_scale <= 1.0`.
3. Neu hop le thi `ACCEPT_AND_EXECUTE`.
4. `step` hien chua duoc validate o server, nen gia tri 0 hoac am van co the di
   tiep vao executor.

Khi execute:

1. Publish feedback 30%:
   - `Planning checker board path` neu `execute=true`.
   - `Planning checker board path (plan-only)` neu `execute=false`.
2. Goi:

```cpp
executor_->checkerBoard(
  goal->step,
  error_msg,
  goal->velocity_scale,
  0.3,
  5.0,
  goal->execute);
```

Trong do:

- `step`: lay tu goal.
- `velocity_scale`: lay tu goal.
- `acceleration_scale`: co dinh `0.3`.
- `planning_time`: co dinh `5.0` giay.
- `execute`: lay tu goal.

Neu executor tra ve false, action abort voi message tu `error_msg`. Neu thanh
cong, publish feedback 100% va succeed.

Cancel request se goi `executor_->stop()`, trong executor ham nay goi
`move_group_->stop()`, clear pose targets va publish text `Motion stopped`.

## 4. Executor `MoveItExecutor::checkerBoard()`

Truoc khi tao path, executor:

1. Khoa `motion_mutex_`, nen cac motion action dung chung executor se khong chay
   chen nhau trong cung mot instance.
2. Kiem tra da initialize chua.
3. Validate `velocity_scale` va `acceleration_scale`, ca hai phai nam trong
   `(0, 1]`.
4. Set planning time, velocity scale, acceleration scale cho MoveGroup.
5. Goi `getCurrentStateForPlanning(2.0, error_msg)`.
6. Neu khong lay duoc current state tu `/joint_states`, executor tu choi plan
   de tranh plan tu zero/default state.
7. Lay `start_pose = move_group_->getCurrentPose().pose`.
8. Xoa marker cu va publish text `Checker_board`.

## 5. Cach tao luoi 3x3

`start_pose` duoc xem la tam cua ban co. Ham tao 9 target theo offset:

```text
row =  1: col = -1, 0, 1
row =  0: col =  1, 0, -1
row = -1: col = -1, 0, 1
```

Voi moi target:

```text
x = start_pose.x + col * step
y = start_pose.y + row * step
z = start_pose.z
orientation = start_pose.orientation
```

Thu tu nay tao duong quet zig-zag tren XY:

```text
(-1, +1) -> (0, +1) -> (+1, +1)
                              |
(+1,  0) -> (0,  0) -> (-1,  0)
                              |
(-1, -1) -> (0, -1) -> (+1, -1)
```

Luu y: diem tam `(0, 0)` cung nam trong 9 target, nen robot se quay lai tam
mot lan trong chuoi scan.

## 6. Cach tao waypoint Cartesian

Danh sach waypoint bat dau bang pose hien tai:

```text
waypoints[0] = start_pose
```

Sau do voi moi target trong 9 target:

1. Tao `travel_pose = target`.
2. Gan `travel_pose.z = target.z + step / 2.0`.
3. Push `travel_pose`.
4. Tao `drop_pose = target`.
5. Push `drop_pose`.

Tong so waypoint hien tai la:

```text
1 start waypoint + 9 * 2 waypoint = 19 waypoint
```

Hinh hoc motion:

- Di tu pose hien tai den diem tren cao cua target dau tien.
- Ha xuong target dau tien.
- Lap lai voi cac target tiep theo.
- Do tat ca waypoints duoc dua vao mot lan `computeCartesianPath()`, duong di
  giua cac diem drop/travel lien tiep cung la mot Cartesian path lien tuc.

## 7. Planning va execute

Executor goi:

```cpp
move_group_->computeCartesianPath(
  waypoints,
  0.01,
  trajectory,
  true);
```

Thong so:

- `eef_step = 0.01`.
- `avoid_collisions = true`.
- Dieu kien thanh cong: `fraction >= 0.99`.

Neu `fraction < 0.99`, action abort voi message:

```text
Cartesian path planning failed, fraction = ...
```

Neu plan thanh cong:

1. Publish trajectory line len RViz visual tools neu co joint model group.
2. Dong goi trajectory vao `MoveGroupInterface::Plan`.
3. Neu `execute=false`, dung lai sau planning va tra ve success:
   `CheckerBoard planning success; execution skipped`.
4. Neu `execute=true`, goi `move_group_->execute(plan)`.
5. Neu execute fail, abort voi message `Execution of Cartesian path failed`.
6. Neu execute success, action succeed voi message:
   `Checker board motion completed successfully`.

## 8. Cach goi hien tai

Launch server that:

```bash
ros2 launch robot_task_manager task_servers.launch.py
```

Hoac sim:

```bash
ros2 launch robot_task_manager task_servers_sim.launch.py
```

Goi action truc tiep:

```bash
ros2 action send_goal /move_checker_board robot_task_manager/action/CheckerBoard \
  "{step: 0.1, velocity_scale: 0.1, execute: true}" --feedback
```

Plan-only:

```bash
ros2 action send_goal /move_checker_board robot_task_manager/action/CheckerBoard \
  "{step: 0.1, velocity_scale: 0.1, execute: false}" --feedback
```

Neu dung `task_manager_client` voi `task_name:=checker_board`, client hien gui
goal mac dinh:

```text
step = 0.40
velocity_scale = 0.2
execute = parameter execute
```

## 9. Diem can luu y / rui ro hien tai

1. `step` chua duoc validate. Neu `step=0`, 9 target trung nhau va
   `lift_height=0`. Neu `step<0`, huong offset va lift height bi dao nguoc,
   co the tao waypoint Z thap hon target.
2. Action server chi feedback 30% va 100%, khong feedback theo tung diem trong
   luoi.
3. Tat ca 19 waypoint duoc plan trong mot Cartesian path duy nhat. Neu mot doan
   nho khong kha thi toan bo action fail voi `fraction < 0.99`.
4. Path chi thay doi position, orientation giu nguyen theo TCP pose luc bat dau.
5. Current state guard da co: neu MoveIt khong lay duoc state that tu
   `/joint_states`, executor se khong plan tu zero/default state.
6. `planning_time=5.0` duoc truyen vao executor nhung Cartesian planning bang
   `computeCartesianPath()` chu yeu bi chi phoi boi waypoints, `eef_step` va
   collision checking.

## 10. Ket luan

CheckerBoard hien tai chay theo mo hinh: action server nhan goal -> validate
velocity -> executor lay current TCP pose lam tam -> sinh 9 target zig-zag tren
XY -> chen waypoint nang/ha Z tai moi target -> tinh Cartesian path voi
collision avoidance -> tuy `execute` ma chi validate plan hoac execute trajectory.

De lam hanh vi an toan hon, nen bo sung validate `step > 0`, co gioi han max
step theo workspace, va neu can debug calibration thi them feedback theo index
target hien tai.
