# Bao cao doi CheckerBoard sang segmented Cartesian

Ngay tao: 2026-06-27

## 1. Nguyen nhan can doi flow cu

Flow cu cua `/move_checker_board` gom toan bo checkerboard vao mot Cartesian
path lon. Cach nay khong co diem dung that tai tung vi tri do, nen khong phu hop
cho bai toan do sai so/toa do tai tung diem checkerboard.

Yeu cau moi la robot phai di theo tung segment rieng, moi khi ha xuong diem Z do
thi dung `measurement_settle_time_s` giay de thiet bi co thoi gian lay mau.

## 2. Flow cu

Trong `MoveItExecutor::checkerBoard()` cu:

- Lay TCP pose hien tai lam tam.
- Sinh 9 target 3x3 zig-zag.
- Tao mot vector waypoint lon:
  - 1 start waypoint.
  - moi target co `travel_pose` va `drop_pose`.
  - tong 19 waypoint.
- Goi `computeCartesianPath()` mot lan cho toan bo 19 waypoint.
- Neu `execute=true`, execute mot trajectory lon.

## 3. Flow moi

Flow moi van giu action name `/move_checker_board` va action type
`robot_task_manager/action/CheckerBoard`.

Trong `MoveItExecutor::checkerBoard()` moi:

1. Validate `step`.
2. Lay current TCP pose lam tam.
3. Sinh 9 target 3x3 zig-zag nhu cu.
4. Voi tung target:
   - tao `travel_pose` voi `z = target.z + step / 2.0`.
   - goi Cartesian segment rieng toi `travel_pose`.
   - tao `drop_pose` tai Z do.
   - goi Cartesian segment rieng toi `drop_pose`.
   - neu `execute=true`, cho `measurement_settle_time_s`.
5. Khong con push toan bo 19 waypoint vao mot path lon.

Moi target co 2 segment, nen checkerboard co 18 lan plan Cartesian segment rieng.

## 4. File da sua

- `robot_task_manager/include/robot_task_manager/moveit_executor.hpp`
- `robot_task_manager/src/moveit_executor.cpp`
- `robot_task_manager/src/move_checker_board_server.cpp`
- `robot_task_manager/launch/task_servers.launch.py`
- `robot_task_manager/launch/task_servers_sim.launch.py`
- `robot_task_manager/Call_action.md`

## 5. Parameter moi

Them parameter cho `checker_board_server`:

```yaml
measurement_settle_time_s: 2.0
```

Default trong code va launch deu la `2.0`. Parameter nay khong lam thay doi
action interface.

## 6. Xu ly `execute=false`

Voi `execute=false`:

- Khong execute robot motion.
- Khong sleep/chong 2 giay tai diem do.
- Van plan lan luot tung segment de kiem tra tinh kha thi.
- Neu segment nao fail, action abort voi message co target index va segment:
  - `Target N/9: move to travel pose failed: ...`
  - `Target N/9: move down to measurement pose failed: ...`
- Neu tat ca segment plan duoc, result message:

```text
CheckerBoard segmented planning success; execution skipped because execute=false
```

## 7. Xu ly `execute=true`

Voi `execute=true`:

- Truoc moi segment, executor goi `getCurrentStateForPlanning()`.
- Neu khong lay duoc state that tu `/joint_states`, fail voi message hien co:

```text
Failed to get current robot state from /joint_states. Refusing to plan from zero/default state.
```

- Moi segment goi `computeCartesianPath()` rieng voi 2 waypoint:
  - current/start pose cua segment.
  - target pose cua segment.
- Dieu kien fraction van la `>= 0.99`.
- Sau moi `drop_pose`, executor publish feedback va cho
  `measurement_settle_time_s`.
- Neu thanh cong, result message:

```text
CheckerBoard segmented motion completed successfully
```

## 8. Validate `step`

Server reject goal neu:

```text
step khong finite hoac step <= 0.0
```

Message log:

```text
Reject goal because CheckerBoard step must be finite and > 0.0
```

Executor cung validate lai va tra ve:

```text
CheckerBoard step must be finite and > 0.0
```

## 9. Feedback moi

Server publish feedback ban dau 0%, executor publish theo tung buoc:

```text
CheckerBoard current pose acquired
CheckerBoard generated 3x3 zig-zag grid
Target 1/9: move to travel pose
Target 1/9: move down to measurement pose
CheckerBoard target 1/9 measurement wait 2.0s
...
Target 9/9: move down to measurement pose
CheckerBoard target 9/9 measurement wait 2.0s
CheckerBoard segmented motion completed
```

Voi `execute=false`, khong co feedback wait measurement.

## 10. Ket qua build

Da chay:

```bash
cd /home/minhquang/ros2_dev
colcon build --packages-select robot_task_manager
```

Ket qua:

```text
Summary: 1 package finished [1min 18s]
```

Sau khi kiem tra GUI van goi action interface cu, da build them:

```bash
cd /home/minhquang/ros2_dev
colcon build --packages-select robot_task_manager robot_gui
```

Ket qua:

```text
Summary: 2 packages finished [3.79s]
```

## 11. Ket qua test runtime

Chua chay runtime action trong phien nay vi can launch MoveIt/action server va
nguon `/joint_states` that.

Trang thai hien tai:

| Test | Trang thai |
| --- | --- |
| `execute=false` | Chua chay runtime; build da pass |
| `execute=true` | Chua chay runtime; build da pass |
| `step=0` | Chua chay runtime; code reject goal |
| `step<0` | Chua chay runtime; code reject goal |

Lenh test de chay tiep:

```bash
ros2 action send_goal /move_checker_board robot_task_manager/action/CheckerBoard \
  "{step: 0.1, velocity_scale: 0.1, execute: false}" --feedback
```

```bash
ros2 action send_goal /move_checker_board robot_task_manager/action/CheckerBoard \
  "{step: 0.1, velocity_scale: 0.1, execute: true}" --feedback
```

```bash
ros2 action send_goal /move_checker_board robot_task_manager/action/CheckerBoard \
  "{step: 0.0, velocity_scale: 0.1, execute: true}" --feedback
```

```bash
ros2 action send_goal /move_checker_board robot_task_manager/action/CheckerBoard \
  "{step: -0.1, velocity_scale: 0.1, execute: true}" --feedback
```

## 12. Xac nhan

- Khong con push toan bo 19 waypoint thanh mot path lon trong
  `MoveItExecutor::checkerBoard()`.
- Moi target checkerboard duoc tach thanh 2 Cartesian segment rieng.
- Voi `execute=true`, co 9 lan cho do sau 9 `drop_pose`.
- Voi `execute=false`, van plan tung segment, khong sleep.
- Truoc moi segment co guard current state tu `/joint_states`.
- Khong fallback ve zero/default state.
- Khong doi action name `/move_checker_board`.
- Khong doi action type `robot_task_manager/action/CheckerBoard`.
- GUI/client cu van goi duoc action vi action interface khong doi.
- Cac action khac khong bi doi API.
