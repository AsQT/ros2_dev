# GoHome action fix report

## Muc tieu

Cap nhat `/gohome` de goal co ca `start` va `execute`, ho tro plan-only khi
`execute=false`, va khong plan tu zero/default state khi chua lay duoc robot
state hien tai tu `/joint_states`.

## Thay doi da thuc hien

- Xac nhan `robot_task_manager/action/GoHome.action` dang dung interface:
  `bool start`, `bool execute`, result `success/message`, feedback
  `current_step/progress`.
- Cap nhat `gohome_server`:
  - `start=false` duoc abort voi message
    `GoHome rejected because start=false`.
  - `execute=true` plan va execute named target `home`.
  - `execute=false` chi plan, khong execute, tra message
    `GoHome plan succeeded; execution skipped because execute=false`.
- Cap nhat `MoveItExecutor::goNamedTarget()` de goi
  `startStateMonitor()`/`getCurrentState()` va set start state that truoc khi
  plan. Neu khong lay duoc state tu `/joint_states`, action fail voi message:
  `Failed to get current robot state from /joint_states. Refusing to plan from zero/default state.`
- Bat lai `gohome_server` trong `task_servers.launch.py`.
- Xac nhan `task_servers_sim.launch.py` da co `gohome_server`.
- Xac nhan `task_manager_client` gui du `start=true` va `execute` theo tham so
  `-p execute:=...`.
- Cap nhat tai lieu `robot_task_manager/Call_action.md` cho `/gohome`.

## Kiem tra

Build:

```bash
cd /home/minhquang/ros2_dev
colcon build --packages-select robot_task_manager --symlink-install
```

Ket qua: thanh cong.

Interface:

```bash
ros2 interface show robot_task_manager/action/GoHome
```

Ket qua:

```text
bool start
bool execute
---
bool success
string message
---
string current_step
float32 progress
```

Runtime voi mock MoveIt:

```bash
ros2 launch robot_moveit moveit_gui.launch.py use_mock:=true gui_delay:=1.0 initial_page:=1
ros2 launch robot_task_manager task_servers.launch.py
```

Action ton tai:

```bash
ros2 action list | rg '^/gohome$'
```

Ket qua: `/gohome`.

Case `start=false`:

```bash
ros2 action send_goal /gohome robot_task_manager/action/GoHome "{start: false, execute: false}" --feedback
```

Ket qua: goal accepted, action abort, `success=false`, message
`GoHome rejected because start=false`.

Case `execute=false`:

```bash
ros2 action send_goal /gohome robot_task_manager/action/GoHome "{start: true, execute: false}" --feedback
```

Ket qua: action succeed, feedback `Home plan validated (execution skipped)`,
message `GoHome plan succeeded; execution skipped because execute=false`.

Case `execute=true`:

```bash
ros2 action send_goal /gohome robot_task_manager/action/GoHome "{start: true, execute: true}" --feedback
```

Ket qua: action succeed, feedback `Home_reached`, message
`Robot moved to home successfully`.

Client:

```bash
ros2 run robot_task_manager task_manager_client --ros-args -p task_name:=gohome -p execute:=false
```

Ket qua: goal accepted, result code succeeded, message
`GoHome plan succeeded; execution skipped because execute=false`.

