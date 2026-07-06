# TCP/Action CSV Logging Usage

## Mục tiêu
Logging trong `robot_task_manager` dùng để đánh giá action theo từng lần gọi:
summary/result, event timeline, metadata, TCP set-vs-actual, joint tracking,
trajectory/error metrics và dữ liệu RL/object khi action có hỗ trợ.

Source of truth đã kiểm tra:
- `include/robot_task_manager/log_paths.hpp`
- `include/robot_task_manager/standard_action_logger.hpp`
- `include/robot_task_manager/per_call_tcp_logger.hpp`
- các server trong `src/*.cpp`
- action definitions trong `action/*.action`

## Root và cấu trúc thư mục
Root mặc định trong C++ là:

```text
$HOME/ros2_dev/Log_robot_data
```

Một số launch hiện truyền literal:

```text
/home/minhquang/ros2_dev/Log_robot_data
```

Cấu trúc chuẩn:

```text
Log_robot_data/
└── <mock|real>/
    ├── 01_baseline_motion_eval/
    ├── 02_rl_motion_eval/
    ├── 03_task_execution_eval/
    └── _debug/
        └── <ActionDisplayName>/
            └── run_YYYYMMDD_HHMMSS_PID[_NNN]/
                └── call_0001/
                    ├── metadata.json
                    ├── events.csv
                    ├── summary.csv
                    └── ... action-specific CSV files ...
```

## Nhóm action
| Group | Action display |
|---|---|
| `01_baseline_motion_eval` | `MoveToPose`, `MoveToPoseCartesian`, `CheckerBoard`, `RepeatabilityTest` |
| `02_rl_motion_eval` | `MovePoseRl`, `MoveTargetRl`, `MoveToPoseObstacle`, `DrlPlanner` |
| `03_task_execution_eval` | `PickPlace`, `PickPlaceRL` |
| `_debug` | `GoHome`, `MoveGripper` |

## Logger chính
| Logger | File/source | Dùng cho |
|---|---|---|
| `StandardActionLogger` | `standard_action_logger.hpp` | `summary.csv`, `events.csv`, `metadata.json` cho action hỗ trợ standard logging. |
| `PerCallTcpLogger` | `per_call_tcp_logger.hpp` | TCP/joint/error tracking per-call cho `MoveToPose`, `MoveToPoseCartesian`, `CheckerBoard`, `RepeatabilityTest`. |
| PickPlace logger riêng | `pickplace_server.cpp` | CSV/summary riêng cho `/pickplace`. |
| Metrics/log writers trong RL servers | `move_pose_rl_server.cpp`, `move_target_rl_server.cpp`, `drl_pickplace_server.cpp`, `move_to_pose_obstacle_server.cpp` | RL input, planning, obstacle/object, trajectory tracking và metrics theo action. |

`robot_task_executor` và `robot_drl_executor` cũng có `ExecutorExperimentLogger`,
nhưng đó là logger executor/service nội bộ, không phải toàn bộ cơ chế action log của
`robot_task_manager`.

## Điều kiện bật log
| Action | Goal flag | Node/launch flag liên quan |
|---|---|---|
| `MoveToPose` | `enable_tcp_log` | `enable_executor_logging:=true` để khởi tạo per-call logger. |
| `MoveToPoseCartesian` | `enable_tcp_log` | `enable_executor_logging:=true`. |
| `CheckerBoard` | `enable_tcp_log` | `enable_executor_logging:=true`. |
| `RepeatabilityTest` | `enable_tcp_log` | `enable_executor_logging:=true`. |
| `PickPlace` | `enable_tcp_log` | Server có logger riêng; dùng field trong goal để quyết định ghi TCP log. |
| `MovePoseRl` | `enable_metrics_log` | Ghi metrics/action-specific CSV khi bật. |
| `MoveTargetRl` | `enable_metrics_log` | Ghi metrics/action-specific CSV khi bật. |
| `MoveToPoseObstacle` | `enable_metrics_log` | Ghi metrics/action-specific CSV khi bật. |
| `DrlPickPlace` | `enable_metrics_log` | Ghi PickPlaceRL metrics/action-specific CSV khi bật. |

`enable_standard_logging` mặc định `true` trong `task_servers.launch.py` và tạo
summary/events/metadata cho các action đã tích hợp logger chuẩn.

## Ví dụ bật logging
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_bringup mock.launch.py enable_executor_logging:=true
```

Gọi `/move_to_pose` có TCP log:

```bash
ros2 action send_goal /move_to_pose robot_task_manager/action/MoveToPose \
  "{target_pose: {position: {x: 0.35, y: 0.0, z: 0.25}, orientation: {w: 1.0}}, velocity_scale: 0.1, execute: true, enable_tcp_log: true}"
```

Gọi `/pickplace` có TCP log:

```bash
ros2 action send_goal /pickplace robot_task_manager/action/PickPlace \
  "{pose_pick: {position: {x: 0.35, y: 0.0, z: 0.08}, orientation: {w: 1.0}}, pose_place: {position: {x: 0.40, y: 0.10, z: 0.08}, orientation: {w: 1.0}}, gripper: 0.025, velocity_scale: 0.1, execute: true, enable_tcp_log: true}"
```

Gọi `/move_pose_rl` có metrics log:

```bash
ros2 action send_goal /move_pose_rl robot_task_manager/action/MovePoseRl \
  "{target_pose: {position: {x: 0.40, y: 0.0, z: 0.12}, orientation: {w: 1.0}}, velocity_scale: 0.1, execute: false, enable_metrics_log: true}"
```

## File thường gặp
- `metadata.json`: runtime mode, action name, run/call id, frame, planner/hardware metadata nếu server cung cấp.
- `events.csv`: timeline stage/event.
- `summary.csv`: kết quả cuối, success/failure, thời gian.
- `tcp_tracking.csv` hoặc file action-specific tương đương: TCP set/actual theo thời gian.
- `joint_tracking.csv`: joint state/setpoint tracking khi có dữ liệu.
- `error_tracking.csv`, `trajectory_metrics.csv`: sai số và metric quỹ đạo.
- `rl_input_15d*.csv`, `planning_*.csv`, `obstacle.csv`, `object_obstacle.csv`: dữ liệu riêng cho action RL/PickPlaceRL.

## Plot/viewer
Các script liên quan:

```bash
ros2 run robot_task_manager log_data_viewer_gui.py
ros2 run robot_task_manager plot_action_log.py
ros2 run robot_task_manager analyze_pickplace_tcp_logs.py
```

Các script này đọc `Log_robot_data` và một số file CSV chuẩn ở trên. Nếu thiếu file,
hãy kiểm tra goal flag, `enable_executor_logging`, `enable_standard_logging` và đúng
nhánh `mock`/`real`.
