# ROS2 Robot Workspace Documentation

> Tài liệu tổng quan cấp workspace là [`../README.md`](../README.md) ở root.
> File này là chỉ mục chi tiết theo package. Source of truth là source code hiện
> tại trong từng package.

## 1. Tổng quan
Workspace `~/ros2_dev/src` gồm **14 package ROS 2** cho mô tả robot, ros2_control,
MoveIt, task action/service, DRL planner, GUI, hardware interface, vision pipeline
và mô phỏng Gazebo.

## 2. Danh sách package
| Package | Vai trò |
|---|---|
| robot_description | URDF/Xacro, mesh, `display.launch` (RViz) |
| robot_control | ros2_control controllers và launch controller manager |
| robot_moveit | MoveIt config, move_group launch, RViz/GUI launch |
| robot_hardware_interface | TCP/RS485 hardware node, ros2_control plugin, service/msg hardware |
| robot_task_manager | Action servers MoveIt/gripper/pick-place/DRL/repeatability + logging CSV |
| robot_task_executor | Service-based MoveIt executor (legacy) |
| robot_task_executor_msgs | Service contracts cho executor |
| robot_drl | DRL planner, mock environment, obstacle tests |
| robot_drl_executor | Cartesian pose sequence MoveIt service cho DRL |
| robot_gui | Qt GUI, hardware controls, action clients, RViz embed |
| robot_vision_pipeline | YOLO/ArUco/depth/object pose pipeline |
| robot_vision_pipeline_msgs | Message contracts cho object detections |
| robot_gazebo | Gazebo world/model/SDF và script spawn object (tách từ robot_description) |
| robot_bringup | Launch tổng hợp mock/sim/real/demo |

## 3. Luồng hoạt động chính
```mermaid
flowchart LR
  Desc[robot_description] --> Control[robot_control]
  Desc --> MoveIt[robot_moveit move_group]
  Control --> MoveIt
  GUI[robot_gui] --> Tasks[robot_task_manager]
  Vision[robot_vision_pipeline] --> DRL[robot_drl]
  DRL --> DrlExec[robot_drl_executor]
  Tasks --> MoveIt
  DrlExec --> MoveIt
  MoveIt --> HW[robot_hardware_interface or Gazebo]
  Bringup[robot_bringup] --> Desc
  Bringup --> MoveIt
  Bringup --> Tasks
  Bringup --> DRL
```

## 4. Build nhanh
```bash
cd ~/ros2_dev
colcon build
source install/setup.bash
```

## 5. Chạy nhanh
- Mock hardware (GUI + MoveIt + task servers): `ros2 launch robot_bringup mock.launch.py`
- Simulation Gazebo: `ros2 launch robot_bringup sim.launch.py`
- Real/GUI: `ros2 launch robot_bringup real.launch.py`
- MoveIt mock: `ros2 launch robot_moveit moveit_mock.launch.py`
- GUI riêng: `ros2 launch robot_gui robot_gui.launch.py`
- Vision full pipeline: `ros2 launch robot_vision_pipeline vision_full_pipeline.launch.py`

Bật vision trong bringup: thêm `use_vision:=true` (mặc định `false` ở `mock`/`sim`,
`true` ở `real`).

## 6. Logging CSV
Root log mặc định `~/ros2_dev/Log_robot_data`, cấu trúc
`<mock|real>/<nhóm>/<TênAction>/run_.../call_.../`. Ba nhóm đánh giá:
`01_baseline_motion_eval`, `02_rl_motion_eval`, `03_task_execution_eval`, cùng
nhóm `_debug` (GoHome/MoveGripper). Chi tiết:
[`robot_task_manager/docs/tcp_csv_logging_usage.md`](robot_task_manager/docs/tcp_csv_logging_usage.md).

## 7. Tài liệu chi tiết
Mỗi package có `README.md`, `DATA_FLOW.md`, `PARAMETERS.md`, `LAUNCH.md`; package liên quan MoveIt/action/service có thêm `MOVEIT_ACTION_FLOW.md`. Báo cáo kỹ thuật/audit nằm trong [`Reports/`](Reports/).
