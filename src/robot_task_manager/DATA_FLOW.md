# robot_task_manager - Data Flow

## 1. Mục tiêu luồng dữ liệu
Chuyển goal/action từ GUI/client thành lệnh MoveIt, gripper, DRL hoặc chuỗi composite và trả feedback/result.

## 2. Input
- Action goal từ GUI/client: pose, velocity_scale, gripper width/position, execute flag, repeat count.
- Parameter launch: `enable_drl_backend`, `planner_node_name`, `drl_calibrated_start_tcp_base`.
- DRL trajectory topic/service khi dùng `drl_pickplace` hoặc `move_pose_rl`.

## 3. Output
- Feedback `stage/current_stage/progress/current_pose` tùy action.
- Result `success`, `message`, `failed_stage` nếu action có.
- Trajectory execute qua MoveIt/controller khi `execute=true`.

## 4. Internal processing
- Server validate goal và velocity scale.
- Server đơn giản gọi `moveit_executor` hoặc `gripper_executor`.
- Server composite gọi action/service con theo thứ tự.
- DRL server set parameter planner, gọi `/drl/plan`, `/drl/execute_forward`, kiểm tra status.

## 5. Sơ đồ luồng dữ liệu
```mermaid
flowchart LR
  GUI[GUI/client] --> Action[robot_task_manager action]
  Action --> MoveItExec[moveit_executor]
  Action --> Gripper[gripper_executor]
  Action --> DRL[/drl services]
  MoveItExec --> MoveGroup[move_group]
  Gripper --> MoveGroup
  MoveGroup --> Controller[ros2_control]
  Action --> Result[feedback/result]
```

## 6. Liên kết với package khác
- `robot_gui` là client operator.
- `robot_moveit` cung cấp `move_group`.
- `robot_drl` cung cấp planner service/topic.
- `robot_bringup` launch server theo mode.

## 7. Các điểm cần chú ý
- Pose dùng mét; orientation dùng quaternion.
- `velocity_scale` phải trong `(0,1]` ở nhiều action.
- Action name trong source thường không có slash khi tạo server nhưng ROS resolve thành namespace hiện tại; launch mặc định namespace root.
