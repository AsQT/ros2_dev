# Workspace Data Flow

## 1. Mục tiêu luồng dữ liệu
Mô tả đường đi dữ liệu từ mô tả robot/camera/operator tới planner, action/service executor, MoveIt, controller và hardware/simulation.

## 2. Input
- Operator input từ `robot_gui` hoặc client script.
- Camera/depth từ `robot_vision_pipeline` hoặc mock target từ `robot_drl`.
- Robot model từ `robot_description` và MoveIt config từ `robot_moveit`.
- YAML/launch arguments từ bringup/control/executor packages.

## 3. Output
- Trajectory tới ros2_control/Gazebo/hardware.
- Action/service result cho GUI/client.
- Vision object pose/marker/debug image.
- Joint state, hardware status và flag.

## 4. Internal processing
- `robot_bringup` include các subsystem theo mode.
- `robot_moveit` cung cấp `move_group`.
- `robot_task_manager`, `robot_task_executor`, `robot_drl_executor` chuyển goal/request thành plan/execute MoveIt.
- `robot_drl` tạo waypoint/trajectory tránh vật cản.
- `robot_vision_pipeline` chuyển ảnh thành pose object.
- `robot_hardware_interface` chuyển command ROS sang protocol robot.

## 5. Sơ đồ luồng dữ liệu
```mermaid
flowchart LR
  Camera[Camera/Static Image] --> Vision[robot_vision_pipeline]
  Vision --> Objects[Box/Wood/Object Pose]
  Objects --> DRL[robot_drl planner]
  GUI[robot_gui] --> TaskActions[robot_task_manager actions]
  Client[Scripts/CLI] --> TaskActions
  DRL --> DrlExec[robot_drl_executor service]
  TaskActions --> MoveIt[robot_moveit move_group]
  DrlExec --> MoveIt
  Executor[robot_task_executor services] --> MoveIt
  Desc[robot_description] --> MoveIt
  Control[robot_control controllers] --> HW[robot_hardware_interface/Gazebo]
  MoveIt --> Control
  HW --> JointStates[/joint_states + flags]
  JointStates --> GUI
```

## 6. Liên kết package
- Bringup là điểm khởi động mode sim/real/demo.
- Description/control/moveit tạo nền robot model và motion planning.
- GUI/task/DRL/vision là tầng ứng dụng.
- Hardware interface là đầu cuối real robot; Gazebo là đầu cuối simulation.

## 7. Điểm cần chú ý
- ROS/MoveIt dùng mét và radian; GUI nhập pose theo mm tại nhiều ô rồi đổi `mm / 1000.0` trước khi gửi action.
- `execute=false` là plan-only ở action/service task.
- Frame quan trọng: `base_link`, `tcp_link`, `world`, `aruco_world`.
- Cần kiểm tra runtime bằng ROS CLI vì launch condition có thể tắt/bật node theo argument.
