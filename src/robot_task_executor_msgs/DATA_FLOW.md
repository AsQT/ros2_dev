# robot_task_executor_msgs - Data Flow

## 1. Mục tiêu luồng dữ liệu
Chuẩn hóa request/response giữa client và MoveIt service executor.

## 2. Input
Service request do client tạo: named target, joint array, pose, Cartesian point, hoặc PoseStamped sequence.

## 3. Output
Service response với `success`, `message`, và `fraction` cho Cartesian service.

## 4. Internal processing
Không xử lý runtime; ROSIDL generate C++/Python type từ `.srv`.

## 5. Sơ đồ luồng dữ liệu
```mermaid
flowchart LR
  Client[Client package] --> Type[robot_task_executor_msgs srv]
  Type --> Executor[robot_task_executor or robot_drl_executor]
  Executor --> MoveIt[MoveIt]
  MoveIt --> Response[success/message/fraction]
```

## 6. Liên kết với package khác
`robot_task_executor`, `robot_drl_executor`, client script hoặc node khác include service type này.

## 7. Các điểm cần chú ý
Pose thường kỳ vọng frame `base_link` ở executor hiện tại; frame rỗng trong `MoveCartesianPoseSequence` được executor DRL thay bằng `base_link`.
