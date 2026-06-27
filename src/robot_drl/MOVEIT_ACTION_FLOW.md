# robot_drl - MoveIt / Action Execution Flow

## 1. Tổng quan
DRL không dùng MoveGroupInterface trực tiếp trong file chính như C++, nhưng phụ thuộc MoveIt qua `/compute_ik`, planning scene và service Cartesian executor `/move_cartesian_pose_sequence`. Plan-only là gọi `/drl/plan` nhưng không execute; execute gọi `/drl/execute_forward`.

## 2. Danh sách action/service liên quan MoveIt
| Action/Service | Type | Server/Client | File source | Vai trò |
|---|---|---|---|---|
| /drl/plan, /drl/execute_forward, /drl/get_execution_status | std_srvs/Trigger + PoseArray | Server/client | drl_planner_node_base.py, drl_unified_planner_node.py | DRL plan rồi gọi Cartesian executor MoveIt |

## 3. Luồng execute chung
Client tạo goal/request với `execute=true` -> server validate -> MoveIt plan/Cartesian path -> controller execute -> result success/fail.

## 4. Luồng plan-only
Client tạo goal/request với `execute=false` -> server chỉ plan/tính path -> không gửi trajectory tới controller -> trả result/fraction nếu có.

## 5. Chi tiết từng action/service
DRL không dùng MoveGroupInterface trực tiếp trong file chính như C++, nhưng phụ thuộc MoveIt qua `/compute_ik`, planning scene và service Cartesian executor `/move_cartesian_pose_sequence`. Plan-only là gọi `/drl/plan` nhưng không execute; execute gọi `/drl/execute_forward`.

## 6. Feedback/result
- Action trong `robot_task_manager` trả stage/progress trong feedback và `success/message` trong result.
- Service executor trả `success/message`; Cartesian service trả thêm `fraction`.

## 7. Điều kiện thành công/thất bại
- Thành công: server/action/service có sẵn, frame hợp lệ, MoveIt plan thành công, controller execute thành công nếu `execute=true`.
- Thất bại: `move_group` chưa chạy, controller inactive, frame khác `base_link` khi executor không hỗ trợ transform, fraction Cartesian thấp, timeout hoặc DRL backend chưa sẵn sàng.

## 8. Phụ thuộc runtime
- `robot_description`, `robot_moveit/move_group`, ros2_control controller, và các action/service con tương ứng.
- Với DRL: cần `drl_unified_planner_node` và `/move_cartesian_pose_sequence`.

## 9. Sơ đồ sequence
```mermaid
sequenceDiagram
  participant C as Client/GUI
  participant S as Action or Service Server
  participant M as MoveIt move_group
  participant R as Controller/Robot
  C->>S: goal/request execute flag
  S->>S: validate parameters and frames
  S->>M: plan or computeCartesianPath
  alt execute=true
    M->>R: execute trajectory
    R-->>S: execution result
  else execute=false
    M-->>S: plan only result
  end
  S-->>C: feedback/result or service response
```
