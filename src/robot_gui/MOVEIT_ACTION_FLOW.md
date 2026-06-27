# robot_gui - MoveIt / Action Execution Flow

## 1. Tổng quan
GUI không plan bằng MoveIt trực tiếp trong task controller; nó gửi action tới `robot_task_manager`. Button Plan gửi `execute=false`, Start gửi `execute=true`. MoveIt chạy ở server phía task manager/move_group.

## 2. Danh sách action/service liên quan MoveIt
| Action/Service | Type | Server/Client | File source | Vai trò |
|---|---|---|---|---|
| /gohome, /move_to_pose, /move_to_pose_cartesian, /move_gripper, /pickplace, /move_checker_board, /move_pose_rl, /repeatability_test | robot_task_manager actions | Client | src/task_action_controller.cpp | Plan/Start từ GUI |

## 3. Luồng execute chung
Client tạo goal/request với `execute=true` -> server validate -> MoveIt plan/Cartesian path -> controller execute -> result success/fail.

## 4. Luồng plan-only
Client tạo goal/request với `execute=false` -> server chỉ plan/tính path -> không gửi trajectory tới controller -> trả result/fraction nếu có.

## 5. Chi tiết từng action/service
GUI không plan bằng MoveIt trực tiếp trong task controller; nó gửi action tới `robot_task_manager`. Button Plan gửi `execute=false`, Start gửi `execute=true`. MoveIt chạy ở server phía task manager/move_group.

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
