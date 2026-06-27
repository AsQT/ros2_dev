# Documentation Update Report

## 1. Danh sách package đã rà soát
| Package | README | DATA_FLOW | PARAMETERS | LAUNCH | MOVEIT_ACTION_FLOW | Ghi chú |
|---|---|---|---|---|---|---|
| robot_bringup | yes | yes | yes | yes | yes | Source scan + docs update |
| robot_control | yes | yes | yes | yes | not required | Source scan + docs update |
| robot_description | yes | yes | yes | yes | not required | Source scan + docs update |
| robot_drl | yes | yes | yes | yes | yes | Source scan + docs update |
| robot_drl_executor | yes | yes | yes | yes | yes | Source scan + docs update |
| robot_gui | yes | yes | yes | yes | yes | Source scan + docs update |
| robot_hardware_interface | yes | yes | yes | yes | not required | Source scan + docs update |
| robot_moveit | yes | yes | yes | yes | yes | Source scan + docs update |
| robot_task_executor | yes | yes | yes | yes | yes | Source scan + docs update |
| robot_task_executor_msgs | yes | yes | yes | yes | yes | Source scan + docs update |
| robot_task_manager | yes | yes | yes | yes | yes | Source scan + docs update |
| robot_vision_pipeline | yes | yes | yes | yes | not required | Source scan + docs update |
| robot_vision_pipeline_msgs | yes | yes | yes | yes | not required | Source scan + docs update |

## 2. File đã tạo mới
- `src/README.md`
- `src/DATA_FLOW.md`
- `src/DOCUMENTATION_UPDATE_REPORT.md`
- `DATA_FLOW.md`, `PARAMETERS.md`, `LAUNCH.md` cho từng package có `package.xml`.
- `MOVEIT_ACTION_FLOW.md` cho package liên quan MoveIt/action/service: robot_bringup, robot_drl, robot_drl_executor, robot_gui, robot_moveit, robot_task_executor, robot_task_executor_msgs, robot_task_manager.
- `robot_gui/GUI_FLOW.md`.

## 3. File đã cập nhật
- `README.md` trong toàn bộ 13 package được chuẩn hóa theo template yêu cầu.
- Các tài liệu package hiện mô tả node/executable, launch, parameter, interface và data flow dựa trên source hiện tại.

## 4. Các action/service/topic quan trọng đã được tài liệu hóa
- `robot_task_manager`: `/gohome`, `/move_to_pose`, `/move_to_pose_cartesian`, `/move_checker_board`, `/move_gripper`, `/pickplace`, `/drl_pickplace`, `/move_pose_rl`, `/repeatability_test`.
- DRL: `/drl/plan`, `/drl/execute_forward`, `/drl/clear_trajectory`, `/drl/get_execution_status`, `/drl/forward_poses`.
- Executor: `/move_cartesian_pose_sequence` và service types trong `robot_task_executor_msgs`.
- Hardware: `/robot_hw/*`, `/joint_states`, `/robot_hw/flags`, `/robot_hw/status_text`.
- Vision: camera image/depth topics, YOLO JSON, `WoodArray`, `BoxArray`, debug image và marker topics.

## 5. Các launch file đã được tài liệu hóa
Đã tài liệu hóa launch trong `robot_bringup`, `robot_control`, `robot_description`, `robot_drl`, `robot_drl_executor`, `robot_gui`, `robot_hardware_interface`, `robot_moveit`, `robot_task_executor`, `robot_task_manager`, `robot_vision_pipeline`. Hai package interface-only không có launch.

## 6. Các parameter mặc định đã được tài liệu hóa
- MoveIt/executor: `move_group_name=arm`, `base_frame=base_link`, `ee_link=tcp_link`, `planning_time=2.0`, `num_planning_attempts=5`, velocity scale `0.1`, acceleration scale `0.5`, Cartesian threshold `0.95` khi có.
- GUI: `embed_rviz=false`, `initial_page=-1`, default velocity scale `0.1`, conversion mm -> m.
- Task manager: DRL backend args, repeatability timeout/velocity, client execute flags.
- DRL/vision/hardware/controller: launch args và YAML/config quan trọng theo source scan.

## 7. Các điểm chưa xác định được từ source
- Một số YAML chỉ có hiệu lực khi launch truyền vào đúng node; cần xác nhận runtime bằng `ros2 param list`.
- Hardware thật cần xác nhận IP/port/protocol/calibration trước khi vận hành.
- Vision topic/model path có thể đổi theo camera thật hoặc launch override.

## 8. Kiểm tra cuối
- Chỉ thay đổi markdown: yes, nhiệm vụ chỉ ghi/cập nhật `.md`.
- Có package nào thiếu tài liệu: no theo checklist file tồn tại.
- Có file nào cần người dùng xác nhận thêm: yes, các giá trị hardware/calibration/model path runtime.
