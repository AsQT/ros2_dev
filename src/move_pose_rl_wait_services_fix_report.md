# MovePoseRL Wait Services Fix Report

Ngày thực hiện: 2026-06-26

## 1. Nguyên nhân cụ thể

`/move_pose_rl` bị kẹt hoặc fail tại stage `wait_for_drl_services` vì `task_servers.launch.py` không luôn launch backend DRL theo mặc định. Khi thiếu backend, `move_pose_rl_server` chờ service:

```text
/drl_unified_planner_node/set_parameters
/drl/plan
/drl/clear_trajectory
```

và với `execute=true` chờ thêm:

```text
/drl/execute_forward
/drl/get_execution_status
```

Trước sửa, log server chỉ báo stage chung `wait_for_drl_services`, chưa chỉ rõ service nào OK/MISSING.

## 2. Service thiếu hoặc sai tên

Tên service trong `move_pose_rl_server` là đúng với backend thực tế. Service bị thiếu khi backend `robot_drl/drl_unified_planner_node` không được launch.

Service parameter đúng là:

```text
/drl_unified_planner_node/set_parameters
```

## 3. Node/executable tạo service

Đã kiểm tra trong `robot_drl/setup.py`:

```text
package: robot_drl
executable: drl_unified_planner_node
entry point: robot_drl.drl_unified_planner_node:main
node name: /drl_unified_planner_node
```

Node này tạo:

```text
/drl/plan
/drl/replan
/drl/clear_trajectory
/drl/execute_forward
/drl/execute_backward
/drl/execute_trajectory
/drl/get_execution_status
/drl/forward_trajectory_poses
/drl_unified_planner_node/set_parameters
```

## 4. File launch đã sửa

```text
robot_task_manager/launch/task_servers.launch.py
robot_task_manager/launch/task_servers_sim.launch.py
```

Thay đổi chính:

- `enable_drl_backend` default là `true`.
- Có thể tắt khi backend chạy ngoài:

```bash
ros2 launch robot_task_manager task_servers.launch.py enable_drl_backend:=false
```

- Launch backend đúng node/executable:

```text
robot_drl/drl_unified_planner_node
name=drl_unified_planner_node
```

- Trong launch task servers, backend DRL được cấu hình không phụ thuộc service ngoài launch:

```yaml
use_planning_scene_obstacles: false
validate_path_with_moveit_ik: false
workspace_max_base: [0.500, 0.150, 0.450]
```

## 5. File server đã sửa

```text
robot_task_manager/src/move_pose_rl_server.cpp
```

Đã thêm log kiểm từng service:

```text
[MovePoseRl] checking service: /drl_unified_planner_node/set_parameters ... OK
[MovePoseRl] checking service: /drl/plan ... OK
[MovePoseRl] checking service: /drl/clear_trajectory ... OK
```

Nếu thiếu service, result fail rõ:

```text
MovePoseRl failed at wait_for_drl_services: missing /drl/plan
```

## 6. Service `/move_pose_rl` cần

`execute=false`:

```text
/drl_unified_planner_node/set_parameters
/drl/plan
/drl/clear_trajectory
/drl/forward_trajectory_poses
```

`execute=true`:

```text
/drl_unified_planner_node/set_parameters
/drl/plan
/drl/clear_trajectory
/drl/execute_forward
/drl/get_execution_status
/drl/forward_trajectory_poses
```

Ngoài DRL service, action cần `/joint_states` và TF `base_link <- tcp_link` để lấy current TCP pose.

## 7. Build

Lệnh:

```bash
cd /home/minhquang/ros2_dev
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_task_manager robot_drl robot_gui
```

Kết quả:

```text
Summary: 3 packages finished
```

## 8. Kết quả `ros2 node list`

Test với `ROS_DOMAIN_ID=44`:

```text
/drl_pickplace_action_server
/drl_unified_planner_node
/move_pose_rl_action_server
/move_to_pose_server
/move_pose_cartesian_server
/move_gripper_server
/pickplace_action_server
/repeatability_test_action_server
/gohome_server
```

## 9. Kết quả `ros2 service list`

Các service DRL/backend thấy được:

```text
/drl/clear_trajectory
/drl/execute_backward
/drl/execute_forward
/drl/execute_trajectory
/drl/get_execution_status
/drl/plan
/drl/replan
/drl_unified_planner_node/set_parameters
/drl_unified_planner_node/set_parameters_atomically
```

## 10. Test `/move_pose_rl execute=false`

Do `task_servers.launch.py` không tự publish robot state, test CLI đã cấp TF và `/joint_states` tối thiểu:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.375 --y 0.0 --z 0.25 \
  --qx 0.0 --qy 0.0 --qz 0.0 --qw 1.0 \
  --frame-id base_link --child-frame-id tcp_link

ros2 topic pub /joint_states sensor_msgs/msg/JointState \
  "{name: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'], position: [0.0,0.0,0.0,0.0,0.0,0.0]}" --rate 10
```

Action:

```bash
ros2 action send_goal /move_pose_rl robot_task_manager/action/MovePoseRl \
  "{target_pose: {position: {x: 0.40, y: 0.10, z: 0.35}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}, velocity_scale: 0.5, execute: false}" \
  --feedback
```

Kết quả:

```text
validate_goal -> get_current_pose -> wait_for_drl_services -> drl_plan -> done_plan_only
success: true
message: DRL plan succeeded; execution skipped because execute=false
```

## 11. Test `/move_pose_rl execute=true`

Action:

```bash
ros2 action send_goal /move_pose_rl robot_task_manager/action/MovePoseRl \
  "{target_pose: {position: {x: 0.40, y: 0.10, z: 0.35}, orientation: {x: 1.0, y: 1.0, z: 0.0, w: 0.0}}, velocity_scale: 0.5, execute: true}" \
  --feedback
```

Kết quả:

```text
validate_goal -> get_current_pose -> wait_for_drl_services -> drl_plan -> execute_forward -> execution_status
failed_stage: execution_status
message: DRL execution failed: FAILED_FORWARD: Service /move_cartesian_pose_sequence not available after 5.0 s.
```

Điểm quan trọng: `execute=true` đã qua service check, DRL plan thành công và gọi `/drl/execute_forward`. Lỗi còn lại thuộc executor chuyển động thấp hơn, không phải `wait_for_drl_services`.

## 12. Test GUI

Không mở GUI trong phiên CLI này. Đường action/backend mà GUI dùng đã được kiểm trực tiếp bằng `/move_pose_rl`:

- `execute=false` thành công.
- `execute=true` qua service check và fail rõ ở execution backend thấp hơn.
- Nếu tắt DRL backend, server hiện trả lỗi thiếu service cụ thể dạng `missing /drl/plan`, không còn log chung chung.

## 13. Xác nhận lỗi đã hết

Lỗi sau đã được xử lý khi chạy launch mặc định:

```text
Planner parameter service not available: /drl_unified_planner_node/set_parameters
```

`task_servers.launch.py` hiện launch `/drl_unified_planner_node` mặc định và service `/drl_unified_planner_node/set_parameters` xuất hiện.

## 14. Không ảnh hưởng action khác

`ros2 action list` sau launch:

```text
/drl_pickplace
/gohome
/move_action
/move_checker_board
/move_gripper
/move_pose_rl
/move_to_pose
/move_to_pose_cartesian
/pickplace
/repeatability_test
```

Các action yêu cầu vẫn có mặt:

```text
/move_to_pose
/move_to_pose_cartesian
/move_checker_board
/pickplace
/drl_pickplace
/repeatability_test
/gohome
```
