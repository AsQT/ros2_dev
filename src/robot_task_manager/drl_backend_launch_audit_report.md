# DRL Backend Launch Audit Report

Ngày thực hiện: 2026-06-26

## Mục tiêu

`/move_pose_rl` và `/drl_pickplace` trong `robot_task_manager` phụ thuộc backend DRL của package `robot_drl`. Trước cập nhật, `task_servers.launch.py` và `task_servers_sim.launch.py` chỉ chạy action server, không chạy node backend nên action RL có thể fail với lỗi như:

```text
Planner parameter service not available: /drl_unified_planner_node/set_parameters
```

## Kết quả audit

Backend đúng là:

```text
package: robot_drl
executable: drl_unified_planner_node
node name: /drl_unified_planner_node
```

Node này được khai báo trong `robot_drl/setup.py` và tự tạo các service/topic mà action RL cần:

```text
/drl_unified_planner_node/set_parameters
/drl/plan
/drl/clear_trajectory
/drl/execute_forward
/drl/get_execution_status
/drl/forward_trajectory_poses
```

Phụ thuộc theo action:

| Action | DRL backend cần có | Phụ thuộc khác |
|---|---|---|
| `/move_pose_rl` | `/drl_unified_planner_node/set_parameters`, `/drl/clear_trajectory`, `/drl/plan`, topic `/drl/forward_trajectory_poses`; nếu `execute=true` thêm `/drl/execute_forward`, `/drl/get_execution_status` | `/joint_states`, TF `base_link <- tcp_link` |
| `/drl_pickplace` | `/drl_unified_planner_node/set_parameters`, `/drl/clear_trajectory`, `/drl/plan`, `/drl/execute_forward`, `/drl/get_execution_status`, topic `/drl/forward_trajectory_poses` | `/move_gripper`, `/move_to_pose_cartesian`, TF nếu pose khác frame |

## Thay đổi đã thực hiện

- `robot_task_manager/launch/task_servers.launch.py`
  - Thêm launch argument `enable_drl_backend`, mặc định `true`.
  - Thêm launch argument `planner_node_name`, mặc định `/drl_unified_planner_node`.
  - Thêm launch argument `drl_calibrated_start_tcp_base`, mặc định `[0.375, 0.000, 0.250]`.
  - Chạy `robot_drl/drl_unified_planner_node` theo điều kiện `enable_drl_backend`.
  - Truyền `planner_node_name` vào cả `drl_pickplace_server` và `move_pose_rl_server`.

- `robot_task_manager/launch/task_servers_sim.launch.py`
  - Cập nhật tương tự launch mock/real.
  - Backend dùng `use_sim_time: true`.

- `robot_task_manager/package.xml`
  - Thêm `exec_depend` tới `robot_drl`.

- `robot_task_manager/README.md` và `robot_task_manager/Call_action.md`
  - Ghi rõ hai launch mặc định chạy DRL backend.
  - Ghi cách tắt backend nội bộ:

```bash
ros2 launch robot_task_manager task_servers.launch.py enable_drl_backend:=false
ros2 launch robot_task_manager task_servers_sim.launch.py enable_drl_backend:=false
```

## Cấu hình backend mặc định

Backend được launch ở chế độ không tự prompt/không tự plan để action server điều khiển hoàn toàn:

```yaml
input_mode: manual
auto_plan_on_start: false
manual_prompt_on_start: false
auto_execute_after_plan: false
preposition_before_plan: false
update_start_tcp_from_tf_before_plan: true
use_planning_scene_obstacles: true
validate_path_with_moveit_ik: default của robot_drl
```

## Kiểm thử

Build:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/minhquang/ros2_dev
colcon build --packages-select robot_task_manager robot_drl robot_gui
```

Kết quả cuối:

```text
Summary: 3 packages finished
```

Kiểm tra launch mock/real với `ROS_DOMAIN_ID=37`:

```bash
ros2 launch robot_task_manager task_servers.launch.py
ros2 action list
ros2 service list
```

Action có mặt:

```text
/drl_pickplace
/move_pose_rl
```

Backend service có mặt:

```text
/drl_unified_planner_node/set_parameters
/drl/plan
/drl/clear_trajectory
/drl/execute_forward
/drl/get_execution_status
```

Kiểm tra `/move_pose_rl execute=false`:

```text
Goal accepted.
Result: ABORTED tại get_current_pose vì chưa có /joint_states.
Message: Failed to receive /joint_states before DRL planning. Refusing to plan from zero/default state.
```

Điểm quan trọng: action không còn fail vì thiếu `/drl_unified_planner_node/set_parameters` hoặc thiếu `/drl/plan`.

Kiểm tra `/drl_pickplace execute=false`:

```text
Goal accepted.
Feedback đi qua WAIT_FOR_SERVERS.
Sau đó timeout ở OPEN_GRIPPER do launch độc lập không có robot state đầy đủ cho chuỗi motion con.
```

Điểm quan trọng: action không còn fail tại bước chờ DRL backend service.

Kiểm tra launch simulation với `ROS_DOMAIN_ID=38`:

```text
/drl_pickplace
/move_pose_rl
/drl_unified_planner_node/set_parameters
/drl/plan
/drl/clear_trajectory
/drl/execute_forward
/drl/get_execution_status
```

Kiểm tra disable backend với `ROS_DOMAIN_ID=39`:

```bash
ros2 launch robot_task_manager task_servers.launch.py enable_drl_backend:=false
```

Kết quả:

```text
Không có node /drl_unified_planner_node.
ros2 service info /drl/plan báo Services count: 0, Clients count: 2.
```

Lưu ý: `ros2 service list` vẫn có thể liệt kê tên `/drl/plan` khi chỉ có client tồn tại. Dùng `ros2 service info /drl/plan` để phân biệt service server thật.

## Ghi chú còn lại

Khi dừng launch bằng Ctrl-C, một số action server MoveIt hiện có có thể in `RCLError` trong destructor do context ROS đã invalid. Hiện tượng này xảy ra lúc shutdown và không ảnh hưởng tới mục tiêu cập nhật DRL backend launch.
