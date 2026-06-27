# DRL Action Fix Report

Date: 2026-06-27
Workspace: `/home/minhquang/ros2_dev`

## 1. Action DRL nào bị lỗi

- `/move_pose_rl` (`robot_task_manager/action/MovePoseRl`)
- `/drl_pickplace` (`robot_task_manager/action/DrlPickPlace`)

Hai action này phụ thuộc backend `robot_drl/drl_unified_planner_node` và các service `/drl/*`.

## 2. Lỗi gốc là gì

Các launch file DRL chạy `drl_unified_planner_node` bằng Python mặc định, không pin interpreter có đủ thư viện RL. Node này load model qua `robot_drl/model_loader.py`, nơi dùng `stable_baselines3`, `gymnasium`, `numpy`, và gián tiếp dùng `torch`.

Ngoài ra có lỗi import message trong flow vision/mock:

```text
from robot_vision_pipeline.msg import Box, BoxDetection -> ImportError
from robot_vision_pipeline_msgs.msg import Box, BoxDetection -> OK
```

## 3. Có phải do thiếu Python venv không

Đúng về mặt cấu hình launch: các node cần RL libs chưa chạy bằng venv bắt buộc.

Trên máy này, `python3` mặc định hiện cũng import được `torch/stable_baselines3/gymnasium/numpy`, nên không tái hiện `ModuleNotFoundError` trong shell hiện tại. Tuy vậy launch vẫn sai vì không bảo đảm chạy bằng:

```bash
/home/minhquang/venvs/ros_rl/bin/python3
```

Venv đã kiểm tra:

```text
Python 3.12.3
torch ok
stable_baselines3 ok
gymnasium ok
numpy ok
rclpy ok
action_msgs ok
geometry_msgs ok
```

## 4. Node nào đã được thêm prefix /home/minhquang/venvs/ros_rl/bin/python3

Đã thêm `prefix=RL_PYTHON` cho mọi `launch_ros.actions.Node` chạy:

```text
package="robot_drl"
executable="drl_unified_planner_node"
```

Không thêm prefix cho:

- `robot_drl_executor_node` vì là C++.
- Các action server C++ trong `robot_task_manager`.
- `mock_environment_node` vì không load model RL.

## 5. File launch nào đã sửa

- `robot_task_manager/launch/task_servers.launch.py`
- `robot_task_manager/launch/task_servers_sim.launch.py`
- `robot_drl/launch/drl_mock_hw.launch.py`
- `robot_drl/launch/drl_gazebo.launch.py`
- `robot_drl/launch/rl_sim_rviz.launch.py`
- `robot_drl/launch/mock_drl.launch.py`
- `robot_drl/launch/mock_drl_rviz.launch.py`
- `robot_drl/launch/main.launch.py`
- `robot_drl/launch/drl_unified_planner.launch.py`
- `robot_bringup/launch/drl_test.launch.py`
- `robot_bringup/launch/rl_pick_place_box_gazebo_demo.launch.py`

## 6. File source nào đã sửa

- `robot_drl/robot_drl/drl_unified_planner_node.py`
  - Đổi lazy import `Box` sang `robot_vision_pipeline_msgs.msg`.
- `robot_drl/robot_drl/mock_environment_node.py`
  - Đổi import `Box`, `BoxDetection` sang `robot_vision_pipeline_msgs.msg`.
- `robot_drl/package.xml`
  - Thêm dependency `robot_vision_pipeline_msgs`.

## 7. Dependency nào đổi từ robot_task_executor sang robot_drl_executor

Trong working tree hiện tại, các flow DRL đã dùng `robot_drl_executor` thay cho node/service runtime từ `robot_task_executor`:

- `robot_drl/package.xml`: runtime exec dependency là `robot_drl_executor`.
- `robot_drl/launch/drl_mock_hw.launch.py`: include `robot_drl_executor/launch/robot_drl_executor.launch.py`.
- `robot_drl/launch/drl_gazebo.launch.py`: include `robot_drl_executor/launch/robot_drl_executor.launch.py`.
- `robot_drl/launch/rl_sim_rviz.launch.py`: include `robot_drl_executor/launch/robot_drl_executor.launch.py`.
- `robot_bringup/launch/drl_test.launch.py`: include `robot_drl_executor/launch/robot_drl_executor.launch.py`.
- `robot_bringup/launch/rl_pick_place_box_gazebo_demo.launch.py`: include `robot_drl_executor/launch/robot_drl_executor.launch.py`.

Giữ nguyên `robot_task_executor_msgs` vì service `/move_cartesian_pose_sequence` vẫn dùng type:

```text
robot_task_executor_msgs/srv/MoveCartesianPoseSequence
```

Không xóa package `robot_task_executor`.

## 8. Lệnh build đã chạy

```bash
cd /home/minhquang/ros2_dev
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robot_drl_executor robot_drl robot_task_manager robot_bringup
```

## 9. Kết quả build

Build thành công:

```text
Summary: 4 packages finished [4.29s]
```

## 10. Lệnh test đã chạy

Mock hardware stack:

```bash
source /opt/ros/jazzy/setup.bash
source /home/minhquang/ros2_dev/install/setup.bash
export ROS_DOMAIN_ID=73
ros2 launch robot_drl drl_mock_hw.launch.py \
  auto_plan_on_start:=false \
  auto_execute_after_plan:=false \
  manual_prompt_on_start:=false \
  input_mode:=manual
```

Task action servers, backend tắt để tránh chạy trùng planner:

```bash
ros2 launch robot_task_manager task_servers.launch.py enable_drl_backend:=false
```

Graph checks:

```bash
ros2 node list
ros2 action list -t
ros2 service list -t
```

Goal test:

```bash
ros2 action send_goal /move_pose_rl robot_task_manager/action/MovePoseRl \
  "{target_pose: {position: {x: 0.42, y: 0.0, z: 0.12}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, velocity_scale: 0.1, execute: false}"
```

## 11. Kết quả ros2 action list -t

Các action DRL xuất hiện:

```text
/drl_pickplace [robot_task_manager/action/DrlPickPlace]
/move_pose_rl [robot_task_manager/action/MovePoseRl]
```

Các action liên quan khác cũng xuất hiện: `/move_action`, `/execute_trajectory`, `/arm_controller/follow_joint_trajectory`, `/gripper_controller/follow_joint_trajectory`.

Service backend xuất hiện:

```text
/drl/clear_trajectory [std_srvs/srv/Trigger]
/drl/execute_forward [std_srvs/srv/Trigger]
/drl/get_execution_status [std_srvs/srv/Trigger]
/drl/plan [std_srvs/srv/Trigger]
/drl_unified_planner_node/set_parameters [rcl_interfaces/srv/SetParameters]
/move_cartesian_pose_sequence [robot_task_executor_msgs/srv/MoveCartesianPoseSequence]
```

## 12. Kết quả gửi goal/action runtime

Goal `/move_pose_rl` được accept và thành công:

```text
Goal accepted with ID: d9912e4dc0e342f0bb93784a9f5a14df
Result:
    success: true
message: DRL plan succeeded; execution skipped because execute=false
failed_stage: ''
Goal finished with status: SUCCEEDED
```

Log action server xác nhận flow:

```text
[MovePoseRl] wait_for_drl_services | 25.0%
[MovePoseRl] checking service: /drl_unified_planner_node/set_parameters ... OK
[MovePoseRl] checking service: /drl/plan ... OK
[MovePoseRl] checking service: /drl/clear_trajectory ... OK
[MovePoseRl] drl_plan | 45.0%
DRL trajectory ready: waypoints=10 final_position_error=0.00000
[MovePoseRl] done_plan_only | 100.0%
```

Log planner xác nhận plan:

```text
[/drl/plan] source=manual+scene | planning_frame=base_link | start_tcp_base=[0.375 0.    0.25 ] | target_base=[0.42 0.   0.12]
Published trajectories | forward=10wp [0.375,0.000,0.250] -> [0.420,0.000,0.120]
[plan_worker] success: Manual plan published. target=(0.4200, 0.0000, 0.1200)
```

## 13. Các lỗi còn lại nếu có

- Sau khi Ctrl-C để dừng launch, `rviz2` và `move_group` có thể exit code `-11`/segfault trong shutdown path. Lỗi này xảy ra sau khi action test đã thành công.
- `ros2 node list` báo warning có node trùng tên. Snapshot vẫn thấy đúng các node chính: `/drl_unified_planner_node`, `/robot_drl_executor_node`, `/move_pose_rl_action_server`, `/drl_pickplace_action_server`, `/mock_environment_node`.
- `move_group`/RViz log warning vật liệu URDF `gripper_l`/`gripper_r` undefined và `/recognize_objects` không có. Các warning này không chặn DRL action plan-only.

## 14. Trạng thái cuối cùng: chạy được/chưa chạy được

Chạy được.

Đã đạt các tiêu chí:

- Build thành công.
- `drl_unified_planner_node` chạy qua `/home/minhquang/venvs/ros_rl/bin/python3`.
- Không còn crash import `Box/BoxDetection` trong `mock_environment_node`.
- `ros2 action list -t` hiển thị `/move_pose_rl` và `/drl_pickplace`.
- Gửi goal tới `/move_pose_rl` thành công, result `SUCCEEDED`.
- Audit report đã tạo trước khi sửa: `robot_drl_executor/drl_action_error_audit_report.md`.
