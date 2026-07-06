# robot_task_manager

## Vai trò
`robot_task_manager` định nghĩa action ROS 2 và triển khai action server C++ cho
motion baseline, gripper, pick-place, DRL/RL task và repeatability evaluation.
Package này là tầng nhận goal từ GUI/client rồi gọi MoveIt, gripper executor,
vision topics hoặc DRL backend `/drl/*`.

## Action definitions
| ROS action name | Type | Executable/server | Ghi chú |
|---|---|---|---|
| `/gohome` | `GoHome` | `gohome_server` | Về named target `home`. |
| `/gohome_2` | `GoHome` | `gohome_server` instance thứ hai | Về named target `home_2`, set bằng parameter `action_name`. |
| `/move_to_pose` | `MoveToPose` | `move_to_pose_server` | PTP MoveIt; goal có `enable_tcp_log`. |
| `/move_to_pose_cartesian` | `MoveToPoseCartesian` | `move_pose_cartesian_server` | Cartesian MoveIt; goal có `enable_tcp_log`. |
| `/move_checker_board` | `CheckerBoard` | `checker_board_server` | Pattern đo/checker-board; goal có `enable_tcp_log`. |
| `/move_gripper` | `MoveGripper` | `move_gripper_server` | Điều khiển gripper group. |
| `/pickplace` | `PickPlace` | `pickplace_server` | Composite baseline: move/pre-pick, Cartesian descend/lift, gripper, place. |
| `/drl_pickplace` | `DrlPickPlace` | `drl_pickplace_server` | PickPlaceRL: dùng DRL plan cho đoạn pick/place và gripper/cartesian sub-actions. |
| `/move_pose_rl` | `MovePoseRl` | `move_pose_rl_server` | MoveToPoseRL trong tài liệu/yêu cầu, endpoint thực là `/move_pose_rl`. |
| `/move_target_rl` | `MoveTargetRl` | `move_target_rl_server` | Chọn target/obstacle từ vision hoặc fallback goal rồi gọi DRL. |
| `/move_to_pose_obstacle` | `MoveToPoseObstacle` | `move_to_pose_obstacle_server` | MoveIt baseline có obstacle box từ vision/fallback. |
| `/repeatability_test` | `RepeatabilityTest` | `repeatability_test_server` | Loop đo repeatability, gọi action con `/move_to_pose*`. |

## DRL và vision
Các server RL dùng `planner_node_name`/`drl_planner_node_name` để set parameter
cho `drl_unified_planner_node`, rồi gọi:
- `/drl/clear_trajectory`
- `/drl/plan`
- `/drl/execute_forward`
- `/drl/get_execution_status`
- `/drl/get_planning_status`

Các action có vision đọc topic:
- `/vision/wood_objects` (`robot_vision_pipeline_msgs/WoodArray`)
- `/vision/box_objects` (`robot_vision_pipeline_msgs/BoxArray`)

Nếu vision không chạy, `MoveTargetRl` và `MoveToPoseObstacle` có fallback fields
trong goal để test mock/Gazebo.

## Logging CSV
Code hiện có ba hướng log chính:
- `StandardActionLogger`: tạo `summary.csv`, `events.csv`, `metadata.json` theo cấu trúc
  `Log_robot_data/<mock|real>/<evaluation_group>/<ActionDisplay>/run_.../call_.../`.
- `PerCallTcpLogger`: opt-in qua `enable_tcp_log` cho `MoveToPose`,
  `MoveToPoseCartesian`, `CheckerBoard`, `RepeatabilityTest` để ghi TCP/joint/error
  tracking theo từng call.
- Logger riêng/metrics cho `PickPlace`, `DrlPickPlace`, `MovePoseRl`,
  `MoveTargetRl`, `MoveToPoseObstacle` khi goal bật `enable_tcp_log` hoặc
  `enable_metrics_log` tùy action.

Mặc định `log_root_dir` trong C++ được dẫn xuất từ `$HOME/ros2_dev/Log_robot_data`;
launch trong workspace này vẫn truyền `/home/minhquang/ros2_dev/Log_robot_data`.

## Chạy nhanh
```bash
cd ~/ros2_dev
colcon build --packages-select robot_task_manager
source install/setup.bash

ros2 launch robot_task_manager task_servers.launch.py
ros2 launch robot_task_manager task_servers_sim.launch.py
```

## Giới hạn cần chú ý
- MoveIt/controller phải sẵn sàng trước khi execute.
- `execute=false` là plan-only ở các action hỗ trợ trường này.
- DRL actions cần `drl_unified_planner_node`, venv/model DRL và services `/drl/*`.
- Real robot path có code/launch hỗ trợ, nhưng mức kiểm chứng thực tế cần xác nhận với phần cứng.
