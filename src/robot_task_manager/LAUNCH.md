# robot_task_manager - Launch Guide

## Launch file
| Launch file | Mục đích | Node/action server |
|---|---|---|
| `task_servers.launch.py` | Chạy task servers cho mock/real MoveIt stack | `drl_unified_planner_node` optional, `gohome_server` x2, `move_to_pose_server`, `move_pose_cartesian_server`, `checker_board_server`, `move_gripper_server`, `pickplace_server`, `drl_pickplace_server`, `move_pose_rl_server`, `move_target_rl_server`, `move_to_pose_obstacle_server`, `repeatability_test_server`. |
| `task_servers_sim.launch.py` | Chạy task servers cho Gazebo/use_sim_time | Tương tự nhưng parameter `use_sim_time=true`, timeout DRL dài hơn ở một số server. |
| `repeatability_test_client.launch.py` | Gửi goal `/repeatability_test` | Python client `repeatability_test_client.py`. |
| `drl_pick_place_random_test.launch.py` | Test random DRL pick-place | Include launch mock/Gazebo tùy `use_gazebo`, chạy random test client. |

## Argument quan trọng
| Argument | Default | Áp dụng |
|---|---:|---|
| `enable_drl_backend` | `true` | Bật `robot_drl/drl_unified_planner_node`; set `false` nếu backend đã chạy riêng. |
| `planner_node_name` | `/drl_unified_planner_node` | DRL servers dùng để gọi service set parameters. |
| `drl_calibrated_start_tcp_base` | `[0.375, 0.000, 0.250]` | Start TCP default cho planner khi launch kèm backend. |
| `runtime_mode` | `mock` | Chọn nhánh log `mock` hoặc `real` trong `task_servers.launch.py`. |
| `enable_standard_logging` | `true` | Bật summary/events/metadata logger cho các action có hỗ trợ. |
| `enable_debug_logging` | `false` | Bật debug log cho helper/child actions nếu server dùng. |
| `enable_executor_logging` | `false` | Bật executor/internal logging ở các server có forward tham số. |
| `log_root_dir` | `/home/minhquang/ros2_dev/Log_robot_data` | Root log được truyền vào server. |

## Ví dụ
```bash
source ~/ros2_dev/install/setup.bash

ros2 launch robot_task_manager task_servers.launch.py
ros2 launch robot_task_manager task_servers.launch.py runtime_mode:=real
ros2 launch robot_task_manager task_servers.launch.py enable_drl_backend:=false
ros2 launch robot_task_manager task_servers_sim.launch.py
ros2 launch robot_task_manager repeatability_test_client.launch.py
```

## Điều kiện runtime
- Cần `move_group` và controller/hardware hoặc Gazebo đã chạy.
- `task_servers.launch.py` build MoveIt config với `use_mock_hardware=true`.
- `task_servers_sim.launch.py` build MoveIt config với `use_sim=true`, `use_mock_hardware=false`.
- Các action RL cần services `/drl/*` và topic `/drl/forward_trajectory_poses`.
- Các action vision cần `/vision/wood_objects` hoặc `/vision/box_objects`, trừ khi goal dùng fallback.
