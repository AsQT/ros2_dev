# robot_task_manager - Parameters

## Launch-level
| Parameter | Default | Nơi khai báo | Ý nghĩa |
|---|---:|---|---|
| `enable_drl_backend` | `true` | `task_servers*.launch.py` | Launch `drl_unified_planner_node` kèm task servers. |
| `planner_node_name` | `/drl_unified_planner_node` | `task_servers*.launch.py` | Tên node DRL planner cho service set parameters. |
| `drl_calibrated_start_tcp_base` | `[0.375, 0.000, 0.250]` | `task_servers*.launch.py` | Start TCP default truyền vào DRL backend. |
| `runtime_mode` | `mock` | `task_servers.launch.py` | Nhánh log `mock` hoặc `real`; sim launch hardcode `mock`. |
| `enable_standard_logging` | `true` | `task_servers.launch.py` | Bật `StandardActionLogger`. |
| `enable_debug_logging` | `false` | `task_servers.launch.py` | Bật log debug ở các server có hỗ trợ. |
| `enable_executor_logging` | `false` | `task_servers.launch.py` | Bật executor/per-call logger ở các server có hỗ trợ. |
| `log_root_dir` | `/home/minhquang/ros2_dev/Log_robot_data` | launch | Root log. |
| `executor_log_dir` | `.../executor_internal` | launch | Được forward nhưng `log_paths.hpp` chuẩn hóa theo action log dir ở nhiều server. |
| `executor_sample_rate_hz` | `50.0` | launch/source | Tần số sample TCP/joint log. |
| `executor_base_frame` | `base_link` | launch/source | Base frame cho logger. |
| `executor_tcp_frame` | `tcp_link` | launch/source | TCP frame cho logger. |

## Server parameters đáng chú ý
| Parameter | Default | Server | Ý nghĩa |
|---|---:|---|---|
| `planning_group` | `arm` | MoveIt action servers | Planning group. |
| `home_target` | `home` / `home_2` | `gohome_server` | Named target SRDF. |
| `action_name` | `gohome` / `gohome_2` | `gohome_server` | Endpoint action cho GoHome. |
| `planning_frame` | `base_link` | RL/composite servers | Frame planning cho DRL actions. |
| `ee_link` | `tcp_link` | RL/composite servers | TCP/end-effector link. |
| `target_class` | `wood` | `move_target_rl_server` | Class target vision mặc định. |
| `obstacle_class` | `box` | `move_target_rl_server`, `move_to_pose_obstacle_server`, RL servers | Class obstacle mặc định. |
| `wood_objects_topic` | `/vision/wood_objects` | `move_target_rl_server` | Topic WoodArray. |
| `box_objects_topic` | `/vision/box_objects` | RL/obstacle servers | Topic BoxArray. |
| `vision_timeout_sec` | `1.0` | vision-dependent servers | Thời gian chờ detection. |
| `drl_timeout_sec` | `120.0` mock, `300.0` sim | RL servers | Timeout DRL plan/execute. |
| `drl_plan_attempts` | `3` | RL servers | Số lần thử plan. |
| `sub_action_timeout_sec` | `60.0` hoặc `180.0` | composite/RL servers | Timeout action con. |
| `use_preposition_before_pre_pick` | `false` | `drl_pickplace_server` | Không ép về preposition cố định trước PLAN_TO_PRE_PICK theo launch hiện tại. |

## Goal flags
- `execute=false`: plan-only khi action hỗ trợ.
- `enable_tcp_log`: có trong `MoveToPose`, `MoveToPoseCartesian`, `CheckerBoard`, `PickPlace`, `RepeatabilityTest`.
- `enable_metrics_log`: có trong `MovePoseRl`, `MoveTargetRl`, `MoveToPoseObstacle`, `DrlPickPlace`.

## Cần xác nhận
- Các giá trị timeout/tolerance tối ưu cho robot thật cần xác nhận bằng chạy thực tế.
- `hardware_plugin` default là `unknown` nếu launch không truyền thêm.
