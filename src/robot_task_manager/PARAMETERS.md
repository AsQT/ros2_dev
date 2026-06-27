# robot_task_manager - Parameters

## 1. Tổng quan
Parameter lấy từ launch và `declare_parameter` trong server/client scripts.

## 2. Bảng parameter
| Parameter | Default | Type | Nơi khai báo | Nơi sử dụng | Ý nghĩa |
|---|---:|---|---|---|---|
| `enable_drl_backend` | `true` | bool | `task_servers*.launch.py` | launch DRL backend | Bật `drl_unified_planner_node` kèm task servers |
| `planner_node_name` | `/drl_unified_planner_node` | string | `task_servers*.launch.py` | DRL servers | Tên node planner để set parameters |
| `drl_calibrated_start_tcp_base` | `[0.375,...]` | array | `task_servers*.launch.py` | DRL backend | TCP start cho planner |
| `server_wait_timeout_s` | `5.0` | double | `repeatability_test_server.cpp` | repeatability | Chờ action con sẵn sàng |
| `action_result_timeout_s` | `120.0` | double | `repeatability_test_server.cpp` | repeatability | Timeout result action con |
| `measurement_settle_time_s` | `2.0` | double | `repeatability_test_server.cpp` | repeatability | Delay ổn định đo |
| `fast_velocity_scale` | `0.1` | double | `repeatability_test_server.cpp` | repeatability | Vận tốc retract/disturb |
| `task_name` | `gohome` | string | `task_manager_client.cpp` | client | Chọn action test |
| `execute` | `true` | bool | client/scripts | action goal | Plan-only hoặc execute |
| `number_of_trials` | `20` | int | `drl_pick_place_random_test_client.py` | random test | Số lần test |
| `gripper_close_width_m` | `0.028` | double | random/demo clients | pick-place | Độ đóng gripper theo mét |

## 3. Parameter theo launch file
- `task_servers.launch.py`, `task_servers_sim.launch.py`: DRL backend và planner node name.
- `repeatability_test_client.launch.py`: `axis`, `repeat_count`, `meas_offset`, `velocity_scale`, `execute`, `frame_id`.
- `drl_pick_place_random_test.launch.py`: `use_gazebo`, `number_of_trials`, `random_seed`, `gripper_close_width_m`, `execute`.

## 4. Parameter theo YAML config
Không có YAML config riêng trong package.

## 5. Giá trị mặc định quan trọng
`velocity_scale=0.1`, `execute=true` ở client, timeout repeatability `120s`.

## 6. Ghi chú thay đổi / rủi ro cấu hình
Sai `planner_node_name` làm DRL action không set được parameter hoặc không gọi được service. Timeout quá thấp dễ fail khi MoveIt/Gazebo khởi động chậm.
