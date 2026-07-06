# robot_bringup - Parameters

## Launch arguments chính
| Parameter | Default | Nơi khai báo | Ý nghĩa |
|---|---:|---|---|
| `use_vision` | `false` trong `mock`/`sim`, `true` trong `real.launch.py` hiện tại | `mock.launch.py`, `sim.launch.py`, `real.launch.py` | Bật `vision_full_pipeline.launch.py` và `aruco_world_static_tf.launch.py`. |
| `enable_executor_logging` | `false` | `mock.launch.py`, `real.launch.py`, `task_servers.launch.py`, `task_executor.launch.py` | Bật CSV executor/internal logging ở các node được launch forward tham số. |
| `log_root_dir` | `/home/minhquang/ros2_dev/Log_robot_data` trong launch | `mock.launch.py`, `real.launch.py` | Root log được truyền vào task/executor; code C++ cũng có default `$HOME/ros2_dev/Log_robot_data`. |
| `executor_log_dir` | tùy mode | `mock.launch.py`, `real.launch.py` | Thư mục log executor nội bộ được forward. |
| `executor_sample_rate_hz` | `50.0` | `mock.launch.py`, `real.launch.py` | Tần số sample TCP/joint cho logger. |
| `executor_base_frame` | `base_link` | `mock.launch.py`, `real.launch.py` | Base frame cho TF lookup log. |
| `executor_tcp_frame` | `tcp_link` | `mock.launch.py`, `real.launch.py` | TCP frame cho TF lookup log. |
| `spawn_demo_woods` | `true` | `sim.launch.py` | Forward vào `robot_gazebo/gazebo.launch.py`. |
| `enable_drl_backend` | `true` | `sim.launch.py`, `task_servers*.launch.py` | Bật/tắt `drl_unified_planner_node` khi task servers chạy. |
| `num_runs` | `20` | `rl_pick_place_campaign.launch.py` | Số lượt campaign DRL pick-place. |
| `output_dir` | empty | `rl_pick_place_campaign.launch.py` | Nơi lưu CSV campaign; empty nghĩa là auto timestamp. |
| `execute` | `true` | demo/campaign launch | Gửi goal plan-only hoặc execute. |

## YAML/config liên quan
- `config/cyclonedds.xml`: được `mock/sim/real.launch.py` set qua `CYCLONEDDS_URI`.
- `config/aruco_world_to_base.yaml`: nguồn transform cho `aruco_world_static_tf.launch.py`.
- `config/common.yaml`, `config/ros2_controllers.yaml`: có trong package nhưng cần kiểm tra launch nào thật sự nạp trước khi xem là runtime-effective.

## Rủi ro cấu hình
- Một số launch hardcode `/home/minhquang/...`; cần chỉnh/override khi chạy trên máy khác.
- Vision là nhánh tùy chọn, thiếu camera/model/calibration sẽ ảnh hưởng vision node nhưng không nên chặn MoveIt/task servers.
- `enable_drl_backend:=false` chỉ hợp lý khi đã có DRL backend bên ngoài chạy sẵn.
