# robot_bringup - Launch Guide

## Danh sách launch file
| Launch file | Node/include chính | Argument quan trọng | Ghi chú |
|---|---|---|---|
| `mock.launch.py` | `robot_moveit/moveit_gui.launch.py`, `robot_task_manager/task_servers.launch.py`, `robot_task_executor/task_executor.launch.py`, optional `vision_full_pipeline` + `aruco_world_static_tf` | `use_vision`, `enable_executor_logging`, `log_root_dir`, `executor_*` | Mock hardware, `use_sim_time=false`. |
| `sim.launch.py` | `robot_gazebo/gazebo.launch.py`, `robot_moveit/moveit.launch.py`, `robot_task_manager/task_servers_sim.launch.py`, controller spawners | `spawn_demo_woods`, `enable_drl_backend`, `use_vision` | Gazebo + `gz_ros2_control`; MoveIt/task servers start bằng timer. |
| `real.launch.py` | `robot_moveit/moveit_gui.launch.py`, `robot_task_manager/task_servers.launch.py`, `robot_task_executor/task_executor.launch.py`, optional vision/static TF | `use_vision`, `enable_executor_logging`, `runtime_mode=real` forwarded | Dùng `use_mock=false`; cần phần cứng/controller sẵn sàng. |
| `aruco_world_static_tf.launch.py` | `tf2_ros/static_transform_publisher` | YAML `config/aruco_world_to_base.yaml` | Static TF tạm thời `parent_frame -> child_frame`, phục vụ vision/world transform. |
| `rl_pick_place_box_gazebo_demo.launch.py` | `sim.launch.py`, `robot_drl_executor`, object spawner, `drl_unified_planner_node`, demo client | object pose/size, `execute`, `run_demo_client`, delay | Demo `/drl_pickplace` trong Gazebo. |
| `rl_pick_place_campaign.launch.py` | `sim.launch.py`, `robot_drl_executor`, object spawner, `drl_unified_planner_node`, campaign runner | `num_runs`, `save_csv`, `output_dir`, randomization, delay | Chạy nhiều lượt DRL pick-place và ghi telemetry. |

## Ví dụ chạy
```bash
source ~/ros2_dev/install/setup.bash

ros2 launch robot_bringup mock.launch.py
ros2 launch robot_bringup mock.launch.py use_vision:=true enable_executor_logging:=true

ros2 launch robot_bringup sim.launch.py
ros2 launch robot_bringup sim.launch.py spawn_demo_woods:=false enable_drl_backend:=true

ros2 launch robot_bringup real.launch.py
ros2 launch robot_bringup rl_pick_place_box_gazebo_demo.launch.py
ros2 launch robot_bringup rl_pick_place_campaign.launch.py num_runs:=50 output_dir:=/home/minhquang/rl_eval_runs/experiment_01
```

## Điều kiện runtime
- Cần `source install/setup.bash` sau build.
- `mock.launch.py`: cần MoveIt mock/controller manager hoạt động.
- `sim.launch.py`: cần Gazebo/ros_gz và controller `joint_state_broadcaster`, `arm_controller`, `gripper_controller`.
- `real.launch.py`: cần phần cứng thật hoặc hardware interface/controller tương ứng; trạng thái thực tế cần xác nhận.
- Vision branch cần camera/model/calibration nếu `use_vision:=true`.
- DRL branch cần venv `ros_rl`, model trong `robot_drl/models`, và service `/drl/*` từ `drl_unified_planner_node`.
