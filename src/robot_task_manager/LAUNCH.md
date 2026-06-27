# robot_task_manager - Launch Guide

## 1. Danh sách launch file
| Launch file | Mục đích | Node được chạy | Điều kiện cần |
|---|---|---|---|
| `task_servers.launch.py` | Chạy action servers real/mock | DRL planner optional + 9 action servers | Cần MoveIt/move_group/controller |
| `task_servers_sim.launch.py` | Chạy action servers simulation | DRL planner optional + 9 action servers | Cần Gazebo/MoveIt sim |
| `repeatability_test_client.launch.py` | Gửi repeatability goal | `repeatability_test_client.py` | Cần `/repeatability_test` |
| `drl_pick_place_random_test.launch.py` | Test random DRL pick-place | include DRL launch + random client | Cần `/drl_pickplace` và DRL backend |

## 2. Chi tiết từng launch file
### task_servers.launch.py
#### Chức năng
Khởi động toàn bộ action servers.
#### Node được khởi tạo
`gohome_server`, `move_to_pose_server`, `move_pose_cartesian_server`, `checker_board_server`, `move_gripper_server`, `pickplace_server`, `drl_pickplace_server`, `move_pose_rl_server`, `repeatability_test_server`, optional `drl_unified_planner_node`.
#### Argument
`enable_drl_backend`, `planner_node_name`, `drl_calibrated_start_tcp_base`.
#### Parameter truyền vào node
Planner node name và start TCP cho DRL servers/backend.
#### Package phụ thuộc
`robot_moveit`, `robot_drl`, `robot_task_manager` actions.
#### Điều kiện thực thi
`move_group` và controller phải sẵn sàng.
#### Lệnh chạy
```bash
ros2 launch robot_task_manager task_servers.launch.py
```
#### Lỗi thường gặp
Action server chạy nhưng execute fail do MoveIt/controller chưa active.

### task_servers_sim.launch.py
Tương tự `task_servers.launch.py` nhưng dùng simulation/use_sim_time. Chạy bằng:
```bash
ros2 launch robot_task_manager task_servers_sim.launch.py
```

### repeatability_test_client.launch.py
Gửi goal repeatability với axis/repeat_count/meas_offset/velocity_scale. Chạy bằng:
```bash
ros2 launch robot_task_manager repeatability_test_client.launch.py
```

### drl_pick_place_random_test.launch.py
Include DRL mock/gazebo launch theo `use_gazebo` và chạy random test client. Chạy bằng:
```bash
ros2 launch robot_task_manager drl_pick_place_random_test.launch.py use_gazebo:=false
```
