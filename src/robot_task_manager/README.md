# robot_task_manager

## 1. Vai trò package
Package action-based task execution cho robot. Nó định nghĩa action, build action servers MoveIt/gripper/composite/DRL/repeatability và cung cấp client demo/test. Source of truth là `action/*.action`, `src/*.cpp`, `include/robot_task_manager/*.hpp`, `launch/*.launch.py`.

## 2. Vị trí trong hệ thống
Nằm giữa GUI/client và MoveIt/DRL/hardware. `robot_gui` gửi action goal tới package này; `robot_bringup` và `robot_moveit` launch các action server; server gọi MoveIt qua `moveit_executor`, gripper executor hoặc gọi DRL service `/drl/*`.

## 3. Thành phần chính
- Action definitions: `GoHome`, `MoveToPose`, `MoveToPoseCartesian`, `CheckerBoard`, `MoveGripper`, `PickPlace`, `DrlPickPlace`, `MovePoseRl`, `RepeatabilityTest`.
- Libraries: `moveit_executor`, `gripper_executor`.
- Action servers: `gohome_server`, `move_to_pose_server`, `move_pose_cartesian_server`, `checker_board_server`, `move_gripper_server`, `pickplace_server`, `drl_pickplace_server`, `move_pose_rl_server`, `repeatability_test_server`.
- Clients/scripts: `task_manager_client`, `drl_pick_place_box_demo_client.py`, `drl_pick_place_random_test_client.py`, `repeatability_test_client.py`.
- Launch: `task_servers.launch.py`, `task_servers_sim.launch.py`, `repeatability_test_client.launch.py`, `drl_pick_place_random_test.launch.py`.
- Tài liệu liên quan: `Call_action.md` và các report hiện có nên được xem như tham khảo, còn file này đồng bộ theo source hiện tại.

## 4. Node / executable
| Node / executable | Nguồn | Vai trò |
|---|---|---|
| `gohome_server` | `src/gohome_server.cpp` | Action `/gohome` |
| `move_to_pose_server` | `src/move_to_pose_server.cpp` | Action `/move_to_pose` |
| `move_pose_cartesian_server` | `src/move_to_pose_cartesian_server.cpp` | Action `/move_to_pose_cartesian` |
| `checker_board_server` | `src/move_checker_board_server.cpp` | Action `/move_checker_board` |
| `move_gripper_server` | `src/move_gripper_server.cpp` | Action `/move_gripper` |
| `pickplace_server` | `src/pickplace_server.cpp` | Composite pick-place |
| `drl_pickplace_server` | `src/drl_pickplace_server.cpp` | DRL pick-place composite |
| `move_pose_rl_server` | `src/move_pose_rl_server.cpp` | DRL move pose action |
| `repeatability_test_server` | `src/repeatability_test_server.cpp` | Repeatability loop |
| `task_manager_client` | `src/task_manager_client.cpp` | CLI/test client |

## 5. Topic / Service / Action
| Interface | Type | Vai trò |
|---|---|---|
| `/gohome` | `GoHome.action` | Move home/named target |
| `/move_to_pose` | `MoveToPose.action` | PTP pose |
| `/move_to_pose_cartesian` | `MoveToPoseCartesian.action` | Cartesian pose |
| `/move_checker_board` | `CheckerBoard.action` | Checker-board pattern |
| `/move_gripper` | `MoveGripper.action` | Gripper target |
| `/pickplace` | `PickPlace.action` | Pick-place sequence |
| `/drl_pickplace` | `DrlPickPlace.action` | DRL pick-place sequence |
| `/move_pose_rl` | `MovePoseRl.action` | DRL move-to-pose |
| `/repeatability_test` | `RepeatabilityTest.action` | Measurement loop |
| `/drl/plan`, `/drl/execute_forward`, `/drl/get_execution_status` | `std_srvs/Trigger` | DRL backend service clients |

## 6. File launch liên quan
`task_servers.launch.py`, `task_servers_sim.launch.py`, `repeatability_test_client.launch.py`, `drl_pick_place_random_test.launch.py`.

## 7. File cấu hình liên quan
Không có YAML config package-level; tham số chủ yếu nằm trong launch và source `declare_parameter`.

## 8. Cách build riêng package
```bash
cd ~/ros2_dev
colcon build --packages-select robot_task_manager
source install/setup.bash
```

## 9. Cách chạy nhanh
```bash
source ~/ros2_dev/install/setup.bash
ros2 launch robot_task_manager task_servers.launch.py
```
Simulation:
```bash
ros2 launch robot_task_manager task_servers_sim.launch.py
```

## 10. Ghi chú kỹ thuật / giới hạn hiện tại
- Cần `move_group` và controller đã chạy trước khi action MoveIt execute thành công.
- `execute=false` dùng để plan-only.
- DRL actions cần `drl_unified_planner_node` và service `/drl/*` khả dụng.
- Repeatability test gọi action con `/move_to_pose` và `/move_to_pose_cartesian` theo loop.
