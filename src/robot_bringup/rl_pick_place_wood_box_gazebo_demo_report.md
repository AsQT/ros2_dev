# RL Pick Place Wood + Obstacle Box Gazebo Demo Report

Date: 2026-06-27
Workspace: `/home/minhquang/ros2_dev/src`

## 1. Mục tiêu

Sửa demo RL pick-place trên Gazebo theo đúng quy ước:

```text
wood = vật cần gắp / pick object
box  = vật cản / obstacle
```

Demo không dùng camera, YOLO, detection topic, hoặc image topic làm input cho pick/place. Pose object và obstacle đi từ ground truth/config trong simulation.

## 2. Vấn đề cũ

Launch cũ `robot_bringup/launch/rl_pick_place_box_gazebo_demo.launch.py` spawn entity `pick_box`, publish `/sim/pick_box_info`, rồi `drl_pick_place_box_demo_client.py` dùng pose đó làm `target_pick`.

Sai lệch chính:

- `box` đang bị dùng làm vật gắp.
- Không có entity rõ ràng `pick_wood`.
- Không có entity rõ ràng `obstacle_box`.
- Không có đường riêng đưa kích thước/pose box obstacle vào DRL planner trong demo này.

## 3. Quy ước mới

- `pick_wood`
  - Role: `pick_object`
  - Topic ground truth: `/sim/pick_wood_info`
  - Pose dùng tạo `target_pick` cho `/drl_pickplace`
- `obstacle_box`
  - Role: `obstacle`
  - Topic ground truth: `/sim/obstacle_box_info`
  - Pose/size được đưa vào MoveIt PlanningScene để `drl_unified_planner_node` đọc qua `/get_planning_scene`

## 4. File đã sửa

- `robot_bringup/launch/rl_pick_place_box_gazebo_demo.launch.py`
  - Chuyển demo sang spawn `pick_wood` + `obstacle_box`.
  - Chạy client mới `drl_pick_place_wood_box_demo_client.py`.
  - Tắt DRL backend mặc định bên trong `sim.launch.py` để tránh chạy trùng planner.
  - Giữ prefix planner: `/home/minhquang/venvs/ros_rl/bin/python3`.
- `robot_bringup/launch/sim.launch.py`
  - Thêm argument `enable_drl_backend` và forward vào `task_servers_sim.launch.py`.
- `robot_gazebo/CMakeLists.txt`
  - Install spawner mới.
- `robot_task_manager/CMakeLists.txt`
  - Install demo client mới.

## 5. File đã tạo mới

- `robot_bringup/rl_pick_place_gazebo_rework_audit_report.md`
- `robot_bringup/rl_pick_place_wood_box_gazebo_demo_report.md`
- `robot_gazebo/gazebo/spawn_pick_wood_obstacle_box.py`
- `robot_task_manager/scripts/drl_pick_place_wood_box_demo_client.py`

## 6. Cách spawn object

Spawner:

```text
robot_gazebo/gazebo/spawn_pick_wood_obstacle_box.py
```

Default object trong launch:

```text
wood name: pick_wood
wood world pose center: (0.440, 0.060, 1.030)
wood base_link marker pose center: (0.440, 0.060, 0.010)
wood size: (0.030, 0.030, 0.030)

box name: obstacle_box
box world pose center: (0.390, -0.020, 1.045)
box base_link marker pose center: (0.390, -0.020, 0.025)
box size: (0.050, 0.050, 0.060)
```

Spawn log chính:

```text
Spawn pick_wood role=pick_object size=(0.03, 0.03, 0.03) world=(0.440, 0.060, 1.030)
Spawn obstacle_box role=obstacle size=(0.05, 0.05, 0.06) world=(0.390, -0.020, 1.045)
```

## 7. Cách lấy ground truth từ Gazebo

Flow dùng spawn/config ground truth:

```text
spawn config/table height
  -> exact world pose for pick_wood and obstacle_box
  -> Marker topics in base_link frame
```

Topics:

- `/sim/pick_wood_info`
- `/sim/obstacle_box_info`

Frame:

- Marker publish ở `base_link`.
- `robot_base_world_z=1.02` được dùng để đổi Z từ world sang base_link.
- X/Y giữ nguyên vì world/base origin hiện trùng trong demo.

Không dùng camera/YOLO/detection/image topic làm input cho demo node. Gazebo stack vẫn bridge một số camera topic sẵn có của robot, nhưng flow pick-place mới không subscribe hoặc phụ thuộc vào chúng.

## 8. Cách truyền object/obstacle vào DRL pick_place

Client:

```text
robot_task_manager/scripts/drl_pick_place_wood_box_demo_client.py
```

Luồng:

```text
/sim/pick_wood_info
  -> target_pick của /drl_pickplace

/sim/obstacle_box_info
  -> CollisionObject id=obstacle_box
  -> /apply_planning_scene
  -> republish /collision_object trong lúc action chạy
  -> /get_planning_scene
  -> robot_drl/drl_unified_planner_node
```

Việc republish `/collision_object` là cần thiết vì các stage Cartesian plan-only có thể publish PlanningScene và làm mất world object trước lần DRL plan kế tiếp.

## 9. Lệnh build đã chạy

Build bắt buộc:

```bash
cd /home/minhquang/ros2_dev
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robot_description robot_drl_executor robot_drl robot_task_manager robot_bringup
```

Build thêm vì có executable mới trong `robot_gazebo`:

```bash
cd /home/minhquang/ros2_dev
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robot_gazebo
```

## 10. Kết quả build

Build bắt buộc lần cuối:

```text
Summary: 5 packages finished [4.90s]
```

Build thêm:

```text
Summary: 1 package finished [0.37s]
```

## 11. Lệnh launch demo Gazebo

Smoke test plan-only:

```bash
cd /home/minhquang/ros2_dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=92
ros2 launch robot_bringup rl_pick_place_box_gazebo_demo.launch.py execute:=false demo_client_delay:=45.0
```

## 12. Kết quả ros2 node/action/service list

Action list xác nhận:

```text
/drl_pickplace [robot_task_manager/action/DrlPickPlace]
/move_pose_rl [robot_task_manager/action/MovePoseRl]
```

Service list xác nhận:

```text
/apply_planning_scene [moveit_msgs/srv/ApplyPlanningScene]
/drl/clear_trajectory [std_srvs/srv/Trigger]
/drl/execute_forward [std_srvs/srv/Trigger]
/drl/get_execution_status [std_srvs/srv/Trigger]
/drl/plan [std_srvs/srv/Trigger]
/get_planning_scene [moveit_msgs/srv/GetPlanningScene]
/move_cartesian_pose_sequence [robot_task_executor_msgs/srv/MoveCartesianPoseSequence]
```

Node list trong lúc chạy xác nhận các node chính:

```text
/drl_pickplace_action_server
/drl_unified_planner_node
/pick_wood_obstacle_box_spawner
/robot_drl_executor_node
```

Trong thời điểm demo gửi goal có thêm:

```text
/drl_pick_place_wood_box_demo_client
```

## 13. Kết quả chạy /drl_pickplace

Goal được client tạo từ ground truth:

```text
wood pose=(0.4400, 0.0600, 0.0100), size=(0.0300, 0.0300, 0.0300)
obstacle pose=(0.3900, -0.0200, 0.0250), size=(0.0500, 0.0500, 0.0600)
target_pick=(0.4400, 0.0600, 0.0700)
target_place=(0.4400, -0.0400, 0.1200)
gripper_close_width_m=0.0250
execute=false
```

Action accepted và chạy đủ flow plan-only:

```text
[DrlPickPlace] VALIDATE_GOAL_PLAN_ONLY
[DrlPickPlace] PLAN_TO_PRE_PICK_EXECUTION_SKIPPED
[DrlPickPlace] PLAN_DESCEND_TO_PICK_EXECUTION_SKIPPED
[DrlPickPlace] PLAN_CLOSE_GRIPPER_EXECUTION_SKIPPED
[DrlPickPlace] PLAN_LIFT_FROM_PICK_EXECUTION_SKIPPED
[DrlPickPlace] PLAN_TO_PLACE_EXECUTION_SKIPPED
[DrlPickPlace] DONE_PLANNING_EXECUTION_SKIPPED
```

Planner nhận đúng obstacle ở cả pre-pick và place plan:

```text
[planning_scene] world_objects=1 | usable_obstacles=1
[planning_scene] obstacle id=obstacle_box:0 ... center_base=[ 0.39  -0.02   0.025] full_size=[0.05 0.05 0.06]
[/drl/plan] source=manual+planning_scene ... policy_obstacle=True
```

Collision validation:

```text
Cartesian obstacle validation OK | samples=517 | min_clearance=0.03874 m
Cartesian obstacle validation OK | samples=513 | min_clearance=0.01964 m
```

Result:

```text
DrlPickPlace demo succeeded: DrlPickPlace planning success; execution skipped
```

## 14. Trạng thái cuối cùng

Chạy được ở mức plan-only trên Gazebo.

Đã đạt:

- Gazebo launch được.
- `pick_wood` spawn đúng vai trò vật cần gắp.
- `obstacle_box` spawn đúng vai trò vật cản.
- Wood và box không chồng nhau.
- Pose wood lấy từ spawn/config ground truth và publish trong `base_link`.
- Pose/size box lấy từ spawn/config ground truth và đưa vào PlanningScene.
- `/drl_pickplace` xuất hiện trong action list và nhận goal.
- DRL planner nhận đúng target pick/place và obstacle.
- Build bắt buộc thành công.
- Audit report đã tạo trước khi sửa.
- Demo report đã tạo sau khi sửa.

Ghi chú còn lại:

- `execute=false` vẫn làm planner thực hiện bước preposition nội bộ do logic hiện tại của `drl_pickplace_server` đặt `preposition_before_plan=true` cho stage pre-pick; đây là hành vi sẵn có của flow action, không phải do camera/vision.
- Khi Ctrl-C shutdown, `move_group` vẫn có segfault shutdown path và Gazebo exit `-2`, giống các report trước. Lỗi này xảy ra sau khi action đã success.
