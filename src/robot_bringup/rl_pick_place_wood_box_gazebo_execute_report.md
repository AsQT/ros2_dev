# RL Pick Place Wood + Random Obstacle Box Gazebo Execute Report

## 1. Mục tiêu

- Chạy demo Gazebo với execute thật, không dừng ở plan-only.
- `pick_wood` là vật cần gắp.
- `obstacle_box` là vật cản.
- `obstacle_box` có kích thước random trong khoảng 0.05 -> 0.15 m.
- Không dùng camera/YOLO/vision input; demo lấy ground truth từ spawn info.

## 2. Nguyên nhân cũ bị execution skipped

Nguyên nhân trực tiếp là goal `/drl_pickplace` được gửi với `execute=false`.

Trace:

- `robot_bringup/launch/rl_pick_place_box_gazebo_demo.launch.py` khai báo launch argument `execute` default cũ là `false`.
- `robot_task_manager/scripts/drl_pick_place_wood_box_demo_client.py` khai báo parameter `execute` default cũ là `False`.
- `robot_task_manager/src/drl_pickplace_server.cpp` có nhánh hợp lệ: nếu goal `execute=false` thì chỉ plan và trả về message `DrlPickPlace planning success; execution skipped`.

Vì vậy log cũ `planning success; execution skipped` không phải do controller/Gazebo fallback, mà do demo chạy plan-only theo default.

## 3. File đã sửa

- `robot_bringup/launch/rl_pick_place_box_gazebo_demo.launch.py`
  - Default `execute=true`.
  - Spawn `pick_wood` và `obstacle_box`.
  - Dùng `robot_gazebo/spawn_pick_wood_obstacle_box.py`.
  - Prefix riêng node RL bằng `/home/minhquang/venvs/ros_rl/bin/python3`.
  - Include `robot_drl_executor.launch.py` để có `/move_cartesian_pose_sequence`.
  - Default `place_xyz=[0.46, 0.12, 0.12]` để policy hội tụ ở chặng place.
- `robot_gazebo/gazebo/spawn_pick_wood_obstacle_box.py`
  - Random size obstacle theo từng trục trong `[0.05, 0.15]`.
  - Kiểm tra không chồng lên wood, target place và robot base.
  - Publish marker/info cho wood và obstacle.
- `robot_task_manager/scripts/drl_pick_place_wood_box_demo_client.py`
  - Default `execute=True`.
  - Default `place_xyz=[0.46, 0.12, 0.12]`.

## 4. File đã tạo mới nếu có

- Tạo mới report này: `robot_bringup/rl_pick_place_wood_box_gazebo_execute_report.md`.

Không tạo launch mới; launch cũ `rl_pick_place_box_gazebo_demo.launch.py` đã được cập nhật để demo mặc định execute thật.

## 5. Logic spawn wood

- Name: `pick_wood`.
- Role: `pick_object`.
- Default pose base: `x=0.44`, `y=0.06`, `z` đặt trên mặt bàn theo chiều cao table infer từ SDF.
- Size: `[0.03, 0.03, 0.03]` m.
- Ground truth được publish qua marker topic `/sim/pick_wood_info`.

Log chạy Gazebo:

```text
Spawn pick_wood role=pick_object size=(0.03, 0.03, 0.03) world=(0.440, 0.060, 1.030)
```

## 6. Logic spawn obstacle box

- Name: `obstacle_box`.
- Role: `obstacle`.
- Random size: random đều từng trục `x/y/z` trong `[0.05, 0.15]` m.
- Default pose: `x=0.34`, `y=-0.09`, `z` đặt trên mặt bàn theo chiều cao box.
- Khi random pose, box được sample trong vùng x/y cấu hình và bị reject nếu:
  - Gần/chồng wood.
  - Chồng target place.
  - Chồng robot base.
  - Không đạt `min_xy_separation`.

Log chạy Gazebo:

```text
Sampled obstacle_box size=(0.133, 0.064, 0.054) in required [0.050, 0.150] m range
Spawn obstacle_box role=obstacle size=(0.1326090135, 0.0641037633, 0.0539311425) world=(0.340, -0.090, 1.042)
```

## 7. Ground truth dùng cho demo

- Wood pose lấy từ marker `/sim/pick_wood_info`.
- Box pose và size lấy từ marker `/sim/obstacle_box_info`.
- Spawner publish marker trong `frame_id=base_link`, đồng thời spawn Gazebo entity bằng world pose đã cộng `robot_base_world_z`.
- Demo client kiểm tra frame marker khớp `base_link`, rồi gửi `target_pick` và `target_place` vào `/drl_pickplace`.

Log client:

```text
Using Gazebo ground truth: wood=pick_wood;role=pick_object pose=(0.4400, 0.0600, 0.0100) size=(0.0300, 0.0300, 0.0300);
obstacle=obstacle_box;role=obstacle pose=(0.3400, -0.0900, 0.0220) size=(0.1326, 0.0641, 0.0539);
pick_z=0.0700 place=(0.4600, 0.1200, 0.1200) gripper=0.0250 execute=True
```

## 8. Cách obstacle được đưa vào DRL/planning

Demo client thêm `obstacle_box` vào MoveIt PlanningScene bằng `CollisionObject` dạng box với pose/size lấy từ marker ground truth.

DRL planner đọc PlanningScene và đưa obstacle vào cả validation lẫn policy observation:

```text
[planning_scene] world_objects=1 | usable_obstacles=1
[planning_scene] obstacle id=obstacle_box:0 ... center_base=[ 0.34  -0.09   0.022] full_size=[0.1326 0.0641 0.0539]
[/drl/plan] policy obstacle selected id=obstacle_box:0 source=manual+planning_scene
[/drl/plan] ... number_of_obstacles=1 | policy_obstacle=True
```

Collision validation cũng chạy:

```text
Cartesian obstacle validation OK | samples=24 | min_clearance=0.09177 m
Cartesian obstacle validation OK | samples=10 | min_clearance=0.09176 m
```

## 9. Cách bật execute=true

- Launch argument `execute` trong `rl_pick_place_box_gazebo_demo.launch.py` default là `true`.
- Demo client default parameter `execute=True`.
- Goal `/drl_pickplace` nhận `execute=True`, nên server gọi `/drl/execute_forward` thay vì nhánh `execution skipped`.

Log xác nhận:

```text
execute=True
[DrlPickPlace] PLAN_TO_PRE_PICK | 22.0%
DRL plan attempt 1/3 mode=execute ...
[/move_cartesian_pose_sequence] ... execute=1
```

## 10. Lệnh build đã chạy

Build bắt buộc:

```bash
cd /home/minhquang/ros2_dev
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robot_description robot_drl_executor robot_drl robot_task_manager robot_bringup
```

Build thêm package spawner:

```bash
colcon build --symlink-install --packages-select robot_gazebo
```

Sau lần chỉnh target place, build lại đầy đủ:

```bash
colcon build --symlink-install --packages-select robot_description robot_drl_executor robot_drl robot_task_manager robot_bringup robot_gazebo
```

## 11. Kết quả build

Kết quả build bắt buộc:

```text
Summary: 5 packages finished [4.32s]
```

Build thêm `robot_gazebo`:

```text
Summary: 1 package finished [0.39s]
```

Build cuối sau chỉnh sửa:

```text
Summary: 6 packages finished [5.12s]
```

Kiểm tra Python:

```bash
python3 -m py_compile robot_bringup/launch/rl_pick_place_box_gazebo_demo.launch.py robot_gazebo/gazebo/spawn_pick_wood_obstacle_box.py robot_task_manager/scripts/drl_pick_place_wood_box_demo_client.py
```

Kết quả: pass.

## 12. Lệnh launch Gazebo đã chạy

```bash
cd /home/minhquang/ros2_dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=95
ros2 launch robot_bringup rl_pick_place_box_gazebo_demo.launch.py
```

Trước đó có một run với `place=(0.44,-0.04,0.12)` bị fail ở `PLAN_TO_PLACE` do DRL rollout không hội tụ. Không sửa model/reward/action space; chỉ đổi default place sang `(0.46,0.12,0.12)` trong workspace huấn luyện và cách obstacle hơn.

## 13. Kết quả node/action/service/controller list

Action list:

```text
/arm_controller/follow_joint_trajectory [control_msgs/action/FollowJointTrajectory]
/drl_pickplace [robot_task_manager/action/DrlPickPlace]
/gripper_controller/follow_joint_trajectory [control_msgs/action/FollowJointTrajectory]
/move_pose_rl [robot_task_manager/action/MovePoseRl]
```

Service bắt buộc:

```text
/drl/clear_trajectory [std_srvs/srv/Trigger]
/drl/execute_forward [std_srvs/srv/Trigger]
/drl/get_execution_status [std_srvs/srv/Trigger]
/drl/plan [std_srvs/srv/Trigger]
/move_cartesian_pose_sequence [robot_task_executor_msgs/srv/MoveCartesianPoseSequence]
```

Controller list:

```text
gripper_controller      joint_trajectory_controller/JointTrajectoryController  active
arm_controller          joint_trajectory_controller/JointTrajectoryController  active
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster          active
```

## 14. Kết quả /drl_pickplace execute

Goal:

```text
target_pick=(0.4400, 0.0600, 0.0700)
target_place=(0.4600, 0.1200, 0.1200)
gripper_close_width_m=0.0250
execute=True
```

Result:

```text
[DrlPickPlace] DONE | 100.0%
DrlPickPlace completed successfully
DrlPickPlace demo succeeded: DrlPickPlace completed successfully
```

Failed stage: none.

Log chính của `drl_pickplace_server`:

```text
[DrlPickPlace] OPEN_GRIPPER | 8.0%
[DrlPickPlace] PLAN_TO_PRE_PICK | 22.0%
DRL execution completed: SUCCEEDED_FORWARD: Executed 14 [forward] waypoints via /move_cartesian_pose_sequence (fraction=1.0000).
PLAN_TO_PRE_PICK pose check ... pos_err=0.00032
[DrlPickPlace] DESCEND_TO_PICK | 38.0%
DESCEND_TO_PICK pose check ... pos_err=0.00001
[DrlPickPlace] CLOSE_GRIPPER | 50.0%
[DrlPickPlace] LIFT_FROM_PICK | 62.0%
LIFT_FROM_PICK pose check ... pos_err=0.00003
[DrlPickPlace] PLAN_TO_PLACE | 82.0%
PLAN_TO_PLACE pose check ... pos_err=0.00009
[DrlPickPlace] OPEN_GRIPPER_AT_PLACE | 96.0%
[DrlPickPlace] DONE | 100.0%
```

Log chính của `drl_unified_planner_node`:

```text
[/drl/plan] target_base=[0.44 0.06 0.12] | number_of_obstacles=1 | policy_obstacle=True
Trajectory planning complete ... target_base=(0.4400, 0.0600, 0.1200) ... forward_waypoints=15
[execute_forward] ... success=True message='Forward execution started.'
[forward] background execution finished | status=SUCCEEDED_FORWARD

[/drl/plan] target_base=[0.46 0.12 0.12] | number_of_obstacles=1 | policy_obstacle=True
Trajectory planning complete ... target_base=(0.4600, 0.1200, 0.1200) ... forward_waypoints=7
[forward] background execution finished | status=SUCCEEDED_FORWARD
```

Log chính của `robot_drl_executor_node`:

```text
/move_cartesian_pose_sequence: 1 poses, execute=1
/move_cartesian_pose_sequence: 14 poses, execute=1
computeCartesianPath fraction=1.000000 trajectory_points=84 threshold=0.95 success=1
[Executor] Trajectory executed successfully.
[/move_cartesian_pose_sequence] Cartesian pose sequence executed successfully (fraction=1.000000)
/move_cartesian_pose_sequence: 6 poses, execute=1
computeCartesianPath fraction=1.000000 trajectory_points=54 threshold=0.95 success=1
[Executor] Trajectory executed successfully.
```

Joint/controller chứng minh robot có di chuyển:

```text
Initial /joint_states:
joint_1=-0.00000009, joint_2=-0.00001657, joint_3=0.00008329, joint_5=-0.00010746

During execution:
joint_1=0.08355781, joint_2=0.35941126, joint_3=0.45167529, joint_5=0.75970766

During/after place:
joint_1=0.23426215, joint_2=0.74374449, joint_3=0.28298359, joint_5=0.54406824

Final:
joint_1=0.25503174, joint_2=0.79101711, joint_3=0.19844374, joint_5=0.58133549
```

Gripper flow:

```text
Gripper moved to opening 0.0500 m
Gripper moved to opening 0.0250 m
Gripper moved to opening 0.0500 m
```

## 15. Trạng thái cuối cùng

Trạng thái: execute được trong Gazebo.

Đạt các tiêu chí:

- Gazebo launch được.
- `pick_wood` spawn đúng là vật cần gắp.
- `obstacle_box` spawn đúng là vật cản.
- Box random size nằm trong 0.05 -> 0.15 m.
- Không dùng camera/YOLO/vision input.
- Pose/size lấy từ spawn info marker ground truth.
- `/drl_pickplace` chạy với `execute=True`.
- Không còn kết quả cuối `execution skipped`.
- Robot joint states thay đổi rõ trong Gazebo.
- `arm_controller` và `gripper_controller` đều nhận action goal và active.
- Gripper mở, đóng, rồi mở lại ở place.
- Action result success.

Không sửa reward, observation dimension, trained model, network architecture hoặc action space.

Ghi chú shutdown: sau khi demo success, Ctrl-C launch vẫn tạo một số lỗi shutdown-path quen thuộc của MoveIt/RViz/Gazebo (`move_group`/`rviz2` segfault, Gazebo exit `-2`, `rclpy.shutdown already called`). Các lỗi này xảy ra sau kết quả success và không làm thay đổi kết luận execute.
