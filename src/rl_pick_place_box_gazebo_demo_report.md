# RL Pick-Place Box Gazebo Demo Report

## Muc tieu

Tao va chay demo:

```text
Gazebo + robot + box 3cm + RL pick_place
```

Demo khong dung YOLO, khong dung camera, khong dung image topic. Thong tin vat the duoc lay tu spawn info / Gazebo ground truth tam thoi va publish ra topic ROS.

## Ket qua chay demo

Demo da chay thanh cong voi launch:

```bash
source install/setup.bash
ros2 launch robot_bringup rl_pick_place_box_gazebo_demo.launch.py demo_client_delay:=45.0
```

Log xac nhan chinh:

```text
Spawn pick_box size=0.030 at x=0.420, y=0.000, z=1.030
Using sim perception: name='pick_box' pose=(0.4200, 0.0000, 0.0100) size=(0.0300, 0.0300, 0.0300)
DrlPickPlace completed successfully
DrlPickPlace demo succeeded: DrlPickPlace completed successfully
```

Sau khi demo thanh cong, Gazebo/MoveIt da duoc dung bang Ctrl-C.

Ghi chu shutdown: khi dung stack bang Ctrl-C, `move_group` co segfault o destructor shutdown. Loi nay xay ra sau khi action da thanh cong, khong lam hong ket qua demo.

## File moi da them

### `robot_description/worlds/pick_box_3cm/model.config`

Mo ta model Gazebo moi cho box demo 3 cm.

### `robot_description/worlds/pick_box_3cm/pick_box_3cm.sdf`

Model SDF toi thieu cho box dong:

```text
size = 0.03 0.03 0.03 m
mass = 0.03 kg
```

Khong sua/xoa model `wood_block` hoac model `worlds/box` cu.

### `robot_description/gazebo/spawn_pick_box.py`

Node spawn box va publish mock perception:

```text
/sim/pick_box_info
```

Message dung:

```text
visualization_msgs/Marker
```

Marker gom:

```text
text              = object name
pose              = pose box
scale.x/y/z       = size box
header.frame_id   = base_link trong launch demo
```

Node tinh Z theo yeu cau:

```text
box_center_z = table_height + box_size_z / 2
```

Trong demo, node doc `table_height` tu:

```text
robot_description/worlds/table/arm_on_the_table.sdf
```

Gia tri suy ra:

```text
table_height = 1.015 m
box_size_z   = 0.03 m
spawn z      = 1.030 m
```

Khi publish theo `base_link`, node tru them:

```text
robot_base_world_z = 1.02
```

Nen perception pose publish co:

```text
z = 1.030 - 1.020 = 0.010 m
```

### `robot_task_manager/scripts/drl_pick_place_box_demo_client.py`

Client demo doc `/sim/pick_box_info`, tao goal `robot_task_manager/action/DrlPickPlace`, va gui toi:

```text
/drl_pickplace
```

Thong so chinh:

```text
gripper_close_width_m = 0.025
pick_z_offset_m       = 0.06
place_xyz             = [0.34, -0.10, 0.12]
```

Ly do co `pick_z_offset_m = 0.06`: policy DRL hien tai khong hoi tu voi target qua thap gan mat ban. Offset nay giup `target_pick` va `pre_pick` nam trong vung policy hien tai co the lap ke hoach.

### `robot_bringup/launch/rl_pick_place_box_gazebo_demo.launch.py`

Launch demo moi, khong sua launch cu.

Launch nay khoi dong:

1. `robot_bringup/launch/sim.launch.py` voi `spawn_demo_woods:=false`
2. `robot_task_executor/launch/task_executor.launch.py`
3. `robot_drl` node `drl_unified_planner_node`
4. `robot_description` node `spawn_pick_box.py`
5. `robot_task_manager` client `drl_pick_place_box_demo_client.py`

Launch arguments quan trong:

```text
randomize_box
box_x
box_y
box_size
table_height
robot_base_world_z
gripper_close_width
pick_z_offset
place_xyz
run_demo_client
spawn_startup_delay
demo_client_delay
```

## File cu da sua toi thieu

### `robot_description/CMakeLists.txt`

Them install script:

```cmake
gazebo/spawn_pick_box.py
```

Ly do: cho phep chay bang:

```bash
ros2 run robot_description spawn_pick_box.py
```

Anh huong: chi them install target, khong doi logic cu.

### `robot_task_manager/CMakeLists.txt`

Them install script:

```cmake
scripts/drl_pick_place_box_demo_client.py
```

Ly do: cho phep launch demo goi executable moi.

Anh huong: chi them install target, khong doi action/server cu.

### `robot_task_manager/package.xml`

Them dependency:

```xml
<depend>visualization_msgs</depend>
```

Ly do: demo client doc `visualization_msgs/msg/Marker`.

Anh huong: bo sung dependency runtime/build metadata, khong doi logic cu.

## Build da thuc hien

Da build cac package lien quan:

```bash
colcon build --packages-select robot_description robot_task_manager robot_bringup
```

Ket qua:

```text
robot_description: OK
robot_task_manager: OK
robot_bringup: OK
```

Co warning cu tu `robot_task_manager` ve `rosidl_target_interfaces()` deprecated. Warning nay da co tu CMake hien tai, khong phai loi cua demo moi.

## Luong demo da xac nhan

1. Gazebo khoi dong voi world table.
2. Robot spawn thanh cong.
3. Controller spawners khoi dong thanh cong.
4. MoveIt va task servers khoi dong.
5. `robot_task_executor` khoi dong va cung cap `/move_cartesian_pose_sequence`.
6. `drl_unified_planner_node` khoi dong.
7. Box 3 cm spawn thanh cong tren ban.
8. `/sim/pick_box_info` publish pose + size cua box.
9. Demo client doc topic mock perception.
10. Client gui goal `/drl_pickplace`.
11. Action mo gripper.
12. DRL plan/execute toi pre-pick.
13. Cartesian descend toi pick.
14. Dong gripper voi width `0.025`.
15. Cartesian lift.
16. DRL plan/execute toi place.
17. Mo gripper.
18. Action ket thuc thanh cong.

## Cach chay lai

```bash
cd /home/minhquang/ros2_dev
source install/setup.bash
ros2 launch robot_bringup rl_pick_place_box_gazebo_demo.launch.py

source install/setup.bash
ros2 launch robot_bringup rl_pick_place_box_gazebo_demo.launch.py demo_client_delay:=45.0
```

Neu muon random box:

```bash
ros2 launch robot_bringup rl_pick_place_box_gazebo_demo.launch.py randomize_box:=true
```

Neu muon doi vi tri dat:

```bash
ros2 launch robot_bringup rl_pick_place_box_gazebo_demo.launch.py place_xyz:="[0.36, -0.08, 0.12]"
```

## Luu y

Policy DRL hien tai khong on dinh voi target Z qua thap gan mat ban. Vi vay launch demo mac dinh:

```text
pick_z_offset = 0.06
place_xyz.z   = 0.12
```

Neu can demo dat vat sat mat ban thuc hon, nen tinh chinh/retrain policy hoac them buoc Cartesian descend/open gripper rieng cho place, tuong tu descend-to-pick.
