# RL Pick Place Object Z Fix Report

## 1. Hien tuong

- Pipeline `DrlPickPlace` tren Gazebo chay duoc nhung robot chua gap duoc wood on dinh.
- Gia tri quan sat trong luong spawn:
  - `current_used_z ~= 1.0200`
  - `measured_expected_z ~= 1.0300`
  - sai lech `z_error = 0.0100 m`
- Da bo sung correction co parameter de cong sai so Z dung mot lan khi tao `target_pick`.

## 2. File da kiem tra

- `robot_bringup/launch/rl_pick_place_box_gazebo_demo.launch.py`
- `robot_gazebo/gazebo/spawn_pick_wood_obstacle_box.py`
- `robot_task_manager/scripts/drl_pick_place_wood_box_demo_client.py`
- `robot_task_manager/src/drl_pickplace_server.cpp`
- `robot_drl/robot_drl/drl_unified_planner_node.py`
- `robot_gazebo/worlds/table/arm_on_the_table.sdf`
- `robot_gazebo/worlds/wood_block/wood_model.sdf`
- `robot_description/urdf/robot_tcp_z.xacro`

## 3. Frame cua tung gia tri

- `1.0200` trong log `[Z_DEBUG] current_used_z` la `robot_base_world_z`, frame/world Z cua robot base do launch truyen vao spawner.
- `1.0300` trong log `[Z_DEBUG] measured_expected_z` la `wood_pose_world.z`, center Z cua wood trong Gazebo world.
- Marker `/sim/pick_wood_info` publish trong `base_link`, voi:

```text
wood_pose_base.z = wood_pose_world.z - robot_base_world_z
wood_pose_base.z = 1.0300 - 1.0200 = 0.0100
```

- `/drl_pickplace` goal tu demo client dung `frame_id=base_link`.
- `drl_pickplace_server.cpp` dung `planning_frame=base_link`; vi input da la `base_link`, server khong transform them cho pick goal.
- MoveIt co `planning_frame=world` trong mot so log, nhung DRL executor dat pose reference frame la `base_link`; `/drl_pickplace` va DRL planner van lam viec theo target base.

## 4. Nguyen nhan sai lech Z

Cong thuc spawn wood dung model origin o tam khoi:

```text
wood_pose_world.z = table_top_z + wood_height / 2
wood_pose_world.z = 1.0150 + 0.0300 / 2 = 1.0300
```

`current_used_z=1.0200` khong phai wood pose; do la `robot_base_world_z`. Sai so 10 mm:

```text
z_error = wood_pose_world.z - robot_base_world_z
z_error = 1.0300 - 1.0200 = 0.0100
```

Nguyen nhan trong luong pick pose la demo client truoc do khong co parameter correction rieng cho sai so Z on dinh nay. File gay anh huong truc tiep:

```text
robot_task_manager/scripts/drl_pick_place_wood_box_demo_client.py
```

Cong thuc cu chi co:

```text
pick_z = wood_marker.pose.position.z + pick_z_offset_m
```

Cac muc da kiem tra:

- Khong phai do lay visual/collision/inertial pose khac nhau: SDF tam trong spawner khong dat `<pose>` cho collision/visual, nen origin cua box la tam khoi.
- Khong phai do lam tron: cac log giu 4 chu so thap phan va sai so dung 0.0100.
- Khong phai clamp `min_pick_z_m`: current demo pick sau offset lon hon `min_pick_z_m=0.0250`.
- Khong thay cong table height hai lan: spawn dung `table_top + h/2`, marker base dung `world - robot_base_world_z`.
- Co rui ro frame: static TF world->base_link trong sim launch la identity, trong khi spawner tu tru `robot_base_world_z`. Vi demo client va planner dung marker da publish trong `base_link`, correction duoc cong o cung frame voi pick goal de tranh cong sai frame.

## 5. Cong thuc truoc khi sua

Trong `drl_pick_place_wood_box_demo_client.py`:

```text
pick_z_before_correction = wood_pose_base.z + pick_z_offset_m
```

Voi full launch plan-only da chay:

```text
wood_pose_base.z = 0.0100
pick_z_offset_m = 0.0600
pick_z_before_correction = 0.0700
```

Voi case debug theo yeu cau `1.02 -> 1.03`:

```text
pick_z_before_correction = 1.0200
```

## 6. Cong thuc sau khi sua

Da them launch argument:

```text
object_z_correction = 0.01
```

Da them client parameter:

```text
object_z_correction_m = 0.01
```

Cong thuc moi:

```text
corrected_pick_z = original_pick_z + object_z_correction_m
pick_z = max(corrected_pick_z, min_pick_z_m)
```

Voi full launch plan-only:

```text
before correction pick_z = 0.0700
z_correction_m = 0.0100
after correction pick_z = 0.0800
```

Voi case debug theo yeu cau:

```text
before correction pick_z = 1.0200
z_correction_m = 0.0100
after correction pick_z = 1.0300
```

Correction chi duoc cong mot lan trong demo client, truoc khi gui `/drl_pickplace`.

## 7. Log kiem chung

Spawner-only:

```text
[Z_DEBUG] table_top_z = 1.0150
[Z_DEBUG] robot_base_world_z = 1.0200
[Z_DEBUG] wood_height = 0.0300
[Z_DEBUG] wood_pose_world.z = 1.0300
[Z_DEBUG] before_transform_world_z = 1.0300
[Z_DEBUG] after_transform_base_z = 0.0100
[Z_DEBUG] transform_delta_z = -1.0200
[Z_DEBUG] wood_pose_base.z = 0.0100
[Z_DEBUG] expected_wood_top_z = 1.0450
[Z_DEBUG] measured_expected_z = 1.0300
[Z_DEBUG] current_used_z = 1.0200
[Z_DEBUG] z_error = 0.0100
```

Client unit check cho case `1.02 -> 1.03`:

```text
[Z_DEBUG] frame=base_link wood_pose_base.z=1.0200 wood_height=0.0300 expected_wood_top_z=1.0350 pick_z_offset_m=0.0000
[Z_DEBUG] before correction pick_z = 1.0200
[Z_DEBUG] z_correction_m = 0.0100
[Z_DEBUG] after correction pick_z = 1.0300
[Z_DEBUG] current_pick_pose.z = 1.0300 target_pick_minus_top=-0.0050 min_pick_z=0.0250 place_z=0.1200
```

Full Gazebo launch plan-only:

```text
[Z_DEBUG] frame=base_link wood_pose_base.z=0.0100 wood_height=0.0300 expected_wood_top_z=0.0250 pick_z_offset_m=0.0600
[Z_DEBUG] before correction pick_z = 0.0700
[Z_DEBUG] z_correction_m = 0.0100
[Z_DEBUG] after correction pick_z = 0.0800
[Z_DEBUG] current_pick_pose.z = 0.0800 target_pick_minus_top=0.0550 min_pick_z=0.0250 place_z=0.1200
[Z_DEBUG] planning_frame=base_link input_pick_frame=base_link target_pick_z=0.0800 target_place_z=0.1200 pick_approach_height=0.0500 close_width=0.0250
[Z_DEBUG] pick_sequence_z pre_pick=0.1300 pick=0.0800 lift=0.1300 pre_pick_minus_pick=0.0500
```

## 8. Ket qua test Gazebo

Da build:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  robot_description \
  robot_drl_executor \
  robot_drl \
  robot_task_manager \
  robot_bringup \
  robot_gazebo
```

Ket qua:

```text
Summary: 6 packages finished
```

Da chay full launch Gazebo voi `execute:=false` de kiem tra luong plan:

```bash
ros2 launch robot_bringup rl_pick_place_box_gazebo_demo.launch.py \
  execute:=false \
  demo_client_delay:=35.0 \
  spawn_startup_delay:=1.0 \
  randomize_box_size:=false
```

Ket qua:

```text
DrlPickPlace completed successfully
DrlPickPlace demo succeeded: DrlPickPlace planning success; execution skipped
```

Chua xac nhan gap vat ly trong Gazebo voi `execute:=true` trong lan nay, vi test da chay o plan-only de thu log correction va tranh keo dai phien. Do do:

- `/drl_pickplace` plan-only tiep tuc chay duoc.
- Robot/gripper tiep xuc wood khi execute that: chua xac nhan trong report nay.

## 9. Trang thai cuoi cung

- Da trace ro `1.02` la `robot_base_world_z`, `1.03` la `wood_pose_world.z`.
- Da them `object_z_correction_m=0.01` tai tang tao `target_pick`.
- Da log `[Z_DEBUG]` truoc/sau correction va log world/base transform.
- Da build thanh cong cac package lien quan.
- Da test full Gazebo plan-only thanh cong.

