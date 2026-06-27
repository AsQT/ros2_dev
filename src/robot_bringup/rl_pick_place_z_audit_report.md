# RL Pick Place Gazebo Z Audit

## Scope

Audit luong Z cho demo `DrlPickPlace` Gazebo:

- `pick_wood` la object can gap.
- `obstacle_box` la obstacle.
- Khong dung camera.
- Pose object/obstacle lay tu spawn/ground truth va publish marker.
- Chua sua reward/model/observation/action space.

## Instrumentation Added

Them log quan sat, khong doi hanh vi:

- `robot_gazebo/gazebo/spawn_pick_wood_obstacle_box.py`
  - `[Z_AUDIT][spawner] table_top_world_z`
  - `robot_base_world_z`
  - `world_z bottom/center/top`
  - `base_z bottom/center/top`
  - model origin ghi ro la `box_center`

- `robot_task_manager/scripts/drl_pick_place_wood_box_demo_client.py`
  - `[Z_AUDIT][client] wood_center_z`
  - `wood_size_z`
  - `wood_top_z`
  - `pick_z_offset`
  - `target_pick_z`
  - `target_pick_minus_top`

- `robot_task_manager/src/drl_pickplace_server.cpp`
  - `[Z_AUDIT][server] target_pick_z`
  - `target_place_z`
  - `pick_approach_height`
  - `pre_pick`, `pick`, `lift`

## Static Findings

### Wood Model Origin

Demo hien tai khong dung truc tiep `robot_gazebo/worlds/wood_block/wood_model.sdf` cho `pick_wood`.

`spawn_pick_wood_obstacle_box.py` tao SDF tam bang `_make_box_sdf()`:

```xml
<model name="pick_wood">
  <link name="link">
    <collision name="collision">
      <geometry><box><size>sx sy sz</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>sx sy sz</size></box></geometry>
    </visual>
  </link>
</model>
```

Collision/visual khong co `<pose>`, nen box duoc dat quanh link/model origin. Vi vay Gazebo model pose Z la tam khoi, khong phai mat day.

### Table Height

`robot_gazebo/worlds/table/arm_on_the_table.sdf`:

- table surface pose Z = `1.0`
- surface box height = `0.03`
- table top = `1.0 + 0.03 / 2 = 1.015 m`

Spawner doc dung cong thuc nay khi `table_height < 0`.

### World To Base

Launch default:

- `robot_base_world_z = 1.02`
- marker frame = `base_link`

Spawner chuyen Z:

```text
marker_base_z = center_world_z - robot_base_world_z
```

X/Y duoc giu nguyen. Khong co TF lookup o spawner. Dieu nay chi dung neu world/base origin trung X/Y va chi lech Z bang `robot_base_world_z`.

Server `drl_pickplace_server.cpp` se transform input goal sang `planning_frame=base_link` neu frame khac. Trong demo, client gui san `base_link`, nen khong transform them.

## Runtime Trace

Da chay spawner-only voi `spawn:=false` de lay so Z that tu node, dung default demo quan trong:

```bash
source /home/minhquang/ros2_dev/install/setup.bash
timeout 5 ros2 run robot_gazebo spawn_pick_wood_obstacle_box.py --ros-args \
  -p spawn:=false \
  -p startup_delay:=0.0 \
  -p publish_rate_hz:=1.0 \
  -p frame_id:=base_link \
  -p wood_size:='[0.03, 0.03, 0.03]' \
  -p wood_x:=0.44 \
  -p wood_y:=0.06 \
  -p randomize_box_size:=false \
  -p box_size:='[0.10, 0.10, 0.10]' \
  -p box_x:=0.34 \
  -p box_y:=-0.09 \
  -p robot_base_world_z:=1.02
```

Ket qua log:

```text
[Z_AUDIT][spawner] table_top_world_z=1.0150 robot_base_world_z=1.0200 model_origin=box_center marker_frame=base_link
[Z_AUDIT][spawner] pick_wood role=pick_object size_z=0.0300 world_z bottom/center/top=(1.0150 1.0300 1.0450) base_z bottom/center/top=(-0.0050 0.0100 0.0250)
[Z_AUDIT][spawner] obstacle_box role=obstacle size_z=0.1000 world_z bottom/center/top=(1.0150 1.0650 1.1150) base_z bottom/center/top=(-0.0050 0.0450 0.0950)
```

`timeout` exit code `124` la binh thuong trong test nay vi node dang spin de publish marker lien tuc.

## Current Formula

Voi default demo:

```text
wood_size_z = 0.0300
table_top_world_z = 1.0150
robot_base_world_z = 1.0200

wood_center_world_z = 1.0150 + 0.0300 / 2 = 1.0300
wood_top_world_z    = 1.0300 + 0.0300 / 2 = 1.0450

wood_center_base_z = 1.0300 - 1.0200 = 0.0100
wood_top_base_z    = 1.0450 - 1.0200 = 0.0250
```

Client hien tai:

```text
pick_z = wood_center_base_z + pick_z_offset
pick_z_offset default launch = 0.0600
pick_z = 0.0100 + 0.0600 = 0.0700
```

Server:

```text
pre_pick_z = pick_z + pick_approach_height_m
pick_approach_height_m default = 0.0500
pre_pick_z = 0.1200
lift_z = pre_pick_z = 0.1200
```

So voi mat tren wood:

```text
target_pick_z - wood_top_base_z = 0.0700 - 0.0250 = 0.0450 m
```

Tuc TCP pick dang cao hon mat tren wood 45 mm.

## Conclusion

Loi Z co kha nang nam o cong thuc pick Z trong client demo:

```text
pick_z = wood_center_z + pick_z_offset
```

Voi object cao 30 mm va offset 60 mm, TCP khong den gan wood. Neu chien luoc la gap tu tren xuong voi TCP can den mat tren object, offset hien tai qua cao.

Khong thay dau hieu cong table height hai lan trong spawn/marker:

- Spawn world dung `table_top + h/2`.
- Marker base dung `world_z - robot_base_world_z`.
- Client dung marker base, khong cong table height.
- Server nhan `base_link`, khong transform them.

Rui ro frame con lai: spawner tu tru `robot_base_world_z` thay vi TF. Neu `base_link` world pose thuc te khac `z=1.02` hoac co lech X/Y, marker se sai. Can so sanh log TF `world -> base_link` khi chay full Gazebo neu van fail sau khi sua pick Z.

## Minimal Patch Proposal

Chua ap dung patch thay doi hanh vi.

Neu xac nhan TCP mong muon nam ngay tren mat tren wood, cong thuc nen doi tu:

```text
pick_z = wood_center_z + pick_z_offset
```

sang:

```text
pick_z = wood_top_z + grasp_clearance_or_tcp_offset
wood_top_z = wood_center_z + wood_size_z / 2
```

Voi current numbers:

```text
wood_top_z = 0.0250
grasp_clearance_or_tcp_offset = 0.0000 .. 0.0050
expected_pick_z = 0.0250 .. 0.0300
```

Neu can giu param hien co, patch nho nhat la doi default launch:

```text
pick_z_offset = 0.015
```

vi client dang cong offset tu center:

```text
0.0100 + 0.0150 = 0.0250
```

Nhung cach ro nghia hon la them/dung cong thuc top:

```text
pick_z = wood_marker.pose.position.z + wood_marker.scale.z / 2.0 + pick_z_offset_m
pick_z_offset_m default = 0.0
```

## Verification

Da chay:

```bash
python3 -m py_compile robot_gazebo/gazebo/spawn_pick_wood_obstacle_box.py robot_task_manager/scripts/drl_pick_place_wood_box_demo_client.py
```

Da chay spawner-only de lay `[Z_AUDIT][spawner]`.

Chua build C++ va chua chay full Gazebo execute trong turn nay. Khi chay full demo, can grep:

```bash
ros2 launch robot_bringup rl_pick_place_box_gazebo_demo.launch.py execute:=false
```

Va xem:

```text
[Z_AUDIT][spawner]
[Z_AUDIT][client]
[Z_AUDIT][server]
```

