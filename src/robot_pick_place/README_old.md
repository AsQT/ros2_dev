# robot_pick_place

GUI PyQt6 tối giản để hiển thị ArUco và gọi action `robot_task_manager/action/PickPlace`.

## Chạy

```bash
ros2 launch robot_pick_place pick_place_gui.launch.py
```

## Topic/action mặc định

- Image: `/aruco/image_annotated`
- Pose: `/aruco_pose`
- Action: `/pickplace`

## Tham số quan trọng

Mặc định GUI dùng orientation cố định cho pick pose, không dùng nguyên quaternion của marker ArUco.
Điều này thường ổn hơn vì quaternion của marker là hướng của mặt marker, không nhất thiết là orientation hợp lệ của TCP/gripper.

```bash
ros2 launch robot_pick_place pick_place_gui.launch.py \
  use_fixed_pick_orientation:=true \
  pick_qx:=0.7071 pick_qy:=0.7071 pick_qz:=0.0 pick_qw:=0.0 \
  place_x:=0.300 place_y:=0.000 place_z:=0.250 \
  place_qx:=0.7071 place_qy:=0.7071 place_qz:=0.0 place_qw:=0.0 \
  gripper:=0.010 velocity_scale:=0.30
```

Nếu muốn thử dùng quaternion từ `/aruco_pose`:

```bash
ros2 launch robot_pick_place pick_place_gui.launch.py use_fixed_pick_orientation:=false
```

Nếu MoveIt fail ở bước approach do z quá thấp, có thể thử tăng tạm z pick gửi vào action:

```bash
ros2 launch robot_pick_place pick_place_gui.launch.py pick_z_offset:=0.03
```

Lưu ý: `PickPlace.action` thường tự tạo approach bằng `pose_pick.z + 0.1`, nên tăng `pick_z_offset` cũng làm điểm gắp cuối cao hơn. Chỉ dùng để debug vùng workspace/collision.
