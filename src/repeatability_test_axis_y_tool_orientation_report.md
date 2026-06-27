# RepeatabilityTest Axis Y Tool Orientation Report

Ngày thực hiện: 2026-06-27

## 1. File đã sửa

- `robot_task_manager/src/repeatability_test_server.cpp`
- `robot_task_manager/CMakeLists.txt`
- `robot_task_manager/Call_action.md`

## 2. Logic cũ

`RepeatabilityTest` tính `meas_pose` từ `retract_pose` bằng cách dịch position theo trục được chọn:

- `axis=0`: đổi `x`
- `axis=1`: đổi `y`
- `axis=2`: đổi `z`

Orientation của `meas_pose` gần như luôn giữ nguyên từ `retract_pose`, nên khi đo theo trục Y tool không đổi hướng phù hợp hướng đo.

## 3. Logic mới

- `axis=0` (`AXIS_X`): dịch `x`, giữ nguyên orientation như `retract_pose`.
- `axis=1` (`AXIS_Y`): dịch `y`, xoay orientation tool thêm `axis_y_tool_yaw_offset_rad` quanh trục Z tool.
- `axis=2` (`AXIS_Z`): dịch `z`, giữ nguyên orientation như `retract_pose`.

Default `axis_y_tool_yaw_offset_rad` là:

```text
1.5707963267948966 rad (+90 deg)
```

## 4. Công thức quaternion

Với axis Y:

```cpp
tf2::Quaternion q_retract;
tf2::fromMsg(retract_orientation, q_retract);

tf2::Quaternion q_yaw_offset;
q_yaw_offset.setRPY(0.0, 0.0, axis_y_tool_yaw_offset_rad_);

tf2::Quaternion q_result = q_retract * q_yaw_offset;
q_result.normalize();
meas_pose.orientation = tf2::toMsg(q_result);
```

Phép nhân `q_retract * q_yaw_offset` áp dụng yaw offset theo local/tool frame.

## 5. Parameter mới

Có thêm parameter:

```text
axis_y_tool_yaw_offset_rad
```

Default:

```text
1.5707963267948966
```

Nếu parameter không finite, server dùng lại default trên và ghi warning.

## 6. Kết quả build

Lệnh:

```bash
cd /home/minhquang/ros2_dev
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_task_manager --symlink-install
```

Kết quả:

```text
Summary: 1 package finished
```

Lần build đầu tiên phát hiện target `repeatability_test_server` thiếu dependency `tf2`/`tf2_geometry_msgs`; đã bổ sung vào `CMakeLists.txt`. Build cuối pass.

## 7. Kết quả test

Launch test:

```bash
ROS_DOMAIN_ID=53 ros2 launch robot_task_manager task_servers.launch.py enable_drl_backend:=false
```

### Axis X execute=false

Goal được accept và server log:

```text
RepeatabilityTest start | mode=plan-only | axis=0 | offset=0.0200 | ... | axis_y_yaw_offset=1.570796
RepeatabilityTest meas_pose | position=(0.4200, 0.0000, 0.1800) | orientation=(0.7071068, 0.7071068, 0.0000000, 0.0000000)
```

Kết luận: axis X dịch X và giữ nguyên orientation.

### Axis Y execute=false

Goal được accept và server log:

```text
RepeatabilityTest start | mode=plan-only | axis=1 | offset=0.0200 | ... | axis_y_yaw_offset=1.570796
RepeatabilityTest meas_pose | position=(0.4000, 0.0200, 0.1800) | orientation=(1.0000000, 0.0000000, 0.0000000, 0.0000000)
```

Với retract orientation `(0.7071068, 0.7071068, 0.0, 0.0)`, kết quả `(1.0, 0.0, 0.0, 0.0)` xác nhận đã nhân thêm yaw +90 quanh Z tool.

### Axis Z execute=false

Goal được accept và server log:

```text
RepeatabilityTest start | mode=plan-only | axis=2 | offset=-0.0200 | ... | axis_y_yaw_offset=1.570796
RepeatabilityTest meas_pose | position=(0.4000, 0.0000, 0.1600) | orientation=(0.7071068, 0.7071068, 0.0000000, 0.0000000)
```

Kết luận: axis Z dịch Z và giữ nguyên orientation.

### Axis Y execute=true

Goal được accept và server log:

```text
RepeatabilityTest start | mode=execute | axis=1 | offset=0.0200 | ... | axis_y_yaw_offset=1.570796
RepeatabilityTest meas_pose | position=(0.4000, 0.0200, 0.1800) | orientation=(1.0000000, 0.0000000, 0.0000000, 0.0000000)
```

Kết luận: execute path cũng dùng orientation Y đã xoay +90. Trong môi trường test hiện tại, action không hoàn tất do sub-action MoveToPose không trả result trước timeout CLI; vì vậy không xác nhận motion thật của robot trong phiên này.

## 8. Action interface

Không đổi action interface:

```text
robot_task_manager/action/RepeatabilityTest
/repeatability_test
```

Không sửa file `.action`.

## 9. Không ảnh hưởng action khác

Thay đổi chỉ nằm trong logic tính `meas_pose` của `repeatability_test_server` và dependency target CMake cho server này. Không đổi flow:

1. MoveToPose tới `retract_pose`
2. Cartesian tới `meas_pose`
3. Wait tại điểm đo
4. Cartesian về `retract_pose`
5. MoveToPose tới `disturb_pose_1`
6. MoveToPose về `retract_pose`

Các action khác không đổi logic.

## 10. Ghi chú test

Khi dừng launch bằng Ctrl-C trong lúc các goal test còn chờ sub-action, một số server MoveIt hiện có in `RCLError`/`Broken promise` ở pha shutdown. Đây là artifact của việc ngắt test giữa chừng, không liên quan tới logic orientation đã sửa.
