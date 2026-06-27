Yêu cầu sửa tiếp action `RepeatabilityTest` trong `robot_task_manager`.

Hiện tượng:
Sau khi sửa `axis=Y` để `meas_pose` xoay tool thêm 90 độ, khi chọn trục Y vẫn plan Cartesian lỗi. Trục X thì plan đúng.

Nguyên nhân:
Hiện mới chỉ đổi orientation của `meas_pose`, còn `retract_pose` và `disturb_pose_1` vẫn giữ orientation cũ. Flow repeatability có đoạn Cartesian:

```text
retract_pose -> meas_pose -> retract_pose
```

Nếu `meas_pose` xoay 90 độ nhưng `retract_pose` không xoay, Cartesian segment phải vừa dịch chuyển vừa đổi orientation 90 độ trong đoạn ngắn, dễ làm `computeCartesianPath()` fail.

Mục tiêu:
Khi `axis == AXIS_Y`, không chỉ `meas_pose`, mà cả các pose dùng trong flow phải đổi sang cùng hướng tool xoay 90 độ:

* `retract_pose`
* `meas_pose`
* `disturb_pose_1`

Tức là với trục Y, toàn bộ chu trình đo phải dùng orientation đã xoay 90 độ, để các đoạn Cartesian có orientation nhất quán.

============================================================

1. LOGIC MONG MUỐN
   ============================================================

Hiện tại có thể đang có logic kiểu:

```cpp
auto retract_pose = goal->retract_pose.pose;
auto disturb_pose_1 = goal->disturb_pose_1.pose;
auto meas_pose = retract_pose;

if (goal->axis == RepeatabilityTest::Goal::AXIS_X) {
  meas_pose.position.x = retract_pose.position.x + goal->meas_offset;
} else if (goal->axis == RepeatabilityTest::Goal::AXIS_Y) {
  meas_pose.position.y = retract_pose.position.y + goal->meas_offset;
  // mới chỉ xoay meas_pose ở đây
} else {
  meas_pose.position.z = retract_pose.position.z + goal->meas_offset;
}
```

Cần đổi thành:

```cpp
auto retract_pose = goal->retract_pose.pose;
auto disturb_pose_1 = goal->disturb_pose_1.pose;

auto working_retract_pose = retract_pose;
auto working_disturb_pose_1 = disturb_pose_1;
auto meas_pose = working_retract_pose;

if (goal->axis == RepeatabilityTest::Goal::AXIS_Y) {
  const auto rotated_orientation = rotateYaw90(retract_pose.orientation);

  working_retract_pose.orientation = rotated_orientation;
  working_disturb_pose_1.orientation = rotated_orientation;
  meas_pose.orientation = rotated_orientation;
}
```

Sau đó mới tính vị trí đo:

```cpp
if (goal->axis == RepeatabilityTest::Goal::AXIS_X) {
  meas_pose.position.x = working_retract_pose.position.x + goal->meas_offset;
} else if (goal->axis == RepeatabilityTest::Goal::AXIS_Y) {
  meas_pose.position.y = working_retract_pose.position.y + goal->meas_offset;
} else {
  meas_pose.position.z = working_retract_pose.position.z + goal->meas_offset;
}
```

Và toàn bộ flow phía sau phải dùng:

```text
working_retract_pose
meas_pose
working_disturb_pose_1
```

Không được dùng lại `retract_pose` / `disturb_pose_1` gốc trong các lệnh move sau khi đã tạo pose làm việc.

============================================================
2. HÀM XOAY ORIENTATION 90 ĐỘ
=============================

Tạo helper rõ ràng, ví dụ:

```cpp
geometry_msgs::msg::Quaternion rotateToolYaw(
  const geometry_msgs::msg::Quaternion& input,
  double yaw_offset_rad)
{
  tf2::Quaternion q_input;
  tf2::fromMsg(input, q_input);

  tf2::Quaternion q_offset;
  q_offset.setRPY(0.0, 0.0, yaw_offset_rad);

  tf2::Quaternion q_result = q_input * q_offset;
  q_result.normalize();

  return tf2::toMsg(q_result);
}
```

Dùng:

```cpp
const auto rotated_orientation =
  rotateToolYaw(retract_pose.orientation, axis_y_tool_yaw_offset_rad_);
```

Default:

```cpp
axis_y_tool_yaw_offset_rad_ = 1.5707963267948966;
```

Nếu project đã thêm parameter `axis_y_tool_yaw_offset_rad` thì dùng parameter đó. Nếu chưa có thì thêm parameter này.

============================================================
3. FLOW SAU KHI SỬA
===================

Flow mới phải dùng pose đã được chuẩn hóa orientation:

```text
1. MoveToPose tới working_retract_pose
2. Mỗi loop:
   - Cartesian từ working_retract_pose tới meas_pose
   - Wait measurement_settle_time_s
   - Cartesian từ meas_pose quay về working_retract_pose
   - MoveToPose tới working_disturb_pose_1
   - MoveToPose quay về working_retract_pose
```

Với axis X:

* `working_retract_pose.orientation = retract_pose.orientation`
* `working_disturb_pose_1.orientation = disturb_pose_1.orientation`
* `meas_pose.orientation = retract_pose.orientation`

Với axis Y:

* `working_retract_pose.orientation = rotated_orientation`
* `working_disturb_pose_1.orientation = rotated_orientation`
* `meas_pose.orientation = rotated_orientation`

Với axis Z:

* `working_retract_pose.orientation = retract_pose.orientation`
* `working_disturb_pose_1.orientation = disturb_pose_1.orientation`
* `meas_pose.orientation = retract_pose.orientation`

============================================================
4. ĐIỂM QUAN TRỌNG
==================

Khi `axis=Y`, không được để các đoạn này có orientation lệch nhau:

```text
working_retract_pose -> meas_pose
meas_pose -> working_retract_pose
```

Hai pose này phải cùng orientation đã xoay 90 độ.

Nếu dùng action con `/move_to_pose` cho `working_retract_pose` và `working_disturb_pose_1`, cũng phải truyền pose đã xoay.

Nếu dùng `/move_to_pose_cartesian` cho `meas_pose`, cũng phải truyền pose đã xoay.

Không sửa action interface.

Không đổi:

* action name `/repeatability_test`
* action type
* field goal
* logic position X/Y/Z
* velocity_scale / fast_velocity_scale
* execute=true / execute=false behavior

============================================================
5. TEST BẮT BUỘC
================

Build:

```bash
cd ~/ros2_dev
colcon build --packages-select robot_task_manager
source install/setup.bash
```

Launch server:

```bash
ros2 launch robot_task_manager task_servers.launch.py
```

Test axis X plan-only, phải vẫn đúng như cũ:

```bash
ros2 action send_goal /repeatability_test robot_task_manager/action/RepeatabilityTest \
  "{retract_pose: {header: {frame_id: 'world'}, pose: {position: {x: 0.40, y: 0.00, z: 0.18}, orientation: {x: 0.7071068, y: 0.7071068, z: 0.0, w: 0.0}}}, disturb_pose_1: {header: {frame_id: 'world'}, pose: {position: {x: 0.35, y: -0.08, z: 0.18}, orientation: {x: 0.7071068, y: 0.7071068, z: 0.0, w: 0.0}}}, axis: 0, meas_offset: 0.02, repeat_count: 1, velocity_scale: 0.15, execute: false}" \
  --feedback
```

Test axis Y plan-only:

```bash
ros2 action send_goal /repeatability_test robot_task_manager/action/RepeatabilityTest \
  "{retract_pose: {header: {frame_id: 'world'}, pose: {position: {x: 0.40, y: 0.00, z: 0.18}, orientation: {x: 0.7071068, y: 0.7071068, z: 0.0, w: 0.0}}}, disturb_pose_1: {header: {frame_id: 'world'}, pose: {position: {x: 0.35, y: -0.08, z: 0.18}, orientation: {x: 0.7071068, y: 0.7071068, z: 0.0, w: 0.0}}}, axis: 1, meas_offset: 0.02, repeat_count: 1, velocity_scale: 0.15, execute: false}" \
  --feedback
```

Kỳ vọng axis Y:

* `working_retract_pose`, `meas_pose`, `working_disturb_pose_1` đều cùng orientation xoay 90 độ.
* Cartesian từ retract xuống meas plan được.
* Cartesian từ meas về retract plan được.
* Không còn fail do orientation đổi đột ngột giữa retract/meas.

Test axis Y execute thật:

```bash
ros2 action send_goal /repeatability_test robot_task_manager/action/RepeatabilityTest \
  "{retract_pose: {header: {frame_id: 'world'}, pose: {position: {x: 0.40, y: 0.00, z: 0.18}, orientation: {x: 0.7071068, y: 0.7071068, z: 0.0, w: 0.0}}}, disturb_pose_1: {header: {frame_id: 'world'}, pose: {position: {x: 0.35, y: -0.08, z: 0.18}, orientation: {x: 0.7071068, y: 0.7071068, z: 0.0, w: 0.0}}}, axis: 1, meas_offset: 0.02, repeat_count: 1, velocity_scale: 0.15, execute: true}" \
  --feedback
```

Kỳ vọng:

* Robot đi tới retract với tool đã xoay 90 độ.
* Cartesian tới meas theo Y thành công.
* Quay về retract thành công.
* Đi tới disturb_pose_1 với cùng hướng tool xoay 90 độ.
* Không fail Cartesian do mismatch orientation.

============================================================
6. CẬP NHẬT TÀI LIỆU
====================

Cập nhật `robot_task_manager/Call_action.md` phần `/repeatability_test`:

Ghi rõ:

* `axis=0`: dịch X, giữ orientation goal gốc.
* `axis=1`: dịch Y, toàn bộ pose trong chu trình đo dùng orientation xoay thêm `axis_y_tool_yaw_offset_rad`, default `+90 deg`.

  * `retract_pose`
  * `meas_pose`
  * `disturb_pose_1`
* `axis=2`: dịch Z, giữ orientation goal gốc.

============================================================
7. BÁO CÁO
==========

Tạo file:

```text
repeatability_test_axis_y_all_pose_orientation_report.md
```

Nội dung bắt buộc:

1. Nguyên nhân axis Y fail Cartesian.
2. File đã sửa.
3. Logic cũ: chỉ xoay `meas_pose`.
4. Logic mới: xoay cả `working_retract_pose`, `meas_pose`, `working_disturb_pose_1`.
5. Giá trị yaw offset dùng cho axis Y.
6. Có dùng parameter `axis_y_tool_yaw_offset_rad` không.
7. Kết quả build.
8. Kết quả test:

   * axis X execute=false
   * axis Y execute=false
   * axis Y execute=true
9. Xác nhận không đổi action interface.
10. Xác nhận không ảnh hưởng axis X/Z.
