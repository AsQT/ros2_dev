Yêu cầu Codex sửa package `robot_gui` C++ hiện tại: phần joint state chưa được hiển thị trên GUI. Cần tham khảo `robot_gui_old` để port lại đúng logic.

## 1. Vấn đề hiện tại

Trong GUI C++ mới:

```text
robot_gui
```

tab Robot/Hardware hiện chưa hiển thị được trạng thái joint hiện tại.

Yêu cầu:

```text
- Tham khảo robot_gui_old.
- Port lại logic hiển thị joint state sang C++.
- Dùng layout từ robot_gui/ui/robot_gui.ui.
- Không tự tạo layout/widget mới.
- Không sửa robot_gui.ui nếu không thật sự cần.
```

Package cũ cần tham khảo:

```text
robot_gui_old
```

Package mới cần sửa:

```text
robot_gui
```

---

## 2. File cần đọc kỹ

Đọc logic cũ trong:

```text
robot_gui_old/robot_gui/main_window.py
robot_gui_old/robot_gui/gui_win.md
robot_gui_old/ui/robot_gui.ui
```

Đọc GUI C++ hiện tại:

```text
robot_gui/ui/robot_gui.ui
robot_gui/src/main_window.cpp
robot_gui/include/robot_gui/main_window.hpp
robot_gui/src/robot_gui_node.cpp
robot_gui/include/robot_gui/robot_gui_node.hpp
```

Mục tiêu là xác định:

```text
- robot_gui_old subscribe topic nào để lấy joint state
- message type gì
- map joint nào vào ô hiển thị nào
- đơn vị hiển thị là rad hay deg
- objectName của các QLabel/QLineEdit/LCDNumber trong .ui
```

Không được đoán objectName.

---

## 3. Topic joint state cần subscribe

Cần subscribe topic chuẩn:

```text
/joint_states
```

Message:

```text
sensor_msgs/msg/JointState
```

C++ include:

```cpp
#include <sensor_msgs/msg/joint_state.hpp>
```

Logic callback:

```cpp
void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
```

---

## 4. Mapping joint name

Cần map theo tên joint trong message, không chỉ dựa vào index nếu có thể.

Các joint cần hiển thị tối thiểu:

```text
joint_1
joint_2
joint_3
joint_4
joint_5
joint_6
joint_gl
joint_gr
```

Nếu GUI chỉ hiển thị 6 trục chính thì:

```text
joint_1 -> Axis 1
joint_2 -> Axis 2
joint_3 -> Axis 3
joint_4 -> Axis 4
joint_5 -> Axis 5
joint_6 -> Axis 6
```

Không phụ thuộc thứ tự mảng nếu message có name khác thứ tự.

Pseudo logic:

```cpp
for (size_t i = 0; i < msg->name.size(); ++i) {
    const auto &name = msg->name[i];
    if (name == "joint_1") axis = 0;
    ...
}
```

---

## 5. Đơn vị hiển thị

Codex phải kiểm tra `robot_gui_old` đang hiển thị đơn vị gì.

Nếu `/joint_states.position` là radian theo chuẩn ROS, nhưng GUI cũ hiển thị độ, thì phải convert:

```cpp
deg = rad * 180.0 / M_PI;
```

Nếu old GUI đã hiển thị rad thì giữ rad.

Yêu cầu trong report ghi rõ:

```text
- Input /joint_states.position unit:
- GUI display unit:
- Conversion used:
```

Không tự ý đổi đơn vị.

---

## 6. Giá trị cần hiển thị

Tối thiểu hiển thị:

```text
- actual position từng joint
```

Nếu trong `.ui` có ô velocity thì hiển thị thêm:

```text
- velocity từng joint nếu msg.velocity có dữ liệu
```

Nếu trong `.ui` có effort thì hiển thị thêm:

```text
- effort nếu msg.effort có dữ liệu
```

Nếu msg không có velocity/effort hoặc size không đủ:

```text
- không crash
- hiển thị "--" hoặc giữ giá trị cũ
- log debug/throttle warning nếu cần
```

---

## 7. Dùng widget có sẵn trong robot_gui.ui

Không được tạo widget mới.

Phải tìm đúng objectName trong:

```text
robot_gui/ui/robot_gui.ui
```

Ví dụ cần tìm các ô kiểu:

```text
Axis1Actual
Axis1Current
txtAxis1Actual
lineEditAxis1Actual
labelAxis1Actual
lcdAxis1Pos
```

Nhưng không được đoán. Phải đọc file `.ui`.

Nếu không tìm thấy widget tương ứng:

```cpp
RCLCPP_WARN(logger, "Joint state widget not found: <objectName>");
```

Không tự tạo widget thay thế.

---

## 8. Thread-safe GUI update

Nếu ROS executor chạy thread riêng, không update Qt widget trực tiếp trong callback.

Dùng Qt signal-slot:

```text
ROS callback /joint_states
    -> emit jointStateUpdated(...)
    -> Qt slot updateJointStateDisplay(...) trên main thread
```

Nếu hiện tại GUI dùng QTimer `spin_some()` trên Qt main thread thì có thể update trực tiếp, nhưng vẫn phải đảm bảo không block GUI.

---

## 9. Format hiển thị

Format đề xuất:

```text
position: 3 chữ số thập phân
velocity: 3 chữ số thập phân nếu có
```

Ví dụ:

```cpp
QString::number(value, 'f', 3)
```

Nếu hiển thị degree:

```text
deg
```

Nếu hiển thị rad:

```text
rad
```

Không làm thay đổi field input setpoint của người dùng, chỉ cập nhật field actual/current.

---

## 10. Không nhầm với flags

Phần `/robot_hw/flags` chỉ dùng cho LED trạng thái.

Phần `/joint_states` dùng cho vị trí/velocity thực tế.

Không dùng `status_f` để hiển thị joint position.

Không dùng `/robot_hw/flags` thay cho `/joint_states`.

---

## 11. Test bắt buộc

Build:

```bash
cd ~/ros2_dev
colcon build --packages-select robot_gui --event-handlers console_direct+
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Chạy GUI standalone:

```bash
ros2 launch robot_gui robot_gui.launch.py embed_rviz:=false
```

Trong terminal khác, publish thử joint state mock:

```bash
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
  name: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_gl','joint_gr'],
  position: [0.1,0.2,0.3,0.4,0.5,0.6,0.01,0.01],
  velocity: [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],
  effort: []
}" -r 5
```

Kỳ vọng:

```text
- GUI không crash.
- Axis 1..6 actual/current position cập nhật.
- Nếu GUI dùng degree thì 0.1 rad hiển thị khoảng 5.730 deg.
- Nếu GUI dùng rad thì hiển thị 0.100.
```

Test với launch MoveIt:

```bash
ros2 launch robot_moveit moveit_gui.launch.py
```

Kỳ vọng:

```text
- /joint_states có dữ liệu.
- GUI hiển thị joint state.
- RViz nếu bật vẫn hoạt động.
```

---

## 12. Test objectName mapping

Thêm log runtime hoặc report:

```text
Joint state display widgets:
Axis1 position widget: found yes/no
Axis2 position widget: found yes/no
Axis3 position widget: found yes/no
Axis4 position widget: found yes/no
Axis5 position widget: found yes/no
Axis6 position widget: found yes/no
```

Nếu widget missing, ghi rõ objectName thiếu.

---

## 13. Không phá phần khác

Không làm hỏng:

```text
- layout từ robot_gui.ui
- ảnh Logo.png
- embedded RViz native
- moveit_gui.launch.py
- /robot_hw/flags LED
- /robot_hw/servo_all
- tab Robot button mapping
```

Không sửa `.ui` nếu chưa cần.

---

## 14. Report

Tạo/cập nhật:

```text
robot_gui/joint_state_display_report.md
```

Nội dung bắt buộc:

```markdown
# Joint State Display Report

## 1. Source reference
- robot_gui_old files checked:

## 2. Topic
- Topic:
- Message:
- Callback file:

## 3. Joint mapping
| Joint name | Axis | Widget objectName | Found | Unit | Test value |
|---|---|---|---|---|---|

## 4. Unit conversion
- Input unit:
- Display unit:
- Conversion:

## 5. Threading
- ROS spin method:
- GUI update method:

## 6. Test result
- Build:
- Mock /joint_states publish:
- MoveIt launch:
- Remaining issues:
```

---

## 15. Output cuối cùng

Output ngắn gọn:

```text
Đã port hiển thị joint state từ robot_gui_old sang robot_gui C++.
Topic: /joint_states
Message: sensor_msgs/msg/JointState
Axis 1..6 display: OK/FAIL
Unit: rad/deg
Mock publish test: OK/FAIL
Report: robot_gui/joint_state_display_report.md
```
