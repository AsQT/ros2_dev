# Gripper ETH Unit Audit Report

## 1. Mục tiêu kiểm tra

Kiểm tra riêng luồng giá trị gripper gửi qua Ethernet/TCP để xác định giá trị cuối cùng có đúng chuẩn mong muốn hay không:

- Đơn vị mong muốn: mm.
- Range mong muốn: `0 -> 50 mm`.
- Chỉ audit, không sửa code/config/firmware.

## 2. Các file đã đọc

- `codex.md`
- `robot_description/urdf/robot.urdf.xacro:10-25`
- `robot_description/urdf/robot.xacro:310-335`
- `robot_description/urdf/ros2_control.xacro:19-98`
- `robot_control/config/robot_controllers.yaml:1-42`
- `robot_control/config/gripper_controller.yaml:1-10`
- `robot_moveit/config/moveit_controllers.yaml:1-26`
- `robot_moveit/config/robot.srdf:9-30`
- `robot_moveit/config/joint_limits.yaml:47-57`
- `robot_task_manager/action/MoveGripper.action:1-8`
- `robot_task_manager/src/move_gripper_server.cpp:19-115`
- `robot_task_manager/src/gripper_executor.cpp:13-180`
- `robot_task_manager/include/robot_task_manager/gripper_executor.hpp:27-39`
- `robot_task_manager/src/pickplace_server.cpp:40-52,211-224,470-568`
- `robot_task_manager/src/drl_pickplace_server.cpp:96-105,652-657,692-755`
- `robot_gui/src/task_action_controller.cpp:69-71,745-765,855-881,966-998`
- `robot_gui/src/robot_gui_node.cpp:77-80,210-218`
- `robot_hardware_interface/CMakeLists.txt:43-49,74-93`
- `robot_hardware_interface/plugin.xml:1-10`
- `robot_hardware_interface/package.xml:35-38`
- `robot_hardware_interface/include/robot_hardware_interface/tcp_client.hpp:14-38,75-80`
- `robot_hardware_interface/include/robot_hardware_interface/tcp_system_hardware.hpp:36-54,108-112`
- `robot_hardware_interface/src/tcp_client.cpp:206-220,224-268,577-639,649-708`
- `robot_hardware_interface/src/tcp_system_hardware.cpp:53-56,313-357,641-705,763-768,929-943,1016-1053,1081-1161`
- `robot_hardware_interface/src/robot_hw_node.cpp:73-76,595-615,692-713,720-738`
- `robot_hardware_interface/src/rs485_protocol.cpp:464-511`
- `robot_hardware_interface/src/rs485_system_hardware.cpp:30-33,285-307,371-396`
- `robot_hardware_interface/config/params.yaml:1-11`

Firmware STM32/MainBoard_ETH parser files such as `Drivers/HARDWARE/ROS_DRIVER/ros_eth_server.c`, `RobotControl.c`, or `scan_lane` were not found in this workspace.

## 3. Luồng dữ liệu gripper từ ROS xuống ETH

Luồng action/task chính:

```text
GUI / action input in mm
  -> robot_gui converts mm to m
  -> /move_gripper goal.position in m
  -> GripperExecutor range check [0.0, 0.05]
  -> MoveIt gripper group joint_gl + joint_gr
  -> gripper_controller FollowJointTrajectory
  -> robot_hardware_interface/RobotSystemHardware
  -> RobotTcpClient::run_all()
  -> TCP command RUN_ALL 0xF3
```

Các bằng chứng chính:

- GUI đổi gripper mm sang m: `robot_gui/src/task_action_controller.cpp:855-881`.
- `MoveGripper.action` chỉ có `float64 position` và `bool execute`: `robot_task_manager/action/MoveGripper.action:1-2`.
- `MoveGripperActionServer` đưa `goal->position` trực tiếp vào `GripperExecutor::moveToOpening`: `robot_task_manager/src/move_gripper_server.cpp:109-115`.
- `GripperExecutor` default range `[0.0, 0.05]` và báo log theo mét: `robot_task_manager/src/gripper_executor.cpp:24-25,116-125,171-177`.
- Nếu gripper group có 2 joint, executor chia đều `opening / joint_values.size()`: `robot_task_manager/src/gripper_executor.cpp:141-156`.
- URDF khai báo `joint_gl` và `joint_gr` là prismatic, limit `-0.001 -> 0.02` m mỗi bên: `robot_description/urdf/robot.xacro:318-335`.
- MoveIt group `gripper` gồm `joint_gl`, `joint_gr`: `robot_moveit/config/robot.srdf:9-11`.
- Controller gripper dùng `joint_trajectory_controller/JointTrajectoryController` với command interface `position`: `robot_control/config/robot_controllers.yaml:9-10,32-39`.
- Real hardware plugin là `robot_hardware_interface/RobotSystemHardware` từ `tcp_system_hardware`: `robot_description/urdf/ros2_control.xacro:27-35`, `robot_hardware_interface/plugin.xml:1-10`.

Nhánh GUI hardware trực tiếp:

- `robot_gui/src/robot_gui_node.cpp:210-218` publish `/gripper_controller/joint_trajectory` với `joint_gl`, `joint_gr`; mỗi joint nhận `width_m / 2.0`.

Nhánh standalone `robot_hw_node`:

- `robot_hardware_interface/config/params.yaml:9-11` default chỉ khai báo `joint_1..joint_6`, `axis_ids [0..5]`, nên khi chạy node standalone theo config này thì không có gripper trong danh sách joint. Tuy nhiên ros2_control real hardware plugin lấy joint từ URDF và mặc định map đủ 8 joint theo thứ tự `0..7`: `robot_hardware_interface/src/tcp_system_hardware.cpp:763-768`.

## 4. Giá trị gripper trong ROS đang là đơn vị gì

Phía ROS/task/gripper đang dùng mét cho action và joint prismatic:

- GUI input hiển thị/nhận mm rồi đổi sang m: `value_m = value_mm / 1000.0` tại `robot_gui/src/task_action_controller.cpp:745-765`.
- Nút gripper gửi `goal.position = target` đã đổi sang m: `robot_gui/src/task_action_controller.cpp:855-881`.
- Default GUI: open `48.0 mm`, close `28.0 mm`, pick `10.0 mm`: `robot_gui/src/task_action_controller.cpp:69-71`.
- `/drl_pickplace` dùng `gripper_open_width_m = 0.05`, `gripper_default_close_width_m = 0.028`: `robot_task_manager/src/drl_pickplace_server.cpp:96-97`.
- `DrlPickPlace` clamp close width về `[0.0, gripper_open_width_m_]`: `robot_task_manager/src/drl_pickplace_server.cpp:652-657`.
- `PickPlace` default open gripper là `0.048`: `robot_task_manager/src/pickplace_server.cpp:40-42`.

Kết luận phần ROS: gripper command ở action/controller là mét, không phải raw mm. GUI có input mm nhưng convert sang m trước khi gửi action/controller.

## 5. Giá trị gripper khi đóng gói TCP đang là đơn vị gì

Trong real ros2_control TCP plugin, 6 joint đầu được xử lý là joint quay rad/deg, còn index `>= 6` có logic riêng:

- Helper trong TCP plugin:
  - `deg2rad(deg)`
  - `rad2deg(rad)`
  - `deg2met(deg) { return deg / 2; }`
  - `met2deg(rad) { return rad * 2; }`
  tại `robot_hardware_interface/src/tcp_system_hardware.cpp:53-56`.
- Khi đọc state cho index `>= 6`, code dùng `hw_pos_[i] = deg2met(pos_deg)`: `robot_hardware_interface/src/tcp_system_hardware.cpp:1031-1053`.
- Khi ghi command cho index `>= 6`, code dùng `pos_deg[i] = met2deg(rad_driver)`: `robot_hardware_interface/src/tcp_system_hardware.cpp:1145-1155`.
- Sau đó `RobotTcpClient::run_all()` pack `pos_deg[i] * 1000.0` thành `int32`: `robot_hardware_interface/src/tcp_client.cpp:618-639`.

Vì `met2deg(x) = x * 2`, với action path chuẩn:

```text
opening_m = W
GripperExecutor đặt joint_gl = W / 2
GripperExecutor đặt joint_gr = W / 2
tcp_system_hardware với i>=6: pos_deg = joint_m * 2 = W
tcp_client payload int32 = round(pos_deg * 1000) = round(W * 1000)
```

Do đó nếu đi qua `/move_gripper`, payload numeric cho mỗi axis gripper thường bằng `opening_m * 1000`, tức bằng trị số mm của tổng độ mở. Ví dụ:

- `0.028 m` close width -> mỗi joint `0.014 m` -> `met2deg=0.028` -> payload `28`.
- `0.048 m` open width -> mỗi joint `0.024 m` -> `met2deg=0.048` -> payload `48`.
- `0.050 m` -> mỗi joint `0.025 m` -> `met2deg=0.050` -> payload `50`.

Tuy nhiên code/protocol vẫn đặt tên và pack theo đường `deg/mdeg`, không có field/unit riêng là `mm`. Nếu command đến trực tiếp như joint trajectory với `joint_gl/joint_gr` không phải nửa tổng width, payload sẽ là `joint_position_m * 2 * 1000`, không phải từng joint mm.

## 6. Payload ETH gửi xuống STM32

- Command id: `RUN_ALL = 0xF3`, định nghĩa tại `robot_hardware_interface/include/robot_hardware_interface/tcp_client.hpp:22-25`.
- TCP frame header:
  - magic `0x55AA` little-endian, wire bytes `AA 55`
  - cmd 1 byte
  - seq 2 bytes little-endian
  - payload length 2 bytes little-endian
  - payload bắt đầu sau header 7 bytes
  - Xem `robot_hardware_interface/src/tcp_client.cpp:206-220`.
- Payload `RUN_ALL`:
  - 8 axes.
  - Mỗi axis 8 bytes: `position int32 little-endian` + `velocity uint32 little-endian`.
  - Payload length: `8 * (4 + 4) = 64 bytes`.
  - Pack tại `robot_hardware_interface/src/tcp_client.cpp:618-639`.
- Offset nếu axis mapping mặc định từ URDF được dùng:
  - `joint_gl` = axis 6: position payload offset `48..51`, velocity payload offset `52..55`; frame offset `55..58` và `59..62`.
  - `joint_gr` = axis 7: position payload offset `56..59`, velocity payload offset `60..63`; frame offset `63..66` và `67..70`.
- Scale:
  - `position_payload = round(pos_deg * 1000.0)`.
  - Với gripper trong `tcp_system_hardware`: `pos_deg = joint_position_m * 2`.
  - Với action path `/move_gripper`: `joint_position_m = opening_m / 2`, nên `position_payload = round(opening_m * 1000.0)`.
- Range:
  - Action executor range: `0.0 -> 0.05 m`, tức `0 -> 50 mm`: `robot_task_manager/src/gripper_executor.cpp:24-25,116-125`.
  - TCP packer clamp chung: `-280.0 -> 280.0` trước khi nhân `1000`: `robot_hardware_interface/src/tcp_client.cpp:629-633`.
  - Không thấy clamp riêng `0 -> 50` ở ETH layer.
  - URDF prismatic limit mỗi joint là `-0.001 -> 0.02 m`: `robot_description/urdf/robot.xacro:318-335`, tương đương tổng mở hình học khoảng `0 -> 40 mm` nếu hai joint mở đối xứng. SRDF open state là `0.0188 + 0.0193 = 0.0381 m`: `robot_moveit/config/robot.srdf:28-30`.
- Velocity:
  - `tcp_system_hardware.cpp:1153-1155` dùng `vel_deg_s[i] = met2deg(i)` cho gripper index `>=6`; đây là theo index `i`, không theo `cmd_vel_[i]`. Kết quả axis 6 velocity payload khoảng `12000`, axis 7 khoảng `14000`. Đây là điểm nghi ngờ sai, nhưng không phải trường position unit.

## 7. Firmware nhận và hiểu giá trị gripper là đơn vị gì

Không đủ dữ liệu để xác nhận phía firmware.

Các file firmware/parser được yêu cầu trong `codex.md` không có trong workspace:

- `Drivers/HARDWARE/ROS_DRIVER/ros_eth_server.c`: không tìm thấy.
- `RobotControl.c`: không tìm thấy.
- `MainBoard_ETH`: không tìm thấy.
- `scan_lane` / parser STM32: không tìm thấy.

Do đó chưa xác nhận được:

- STM32 parser hiểu `RUN_ALL 0xF3` axis 6/7 là mm, mdeg, tick, pulse, percent hay raw.
- STM32 lưu vào `JOINT[6].CMD.abs_pos_set` / `JOINT[7].CMD.abs_pos_set` hay biến riêng.
- STM32 có đổi đơn vị thêm trước khi gửi xuống driver gripper hay không.

Phía ROS/TCP chỉ chứng minh được payload numeric cho action path thường bằng trị số mm, nhưng protocol code vẫn dùng hàm/field kiểu degree/milli-degree.

## 8. JOINT[6] / JOINT[7] có xử lý đặc biệt hay bị xử lý như joint quay

Trong ROS hardware plugin:

- `joint_gl` và `joint_gr` được xử lý đặc biệt theo index `i >= 6` trong `tcp_system_hardware.cpp`.
- Đọc state: axis 6/7 dùng `deg2met(pos_deg)` thay vì `deg2rad(pos_deg)`: `robot_hardware_interface/src/tcp_system_hardware.cpp:1031-1053`.
- Ghi command: axis 6/7 dùng `met2deg(rad_driver)` thay vì `rad2deg(rad_driver)`: `robot_hardware_interface/src/tcp_system_hardware.cpp:1145-1155`.
- Tuy nhiên sau bước này, `RobotTcpClient::run_all()` vẫn pack axis 6/7 giống mọi axis khác: `int32 round(pos * 1000)` + `uint32 round(vel * 1000)`: `robot_hardware_interface/src/tcp_client.cpp:618-639`.

Trong firmware:

- Chưa xác nhận được `JOINT[6]` / `JOINT[7]` có logic riêng hay bị xử lý như joint quay, vì thiếu firmware parser.

## 9. So sánh với chuẩn mong muốn 0 -> 50 mm

So sánh theo từng lớp:

- GUI/task action:
  - Có input mm và convert sang m.
  - Range action executor là `0.0 -> 0.05 m`.
  - Phù hợp về mặt API người dùng, nhưng đơn vị nội bộ ROS là m.
- MoveIt/URDF:
  - `joint_gl/joint_gr` là prismatic, đơn vị m.
  - Mỗi joint limit `upper=0.02 m`, nên tổng mở hình học khoảng `40 mm`, không phải `50 mm`.
  - SRDF open state khoảng `38.1 mm`.
- TCP hardware plugin:
  - Có logic riêng cho axis 6/7 để đổi m sang pseudo-degree `x * 2`.
  - Với action path chia đôi gripper, payload numeric position bằng `opening_mm`.
  - Không có field/protocol rõ ràng ghi là mm; code đang pack theo mdeg-style `int32 * 1000`.
  - Không có clamp riêng `0 -> 50` trong `run_all`, chỉ có clamp chung `-280 -> 280`.
- Firmware:
  - Chưa kiểm tra được.

## 10. Kết luận

Kết luận C:

Không đủ dữ liệu để xác nhận phía firmware vì thiếu file firmware. Phía ROS hiện tại đang gửi gripper action/controller bằng mét. Khi đi qua `tcp_system_hardware` và `RobotTcpClient::run_all`, payload position cho axis 6/7 là `int32 little-endian` theo công thức:

```text
payload_pos = round((joint_position_m * 2) * 1000)
```

Với luồng `/move_gripper` chuẩn, vì `GripperExecutor` chia tổng độ mở cho 2 joint, công thức rút gọn thành:

```text
payload_pos = round(opening_m * 1000)
```

Nghĩa là trị số payload thường bằng số mm của tổng độ mở gripper, ví dụ `0.028 m -> 28`, `0.048 m -> 48`, `0.050 m -> 50`.

Nhưng không thể kết luận “gripper qua ETH hiện đang gửi đúng đơn vị mm, range 0->50” vì:

- Protocol code vẫn dùng đường `deg/mdeg` chung cho `RUN_ALL`, không có command/field gripper-mm riêng.
- Firmware parser không có trong workspace để xác nhận nó hiểu axis 6/7 là mm.
- ETH layer không clamp riêng `0 -> 50`; clamp chung là `-280 -> 280`.
- URDF/MoveIt limit hiện tại cho `joint_gl/joint_gr` là `0.02 m` mỗi bên, tương đương tổng khoảng `40 mm`, không khớp hoàn toàn với chuẩn mong muốn `50 mm`.
- Velocity gripper trong `tcp_system_hardware` có dấu hiệu nghi ngờ: `vel_deg_s[i] = met2deg(i)` dùng index thay vì command velocity.

Nguy cơ gây lỗi gripper:

- Nếu firmware đang hiểu axis 6/7 là mdeg giống joint quay, payload `28/48/50` sẽ bị hiểu là `0.028/0.048/0.050 deg`, không phải `28/48/50 mm`.
- Nếu firmware đang hiểu axis 6/7 là raw/tick/pulse, chưa có mapping xác nhận nên giá trị có thể sai scale.
- Nếu MoveIt limit `0.02 m` mỗi bên được enforce, lệnh mở `50 mm` có thể vượt limit hình học dù action executor cho phép `0.05 m`.

## 11. Đề xuất sửa ở task sau

- Bổ sung hoặc đưa firmware STM32/MainBoard_ETH vào workspace để audit parser `RUN_ALL 0xF3`.
- Xác nhận firmware axis 6/7 nhận `int32` payload là `mm`, `mdeg`, tick/pulse hay raw.
- Nếu chuẩn thật là `0 -> 50 mm`, nên làm rõ bằng tên hàm/biến ở hardware layer thay vì `met2deg/deg2met`.
- Thêm clamp rõ ràng `0 -> 50` cho gripper trước khi pack TCP, nếu firmware yêu cầu mm.
- Đồng bộ URDF/MoveIt gripper limit với chuẩn `50 mm` hoặc ghi rõ cơ khí chỉ mở khoảng `40 mm`.
- Kiểm tra và sửa riêng velocity gripper `vel_deg_s[i] = met2deg(i)` nếu đó là lỗi ngoài ý muốn.
