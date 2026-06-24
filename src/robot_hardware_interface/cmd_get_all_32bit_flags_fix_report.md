# CMD_GET_ALL 32-bit Flags Fix Report

## 1. File đã sửa

- `robot_hardware_interface/include/robot_hardware_interface/tcp_client.hpp`
- `robot_hardware_interface/src/tcp_client.cpp`
- `robot_hardware_interface/src/tcp_system_hardware.cpp`
- `robot_hardware_interface/src/robot_hw_node.cpp`
- `robot_hardware_interface/src/rs485_protocol.cpp`
- `robot_hardware_interface/src/rs485_hw_node.cpp`
- `robot_hardware_interface/CMakeLists.txt`
- `robot_hardware_interface/test/tcp_get_all_parser_test.cpp`
- `robot_hardware_interface/README.md`
- `robot_gui/gui_win.md`
- `robot_bringup/README.md`
- `robot_hardware_interface/flags_publish_runtime_report.md`
- `robot_hardware_interface/servo_all_flags_test_report.md`

## 2. Command parser

Parser nằm ở:

```text
robot_hardware_interface/src/tcp_client.cpp
RobotTcpClient::get_all_state(...)
RobotTcpClient::parse_status_all_payload(...)
```

Command trong code vẫn là:

```text
cmd::STATUS_ALL
```

Comment/document đã ghi rõ đây là response format của `STATUS_ALL / CMD_GET_ALL`.
Không đổi command ID.

## 3. Payload CMD_GET_ALL hiện tại

Format duy nhất được TCP parser chấp nhận:

```text
[pos:i32][vel:u32][flag:u32] * NUM_AXIS
```

Với 6 axis:

```text
axis_bytes = 12
payload = 72 bytes
flag = uint32 little-endian
```

Parser dùng:

```cpp
pos_i32  = unpack_i32_le(base + 0);
vel_u32  = unpack_u32_le(base + 4);
flag_u32 = unpack_u32_le(base + 8);
```

## 4. Đã bỏ path parse 16-bit

Trong TCP path:

- Đã bỏ autodetect 10/12 byte.
- Đã bỏ `kBytesPerAxisStatus16`.
- Đã bỏ `unpack_u16_le(base + 8)`.
- Payload khác 72 byte sẽ throw lỗi protocol mismatch.

Lỗi khi nhận payload cũ 60 byte:

```text
CMD_GET_ALL protocol mismatch: expected 72 bytes for 6 axes x 12 bytes, got 60
```

Các `uint16` còn lại trong package là:

- TCP header helper `unpack_u16_le_raw` cho magic/length.
- Path RS485 legacy, đã comment rõ không dùng cho TCP `/robot_hw/flags`.

## 5. Publisher /robot_hw/flags

Publisher chính nằm ở:

```text
robot_hardware_interface/src/tcp_system_hardware.cpp
RobotSystemHardware::publish_status_flags(...)
```

Standalone node tương ứng:

```text
robot_hardware_interface/src/robot_hw_node.cpp
RobotHwNode::publish_status(...)
```

Flags giữ dạng:

```cpp
std::vector<uint32_t>
```

Publish:

```cpp
msg.axes[i].status_f = st;
```

Bool `servo_on` dùng đúng mask 32-bit:

```cpp
STATUS_SERVO_ON = 0x00100000;
a.servo_on = (st & STATUS_SERVO_ON) != 0;
```

`motionning` giữ nguyên tên field message hiện tại và map từ bit `RUNNING = 0x08000000`.

## 6. GUI

GUI đọc:

```text
/robot_hw/flags
robot_hardware_interface/msg/FlagStatus
```

Field đọc:

```python
flags = [axis.status_f for axis in msg.axes]
```

Không đọc `msg.flags`, `msg.data`, hoặc `axis.flag`.

Mapping giữ đúng:

```text
msg.axes[0].status_f -> Axis 1
...
msg.axes[5].status_f -> Axis 6
```

Nút robot enable/disable dựa trên:

```python
all((status_f & SERVO_ON) != 0 for status_f in flags[:6])
```

## 7. Runtime log

Log frame đầu tiên đã đổi sang dạng:

```text
CMD_GET_ALL payload_len=72 axis_bytes=12 axes=6 axis[0]: pos=<...> vel=<...> flag=0x...
```

Nếu payload không đúng 72 byte, parser throw lỗi:

```text
CMD_GET_ALL protocol mismatch: expected 72 bytes for 6 axes x 12 bytes, got <actual>
```

## 8. Test parser 12 byte/axis

Đã thêm test C++:

```text
robot_hardware_interface/test/tcp_get_all_parser_test.cpp
```

Mock payload 6 axis:

```text
flags[0] = 0x00100000
flags[1] = 0x08000000
flags[2] = 0x02000000
flags[3] = 0x00200000
flags[4] = 0x00000018
flags[5] = 0x00010001
```

Kết quả:

```text
ctest --test-dir build/robot_hardware_interface --output-on-failure -R tcp_get_all_parser_test
100% tests passed, 0 tests failed out of 1
```

## 9. Test reject payload 10 byte/axis

Trong cùng test C++, payload cũ 60 byte:

```text
[pos:i32][vel:u32][flag:u16] * 6
```

Kỳ vọng:

```text
throw runtime_error có "protocol mismatch"
```

Kết quả:

```text
OK
```

## 10. Build và test GUI

Clean build đã chạy:

```text
rm -rf build/robot_hardware_interface install/robot_hardware_interface
rm -rf build/robot_gui install/robot_gui
rm -rf build/robot_bringup install/robot_bringup

colcon build --packages-select robot_hardware_interface robot_gui robot_bringup --event-handlers console_direct+
```

Kết quả:

```text
Summary: 3 packages finished
```

GUI tests:

```text
pytest src/robot_gui/test
28 passed
```

Interface check:

```text
ros2 interface show robot_hardware_interface/msg/FlagStatus
robot_hardware_interface/AxisFlag[6] axes
  ...
  uint32 status_f
```

## 11. Runtime /robot_hw/flags và servo_all

Trong shell hiện tại chưa chạy `ros2 launch robot_bringup real.launch.py`, nên không có runtime topic/service:

```text
ros2 topic info -v /robot_hw/flags
Unknown topic '/robot_hw/flags'

ros2 topic echo --once /robot_hw/flags
WARNING: topic [/robot_hw/flags] does not appear to be published yet
Could not determine the type for the passed topic

ros2 service list | grep servo
<no output>
```

Test script servo safety vẫn chạy được bằng `ros2 run`, nhưng không thể gọi service vì service chưa tồn tại:

```text
ros2 run robot_hardware_interface test_servo_all_flags.py --timeout 1 --samples 1 --settle 0.1
ERROR during ON/read phase: /robot_hw/servo_all service not available within 1.00s
ERROR while forcing servo_all OFF: /robot_hw/servo_all service not available within 1.00s
```

Do đó:

- `ros2 topic echo /robot_hw/flags`: chưa kiểm tra runtime thật được trong shell này.
- Bật/tắt servo bằng `/robot_hw/servo_all`: chưa kiểm tra runtime thật được trong shell này.
- `status_f` sau khi bật servo có bit `SERVO_ON = 0x00100000` chưa: chưa kết luận được vì chưa có robot runtime.

## 12. Kết luận

ROS-side parser hiện đã đồng bộ với STM32 protocol mới:

```text
[pos:i32][vel:u32][flag:u32] * 6
72 bytes
```

Nếu chạy robot thật mà `/robot_hw/servo_all` trả success nhưng `/robot_hw/flags axes[i].status_f` vẫn `0x00000000`, lỗi còn lại nhiều khả năng nằm ở frame STM32 `CMD_GET_ALL` chưa cập nhật flag runtime, không phải ROS parser.
