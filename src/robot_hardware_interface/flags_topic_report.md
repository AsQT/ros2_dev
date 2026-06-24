Yêu cầu sửa dứt điểm luồng `/robot_hw/flags`: STM32 phải gửi cờ `status/flags` dạng 32-bit để khớp với các bit GUI đang kiểm tra.

## 1. Bối cảnh

Hiện tại GUI cần kiểm tra các bit trạng thái:

```c
#define ERROR_ALL    0x00000001
#define SOF_LIMIT_P  0x00000008
#define SOF_LIMIT_M  0x00000010
#define EMG          0x00010000
#define SERVO_ON     0x00100000
#define ALARM        0x00200000
#define ORG_SET_OK   0x02000000
#define RUNNING      0x08000000
```

Các bit sau vượt quá phạm vi 16-bit:

```text
EMG        = 0x00010000
SERVO_ON   = 0x00100000
ALARM      = 0x00200000
ORG_SET_OK = 0x02000000
RUNNING    = 0x08000000
```

Vì vậy nếu STM32 chỉ gửi `uint16_t status` thì GUI không bao giờ đọc đúng các LED quan trọng như Servo On, Alarm, Origin OK, Running.

Yêu cầu sửa theo hướng:

```text
STM32 STATUS_ALL frame phải gửi status/flags 32-bit cho từng trục.
```

Không được cắt còn 16-bit.

---

## 2. Chuẩn frame STATUS_ALL mới

Mỗi axis trong payload `STATUS_ALL` phải có format cố định:

```c
int32_t  pos_mdeg;        // 4 bytes
uint32_t vel_mdeg_s;      // 4 bytes
uint32_t status_f;        // 4 bytes
```

Tổng mỗi trục:

```text
12 bytes / axis
```

Với 6 trục:

```text
6 * 12 = 72 bytes
```

Mapping bắt buộc:

```text
axis 0 -> Axis 1
axis 1 -> Axis 2
axis 2 -> Axis 3
axis 3 -> Axis 4
axis 4 -> Axis 5
axis 5 -> Axis 6
```

Endianness:

```text
little-endian
```

Tức là STM32 phải serialize raw theo little-endian đúng thứ tự:

```text
[pos_i32][vel_u32][status_u32]
[pos_i32][vel_u32][status_u32]
...
```

Không dùng `uint16_t status`.
Không dùng `status & 0xFFFF`.
Không ép kiểu `uint32_t -> uint16_t`.

---

## 3. Sửa firmware STM32

Tìm phần STM32 tạo frame phản hồi `STATUS_ALL`.

Sửa các struct / buffer / hàm đóng gói để status gửi lên là `uint32_t`.

Ví dụ logic đúng:

```c
typedef struct __attribute__((packed)) {
    int32_t  pos_mdeg;
    uint32_t vel_mdeg_s;
    uint32_t status_f;
} AxisStateTcp_t;
```

Hoặc nếu không dùng struct thì ghi từng field vào buffer theo 4 byte little-endian.

Nguồn `status_f` phải lấy từ thanh ghi status thật của driver/servo, giữ nguyên 32-bit.

Nếu hiện tại code đang có kiểu:

```c
uint16_t status;
uint16_t flag;
```

hoặc:

```c
buffer[offset++] = status & 0xFF;
buffer[offset++] = (status >> 8) & 0xFF;
```

thì phải sửa thành 4 byte:

```c
buffer[offset++] = status & 0xFF;
buffer[offset++] = (status >> 8) & 0xFF;
buffer[offset++] = (status >> 16) & 0xFF;
buffer[offset++] = (status >> 24) & 0xFF;
```

Sau khi sửa, payload length của `STATUS_ALL` phải là `72` byte cho 6 trục.

---

## 4. Sửa `robot_hardware_interface`

Kiểm tra các file:

```text
robot_hardware_interface/src/tcp_client.cpp
robot_hardware_interface/src/robot_hw_node.cpp
robot_hardware_interface/src/tcp_system_hardware.cpp
robot_hardware_interface/msg/AxisFlag.msg
robot_hardware_interface/msg/FlagStatus.msg
```

Hiện tại `AxisFlag.msg` đã có:

```text
uint32 status_f
```

Nếu đúng thì không cần đổi message.

Nhưng phải sửa parser TCP để ưu tiên/bắt buộc format 12 byte/trục.

Trong `RobotTcpClient::get_all_state()`:

* Không được parse status theo 16-bit cho protocol mới.
* Nếu payload vẫn là 10 byte/trục thì phải log error/warning rõ:

```text
STATUS_ALL payload uses old 16-bit status format; expected 12 bytes/axis with uint32 status_f
```

* Chuẩn mới phải parse:

```cpp
pos_i32  = unpack_i32_le(base + 0);
vel_u32  = unpack_u32_le(base + 4);
status_f = unpack_u32_le(base + 8);
```

* `flag_s` phải là:

```cpp
std::vector<uint32_t>
```

* Không dùng:

```cpp
unpack_u16_le(base + 8)
```

cho protocol mới.

Nếu muốn giữ tương thích tạm thời với frame cũ 10 byte/trục thì vẫn được, nhưng phải log warning cực rõ rằng frame 16-bit không đủ cho GUI LED. Tuy nhiên mục tiêu cuối cùng là STM32 gửi 12 byte/trục.

---

## 5. Publish `/robot_hw/flags`

Topic:

```text
/robot_hw/flags
```

Message hiện tại:

```text
robot_hardware_interface/msg/FlagStatus
```

Với cấu trúc:

```text
AxisFlag[6] axes
```

Mỗi axis phải publish:

```cpp
msg.axes[i].status_f = flags[i];
```

Các bool parse sẵn trong `AxisFlag` cũng phải dùng `status_f` 32-bit:

```cpp
a.servo_on = (st & SERVO_ON) != 0;
a.error_all = (st & ERROR_ALL) != 0;
a.org_ok = (st & ORG_SET_OK) != 0;
a.motionning = (st & RUNNING) != 0;
a.limit_pos = (st & SOF_LIMIT_P) != 0;
a.limit_neg = (st & SOF_LIMIT_M) != 0;
a.emg = (st & EMG) != 0;
```

Cần kiểm tra lại toàn bộ mask trong C++ để đúng với firmware và GUI.

Không được dùng mask 16-bit cũ nếu không còn phù hợp.

---

## 6. Sửa GUI đọc đúng field

Trong `robot_gui`, subscriber `/robot_hw/flags` phải đọc field:

```python
flags = [axis.status_f for axis in msg.axes]
```

Không được đọc nhầm field bool nếu GUI đang cần raw bitmask.

Sau đó update LED bằng:

```python
active = (axis_status & MASK) != 0
```

Mapping:

```text
msg.axes[0].status_f -> Axis 1
msg.axes[1].status_f -> Axis 2
msg.axes[2].status_f -> Axis 3
msg.axes[3].status_f -> Axis 4
msg.axes[4].status_f -> Axis 5
msg.axes[5].status_f -> Axis 6
```

---

## 7. Kiểm tra build/source workspace

Sau khi sửa message/code, chạy lại build:

```bash
cd ~/ros2_dev
colcon build --packages-select robot_hardware_interface robot_gui robot_bringup
source install/setup.bash
```

Nếu interface vẫn lỗi, clean build riêng:

```bash
rm -rf build/robot_hardware_interface install/robot_hardware_interface
colcon build --packages-select robot_hardware_interface
source install/setup.bash
```

Kiểm tra:

```bash
ros2 pkg prefix robot_hardware_interface
ros2 interface show robot_hardware_interface/msg/AxisFlag
ros2 interface show robot_hardware_interface/msg/FlagStatus
```

---

## 8. Test runtime bắt buộc

Sau khi flash STM32 và launch real:

```bash
ros2 launch robot_bringup real.launch.py
```

Kiểm tra topic:

```bash
ros2 topic list | grep robot_hw
ros2 topic info /robot_hw/flags
ros2 topic info -v /robot_hw/flags
ros2 topic echo /robot_hw/flags
```

Kỳ vọng `ros2 topic echo /robot_hw/flags` phải hiển thị được dạng tương tự:

```yaml
axes:
- servo_on: true
  error_all: false
  org_ok: true
  motionning: false
  limit_pos: false
  limit_neg: false
  emg: false
  status_f: 34603008
...
```

Quan trọng nhất là `status_f` phải có khả năng chứa các giá trị lớn hơn `0xFFFF`.

---

## 9. Test xác nhận 32-bit

Tạo log hoặc test để xác nhận parser nhận 12 byte/trục.

Cần log một lần khi nhận `STATUS_ALL`:

```text
STATUS_ALL axis_bytes=12, axes=6, payload_len=72
axis[0].status_f=0x00100000
```

Nếu còn thấy:

```text
axis_bytes=10
```

thì firmware vẫn đang gửi format cũ 16-bit, chưa đạt yêu cầu.

---

## 10. Test bit cụ thể

Dùng trạng thái thật hoặc mock từ STM32 để kiểm tra:

### Servo ON

STM32 gửi:

```c
status_f = 0x00100000;
```

Kỳ vọng:

```text
/robot_hw/flags axes[i].status_f = 1048576
GUI LED Servo On xanh
```

### Running

STM32 gửi:

```c
status_f = 0x08000000;
```

Kỳ vọng:

```text
/robot_hw/flags axes[i].status_f = 134217728
GUI LED Running xanh
```

### Origin OK

STM32 gửi:

```c
status_f = 0x02000000;
```

Kỳ vọng:

```text
/robot_hw/flags axes[i].status_f = 33554432
GUI LED Origin OK xanh
```

### Alarm

STM32 gửi:

```c
status_f = 0x00200000;
```

Kỳ vọng:

```text
/robot_hw/flags axes[i].status_f = 2097152
GUI LED Alarm đỏ
```

Nếu các giá trị trên bị mất hoặc chỉ còn 16-bit thấp thì parser/frame vẫn sai.

---

## 11. Không phá các phần khác

Không làm hỏng:

* TCP frame header hiện tại.
* Position `int32` theo milli-degree.
* Velocity `uint32`.
* Axis index 0 -> Axis 1.
* Service `/robot_hw/servo_all`.
* Topic `/robot_hw/connected`.
* Topic `/robot_hw/status_text`.
* GUI layout `robot_gui/ui/robot_gui.ui`.
* Các nút điều khiển giống tab Hardware của `robot_gui_old`.

---

## 12. Báo cáo sau khi sửa

Sau khi sửa và test xong, cập nhật hoặc tạo file:

```text
robot_hardware_interface/flags_32bit_fix_report.md
```

Nội dung cần ghi:

1. Đã sửa firmware STM32 file/hàm nào.
2. `STATUS_ALL` payload hiện tại bao nhiêu byte/trục.
3. Status/flags hiện là `uint32_t` hay chưa.
4. `robot_hardware_interface` parse status ở file/hàm nào.
5. `/robot_hw/flags` publish field nào.
6. GUI đọc field nào.
7. Kết quả `ros2 interface show robot_hardware_interface/msg/FlagStatus`.
8. Kết quả `ros2 topic echo /robot_hw/flags`.
9. Log xác nhận `axis_bytes=12`.
10. Test LED Servo On / Running / Origin OK / Alarm đã đúng chưa.

---

## 13. Output cuối cùng

Output cuối cùng cần ghi rõ:

```text
Đã sửa STM32 STATUS_ALL sang status_f 32-bit.
Đã sửa robot_hardware_interface parse/publish uint32 status_f.
Đã xác nhận /robot_hw/flags echo được.
Đã xác nhận GUI đọc msg.axes[i].status_f.
```

Nếu còn lỗi, phải ghi rõ lỗi nằm ở bước nào:

```text
STM32 frame
TCP parser
ROS message build/source
/robot_hw/flags publisher
robot_gui subscriber
LED widget update
```
