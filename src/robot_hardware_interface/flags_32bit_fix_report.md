# /robot_hw/flags 32-bit Fix Report

## 1. STM32 firmware

Chua sua duoc firmware STM32 trong workspace nay vi khong tim thay source firmware/STM32. Lenh quet file `.c`, `.h`, `.cpp`, `.hpp` trong `/home/minhquang/ros2_dev/src` chi thay cac file ROS-side cua `robot_hardware_interface`, khong thay project firmware tao frame `STATUS_ALL`.

Firmware van can sua o repo/project STM32 that:

```c
typedef struct __attribute__((packed)) {
  int32_t  pos_mdeg;
  uint32_t vel_mdeg_s;
  uint32_t status_f;
} AxisStateTcp_t;
```

Payload `STATUS_ALL` muc tieu cho 6 truc la:

```text
6 axes * 12 bytes/axis = 72 bytes
```

Moi axis phai serialize little-endian theo thu tu:

```text
[int32 pos_mdeg][uint32 vel_mdeg_s][uint32 status_f]
```

## 2. STATUS_ALL payload bytes/axis

Da sua ROS-side TCP parser trong:

```text
robot_hardware_interface/src/tcp_client.cpp
```

Ham:

```cpp
RobotTcpClient::get_all_state(int timeout_ms)
```

Parser hien chi chap nhan format moi:

```text
12 bytes / axis
```

Neu payload van la format cu 10 bytes/axis, parser se throw loi ro rang:

```text
STATUS_ALL payload uses old or invalid status format ... Expected 12 bytes/axis with uint32 status_f ... 10 bytes/axis 16-bit status is not supported.
```

## 3. Status/flags uint32

ROS-side status/flags hien di bang `uint32_t`:

```cpp
std::vector<uint32_t> flags
```

Parser moi doc:

```cpp
pos_i32 = unpack_i32_le(base + 0);
vel_u32 = unpack_u32_le(base + 4);
flag    = unpack_u32_le(base + 8);
```

Khong con dung fallback:

```cpp
unpack_u16_le(base + 8)
```

trong TCP parser moi.

Ghi chu: legacy RS485 path van co code 16-bit trong `rs485_protocol.cpp` / `rs485_hw_node.cpp`; path do publish `/hardware/flags`, khong phai `/robot_hw/flags`. Neu van dung RS485 path cho LED 32-bit thi can sua tiep.

## 4. robot_hardware_interface parse status

File/hàm parse TCP:

```text
robot_hardware_interface/src/tcp_client.cpp
RobotTcpClient::get_all_state(int timeout_ms)
```

Da bat buoc parse 12 bytes/axis va raw status 32-bit.

## 5. /robot_hw/flags publish field

Publisher `/robot_hw/flags` giu message:

```text
robot_hardware_interface/msg/FlagStatus
```

Message:

```text
AxisFlag[6] axes
```

Raw field:

```text
axes[i].status_f
```

Kieu field:

```text
uint32 status_f
```

Publisher trong:

```text
robot_hardware_interface/src/robot_hw_node.cpp
robot_hardware_interface/src/tcp_system_hardware.cpp
```

deu publish:

```cpp
msg.axes[i].status_f = flags[i];
```

Bool parse san trong `AxisFlag` van duoc tinh tu `uint32_t st`.

## 6. GUI field

GUI dang doc dung raw field:

```python
flags = [int(getattr(axis_flags, "status_f", 0)) for axis_flags in msg.axes]
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

## 7. ros2 interface show

Sau build va source workspace:

```bash
source /home/minhquang/ros2_dev/install/setup.bash
ros2 interface show robot_hardware_interface/msg/FlagStatus
ros2 interface show robot_hardware_interface/msg/AxisFlag
```

Ket qua:

```text
robot_hardware_interface/AxisFlag[6] axes
  bool servo_on
  bool error_all
  bool org_ok
  bool motionning
  bool org_retunning
  bool limit_pos
  bool limit_neg
  bool org_sensor
  bool alarm_rst
  bool emg
  bool stop
  bool communi_err
  uint32 status_f
```

`ros2 pkg prefix robot_hardware_interface`:

```text
/home/minhquang/ros2_dev/install/robot_hardware_interface
```

## 8. ros2 topic echo /robot_hw/flags

Chua xac nhan duoc runtime echo trong phien nay vi `real.launch.py` / STM32 khong chay.

Ket qua khi khong co publisher runtime:

```text
$ ros2 topic info /robot_hw/flags
Unknown topic '/robot_hw/flags'

$ ros2 topic echo /robot_hw/flags --once --timeout 1
WARNING: topic [/robot_hw/flags] does not appear to be published yet
Could not determine the type for the passed topic
```

Can chay lai sau khi flash STM32 va launch real.

## 9. Log xac nhan axis_bytes=12

Da them/giu log xac nhan:

### robot_hw_node.cpp

Log first valid state frame gom:

```text
payload_length
offset
axis_bytes
axes
configured_joints
pos_mdeg[0..5]
status_f[0..5]
```

### tcp_system_hardware.cpp

Log one-time khi hardware plugin doc state thanh cong:

```text
STATUS_ALL axis_bytes=12, axes=6, payload_len=72, status_f[0..5]=[...]
```

Neu thay `axis_bytes=10` thi firmware van gui format cu. Voi parser moi, format 10 bytes/axis se bi reject va log loi parse.

## 10. LED bit tests

Chua the test voi STM32 that trong phien nay. Dieu kien can xac nhan runtime:

```text
status_f = 0x00100000 -> GUI LED Servo On xanh
status_f = 0x08000000 -> GUI LED Running xanh
status_f = 0x02000000 -> GUI LED Origin OK xanh
status_f = 0x00200000 -> GUI LED Alarm do
```

Neu `/robot_hw/flags axes[i].status_f` echo ra cac gia tri lon hon `0xFFFF`, ROS-side da giu du 32-bit.

## 11. Build result

Da chay:

```bash
cd /home/minhquang/ros2_dev
colcon build --packages-select robot_hardware_interface
colcon build --packages-select robot_hardware_interface robot_gui robot_bringup
```

Ket qua:

```text
Summary: 1 package finished
Summary: 3 packages finished
```

## 12. Ket luan

- Da sua `robot_hardware_interface` TCP parser de bat buoc status 32-bit 12 bytes/axis.
- `AxisFlag.status_f` da la `uint32`, khong can doi message.
- `/robot_hw/flags` publisher da publish `axes[i].status_f` tu `uint32_t`.
- GUI da doc `msg.axes[i].status_f`.
- Chua sua duoc STM32 firmware trong workspace nay vi khong co source firmware.
- Chua xac nhan duoc `/robot_hw/flags echo` va LED runtime vi chua co `real.launch.py`/STM32 dang chay trong phien nay.

## 13. Viec can lam tiep

1. Sua/flash firmware STM32 de `STATUS_ALL` gui 12 bytes/axis.
2. Chay:

   ```bash
   source /home/minhquang/ros2_dev/install/setup.bash
   ros2 launch robot_bringup real.launch.py
   ```

3. Kiem tra:

   ```bash
   ros2 topic info -v /robot_hw/flags
   ros2 topic echo /robot_hw/flags
   ```

4. Xac nhan log:

   ```text
   axis_bytes=12
   payload_len=72
   status_f[0]=0x00100000
   ```

