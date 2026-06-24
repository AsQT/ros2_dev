# Servo All Flags Test Report

## 1. Service /robot_hw/servo_all

- Service type from implementation: `std_srvs/srv/SetBool`
- Interface verified with `ros2 interface show std_srvs/srv/SetBool`:

```text
bool data
---
bool success
string message
```

- Request field used to turn servo ON/OFF: `data`
- Response fields: `success`, `message`
- Runtime service availability in this shell: not available

Command result:

```text
ros2 service list | grep servo: no output
ros2 service type /robot_hw/servo_all: no output
```

## 2. Flags before servo on

Runtime topic was not available in this shell, so no real robot flag sample could be captured.

Command result:

```text
ros2 topic echo --once /robot_hw/flags
WARNING: topic [/robot_hw/flags] does not appear to be published yet
Could not determine the type for the passed topic

ros2 topic info -v /robot_hw/flags
Unknown topic '/robot_hw/flags'
```

Recorded values:

```text
Axis 1 status_f: <no message>
Axis 2 status_f: <no message>
Axis 3 status_f: <no message>
Axis 4 status_f: <no message>
Axis 5 status_f: <no message>
Axis 6 status_f: <no message>
```

## 3. Servo all ON result

Created safety test script:

```text
robot_hardware_interface/scripts/test_servo_all_flags.py
```

The script:

1. Subscribes `/robot_hw/flags`.
2. Reads flags before test.
3. Calls `/robot_hw/servo_all` with `data=True`.
4. Reads flags after ON.
5. Uses `finally` to always call `/robot_hw/servo_all` with `data=False`.
6. Reads flags after OFF.
7. Prints `status_f` as hex and checks bit `0x00100000`.

The script is also installed by `robot_hardware_interface/CMakeLists.txt` to:

```text
install/robot_hardware_interface/lib/robot_hardware_interface/test_servo_all_flags.py
```

Runtime result in this shell:

```text
before servo_all ON: no /robot_hw/flags samples within 1.00s
before_flags:
  <no message>
Calling /robot_hw/servo_all data=True
ERROR during ON/read phase: /robot_hw/servo_all service not available within 1.00s
```

- Service response: no response because service was unavailable
- Flags after ON: not captured
- SERVO_ON bit detected: no data

## 4. Servo all OFF result

The script did enter the `finally` block and attempted to force servo OFF.

Runtime result:

```text
Calling /robot_hw/servo_all data=False
ERROR while forcing servo_all OFF: /robot_hw/servo_all service not available within 1.00s
```

- Service response: no response because service was unavailable
- Flags after OFF: not captured
- Safety status: no ON command reached a service in this shell; OFF was still attempted by `finally`

## 5. TCP parser status

Code inspection:

- Active `real.launch.py` path uses `robot_hardware_interface::RobotSystemHardware` in:

```text
robot_hardware_interface/src/tcp_system_hardware.cpp
```

- `/robot_hw/flags` publisher:

```text
RobotSystemHardware::setup_ros_api()
```

- Message publish function:

```text
RobotSystemHardware::publish_status_flags(...)
```

- `servo_on` bool uses the correct 32-bit mask:

```cpp
constexpr uint32_t STATUS_SERVO_ON = 0x00100000;
a.servo_on = (st & STATUS_SERVO_ON) != 0;
a.status_f = st;
```

- TCP parser supports only the current STM32 `CMD_GET_ALL` format:

```text
12 bytes/axis: int32 pos_mdeg, uint32 vel_mdeg_s, uint32 status_f
```

- First valid runtime frame is logged by hardware as:

```text
CMD_GET_ALL payload_len=72 axis_bytes=12 axes=6 axis[0]: pos=<...> vel=<...> flag=0x...
```

Runtime values in this shell:

```text
STATUS_ALL payload_len: not available
axis_bytes: not available
raw status_f per axis: not available
```

## 6. Kết luận

- Service command có chạy không?
  - Chưa chạy được trong shell này vì `/robot_hw/servo_all` không tồn tại. Có vẻ `ros2 launch robot_bringup real.launch.py` chưa chạy trong phiên terminal này.

- `/robot_hw/flags` có đổi sau khi bật servo không?
  - Chưa kết luận được vì topic `/robot_hw/flags` cũng không tồn tại trong shell này.

- `status_f` có giữ được bit 32-bit không?
  - Code `robot_hardware_interface` đang giữ `uint32 status_f` và publish `servo_on` bằng mask đúng `0x00100000`.
  - Cần chạy lại script khi robot runtime đang hoạt động để xác nhận dữ liệu thực tế từ STM32.

- Nếu khi chạy trên robot thật mà service trả `success=True` nhưng `status_f` vẫn `0x00000000`:
  - Khả năng cao STM32/firmware chưa gửi hoặc chưa cập nhật flag SERVO_ON trong `STATUS_ALL`.
  - Khi đó cần kiểm tra firmware/frame `STATUS_ALL` phía STM32, không phải GUI mask.

## 7. Command chạy lại khi robot runtime đang bật

Terminal 1:

```bash
cd ~/ros2_dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_bringup real.launch.py
```

Terminal 2:

```bash
cd ~/ros2_dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 service list | grep servo
ros2 service type /robot_hw/servo_all
ros2 interface show std_srvs/srv/SetBool
ros2 topic echo --once /robot_hw/flags

ros2 run robot_hardware_interface test_servo_all_flags.py --timeout 3 --samples 5 --settle 0.5
```

Expected useful output if STM32 sends the 32-bit flag correctly:

```text
after_enable_flags:
  Axis 1: status_f=0x00100000, servo_on=True, servo_on_bit=True
```

If output remains:

```text
after_enable_flags:
  Axis 1: status_f=0x00000000, servo_on=False, servo_on_bit=False
```

while `/robot_hw/servo_all` returns `success=True`, then `/robot_hw/flags` is not changing after `servo_all ON`; check STM32 `STATUS_ALL` / 32-bit flag update.
