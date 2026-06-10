# robot_hardware_interface

ROS 2 C++ package providing RS485 hardware communication and a ros2_control `SystemInterface` plugin. Handles AA BB ... AA FF framing, byte stuffing, and Modbus CRC16 for motor control.

## Package Structure

```
robot_hardware_interface/
├── src/                       # C++ source files
├── include/                   # C++ headers
├── config/
│   └── params.yaml           # Default parameters
├── launch/
│   └── hardware_interface.launch.py
├── msg/                       # Custom messages (if any)
├── srv/                       # Custom services
├── plugin.xml                 # ros2_control plugin declaration
├── CMakeLists.txt
└── package.xml
```

## Protocol

| Field | Value |
|-------|-------|
| HEADER | `0xAA 0xBB` |
| TAIL | `0xAA 0xFF` |
| STUFF | Duplicate `0xAA` inside body (byte stuffing) |
| CRC | CRC16/Modbus over (cmd + payload), little-endian |

### Commands

| CMD | Name | Payload | Reply |
|-----|------|---------|-------|
| `0xA0` | SERVO ON/OFF axis | `[id][0/1]` | — |
| `0xA2` | GET POS axis | `[id]` | pos_i32 at bytes [2:6] |
| `0xA5` | RUN axis | `[id][pos_i32][vel_u32]` (scaled *1000) | — |
| `0xA7` | JOG | `[id][dir][vel_u32]` (vel=0 → stop) | — |
| `0xF1` | SERVO ON/OFF ALL | `[token][0/1]` | — |
| `0xF2` | STATUS ALL | `[]` | 6 * u32 flags |
| `0xF3` | RUN ALL | `6 * (pos_i32 + vel_u32)` | — |

## ros2_control Plugin

**Plugin name:** `robot_hardware_interface/RobotSystemHardware`

### URDF Snippet

```xml
<ros2_control name="robot_system" type="system">
  <hardware>
    <plugin>robot_hardware_interface/RobotSystemHardware</plugin>
    <param name="port">/dev/ttyUSB0</param>
    <param name="baudrate">115200</param>
    <param name="serial_timeout_s">0.2</param>
    <param name="pos_timeout_s">0.35</param>
    <param name="status_timeout_s">0.5</param>
    <param name="all_token">153</param>
    <param name="axis_ids">1,2,3,4,5,6</param>
    <param name="direction_sign">1,1,1,1,1,1</param>
    <param name="rad_offset">0,0,0,0,0,0</param>
    <param name="default_vel_deg_s">10.0</param>
  </hardware>
  <!-- joint interfaces declared here -->
</ros2_control>
```

## Topics

| Topic | Direction | Description |
|-------|-----------|-------------|
| `/joint_states` | Publish | Current joint positions/velocities |
| `/rs485_hw/status_flags` | Publish | Status flags from all axes |
| `/rs485_hw/connected` | Publish | Connection state |
| `/rs485_hw/cmd_servo_axis` | Subscribe | Servo on/off per axis |
| `/rs485_hw/cmd_run_axis` | Subscribe | Run command per axis |
| `/rs485_hw/cmd_jog` | Subscribe | Jog command |
| `/rs485_hw/cmd_run_all` | Subscribe | Run all axes |
| `/rs485_hw/joint_trajectory` | Subscribe | Trajectory command |

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/rs485_hw/connect` | `Trigger` | Connect to serial port |
| `/rs485_hw/disconnect` | `Trigger` | Disconnect |
| `/rs485_hw/servo_all` | `SetBool` | Servo on/off all |
| `/rs485_hw/poll_now` | `Trigger` | Trigger immediate poll |
| `/rs485_hw/servo_on_axis` | `ServoOnAxis` | Per-axis servo control |
| `/rs485_hw/run_axis` | `RunAxis` | Per-axis run |
| `/rs485_hw/jog` | `Jog` | Jog control |
| `/rs485_hw/home` | `Home` | Go to home position |
| `/rs485_hw/stop_axis` | `StopAxis` | Stop per axis |
| `/rs485_hw/stop_all` | `StopAll` | Stop all axes |

## Build & Run

```bash
cd ~/ros2
colcon build --packages-select robot_hardware_interface
source install/setup.bash

# Run via launch
ros2 launch robot_hardware_interface hardware_interface.launch.py

# Or directly
ros2 run robot_hardware_interface rs485_hw_node \
    --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=115200
```
