# rs485_hardware_cpp

ROS2 C++ node for RS485 protocol:

- HEADER: 0xAA 0xBB
- TAIL:   0xAA 0xFF
- STUFF: duplicate 0xAA inside body (byte stuffing)
- CRC: CRC16/MODBUS over (cmd + payload), little-endian appended

Commands (from provided Python reference):
- 0xA0: SERVO ON/OFF axis   payload [id][0/1]
- 0xA2: GET POS axis        payload [id]   reply payload contains pos_i32 at bytes [2:6]
- 0xA5: RUN axis            payload [id][pos_i32][vel_u32]  (pos,vel scaled *1000)
- 0xA7: JOG                 payload [id][dir][vel_u32]      (vel=0 => stop)
- 0xF1: SERVO ON/OFF ALL    payload [token][0/1]
- 0xF2: STATUS ALL          payload [] reply: 6*u32 flags
- 0xF3: RUN ALL             payload 6*(pos_i32 + vel_u32)

## Build
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Run
```bash
ros2 run rs485_hardware_cpp rs485_hw_node --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=115200
```

## Topics
- Publishes: /joint_states, /rs485_hw/status_flags, /rs485_hw/connected
- Subscribes: /rs485_hw/cmd_servo_axis, /rs485_hw/cmd_run_axis, /rs485_hw/cmd_jog, /rs485_hw/cmd_run_all, /rs485_hw/joint_trajectory

## Services
- /rs485_hw/connect (Trigger)
- /rs485_hw/disconnect (Trigger)
- /rs485_hw/servo_all (SetBool)
- /rs485_hw/poll_now (Trigger)

Typed services (recommended for GUI):
- /rs485_hw/servo_on_axis (rs485_hardware_cpp/srv/ServoOnAxis)
- /rs485_hw/servo_on_all  (rs485_hardware_cpp/srv/ServoOnAll)
- /rs485_hw/jog           (rs485_hardware_cpp/srv/Jog)
- /rs485_hw/run_axis      (rs485_hardware_cpp/srv/RunAxis)
- /rs485_hw/home          (rs485_hardware_cpp/srv/Home)  # implemented as RUN to configured home_positions_rad
- /rs485_hw/stop_axis     (rs485_hardware_cpp/srv/StopAxis)
- /rs485_hw/stop_all      (rs485_hardware_cpp/srv/StopAll)


## ros2_control hardware plugin (SystemInterface)

This package also provides a ros2_control hardware plugin:

- **Plugin name:** `rs485_hardware_cpp/Rs485SystemHardware`

### URDF snippet (real robot)

```xml
<ros2_control name="RS485Hardware" type="system">
  <hardware>
    <plugin>rs485_hardware_cpp/Rs485SystemHardware</plugin>

    <!-- Same naming as config/params.yaml (node), but here is for ros2_control -->
    <param name="port">/dev/ttyUSB0</param>
    <param name="baudrate">115200</param>
    <param name="serial_timeout_s">0.2</param>
    <param name="pos_timeout_s">0.35</param>
    <param name="status_timeout_s">0.5</param>
    <param name="all_token">153</param>

    <!-- Optional -->
    <param name="axis_ids">1,2,3,4,5,6</param>
    <param name="direction_sign">1,1,1,1,1,1</param>
    <param name="rad_offset">0,0,0,0,0,0</param>
    <param name="default_vel_deg_s">10.0</param>
  </hardware>

  <!-- declare joint interfaces here ... -->
</ros2_control>
```

## GUI bring-up (2 joints)

After building this package, you can run the GUI to quickly test **axis 0 & 1**:

```bash
# terminal 1
ros2 launch rs485_hardware_cpp rs485_hw_with_gui.launch.py

# or launch the node only:
ros2 launch rs485_hardware_cpp rs485_hw.launch.py

# terminal 2 (if not launched with GUI)
ros2 run rs485_hardware_cpp rs485_hw_gui.py
```

GUI uses:
- Services: `/rs485_hw/connect`, `/rs485_hw/disconnect`, `/rs485_hw/poll_now`, `/rs485_hw/servo_all`
- Topics: `/joint_states` (positions), `/rs485_hw/cmd_run_axis` (HOME sends RUN to axis 0/1)

If your joint names differ, set GUI params:
```bash
ros2 run rs485_hardware_cpp rs485_hw_gui.py --ros-args -p watch_joints:="['joint1','joint2']" -p axis_ids:="[0,1]"
```
