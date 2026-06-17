# robot_hardware_interface

ROS 2 C++ package providing a `ros2_control` `SystemInterface` plugin and a small hardware service node for a robot TCP controller.

The ROS 2 PC is a TCP client. The robot controller, currently implemented on STM32F407 firmware, is expected to run as a TCP server and keep handling the downstream motor-driver bus internally.

## ros2_control Plugin

Plugin name:

```xml
<plugin>robot_hardware_interface/RobotSystemHardware</plugin>
```

Default hardware parameters:

```xml
<param name="robot_ip">192.168.2.50</param>
<param name="robot_port">5000</param>
<param name="connect_timeout_ms">2000</param>
<param name="read_timeout_ms">50</param>
```

The plugin keeps the existing joint names and command/state interfaces. Position and velocity conversion logic follows the previous package.

## TCP Protocol

The transport frame is a compact binary TCP frame:

```text
MAGIC | CMD | SEQ | LENGTH | PAYLOAD
```

Header layout:

```text
AA 55 | cmd:u8 | seq:u16_le | length:u16_le | payload
```

Magic is `0x55AA` stored little-endian on the wire as `AA 55`. There is no CRC16, byte stuffing, or UART-style tail parser. TCP connect and read operations use separate timeouts so state polling does not block ROS execution.

## Build

```bash
cd ~/ros2_dev
colcon build --packages-select robot_hardware_interface
source install/setup.bash
```

## Run Debug Node

```bash
ros2 launch robot_hardware_interface hardware_interface.launch.py
```

Override TCP parameters if needed:

```bash
ros2 run robot_hardware_interface robot_hw_node --ros-args \
  -p robot_ip:=192.168.2.50 \
  -p robot_port:=5000 \
  -p connect_timeout_ms:=2000 \
  -p read_timeout_ms:=50
```

Main runtime topics and services use `/robot_hw/*`, including `/robot_hw/connect`,
`/robot_hw/connected`, `/robot_hw/status_text`, command topics, and typed motion services.
The node publishes actual robot state on `/joint_states` after successful polling.
