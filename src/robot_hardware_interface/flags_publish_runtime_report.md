# /robot_hw/flags runtime publish report

## Node path

`robot_bringup/launch/real.launch.py` includes MoveIt, which includes `robot_control/launch/controllers.launch.py`.
That launch starts `controller_manager` executable `ros2_control_node`; the robot hardware plugin is selected by:

```text
robot_description/urdf/ros2_control.xacro
<plugin>robot_hardware_interface/RobotSystemHardware</plugin>
```

So the active node named `robot` uses `robot_hardware_interface::RobotSystemHardware` from:

```text
robot_hardware_interface/src/tcp_system_hardware.cpp
```

`robot_hardware_interface/src/robot_hw_node.cpp` is the standalone hardware node path and is not the main path for `real.launch.py`.

## Publisher and publish function

`/robot_hw/flags` is created in:

```text
RobotSystemHardware::setup_ros_api()
```

The message is published by:

```text
RobotSystemHardware::publish_status_flags(...)
```

The standalone node equivalent is:

```text
RobotHwNode::publish_status(...)
```

Both paths now publish all 6 axes every time. Missing flags are filled with `status_f = 0` and a throttled warning is logged.

## Root cause

Before this fix, `RobotSystemHardware::read()` returned early when TCP was not connected or when `get_all_state()` failed. Those branches published `/robot_hw/connected`, but did not publish `/robot_hw/flags`.

That created this runtime symptom:

```text
Publisher count: 1
ros2 topic echo /robot_hw/flags: no data
```

The publisher existed, but no `FlagStatus` message had been sent yet.

## Fix summary

`RobotSystemHardware::setup_ros_api()` now publishes one default message at startup:

```text
6 axes, all status_f = 0
```

`RobotSystemHardware::read()` now publishes default flags before returning on:

```text
not connected
get_all_state() exception
empty pos/vel state
```

The error log is explicit:

```text
get_all_state failed: ... (keeping last state, publishing default flags)
```

`RobotHwNode` standalone was updated with the same behavior.

## STATUS_ALL frame format

`RobotTcpClient::get_all_state()` now accepts only the current STM32 format:

```text
12 bytes/axis: int32 pos_mdeg, uint32 vel_mdeg_s, uint32 status_f
```

The legacy 10-byte format is deprecated and rejected as a protocol mismatch. If
the STM32 still sends 60 bytes for 6 axes, the parser reports:

```text
CMD_GET_ALL protocol mismatch: expected 72 bytes for 6 axes x 12 bytes, got 60
```

On the first valid frame, runtime logs include:

```text
CMD_GET_ALL payload_len=72 axis_bytes=12 axes=6 axis[0]: pos=<...> vel=<...> flag=0x...
```

## Runtime status from this shell

Build and interface verification passed:

```text
colcon build --packages-select robot_hardware_interface robot_bringup robot_gui: OK
ros2 interface show robot_hardware_interface/msg/FlagStatus: OK
```

Runtime topic verification could not be completed in this shell because `real.launch.py` was not running here:

```text
ros2 topic info -v /robot_hw/flags: Unknown topic
ros2 topic echo /robot_hw/flags --once: no publisher/type in this shell
ros2 topic hz /robot_hw/flags: no publisher in this shell
```

When `ros2 launch robot_bringup real.launch.py` is running, `/robot_hw/flags` should now publish at least a default all-zero message immediately, and should continue publishing default flags on TCP read failures.
