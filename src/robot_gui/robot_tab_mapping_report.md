# Robot Tab Mapping Report

## 1. Source reference

- robot_gui_old files checked:
  - `robot_gui_old/robot_gui/main_window.py`
  - `robot_gui_old/gui_win.md`
  - `robot_gui_old/launch/robot_gui.launch.py`
  - `robot_gui_old/launch/embedded_rviz_test.launch.py`
  - `robot_gui_old/test/test_axis_ros_services.py`
  - `robot_gui_old/test/test_axis_status_leds.py`
  - `robot_gui_old/test/test_led_widget_mapping.py`
  - `robot_gui_old/test/test_navigation_buttons.py`
  - `robot_gui_old/test/test_robot_flags_leds.py`
  - `robot_gui_old/test/test_robot_servo_service.py`

## 2. UI object mapping

| Function | Old objectName | New objectName | Found in .ui | Connected |
|---|---|---|---|---|
| Robot enable/toggle | `btnRobotEnable` | `btnRobotEnable` | yes | yes |
| Robot disable | `btnRobotDisable` | `btnRobotDisable` | yes | yes |
| Stop all / emergency stop | `btnStopTask` | `btnStopTask` | yes | yes |
| Axis enable | `btnAxis{1..6}Enable` | `btnAxis{1..6}Enable` | yes | yes |
| Axis home | `btnAxis{1..6}Home` | `btnAxis{1..6}Home` | yes | yes |
| Axis alarm reset | `btnAxis{1..6}AlarmReset` | `btnAxis{1..6}AlarmReset` | yes | log only |
| Axis run absolute | `btnAxis{1..6}Run` | `btnAxis{1..6}Run` | yes | yes |
| Axis stop | `btnAxis{1..6}Stop` | `btnAxis{1..6}Stop` | yes | yes |
| Axis jog positive | `btnAxis{1..6}JogPositive` | `btnAxis{1..6}JogPositive` | yes | yes |
| Axis jog negative | `btnAxis{1..6}JogNegative` | `btnAxis{1..6}JogNegative` | yes | yes |
| Command position | `txtAxis{1..6}CommandPos` | `txtAxis{1..6}CommandPos` | yes | read |
| Command velocity | `txtAxis{1..6}CommandVel` | `txtAxis{1..6}CommandVel` | yes | read |
| Servo On LED | `ledAxis{1..6}ServoOn` | `ledAxis{1..6}ServoOn` | yes | yes |
| Running LED | `ledAxis{1..6}Running` | `ledAxis{1..6}Running` | yes | yes |
| Origin OK LED | `ledAxis{1..6}OrgOK` | `ledAxis{1..6}OrgOK` | yes | yes |
| Limit+ LED | `ledAxis{1..6}LimitPositive` | `ledAxis{1..6}LimitPositive` | yes | yes |
| Limit- LED | `ledAxis{1..6}LimitNegative` | `ledAxis{1..6}LimitNegative` | yes | yes |
| Alarm LED | `ledAxis{1..6}Alarm` | `ledAxis{1..6}Alarm` | yes | yes |
| EMG LED | `ledAxis{1..6}EMG` | `ledAxis{1..6}EMG` | yes | yes |
| Error All LED | `ledAxis{1..6}ErrorAll` | `ledAxis{1..6}ErrorAll` | yes | yes |

## 3. ROS interfaces

- `/robot_hw/flags`:
  - Type: `robot_hardware_interface/msg/FlagStatus`
  - GUI reads: `msg->axes[i].status_f`
  - Axis mapping: `axes[0..5]` -> GUI Axis 1..6
  - If fewer than 6 axes arrive, missing axes are set inactive.
- `/robot_hw/servo_all`:
  - Runtime type verified: `std_srvs/srv/SetBool`
  - Request field: `data=true` for ON, `data=false` for OFF
  - C++ client fixed from wrong `ServoOnAll` client to `std_srvs::srv::SetBool`.
- Home:
  - Service: `/robot_hw/home`
  - Type: `robot_hardware_interface/srv/Home`
  - Request field: `id`, GUI axis 1..6 maps to hardware id 0..5 by default.
- Jog:
  - Service: `/robot_hw/jog`
  - Type: `robot_hardware_interface/srv/Jog`
  - Request fields: `id`, `vel` rad/s, `dir` 1 for positive and 0 for negative.
- Run:
  - Service: `/robot_hw/run_axis`
  - Type: `robot_hardware_interface/srv/RunAxis`
  - Request fields: `id`, `pos` rad, `vel` rad/s.
  - GUI reads deg and converts to rad.
- Stop:
  - Axis service: `/robot_hw/stop_axis`
  - All service: `/robot_hw/stop_all`

## 4. LED mapping

| Axis | LED | Bitmask | Widget objectName | Test result |
|---|---|---|---|---|
| 1..6 | Servo On | `0x00100000` | `ledAxis{N}ServoOn` | OK |
| 1..6 | Running | `0x08000000` | `ledAxis{N}Running` | OK |
| 1..6 | Origin OK | `0x02000000` | `ledAxis{N}OrgOK` | OK |
| 1..6 | Limit + | `0x00000008` | `ledAxis{N}LimitPositive` | OK |
| 1..6 | Limit - | `0x00000010` | `ledAxis{N}LimitNegative` | OK |
| 1..6 | Alarm | `0x00200000` | `ledAxis{N}Alarm` | OK |
| 1..6 | EMG | `0x00010000` | `ledAxis{N}EMG` | OK |
| 1..6 | Error All | `0x00000001` | `ledAxis{N}ErrorAll` | OK |

## 5. Button mapping

| Button | Service/action/topic | Request fields | Test result |
|---|---|---|---|
| `btnRobotEnable` | `/robot_hw/servo_all` | `std_srvs/SetBool.data = !robot_servo_on` | type fixed; live click not sent to avoid robot command |
| `btnRobotDisable` | `/robot_hw/servo_all` | `std_srvs/SetBool.data = false` | type fixed; live click not sent to avoid robot command |
| `btnAxis{N}Enable` | `/robot_hw/servo_all` | `data = true` | connected |
| `btnAxis{N}Home` | `/robot_hw/home` | `id = N - 1` by default | connected |
| `btnAxis{N}Run` | `/robot_hw/run_axis` + `/arm_controller/joint_trajectory` | `id`, `pos` rad, `vel` rad/s | connected |
| `btnAxis{N}Stop` | `/robot_hw/stop_axis` | `id = N - 1` by default | connected |
| `btnAxis{N}JogPositive` | `/robot_hw/jog` while pressed, `/robot_hw/stop_axis` on release | `id`, `vel`, `dir=1` | connected |
| `btnAxis{N}JogNegative` | `/robot_hw/jog` while pressed, `/robot_hw/stop_axis` on release | `id`, `vel`, `dir=0` | connected |
| `btnStopTask` | `/robot_hw/stop_all` | empty request | connected |

## 6. Removed old connected blocking

- Locations:
  - Robot tab C++ button handlers do not check an old `connected` flag.
  - Service calls check actual ROS client availability with `service_is_ready()`.
- New behavior:
  - If service is available, request is sent.
  - If service is unavailable, a clear ROS log warning is emitted.
  - Missing `/robot_hw/flags` leaves LEDs inactive and GUI remains usable.

## 7. Test result

- Build:
  - `colcon build --packages-select robot_gui robot_hardware_interface --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`: OK.
- GUI standalone:
  - `ros2 launch robot_gui robot_gui.launch.py embed_rviz:=false initial_page:=2`: OK.
  - GUI exits cleanly on Ctrl-C.
- Widget found logs:
  - All checked Robot tab buttons, command inputs, and LED widgets for Axis 1..6 reported `found yes`.
- Flags update:
  - Mock publish to `/robot_hw/flags` verified LED update visually.
  - Screenshot: `/tmp/robot_gui_robot_tab_flags.png`.
  - Tested values:
    - Axis 1 `0x00100000` -> Servo On green.
    - Axis 2 `0x08000000` -> Running green.
    - Axis 3 `0x00200000` -> Alarm red.
    - Axis 4 `0x00010000` -> EMG red.
    - Axis 5 `0x00000001` -> Error All red.
    - Axis 6 `0x02000000` -> Origin OK green.
- Servo all:
  - Runtime service type verified as `std_srvs/srv/SetBool`.
  - C++ client now matches `/robot_hw/servo_all`.
  - Live button click was not executed during this task to avoid sending an unintended servo command to the real robot.
- Remaining issues:
  - `robot_gui.ui` still logs stylesheet parse warnings for `btnAxis1Enable` through `btnAxis6Enable`; this task did not modify `.ui`.
  - `btnAxis{N}AlarmReset` remains log-only because no active alarm reset service is wired in the current C++ GUI.
