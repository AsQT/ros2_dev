# Robot GUI Joint State Display Report

## Scope

Task: port joint state display logic from `robot_gui_old` to the current C++ `robot_gui` package without changing `robot_gui/ui/robot_gui.ui`.

Checked files:

- `codex.md`
- `robot_gui_old/robot_gui/main_window.py`
- `robot_gui_old/ui/robot_gui.ui`
- `robot_gui/ui/robot_gui.ui`
- `robot_gui/src/main_window.cpp`
- `robot_gui/include/robot_gui/main_window.hpp`
- `robot_gui/src/robot_gui_node.cpp`
- `robot_gui/include/robot_gui/robot_gui_node.hpp`

Note: `robot_gui_old/robot_gui/gui_win.md` was listed in `codex.md`, but that file is not present in the current workspace.

## Implemented Behavior

- Subscribed topic: `/joint_states`
- Message type: `sensor_msgs/msg/JointState`
- ROS callback location: `robot_gui/src/robot_gui_node.cpp`
- GUI update location: `robot_gui/src/main_window.cpp`
- Threading: ROS executor callback emits `joint_state_received(...)`; Qt slot `update_joint_state_display(...)` updates widgets on the GUI thread.

## Widget Mapping

The existing `.ui` widgets are used directly. No new layout or widget was created.

| Joint name | GUI axis | Position widget | Velocity widget | Status |
|---|---:|---|---|---|
| `joint_1` | Axis 1 | `txtAxis1ActualPos` | `txtAxis1ActualVel` | OK |
| `joint_2` | Axis 2 | `txtAxis2ActualPos` | `txtAxis2ActualVel` | OK |
| `joint_3` | Axis 3 | `txtAxis3ActualPos` | `txtAxis3ActualVel` | OK |
| `joint_4` | Axis 4 | `txtAxis4ActualPos` | `txtAxis4ActualVel` | OK |
| `joint_5` | Axis 5 | `txtAxis5ActualPos` | `txtAxis5ActualVel` | OK |
| `joint_6` | Axis 6 | `txtAxis6ActualPos` | `txtAxis6ActualVel` | OK |

`joint_gl` and `joint_gr` are accepted in `/joint_states`, but the current GUI layout only has actual position/velocity fields for six main axes.

## Units

- Input `/joint_states.position` unit: rad
- Input `/joint_states.velocity` unit: rad/s
- GUI display unit: deg and deg/s, matching `robot_gui_old`
- Conversion used: `value_deg = value_rad * 180.0 / M_PI`

## Test

Build:

```bash
colcon build --packages-select robot_gui --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Result: OK.

Standalone GUI:

```bash
ros2 launch robot_gui robot_gui.launch.py embed_rviz:=false initial_page:=2
```

Result: OK.

Mock publish:

```bash
ros2 topic pub --once /joint_states sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, name: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_gl','joint_gr'], position: [0.1,0.2,0.3,0.4,0.5,0.6,0.01,0.01], velocity: [0.01,0.02,0.03,0.04,0.05,0.06,0.0,0.0], effort: []}"
```

Observed display:

| Axis | Position display | Velocity display |
|---:|---:|---:|
| 1 | `5.730 deg` | `0.573 deg/s` |
| 2 | `11.459 deg` | `1.146 deg/s` |
| 3 | `17.189 deg` | `1.719 deg/s` |
| 4 | `22.918 deg` | `2.292 deg/s` |
| 5 | `28.648 deg` | `2.865 deg/s` |
| 6 | `34.377 deg` | `3.438 deg/s` |

Mock publish test: OK.

Screenshot saved for local inspection:

```text
/tmp/robot_gui_joint_state_display.png
```

## Remaining Notes

- Qt still prints stylesheet parse warnings for `btnAxis1Enable` through `btnAxis6Enable`. This comes from the existing `.ui` stylesheet and was not changed because this task required preserving the current layout/UI file.
