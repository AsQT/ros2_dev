# robot_gui

Qt-based (QtPy/PyQt6) robot GUI providing hardware control, TF visualization, and MoveIt planning interface.

## Package Structure

```
robot_gui/
├── robot_gui/
│   └── (Python modules)
├── launch/
│   └── robot_gui.launch.py
└── package.xml
```

## Tabs

| Tab | Function |
|-----|----------|
| **Hardware** | RS-485 service controls + joint_states readout |
| **TF** | Live pose `base_link` -> `tcp_link` at 0.5 Hz |
| **Planning** | MoveIt plan / execute via action interface |

## Build

```bash
cd ~/ros2
colcon build --packages-select robot_gui
source install/setup.bash
```

## Run

```bash
# Direct
ros2 run robot_gui robot_gui

# Or via launch
ros2 launch robot_gui robot_gui.launch.py
```

## Dependencies

- `robot_hardware_interface` — RS485 hardware service types
- MoveIt up and running (move_group + `/query_planner_interface` + `/display_planned_path`)
- `tf2_ros` — Transform lookups
- `geometry_msgs` / `moveit_msgs` / `shape_msgs` / `sensor_msgs`

## Hardware Tab Notes

The Hardware tab communicates with `robot_hardware_interface` services. Make sure the hardware node is running before using this tab.
