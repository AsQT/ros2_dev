# robot_control

ROS 2 `ament_cmake` package providing ros2_control configuration and launch files for the robot. Defines joint controllers, controller manager setup, and URDF xacro snippets for ros2_control hardware plugin selection.

## Package Structure

```
robot_control/
├── launch/
│   ├── controllers.launch.py    # Main controller manager + spawners
│   └── slider_controller.launch.py  # Joint slider GUI for testing
├── config/
│   └── robot_controllers.yaml   # Controller definitions (arm + gripper)
└── package.xml
```

## Launch Files

### `controllers.launch.py`

Spawns the ros2_control node and starts all controllers.

```bash
ros2 launch robot_control controllers.launch.py
```

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `false` | Use simulation time |
| `use_mock` | `true` | Use mock hardware instead of real RS485 |
| `start_controller_manager` | `true` | Start local ros2_control node |

Spawns in order:
1. `joint_state_broadcaster`
2. `arm_controller` (after broadcaster is ready)
3. `gripper_controller` (after arm is ready)

### `slider_controller.launch.py`

Joint slider GUI for manual joint control. Forwards `/joint_states` as `/joint_commands`.

```bash
ros2 launch robot_control slider_controller.launch.py is_sim:=false is_ignition:=false use_mock_hardware:=true
```

## ros2_control Hardware Selection (URDF)

The xacro macro `robot_ros2_control` in `robot_description` selects hardware based on flags:

| Mode | Plugin |
|------|--------|
| Simulation (Gazebo) | `gz_ros2_control/GazeboSimSystem` |
| Real + mock | `mock_components/GenericSystem` |
| Real hardware | `robot_hardware_interface/RobotSystemHardware` |

## Joint Interfaces

All joints expose both **position** and **velocity** command/state interfaces:

- `joint_1` through `joint_6` — 6-DOF arm
- `joint_gl`, `joint_gr` — gripper left/right

## Build

```bash
cd ~/ros2
colcon build --packages-select robot_control
source install/setup.bash
```
