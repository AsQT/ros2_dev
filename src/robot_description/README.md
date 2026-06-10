# robot_description

URDF description package for the robot. Contains xacro files, robot model definitions, and ros2_control hardware configuration.

## Package Structure

```
robot_description/
├── urdf/
│   ├── robot.urdf.xacro       # Main robot model (includes other xacros)
│   └── ros2_control.xacro     # ros2_control hardware plugin macro
├── launch/
│   └── gazebo.launch.py       # Gazebo world + robot spawner
├── rviz/
│   └── robot.rviz              # RViz configuration
├── meshes/                     # STL/DAE mesh files (if any)
└── package.xml
```

## Build

```bash
cd ~/ros2
colcon build --packages-select robot_description
source install/setup.bash
```

## Generate URDF from Xacro

```bash
# View generated URDF
xacro ~/ros2/src/robot_description/urdf/robot.urdf.xacro

# With parameters
xacro ~/ros2/src/robot_description/urdf/robot.urdf.xacro use_sim:=false use_mock:=false
```

## ros2_control Hardware Selection

The `ros2_control.xacro` macro supports three hardware backends:

```xml
<!-- Simulation (Gazebo) -->
<xacro:robot_ros2_control name:="robot_system" use_sim:="true" />

<!-- Real robot with mock (testing without hardware) -->
<xacro:robot_ros2_control name:="robot_system" use_sim:="false" use_mock:="true" />

<!-- Real robot with RS485 hardware -->
<xacro:robot_ros2_control name:="robot_system" use_sim:="false" use_mock:="false" />
```

## Joint Definitions

| Joint | Type | Description |
|-------|------|-------------|
| `joint_1` – `joint_6` | 6-DOF Arm | Position + velocity interfaces |
| `joint_gl` | Gripper Left | Position + velocity interfaces |
| `joint_gr` | Gripper Right | Position + velocity interfaces |

## Dependencies

- `robot_state_publisher` — publishes TF from URDF
- `joint_state_publisher_gui` — joint slider GUI
- `rviz2` — 3D visualization
- `xacro` — URDF generation
- `ros_gz_interfaces` — Gazebo simulation bridge
