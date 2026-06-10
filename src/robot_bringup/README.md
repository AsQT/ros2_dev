# robot_bringup

Top-level launch package for bringing up the full robot system. Provides simulation and real-hardware launch files that orchestrate Gazebo, MoveIt, hardware interface, GUI, and task servers.

## Package Structure

```
robot_bringup/
├── launch/
│   ├── sim.launch.py         # Simulation bringup (Gazebo + MoveIt + task servers)
│   └── real.launch.py        # Real hardware bringup (RS485 + GUI + MoveIt + task servers)
└── package.xml
```

## Launch Files

### `sim.launch.py` — Simulation bringup

Starts the robot in Gazebo simulation with MoveIt and task servers.

```bash
ros2 launch robot_bringup sim.launch.py
```

Includes:
- Gazebo simulation
- MoveIt (with `use_sim_time:=true`)
- Task server for simulation

### `real.launch.py` — Real hardware bringup

Starts the robot with real RS485 hardware, GUI, MoveIt, and task servers.

```bash
ros2 launch robot_bringup real.launch.py
```

Includes:
- `robot_hardware_interface` RS485 hardware node
- `robot_gui` Qt-based robot GUI
- MoveIt (with `use_sim_time:=false`)
- Task server for real hardware

## Architecture

This package is the **entry point** for launching the robot. It coordinates:

```
robot_bringup
├── robot_description        # URDF / xacro definitions
├── robot_moveit             # MoveIt configuration and move_group
├── robot_hardware_interface # RS485 communication (real only)
├── robot_gui                # GUI for hardware control (real only)
└── robot_task_manager      # High-level task actions
```

## Dependencies

The launch files depend on:
- `robot_description`
- `robot_moveit`
- `robot_hardware_interface`
- `robot_gui`
- `robot_task_manager`
