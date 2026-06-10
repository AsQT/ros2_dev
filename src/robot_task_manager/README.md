# robot_task_manager

ROS 2 task manager providing high-level actions (`GoHome`, `MoveToPose`, `MoveGripper`, `PickPlace`, `CheckerBoard`, etc.) built on top of MoveIt 2.

## Package Structure

```
robot_task_manager/
├── src/
│   ├── pickplace_server.cpp      # PickPlace action server
│   └── (other action servers)
├── include/
│   └── robot_task_manager/       # C++ headers
├── action/
│   ├── GoHome.action
│   ├── MoveToPose.action
│   ├── MoveGripper.action
│   ├── PickPlace.action
│   └── (other actions)
├── launch/
│   ├── task_servers.launch.py   # Real hardware
│   └── task_servers_sim.launch.py  # Simulation
├── CMakeLists.txt
└── package.xml
```

## Build

```bash
cd ~/ros2
colcon build --packages-select robot_task_manager
source install/setup.bash
```

## Run

```bash
# Real hardware
ros2 launch robot_task_manager task_servers.launch.py

# Simulation
ros2 launch robot_task_manager task_servers_sim.launch.py
```

## Actions

### GoHome

Return robot to home position.

```bash
ros2 action send_goal /gohome robot_task_manager/action/GoHome "{start: true}" --feedback
```

### MoveToPose

Move end-effector to a target pose.

```bash
ros2 action send_goal /move_to_pose robot_task_manager/action/MoveToPose \
  "{target_pose: {position: {x: 0.4, y: 0.1, z: 0.35}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}, velocity_scale: 0.5}" --feedback
```

### MoveGripper

Open/close the gripper.

```bash
ros2 action send_goal /move_gripper robot_task_manager/action/MoveGripper "{position: 0.03}" --feedback
```

### PickPlace

Pick an object from `pose_pick` and place it at `pose_place`.

```bash
ros2 action send_goal /pickplace robot_task_manager/action/PickPlace \
  "{pose_pick: {position: {x: 0.40, y: 0.10, z: 0.03}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}, \
   pose_place: {position: {x: 0.30, y: -0.10, z: 0.1}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}, \
   gripper: 0.025, velocity_scale: 0.2}" --feedback
```

### CheckerBoard

Scan a grid of positions on the table (useful for calibration / workspace mapping).

```bash
ros2 action send_goal /move_checker_board robot_task_manager/action/CheckerBoard \
  "{step: 0.1, velocity_scale: 0.1}" --feedback
```

## Robot Workspace Limits (approximate)

| Axis | Range |
|------|-------|
| x | 0.25 – 0.55 m |
| y | -0.45 – 0.45 m |
| z | 0.05 – 0.20 m |

## Dependencies

- `moveit_ros_planning_interface` — MoveIt planning and execution
- `moveit_visual_tools` — Visualization in RViz
- `moveit_core` — Core MoveIt types
- `rclcpp_action` — Action server/client
- `tf2` / `tf2_ros` — Transform handling
