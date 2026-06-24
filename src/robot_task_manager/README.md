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
│   ├── RepeatabilityTest.action
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

### RepeatabilityTest

Run a GT10 repeatability measurement sequence without reading GT10 data in ROS. The action name is `/repeatability_test` and the server executable is `repeatability_test_server`.

Sequence:

1. `MoveToPose` to `retract_pose`
2. For each loop: Cartesian move to `meas_pose`, wait 2 seconds, Cartesian back to `retract_pose`, move to `disturb_pose_1`, move to `disturb_pose_2`, move back to `retract_pose`

`meas_pose` is computed from `retract_pose`; `axis=0` adds `meas_offset` to `x`, and `axis=1` adds it to `y`. Orientation is always copied from `retract_pose`. `velocity_scale` is passed to both `MoveToPose` and `MoveToPoseCartesian`.

Build:

```bash
cd ~/ros2_dev
colcon build --packages-select robot_task_manager
source install/setup.bash
```

Run servers with mock hardware:

```bash
ros2 launch robot_task_manager task_servers.launch.py
```

Quick X-axis test, 3 loops:

```bash
ros2 launch robot_task_manager repeatability_test_client.launch.py \
  axis:=0 repeat_count:=3 meas_offset:=0.02 velocity_scale:=0.25
```

Quick Y-axis test:

```bash
ros2 launch robot_task_manager repeatability_test_client.launch.py \
  axis:=1 repeat_count:=3 meas_offset:=0.02 velocity_scale:=0.25
```

Direct action command:

```bash
ros2 action send_goal /repeatability_test robot_task_manager/action/RepeatabilityTest \
  "{retract_pose: {header: {frame_id: 'world'}, pose: {position: {x: 0.40, y: 0.00, z: 0.18}, orientation: {x: 0.7071068, y: 0.7071068, z: 0.0, w: 0.0}}}, disturb_pose_1: {header: {frame_id: 'world'}, pose: {position: {x: 0.35, y: -0.08, z: 0.18}, orientation: {x: 0.7071068, y: 0.7071068, z: 0.0, w: 0.0}}}, disturb_pose_2: {header: {frame_id: 'world'}, pose: {position: {x: 0.45, y: 0.08, z: 0.18}, orientation: {x: 0.7071068, y: 0.7071068, z: 0.0, w: 0.0}}}, axis: 0, meas_offset: 0.02, repeat_count: 3, velocity_scale: 0.25}" --feedback
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
