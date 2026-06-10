# `robot_task_executor`

High-level task executor for the robot using MoveIt 2 (C++). Provides a service-based interface to MoveIt, abstracting away trajectory planning and execution.

**Role in the project:** Sits between high-level callers (GUI, DRL nodes, CLI) and the MoveIt `move_group` action server. All motion execution goes through MoveIt — no direct joint streaming.

## Package Structure

```
robot_task_executor/
├── src/
│   └── task_executor_node.cpp       # Main C++ executable
├── include/
│   └── robot_task_executor/         # C++ headers
├── config/
│   ├── joint_waypoints.yaml       # Named joint targets (degrees)
│   ├── pose_waypoints.yaml        # Named pose targets (m + rad)
│   └── cartesian_points.yaml       # Named Cartesian points (m only)
├── launch/
│   └── task_executor.launch.py
├── CMakeLists.txt
└── package.xml
```

## Node: `task_executor_node`

```bash
ros2 launch robot_task_executor task_executor.launch.py
```

### Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `move_group_name` | `arm` | MoveIt planning group |
| `base_frame` | `base_link` | Base reference frame |
| `ee_link` | `tcp_link` | End-effector link |
| `planning_time` | `2.0` | Planning time (s) |
| `num_planning_attempts` | `5` | Number of planning attempts |
| `max_velocity_scaling_factor` | `0.5` | Velocity scaling (0–1) |
| `max_acceleration_scaling_factor` | `0.5` | Acceleration scaling (0–1) |
| `waypoints_config` | `config/joint_waypoints.yaml` | Joint waypoints file |
| `cartesian_points_config` | `config/cartesian_points.yaml` | Cartesian points file |
| `pose_waypoints_config` | `config/pose_waypoints.yaml` | Pose waypoints file |

## Services

All services use types from `robot_task_executor_msgs`:

| Service | Type | Purpose |
|---------|------|---------|
| `/move_to_named_target` | `MoveToNamedTarget` | Move to an SRDF named state |
| `/move_to_joint_target` | `MoveToJointTarget` | Move to a joint-space goal |
| `/move_to_pose_target` | `MoveToPoseTarget` | Move to a Cartesian pose goal |
| `/move_to_cartesian_target` | `MoveToCartesianTarget` | Move to xyz goal (fixed orientation) |
| `/move_to_named_cartesian_target` | `MoveToNamedCartesianTarget` | Move to a named Cartesian point |
| `/move_to_named_pose_target` | `MoveToNamedPoseTarget` | Move to a named pose waypoint |
| `/move_sequence` | `MoveSequence` | Execute sequence of named joint waypoints |
| `/move_cartesian_sequence` | `MoveCartesianSequence` | Execute sequence of named Cartesian points |
| `/move_cartesian_pose_sequence` | `MoveCartesianPoseSequence` | Execute arbitrary Cartesian poses (DRL output) |

## Config Files

### `config/joint_waypoints.yaml`
Named joint targets. **All values are in degrees** (converted to radians at load time).

```yaml
waypoints:
  start:
    description: "DRL task starting position"
    joints: [-43.32, 6.88, -26.96, 0.0, -56.16, -43.32]
  origin:
    description: "Robot at zero position"
    joints: [0.0, 0.0, 0.0, 0.0, -90.0, 0.0]
  P1:
    joints: [0.0, 6.05, -26.3, 0.0, -57.65, 0.0]
```

Joint order: `joint_1, joint_2, joint_3, joint_4, joint_5, joint_6`.

### `config/pose_waypoints.yaml`
Named pose targets. **Position in meters, RPY in radians.**

```yaml
pose_waypoints:
  pose_A:
    position: [0.5241, 0.0, 0.315]
    rpy: [3.1416, 0.0, 0.0]
```

### `config/cartesian_points.yaml`
Named Cartesian points. **Position in meters, fixed RPY=[π, 0, 0]** (tool pointing downward).

```yaml
cartesian_points:
  pose_A:
    frame_id: "base_link"
    xyz: [0.000, -0.481, 0.315]
```

## Usage Examples

```bash
# Move to named target
ros2 service call /move_to_named_target \
  robot_task_executor_msgs/srv/MoveToNamedTarget \
  "{target_name: 'start', execute: true}"

# Move to joint target
ros2 service call /move_to_joint_target \
  robot_task_executor_msgs/srv/MoveToJointTarget \
  "{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'], \
   positions: [-0.7561, 0.1201, -0.4705, 0.0, -0.9802, -0.7561], execute: true}"

# Execute DRL-generated pose sequence
ros2 service call /move_cartesian_pose_sequence \
  robot_task_executor_msgs/srv/MoveCartesianPoseSequence \
  "{poses: [{header: {frame_id: 'base_link'}, \
            pose: {position: {x: 0.3, y: -0.3, z: 0.2}, \
                  orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}], execute: true}"
```

## Troubleshooting

### "Target not found"
The `target_name` field is a plain `string`, not a nested message:
```
Wrong:  '{target_name: {data: start}}'
Right: "'{target_name: \\'start\\'}'"
```

### Planning fails with "start state invalid"
1. Move to the `start` named target first.
2. Increase `num_planning_attempts` or `planning_time`.

### Service call format errors
```bash
ros2 srv show robot_task_executor_msgs/srv/MoveToNamedTarget
```
