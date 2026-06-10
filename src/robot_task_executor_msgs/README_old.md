# `robot_task_executor_msgs`

**Purpose:** ROS 2 service definitions for `robot_task_executor`. Contains only message definitions — no nodes, no executables.

**Role in the Project:** All services for commanding the robot through MoveIt 2 are defined here. Other packages (`robot_task_executor`, `robot_drl`, `robot_gui`) depend on this package for the service types.

---

## Package Structure

```
robot_task_executor_msgs/
├── srv/
│   ├── MoveToNamedTarget.srv
│   ├── MoveToJointTarget.srv
│   ├── MoveToPoseTarget.srv
│   ├── MoveToCartesianTarget.srv
│   ├── MoveToNamedCartesianTarget.srv
│   ├── MoveToNamedPoseTarget.srv
│   ├── MoveSequence.srv
│   ├── MoveCartesianSequence.srv
│   └── MoveCartesianPoseSequence.srv
├── CMakeLists.txt
└── package.xml
```

---

## Main Nodes

None. This package contains only service definitions.

---

## Launch Files

None.

---

## Services

### `MoveToNamedTarget.srv`

Move to an SRDF named state (e.g., `"Home"`, `"start"`).

```
string target_name
bool execute
---
bool success
string message
```

### `MoveToJointTarget.srv`

Move to a joint-space goal.

```
string[] joint_names
float64[] positions
bool execute
---
bool success
string message
```

### `MoveToPoseTarget.srv`

Move to a Cartesian pose goal for the end-effector.

```
geometry_msgs/Pose pose
string frame_id
bool execute
---
bool success
string message
```

### `MoveToCartesianTarget.srv`

Move to an xyz goal with fixed orientation (RPY = [π, 0, 0]).

```
float64 x
float64 y
float64 z
string frame_id
bool execute
---
bool success
string message
float64 fraction
```

### `MoveToNamedCartesianTarget.srv`

Move to a named Cartesian point from `cartesian_points.yaml`.

```
string target_name
bool execute
---
bool success
string message
float64 fraction
```

### `MoveToNamedPoseTarget.srv`

Move to a named pose from `pose_waypoints.yaml`.

```
string target_name
bool execute
---
bool success
string message
```

### `MoveSequence.srv`

Execute a sequence of named joint waypoints.

```
string[] waypoint_names
bool execute
---
bool success
string message
```

### `MoveCartesianSequence.srv`

Execute a sequence of named Cartesian points from `cartesian_points.yaml`.

```
string[] waypoint_names
bool execute
---
bool success
string message
float64 fraction
```

### `MoveCartesianPoseSequence.srv`

Execute an arbitrary sequence of Cartesian poses. Used by `robot_drl` to send pre-computed DRL waypoints.

```
geometry_msgs/PoseStamped[] poses
bool execute
---
bool success
string message
float64 fraction
```

- `poses`: All poses must be in `base_link` frame. Empty `frame_id` defaults to `base_link`. Zero quaternion is replaced with RPY=[π, 0, 0].
- `fraction`: MoveIt Cartesian path fraction (0.0–1.0). Below 0.95 → service returns `success=false`.

---

## How to Run

```bash
colcon build --packages-select robot_task_executor_msgs
source install/setup.bash

ros2 srv show robot_task_executor_msgs/srv/MoveToNamedTarget
ros2 srv show robot_task_executor_msgs/srv/MoveCartesianPoseSequence
```

---

## Debugging / Troubleshooting

### Service type not found
```bash
ros2 pkg list | grep robot_task_executor_msgs
ros2 pkg list | grep robot_task_executor
```

Rebuild `robot_task_executor` after building `robot_task_executor_msgs` to ensure headers are regenerated.

---

## TODO

- [ ] No TODOs for this package — it contains only static interface definitions
