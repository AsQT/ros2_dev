# robot_pick_place

PyQt6 GUI for ArUco-based pick and place. Displays annotated camera feed, detects ArUco markers, and calls the `robot_task_manager/PickPlace` action.

## Package Structure

```
robot_pick_place/
├── robot_pick_place/
│   └── (Python GUI modules)
├── launch/
│   └── pick_place_gui.launch.py
└── package.xml
```

## Build

```bash
cd ~/ros2
colcon build --packages-select robot_pick_place
source install/setup.bash
```

## Run

```bash
ros2 launch robot_pick_place pick_place_gui.launch.py
```

## Default Topics / Actions

| Resource | Name |
|----------|------|
| Image topic | `/aruco/image_annotated` |
| Pose topic | `/aruco_pose` |
| Action | `/pickplace` |

## Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_fixed_pick_orientation` | `true` | Use fixed orientation for pick (recommended) |
| `pick_qx`, `pick_qy`, `pick_qz`, `pick_qw` | `0.7071, 0.7071, 0.0, 0.0` | Fixed pick quaternion |
| `pick_z_offset` | `0.0` | Z offset above pick point |
| `gripper` | `0.010` | Gripper opening width |
| `velocity_scale` | `0.30` | Motion velocity scale |

### Use Fixed vs. ArUco Orientation

```bash
# Fixed orientation (recommended — avoids singularity)
ros2 launch robot_pick_place pick_place_gui.launch.py use_fixed_pick_orientation:=true

# Use quaternion from /aruco_pose (may cause IK issues)
ros2 launch robot_pick_place pick_place_gui.launch.py use_fixed_pick_orientation:=false
```

### Custom Pick/Place Positions

```bash
ros2 launch robot_pick_place pick_place_gui.launch.py \
  use_fixed_pick_orientation:=true \
  pick_qx:=0.7071 pick_qy:=0.7071 pick_qz:=0.0 pick_qw:=0.0 \
  place_x:=0.300 place_y:=0.000 place_z:=0.250 \
  place_qx:=0.7071 place_qy:=0.7071 place_qz:=0.0 place_qw:=0.0 \
  gripper:=0.010 velocity_scale:=0.30
```

## Dependencies

- `cv_bridge` — OpenCV ↔ ROS image conversion
- `robot_vision_pipeline_msgs` — ArUco pose types
- `robot_task_manager` — PickPlace action
- `python3-opencv`, `python3-numpy`, `python3-pyqt6`
