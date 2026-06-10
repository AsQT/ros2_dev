# robot_reachability

ROS 2 + MoveIt 2 package for scanning the robot's reachable workspace. Sweeps through a 3D grid of positions and orientation angles, queries MoveIt IK and state validity, and produces a CSV reachability map and RViz marker visualization.

## Package Structure

```
robot_reachability/
├── robot_reachability/
│   └── (Python node modules)
├── config/
│   └── reachability_scan.yaml  # Scan parameters
├── launch/
│   └── reachability_scan.launch.py
└── package.xml
```

## What It Does

At each grid point `(x, y, z)`, the node tries many TCP orientations:

- `roll`: from `-45°` to `45°`
- `pitch`: from `-45°` to `45°`
- `yaw`: fixed at `0°`

For each orientation, it calls MoveIt services:
- `/compute_ik`
- `/check_state_validity`

A point is **reachable** if it has at least one valid IK solution.

## Prerequisites

MoveIt must be running before starting this package:

```bash
ros2 service list | grep compute_ik
ros2 service list | grep check_state_validity
```

## Run

```bash
ros2 launch robot_reachability reachability_scan.launch.py
```

Or directly:

```bash
ros2 run robot_reachability reachability_scan_node \
    --ros-args \
    --params-file ~/ros2/src/robot_reachability/config/reachability_scan.yaml
```

## Output

### CSV File

Default path: `/tmp/reachability_map.csv`

| Column | Description |
|--------|-------------|
| `x_m, y_m, z_m` | TCP position in `base_frame` (meters) |
| `valid_orientation_count` | Number of valid roll/pitch pairs |
| `first_valid_roll_deg` | First valid roll angle |
| `first_valid_pitch_deg` | First valid pitch angle |
| `yaw_deg` | Fixed yaw angle |
| `valid_roll_pitch_pairs_deg` | List of valid `(roll:pitch)` pairs |

### RViz Visualization

| Display | Value |
|---------|-------|
| Type | `MarkerArray` |
| Topic | `/reachability_markers` |
| Fixed Frame | `world` (or `base_frame`) |

## Scan Parameters

Edit `config/reachability_scan.yaml`:

```yaml
base_frame: "world"
group_name: "arm"
ik_link_name: "tcp_link"

x_min: 0.20
x_max: 0.75
y_min: -0.35
y_max: 0.35
z_min: 0.05
z_max: 0.55
xyz_step: 0.05          # Coarse scan

roll_min_deg: -45.0
roll_max_deg: 45.0
pitch_min_deg: -45.0
pitch_max_deg: 45.0
angle_step_deg: 15.0   # Coarse orientation
yaw_deg: 0.0

check_approach: true    # Require approach pose too
approach_offset_z: 0.10
```

For pick/place: first do a coarse scan (`step: 0.05, angle_step: 15°`), then a fine scan (`step: 0.025, angle_step: 10°`) on the promising region.

## Dependencies

- `moveit_msgs` — IK and state validity services
- `geometry_msgs` — pose construction
- `visualization_msgs` — marker publishing
