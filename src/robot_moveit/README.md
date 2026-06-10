# robot_moveit

MoveIt 2 configuration and launch package for the robot arm. Provides the `move_group` node, RViz visualization, semantic robot description (SRDF), and controller configuration.

## Package Structure

```
robot_moveit/
├── launch/
│   └── moveit.launch.py       # Main MoveIt launch
├── config/
│   ├── robot.srdf            # Semantic robot description (named poses, groups)
│   ├── moveit_controllers.yaml  # Controller mapping
│   ├── ompl_planning.yaml    # OMPL planner config
│   ├── moveit.rviz           # RViz configuration
│   └── (other MoveIt configs)
├── CMakeLists.txt
└── package.xml
```

## Launch

```bash
ros2 launch robot_moveit moveit.launch.py
```

### Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `false` | Use simulation time (true for Gazebo) |
| `use_mock` | `true` | Use mock hardware |
| `start_controller_manager` | `true` | Start local ros2_control node |

### What It Starts

- `move_group` node (MoveIt planning)
- `static_transform_publisher` — `world` -> `base_link`
- `robot_state_publisher` — from URDF
- `controller_manager` + spawners (`joint_state_broadcaster`, `arm_controller`, `gripper_controller`)

RViz is **commented out** by default. To enable, uncomment in `moveit.launch.py`.

## Planning Group

The default planning group is named `arm`, containing joints `joint_1` through `joint_6`.

End-effector link: `tcp_link`

## Dependencies

This package depends on:
- `robot_description` — URDF and xacro files
- `controller_manager` / `ros2_controllers` — joint controllers
- `moveit_ros_move_group` — planning core
- `moveit_ros_visualization` — RViz integration
- `moveit_configs_utils` — configuration builder utilities

## Build

```bash
cd ~/ros2
colcon build --packages-select robot_moveit
source install/setup.bash
```
