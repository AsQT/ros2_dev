# `robot_drl`

Deep Reinforcement Learning inference package for autonomous robot pick-and-place. Runs pre-trained DDPG/SAC/TD3 policies to compute smooth robot trajectories from vision detections or manual input, then executes them via MoveIt.

**Preferred node:** `drl_unified_planner_node` — a single unified node that handles planning, execution, and visualization for both manual and vision modes.

## Package Structure

```
robot_drl/
├── robot_drl/
│   ├── __init__.py
│   ├── config.py                 # FRAME_Z_OFFSET, ACTION_STEP constants
│   ├── model_loader.py           # Stable-Baselines3 model loading + inference
│   ├── state_builder.py         # 15D observation vector builder
│   ├── drl_planner_core.py      # Trajectory computation logic
│   ├── drl_planner_node_base.py # Base class (publishers, services, execution)
│   ├── drl_unified_planner_node.py  # [PREFERRED] single unified node
│   └── mock_environment_node.py  # Synthetic vision data for simulation
├── launch/
│   ├── main.launch.py            # Full stack: robot + vision + planner
│   ├── drl_unified_planner.launch.py  # Planner + robot bringup
│   ├── mock_environment.launch.py     # Synthetic vision only
│   ├── rl_sim_rviz.launch.py         # RL sim + RViz
│   └── rviz_drl.launch.py             # RViz only
├── models/                       # Saved policy (.zip) + VecNormalize stats
├── rviz/
│   ├── DRL_Rviz.rviz
│   └── drl_markers.rviz
├── setup.py
└── package.xml
```

## Quick Start

```bash
# Full stack via unified planner (recommended)
ros2 launch robot_drl main.launch.py

# Or directly
ros2 launch robot_drl drl_unified_planner.launch.py
```

## Node: `drl_unified_planner_node` (preferred)

Single node handling both manual and vision input modes, planning, and execution.

```bash
# Manual mode (terminal prompt)
ros2 run robot_drl drl_unified_planner_node \
    --ros-args \
    -p input_mode:=manual \
    -p auto_plan_on_start:=true

# Vision mode (live detections)
ros2 run robot_drl drl_unified_planner_node \
    --ros-args \
    -p input_mode:=vision \
    -p auto_plan_on_start:=false
```

### Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `input_mode` | `manual` | `manual` (terminal) or `vision` (live detections) |
| `auto_plan_on_start` | `true` | Auto-plan on startup (manual mode only) |
| `calibrated_start_tcp_base` | `[0.5241, 0.000, 0.315]` | Start TCP in base_link [x, y, z] m |
| `use_sim_time` | `false` | Use /clock for simulation |

Trained workspace in `base_link`: `x=[0.425, 0.675]`, `y=[-0.200, 0.200]`, `z=[0.020, 0.600]`.

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/drl/forward_trajectory_marker` | `MarkerArray` | Forward trajectory markers |
| `/drl/backward_trajectory_marker` | `MarkerArray` | Backward trajectory markers |
| `/drl/forward_trajectory_poses` | `PoseArray` | Forward trajectory waypoints |
| `/drl/backward_trajectory_poses` | `PoseArray` | Backward trajectory waypoints |
| `/drl/next_pose` | `PoseStamped` | Streaming waypoint during execution |
| `/drl/execution_status` | `String` | Execution state at 2 Hz |

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/drl/plan` | `std_srvs/Trigger` | Plan a new trajectory |
| `/drl/replan` | `std_srvs/Trigger` | Force a replan |
| `/drl/execute_forward` | `std_srvs/Trigger` | Execute forward trajectory |
| `/drl/execute_backward` | `std_srvs/Trigger` | Execute backward trajectory |
| `/drl/clear_trajectory` | `std_srvs/Trigger` | Clear all trajectory state |
| `/drl/get_execution_status` | `std_srvs/Trigger` | Query execution state |

## Coordinate Frames

| Frame | Description | Z reference |
|-------|-------------|-------------|
| `base_link` | Robot kinematic root | 0 = mounting surface |
| `WORLD` / `DRL` | PyBullet training frame | 0 = table surface |

Conversion: `base_link.z = WORLD/DRL.z + 0.330`

## Coordinate Frame Notes

- All trajectory poses are in `base_link` frame.
- The DRL model was trained in a PyBullet frame where `z=0` is the table surface.
- The `FRAME_Z_OFFSET = 0.330` maps PyBullet `z` to `base_link.z`.
- Gripper tool points downward: `RPY = [π, 0, 0]`.

## Removed Deprecated Nodes

| Node | Replacement |
|------|-------------|
| `trajectory_test_node` | `drl_unified_planner_node` |
| `manual_trajectory_test_node` | `drl_unified_planner_node input_mode:=manual` |
| `vision_trajectory_preview_node` | `drl_unified_planner_node input_mode:=vision` |
| `drl_inference_node` | `drl_unified_planner_node` |
| `drl_action_bridge_node` | `drl_unified_planner_node` |

## Dependencies

- `stable-baselines3` — DRL policy inference
- `robot_task_executor_msgs` — service types
- `robot_vision_pipeline` — object detection input (vision mode)
- `moveit_ros_planning_interface` — trajectory execution
