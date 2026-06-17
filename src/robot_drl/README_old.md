# `robot_drl`

**Purpose:** Deep Reinforcement Learning inference for autonomous pick-and-place — runs a pre-trained DDPG/SAC/TD3 policy to compute robot trajectories from vision detections or manual input.

**Preferred node:** `drl_unified_planner_node`

---

## Quick Start

```bash
# Full stack via unified planner (recommended)
ros2 launch robot_drl main.launch.py

# Or directly via drl_unified_planner.launch.py
ros2 launch robot_drl drl_unified_planner.launch.py
```

---

## Package Structure

```
robot_drl/
├── robot_drl/
│   ├── __init__.py
│   ├── config.py                     # FRAME_Z_OFFSET, ACTION_STEP constants
│   ├── model_loader.py               # Stable-Baselines3 model loading + inference
│   ├── state_builder.py              # 15D observation vector builder
│   ├── drl_planner_core.py           # Trajectory computation logic
│   ├── drl_planner_node_base.py      # Base class (publishers, services, execution)
│   ├── drl_unified_planner_node.py  # [PREFERRED] single unified node
│   └── mock_environment_node.py       # Synthetic vision data for simulation
├── launch/
│   ├── main.launch.py               # Full stack (robot + vision + planner)
│   ├── drl_unified_planner.launch.py # Unified planner + robot bringup
│   └── mock_environment.launch.py    # Synthetic vision data only
├── models/                           # Saved policy (.zip) + VecNormalize stats
├── scripts/
│   └── export_joint_state.py
├── setup.py
└── package.xml
```

---

## Node

### `drl_unified_planner_node` (preferred)

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

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `input_mode` | `manual` | `manual` (terminal) or `vision` (live detections) |
| `auto_plan_on_start` | `true` | Auto-plan on startup (manual mode only) |
| `calibrated_start_tcp_base` | `[0.5241, 0.000, 0.315]` | Start TCP in base_link frame [x, y, z] m |
| `use_sim_time` | `false` | Use /clock for simulation time |

Trained workspace in `base_link`: `x=[0.425, 0.675]`, `y=[-0.200, 0.200]`, `z=[0.020, 0.600]`.

#### Node Parameters (advanced)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `fk_fallback_warning` | `true` | Warn if FK fallback is used |
| `execute_mode` | `pose_sequence` | Execution mode |
| `task_executor_service_timeout_sec` | `5.0` | Service call timeout |
| `task_executor_result_timeout_sec` | `120.0` | Execution timeout |
| `publish_next_pose_during_execute` | `true` | Stream waypoints during execution |
| `start_pose_tolerance` | `0.001` | Start pose tolerance (m) |

---

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/drl/forward_trajectory_marker` | `MarkerArray` | Forward trajectory (start → target) |
| `/drl/backward_trajectory_marker` | `MarkerArray` | Backward trajectory (target → start) |
| `/drl/forward_trajectory_poses` | `PoseArray` | Forward trajectory waypoints |
| `/drl/backward_trajectory_poses` | `PoseArray` | Backward trajectory waypoints |
| `/drl/next_pose` | `PoseStamped` | Streaming waypoint during execution |
| `/drl/execution_status` | `String` | Execution state at 2 Hz |

---

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/drl/plan` | `std_srvs/Trigger` | Plan a new trajectory |
| `/drl/replan` | `std_srvs/Trigger` | Force a replan |
| `/drl/execute_forward` | `std_srvs/Trigger` | Execute forward trajectory |
| `/drl/execute_backward` | `std_srvs/Trigger` | Execute backward trajectory |
| `/drl/clear_trajectory` | `std_srvs/Trigger` | Clear all trajectory state |
| `/drl/get_execution_status` | `std_srvs/Trigger` | Query execution state |

---

## Coordinate Frames

| Frame | Description | Z reference |
|-------|-------------|-------------|
| `base_link` | Robot kinematic root | 0 = mounting surface |
| `WORLD` / `DRL` | PyBullet training frame | 0 = table surface |

Conversion: `base_link.z = WORLD/DRL.z + 0.330`

---

## Visualization

Primary RViz topic: `/scene/markers` (via `robot_visualization/scene_visualization_node`)

Debug source topics:
- `/drl/forward_trajectory_marker`
- `/drl/backward_trajectory_marker`

See `robot_visualization/README.md` for details.

---

## Launch Files

### `main.launch.py` — Full stack (recommended)

Starts robot bringup + vision + unified planner in one command.

```bash
ros2 launch robot_drl main.launch.py
ros2 launch robot_drl main.launch.py input_mode:=vision
ros2 launch robot_drl main.launch.py vision:=real
```

### `drl_unified_planner.launch.py` — Planner + bringup

```bash
ros2 launch robot_drl drl_unified_planner.launch.py
ros2 launch robot_drl drl_unified_planner.launch.py input_mode:=vision
```

### `mock_environment.launch.py` — Synthetic vision only

```bash
ros2 launch robot_drl mock_environment.launch.py
ros2 launch robot_drl mock_environment.launch.py target_x:=0.2 target_y:=-0.3 target_z:=0.4
```

---

## Removed Old Nodes

The following nodes have been removed from the package:

| Node | Reason |
|------|--------|
| `trajectory_test_node` | Deprecated — replaced by `drl_unified_planner_node` |
| `manual_trajectory_test_node` | Replaced by `drl_unified_planner_node input_mode:=manual` |
| `vision_trajectory_preview_node` | Replaced by `drl_unified_planner_node input_mode:=vision` |
| `drl_inference_node` | Replaced by `drl_unified_planner_node` |
| `drl_action_bridge_node` | Replaced by `drl_unified_planner_node` |

All planning, execution, and visualization logic is now handled by the single `drl_unified_planner_node`.

---

## Debugging

### No trajectory markers in RViz
1. Set Fixed Frame to `base_link`
2. Check: `ros2 topic echo /drl/forward_trajectory_marker --once`
3. Verify `robot_visualization` is running: `ros2 node list | grep scene`

### Trajectory offset from robot
Verify `FRAME_Z_OFFSET = 0.330` matches actual table height.

### DRL policy not loading
```bash
ls ~/ros2/src/robot_drl/models/
```
ros2 launch robot_drl drl_mock_hw_obstacle_test.launch.py \
  execute:=true \
  case_count:=1 \
  random_seed:=2 \
  randomize_obstacle_count:=false \
  obstacle_count:=1

  ros2 launch robot_drl drl_mock_hw.launch.py \
  input_mode:=manual \
  auto_plan_on_start:=true \
  auto_execute_after_plan:=true