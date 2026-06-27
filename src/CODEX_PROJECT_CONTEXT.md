# Codex Project Context - ros2_dev

> **Purpose:** Onboarding / audit reference for the workspace. This is a *source-of-truth
> orientation document*, not a task. It was produced by reading the actual source tree
> (not by trusting older reports). Every concrete value below was spot-checked against a
> file. Older `*_report.md` / `*_audit.md` / `*_fix.md` files are **history only** and are
> NOT authoritative.
>
> Generated: 2026-06-27. If the tree changes, re-verify before relying on a specific value.

---

## 1. Repository

| Item | Value |
|---|---|
| Workspace root | `/home/minhquang/ros2_dev_2` |
| Source dir | `/home/minhquang/ros2_dev_2/src` |
| Branch read | `main` |
| ROS distro | **ROS 2 Jazzy** (target) |
| Build tool | `colcon` (ament) |
| Sim stack | Gazebo (Sim / `ros_gz`) + `gz_ros2_control` |

> **Discrepancy to note:** the onboarding brief refers to `~/ros2_dev`, but the actual
> workspace on this machine is `~/ros2_dev_2`. Use `~/ros2_dev_2`.

---

## 2. Package inventory

14 packages under `src/`. Only `robot_drl` is `ament_python`; all others are `ament_cmake`.

| Package | Build | Role | Main node(s)/exe | Main launch | Key interfaces | Key deps |
|---|---|---|---|---|---|---|
| **robot_description** | ament_cmake | URDF/Xacro source of truth (robot geometry, frames, ros2_control macro, meshes, worlds) | *(no exe; xacro + helper scripts)* | `display.launch.py` (legacy `gazebo*.launch.py`) | `/robot_description`, TF | xacro, robot_state_publisher |
| **robot_control** | ament_cmake | ros2_control config (controllers) | *(no exe; spawns controller_manager nodes)* | `controllers.launch.py` | `arm_controller`, `gripper_controller`, `joint_state_broadcaster`, `/joint_states` | controller_manager, ros2_controllers |
| **robot_moveit** | ament_cmake | MoveIt 2 config + move_group + RViz | *(move_group; `moveit_gui.py`)* | `moveit.launch.py`, `moveit_gui.launch.py` | `/move_group/*` actions+services | moveit_*, robot_description |
| **robot_hardware_interface** | ament_cmake | Real HW abstraction (TCP/RS485), ros2_control plugin | `robot_hw_node`, `tcp_system_hardware` (plugin) | `hardware_interface.launch.py` | `/robot_hw/*` services, `FlagStatus` msg | hardware_interface, pluginlib |
| **robot_gazebo** | ament_cmake | Gazebo world + robot spawn + ros_gz bridge + object spawners | *(no exe; spawn scripts)* | `gazebo.launch.py` (current) | `/clock`, sim camera, `/sim/*_info` | robot_description, ros_gz_* |
| **robot_task_manager** | ament_cmake | **Action layer** — orchestrates MoveIt/gripper/DRL | 9 `*_server` exes + `task_manager_client` | `task_servers.launch.py`, `task_servers_sim.launch.py` | 9 actions (see §5) | rclcpp_action, moveit, robot_drl |
| **robot_task_executor** | ament_cmake | **Service layer** — MoveIt move/sequence services (YAML waypoints) | `task_executor_node` | `task_executor.launch.py` | `Move*` services | moveit, robot_task_executor_msgs |
| **robot_task_executor_msgs** | ament_cmake | Interface-only: 9 `Move*` service defs | *(none)* | *(none)* | srv defs | geometry_msgs |
| **robot_drl** | **ament_python** | DRL trajectory planner (SB3 TD3/DDPG/SAC) | `drl_unified_planner_node` (+ mock/test nodes) | `drl_unified_planner.launch.py`, mock/gazebo tests | `/drl/*` services + trajectory topics | rclpy, stable-baselines3 |
| **robot_drl_executor** | ament_cmake | Cartesian pose-sequence executor for DRL paths | `robot_drl_executor_node` | `robot_drl_executor.launch.py` | `/move_cartesian_pose_sequence` | moveit, robot_task_executor_msgs |
| **robot_gui** | ament_cmake | Qt5 GUI + embedded RViz, sends task actions | `robot_gui_node` | `robot_gui.launch.py` | subscribes joint/image; action clients | Qt5, rviz_common, robot_task_manager |
| **robot_vision_pipeline** | ament_cmake (+py) | YOLO + ArUco perception, pixel→base pose | `yolo_detect_node`, `pixel_to_base_mapper_node`, … | `vision_full_pipeline.launch.py`, `yolo_detect_*.launch.py` | `/vision/*` topics | cv_bridge, ultralytics, vision_msgs |
| **robot_vision_pipeline_msgs** | ament_cmake | Interface-only: `Wood/WoodArray/Box/BoxArray/BoxDetection` | *(none)* | *(none)* | msg defs | geometry_msgs |
| **robot_bringup** | ament_cmake | Top-level orchestration launches | *(none)* | `sim/real/mock/rl_pick_place_box_gazebo_demo.launch.py` | composes other launches | (launch-only) |

---

## 3. System architecture

```
                        robot_description  (URDF/Xacro = SINGLE SOURCE OF TRUTH)
                                   │  robot.urdf.xacro → robot.xacro (+ robot_tcp_xy / robot_tcp_z)
                                   │  ros2_control.xacro selects HW plugin:
                                   │    use_sim → gz_ros2_control/GazeboSimSystem
                                   │    use_mock → mock_components/GenericSystem
                                   │    real     → robot_hardware_interface/RobotSystemHardware
                                   ▼
        ┌──────────────────────┬──────────────────────┬─────────────────────────┐
        ▼                      ▼                      ▼                         ▼
  robot_control           robot_moveit          robot_gazebo            robot_hardware_interface
  (ros2_control:          (move_group +         (Gazebo world +         (real arm over TCP/RS485
   arm/gripper/jsb)        RViz, SRDF, IK)       ros_gz bridge +         192.168.2.50:5000,
        │                      │                  object spawners)        /robot_hw/* services)
        └───────────┬──────────┘                      │ /clock, sim cam, /sim/*_info
                    ▼  /joint_states, trajectory                         │
            ┌───────────────────────────── TASK / ACTION LAYER ─────────────────────────┐
            │  robot_task_manager  (9 ACTION servers; every action has `execute` flag)   │
            │     ├─ MoveIt-backed:  gohome, move_to_pose, move_to_pose_cartesian,       │
            │     │                  move_gripper, pickplace, checker_board, repeat_test │
            │     └─ DRL-backed:     drl_pickplace, move_pose_rl                          │
            │  robot_task_executor / robot_drl_executor  (SERVICE layer: Move* + Cartesian)│
            └───────────────────────────────┬───────────────────────────────────────────┘
                                            │  /drl/plan, /drl/execute_forward, …
                                            ▼
                              robot_drl  (Python SB3 policy → 15D obs → Cartesian waypoints)
                                            │  /move_cartesian_pose_sequence
                                            ▼
                              robot_drl_executor  (waypoints → MoveIt Cartesian path → controllers)

  robot_vision_pipeline ── /vision/woods,/vision/boxes ──▶ (DRL vision mode / GUI)   [optional, off by default in demo]
  robot_gui ── action clients ──▶ robot_task_manager ;  subscribes /joint_states + image topics + RViz
```

Dependency direction is one-way: `robot_description` is depended on by control, moveit,
gazebo, hardware. Nothing modifies it in return.

---

## 4. Launch map

**Top-level entry points (`robot_bringup/launch/`):**

| Launch | Purpose | Composes |
|---|---|---|
| `sim.launch.py` | **Main simulation entry** | `robot_gazebo/gazebo.launch.py` → (t=4s) `robot_moveit/moveit.launch.py` (`use_mock:=false`, `start_controller_manager:=false`) + controller spawners → (t=6s) `robot_task_manager/task_servers_sim.launch.py`. Args: `spawn_demo_woods=true`, `enable_drl_backend=true`. Controllers spawn chained: jsb → arm → gripper against the `gz_ros2_control` `/controller_manager`. |
| `real.launch.py` | **Main launch to drive the real robot + GUI** | `robot_moveit/moveit_gui.launch.py` (`use_mock:=false`) + `task_servers.launch.py`. With `use_mock:=false`, `controllers.launch.py` makes `ros2_control_node` load the `robot_hardware_interface/RobotSystemHardware` (TCP) plugin → **hardware comes up automatically inside the ros2_control process** (no separate `hardware_interface.launch.py` needed). Also brings up the **embedded-RViz GUI** (see `moveit_gui` below). Confirmed by user. |
| `mock.launch.py` | Mock MoveIt + GUI (no sim, no real HW, **no vision**) | `robot_moveit/moveit_gui.launch.py` (`use_mock:=true`) + `task_servers.launch.py`. Same launch as real but with mock hardware. |
| `rl_pick_place_box_gazebo_demo.launch.py` | Integrated DRL pick-place demo (ground-truth objects) | `sim.launch.py` (with `spawn_demo_woods=false`, `enable_drl_backend=false`) + `robot_drl_executor` + `drl_unified_planner_node` + `spawn_pick_wood_obstacle_box.py` + `drl_pick_place_wood_box_demo_client.py`. Uses spawn ground truth, NOT camera/YOLO. |

**Sub-launches by category:**

- **gazebo:** `robot_gazebo/launch/gazebo.launch.py` (**current**). `robot_description/launch/gazebo.launch.py` and `gazebo_new.launch.py` are **legacy** (superseded; avoid).
- **moveit:** `robot_moveit/launch/moveit.launch.py` (used by sim — move_group + RViz only). `moveit_gui.launch.py` (used by **mock/real**) is the integrated control launch: static TF `world→base_link`, `move_group`, `controllers.launch.py` (conditional on `start_controller_manager`, passes `use_mock`; default `use_mock:=true`), and **`robot_gui_node` with `embed_rviz:=true`** (delayed `gui_delay=3s`, default page 1 = MAIN with embedded RViz). `moveit_mock.launch.py` is legacy.
- **gui:** in mock/real, the GUI is launched **inside** `moveit_gui.launch.py` (embedded RViz). `robot_gui/launch/robot_gui.launch.py` is the standalone variant.
- **task servers:** `task_servers.launch.py` (non-sim), `task_servers_sim.launch.py` (sim; conditionally starts `drl_unified_planner_node` via `enable_drl_backend`).
- **drl:** backend node started by `task_servers_sim.launch.py`; standalone test launches live in `robot_drl/launch/` (`drl_gazebo*`, `drl_mock_hw*`, `mock_drl*`) — dev/test only.
- **gui:** `robot_gui/launch/robot_gui.launch.py`.
- **vision (optional, not in demo):** `vision_full_pipeline.launch.py`, `yolo_detect_real.launch.py`, `yolo_detect_sim.launch.py`, `aruco_detect.launch.py`, etc.

---

## 5. Interface map

### Actions — `robot_task_manager` (every action carries a `bool execute` field)
| Action topic | Server exe | Backend |
|---|---|---|
| `/gohome` | `gohome_server` | MoveIt |
| `/move_to_pose` | `move_to_pose_server` | MoveIt (PTP) |
| `/move_to_pose_cartesian` | `move_pose_cartesian_server` | MoveIt (Cartesian) |
| `/move_gripper` | `move_gripper_server` | gripper_controller |
| `/pickplace` | `pickplace_server` | MoveIt + gripper |
| `/move_checker_board` | `checker_board_server` | MoveIt |
| `/repeatability_test` | `repeatability_test_server` | calls `/move_to_pose*` |
| `/drl_pickplace` | `drl_pickplace_server` | DRL + `/move_gripper` + `/move_to_pose_cartesian` |
| `/move_pose_rl` | `move_pose_rl_server` | DRL |

Action definitions live in `robot_task_manager/action/` (9 `.action` files).

### DRL services & topics — `robot_drl/drl_unified_planner_node`
- Services (all `std_srvs/Trigger`): `/drl/plan`, `/drl/replan` (alias), `/drl/execute_forward`, `/drl/execute_backward`, `/drl/execute_trajectory` (alias), `/drl/clear_trajectory`, `/drl/get_execution_status`.
- Topics: `/drl/forward_trajectory_poses` (PoseArray), `/drl/forward_trajectory_marker`, `/drl/backward_trajectory_*`, `/drl/next_pose`, `/drl/execution_status` (2 Hz).
- `drl_pickplace_server` drives DRL via these `/drl/*` services and sets params on `/drl_unified_planner_node` (e.g. `calibrated_start_tcp_base`).
- Observation is a **15D** vector (`state_builder.build_observation_15d`): TCP xyz, target xyz, error xyz, normalized obstacle rel-pos xyz, normalized obstacle half-extent xyz (WORLD/DRL frame).

### Move services — `robot_task_executor_msgs` (srv), served by `robot_task_executor` and/or `robot_drl_executor`
`MoveCartesianPoseSequence`, `MoveCartesianSequence`, `MoveSequence`, `MoveToCartesianTarget`, `MoveToJointTarget`, `MoveToNamedCartesianTarget`, `MoveToNamedPoseTarget`, `MoveToNamedTarget`, `MoveToPoseTarget` — all have a `bool execute` (plan-only vs execute).
> **`/move_cartesian_pose_sequence` has two possible providers** — `robot_task_executor_node` and `robot_drl_executor_node`. In the DRL demo it is the **`robot_drl_executor`** that provides it. Do not run both providers at once.

### Controllers — `robot_control` / ros2_control
- `joint_state_broadcaster` → `/joint_states`. update_rate **10 Hz**.
- `arm_controller` (JointTrajectoryController) — joints `joint_1..joint_6`, command `position+velocity`.
- `gripper_controller` (JointTrajectoryController) — joints `joint_gl,joint_gr`, command `position`.

### Hardware services — `robot_hardware_interface` (srv)
`Home`, `Jog`, `RunAll`, `RunAxis`, `ServoOnAll`, `ServoOnAxis`, `StopAll`, `StopAxis`, `ArlarmRST` — namespaced under `/robot_hw/*`. Default `robot_ip: 192.168.2.50`, `robot_port: 5000`, joints `joint_1..joint_6`.

### Vision — `robot_vision_pipeline` / `robot_vision_pipeline_msgs`
- Msgs: `Wood`, `WoodArray`, `Box`, `BoxArray`, `BoxDetection`.
- Topics: `/vision/woods`, `/vision/boxes`, `/vision/detection_markers`, `/vision/yolo/*`.

### Simulation ground-truth — `robot_gazebo/spawn_pick_wood_obstacle_box.py`
- Publishes `Marker` on `/sim/pick_wood_info` and `/sim/obstacle_box_info` (in `base_link` frame).
- `drl_pick_place_wood_box_demo_client.py` subscribes to these, applies `pick_z_offset_m` (0.06) + `object_z_correction_m` (0.01), and sends the `drl_pickplace` action goal. **This path uses spawn ground truth, not the camera/YOLO pipeline.**

---

## 6. Current source-of-truth notes

**Trusted / current:**
- Robot model: `robot_description/urdf/robot.urdf.xacro` → `robot.xacro`. **TCP variants** `robot_tcp_xy.xacro` / `robot_tcp_z.xacro` are **only for tool-change accuracy evaluation in mock_hw / real robot** (preview tool in mock, run on real). **Sim (Gazebo) uses only `robot.xacro`** (no tcp variant). Confirmed by user.
- DRL deployed model: `robot_drl/models/run2/` (confirmed by user). Other `run/run_0/run0/run1` are older.
- Controllers: `robot_control/config/robot_controllers.yaml`.
- MoveIt: `robot_moveit/config/robot.srdf`, `kinematics.yaml`, `joint_limits.yaml`, `moveit_controllers.yaml`.
- Sim: `robot_gazebo/launch/gazebo.launch.py` + `worlds/table/arm_on_the_table.sdf`.
- Pick-place server: `robot_task_manager/src/pickplace_server.cpp`.
- DRL policy weights: `robot_drl/models/run2/` (latest run).
- Top-level launches: the 4 in `robot_bringup/launch/`.

**Legacy / do not use by mistake:**
- `robot_description/launch/gazebo.launch.py` & `gazebo_new.launch.py` — superseded by `robot_gazebo`.
- `robot_task_manager/src/pickplace_server_v1.cpp` — old; current is `pickplace_server.cpp`.
- `robot_moveit/launch/moveit_mock.launch.py` — superseded by `moveit_gui.launch.py use_mock:=true`.
- `README_old.md` files in several packages.
- All `*_report.md` / `*_audit.md` / `*_fix.md` (e.g. `rl_pick_place_object_z_fix_report.md`, `rl_pick_place_z_audit_report.md`, `GAZEBO_MIGRATION_REPORT.md`, `src/codex.md`) — **historical context only, not authoritative.**

---

## 7. Build instructions

```bash
# Full workspace
cd ~/ros2_dev_2
colcon build
source install/setup.bash
```

Group / incremental builds (build `*_msgs` before their consumers):

```bash
# Interfaces first
colcon build --packages-select robot_task_executor_msgs robot_vision_pipeline_msgs

# Description / control / moveit
colcon build --packages-select robot_description robot_control robot_moveit

# Task + DRL layer
colcon build --packages-select robot_task_executor robot_drl_executor robot_task_manager robot_drl

# GUI / vision
colcon build --packages-select robot_gui robot_vision_pipeline

# Bringup
colcon build --packages-select robot_bringup
```

Notes:
- `robot_drl` is `ament_python` and loads Stable-Baselines3 models. **DRL venv** (confirmed
  by user): `source ~/venvs/ros_rl/bin/activate` (has `stable-baselines3` / `torch`).
  This must be active in the shell/env where `drl_unified_planner_node` runs.
- After any code change, rebuild the affected package and `source install/setup.bash`.

---

## 8. Runtime validation checklist

After a launch, confirm the graph is healthy:

```bash
ros2 node list                 # move_group, task *_server nodes, drl_unified_planner_node present
ros2 topic list                # /joint_states, /tf, /clock (sim), /drl/* present
ros2 service list -t           # /drl/plan, /move_cartesian_pose_sequence, /robot_hw/* (real)
ros2 action list -t            # /gohome, /move_to_pose, /drl_pickplace, /move_pose_rl, ...
ros2 control list_controllers  # arm_controller, gripper_controller, joint_state_broadcaster = active
```

Expected:
- All 3 controllers report **`active`** (chained spawn: jsb → arm → gripper).
- `move_group` up; the 9 task action servers reachable.
- `/joint_states` publishing continuously.
- `/clock` present and `use_sim_time:=true` honored when running sim.
- For DRL: `drl_unified_planner_node` up and `/drl/plan` etc. listed.

---

## 9. Risks / things not to break

- **Frames / Z offset:** `world` → `base_link` (robot base is raised in Gazebo; table top ≈ table height). Marker poses on `/sim/*_info` are in `base_link`; the demo client applies `pick_z_offset_m` + `object_z_correction_m`. Changing any one of these silently breaks grasp height.
- **`execute=false` vs `execute=true`:** every action/service has an `execute` flag. With `execute=false` the system only PLANS (`drl_pickplace_server` returns "execution skipped"). **Never report success of a real motion when only plan-only ran.**
- **MoveIt availability:** action/service callers assume `move_group` is up; otherwise actions hang/timeout.
- **Controller startup ordering:** controllers are spawned with 15s/20s timeouts and chained event handlers. They are not `active` instantly after launch — wait/verify before commanding motion.
- **DRL venv/model:** planner needs the SB3 venv and a valid model under `robot_drl/models/` (`run2`, **SAC**). Missing venv/model → `/drl/plan` fails.
- **DRL workspace bounds (critical):** the policy is only valid inside `x[0.25,0.50] y[-0.15,0.15] z[0.02,0.30]` m (base frame). Pick/place/start/obstacle poses outside these bounds (see Appendix A) can produce wrong/unsafe motion. Obstacle must respect `safety_margin = 0.03 m`.
- **GUI ↔ `.ui` coupling:** `robot_gui_node` binds to widget `objectName`s in `ui/robot_gui.ui`. Renaming widgets breaks the C++ lookups.
- **Hardware naming:** real arm at `192.168.2.50:5000`, services under `/robot_hw/*`. Don't rename without updating the plugin/config. `real.launch.py` does not itself start `hardware_interface.launch.py`.
- **New `robot_gazebo` vs old files in `robot_description`:** Gazebo launches/spawners exist in BOTH. Use `robot_gazebo`; the `robot_description` copies are legacy.
- **Single provider for `/move_cartesian_pose_sequence`:** don't run `robot_task_executor` and `robot_drl_executor` such that both advertise it.
- **Renaming topics/services/actions:** update *all* dependents (servers, clients, launch params, GUI) — names are wired across C++, Python, and launch files.

---

## 10. Suggested next steps (how to safely take a code task)

1. **Read the relevant package's `README.md` first** (and confirm it isn't `README_old.md`).
2. Confirm current vs legacy file via §6 before editing — never edit a `*_v1` / legacy path.
3. Make the minimal change; do not restructure architecture for a small fix.
4. If you touch interfaces (topic/service/action/frame names), grep the whole tree and
   update every producer/consumer + launch arg + GUI binding.
5. Rebuild only the affected package(s): `colcon build --packages-select <pkg>` then re-source.
6. For MoveIt/Gazebo/action/service tasks, run the §8 checklist and confirm controllers are `active`.
7. If the task requires real execution, verify `execute=true` actually ran motion — don't trust a plan-only success.
8. Keep DRL model/reward/observation/action-space untouched unless the task explicitly asks.
9. In Gazebo demos that say "ground truth", use `/sim/*_info` spawn data — do NOT pull in camera/YOLO.
10. Treat all `*_report/*_audit/*_fix.md` as history; verify against live source before acting on them.

---

## Appendix A — DRL trained-model spec (run2, **SAC**)

Source of truth: **`robot_drl/models/run2/config.yaml`** (verified; values below match it
and the user's spec). Current algorithm = **SAC**. All units are **metres**, frame = robot
**base** (then shifted to the WORLD/DRL frame by `calibrated_start_tcp_base`).

**Workspace (search region)** — positions OUTSIDE this box may behave incorrectly:
```
x: [0.25, 0.50]   y: [-0.15, 0.15]   z: [0.02, 0.30]
```

**Start position** (`mode: random`, fixed fallback `[0.30, 0.00, 0.30]`):
```
random_bounds: min [0.25, -0.15, 0.10]  max [0.5, 0.15, 0.30]
```

**Target region = pick coordinate** (`mode: random`, fixed fallback `[0.30, 0.00, 0.10]`):
```
random_bounds: min [0.25, -0.15, 0.02]  max [0.5, 0.15, 0.15]
```

**Obstacle / box** (`enabled: true`, `mode: random`, `type: box`):
- `safety_margin: 0.03` — min distance to obstacle required for success.
- center random_bounds: `min [0.28, -0.10, 0.080]  max [0.42, 0.10, 0.080]`
- size random: length `[0.05, 0.10]`, width `[0.05, 0.10]`, height `[0.05, 0.15]`.

**Observation** — `Box(-inf, inf, shape=(15,), float32)`, concatenated in order:
```
observation = [ current_pos(3),
                target_pos(3),
                err = target_pos - current_pos (3),
                rel_obs = (obstacle_center - current_pos) / workspace_range (3),
                obs_size = obstacle_half_extent / workspace_range (3) ]
```

**Action** — `Box(-1.0, 1.0, shape=(3,), float32)`, normalized Cartesian delta on x/y/z:
```
delta    = action * action_step        # action_step = 0.01 m
next_pos = current_pos + delta
```
Orientation is **kept fixed** during planning (DRL only moves position; orientation held).

> Practical implication: the demo's pick/place/obstacle poses must stay within the
> workspace box and obstacle bounds above, otherwise the policy can produce wrong/unsafe
> motions. Keep `safety_margin = 0.03 m` in mind when placing obstacles.
