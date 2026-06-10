# DRL Robot Manipulator — robot

Deep Reinforcement Learning training and evaluation framework for the **robot** 6-DOF industrial robotic arm, using **PyBullet** for physics simulation, **Gymnasium** for the RL environment API, and **Stable-Baselines3** for DRL algorithm implementations.

---

## Project Overview

This framework trains DRL agents (DDPG, SAC, TD3) to solve a **robotic reaching task**: moving the TCP (tool centre point) from a home configuration to a randomly sampled target position within the robot's reachable workspace.

| Mode | Description |
|------|-------------|
| `Default` | Free-space reaching — no obstacle |
| `Collision-Free` | Reaching with a 0.1 m cube obstacle that the policy must navigate around |

---

## System Architecture

```
PyBullet Physics Engine (physics + OpenGL rendering)
         │
         ▼
  RobotReachPyBulletEnv  (Gymnasium API)
         │
         ▼
  Stable-Baselines3  (DDPG / SAC / TD3)
         │
         ▼
  Neural Network Policy  →  final_model.zip
```

### Supporting Libraries

| Component | Role |
|-----------|------|
| `src/PyBullet/` | Robot_Cls — PyBullet robot interface, URDF loading, scene setup |
| `src/Industrial_Robotics_Gym/` | Gymnasium env wrapper, environment registration |
| `src/RoLE/` | Robotics Library: FK/IK kinematics, collision detection, trajectory generation, HTM/Quaternion transforms |
| `Training/` | Training scripts (DDPG, SAC, TD3) |
| `Evaluation/` | Prediction, visualization, and IK validation scripts |

---

## Repository Structure

```
DRL_Robot_Manipulator/
├── src/
│   ├── config_loader.py                  # Project root resolver
│   ├── Industrial_Robotics_Gym/
│   │   ├── __init__.py                  # Gymnasium env registration
│   │   └── Environment/
│   │       └── RobotReachPyBulletEnv.py   # Main Gym env (PyBullet-backed)
│   ├── PyBullet/
│   │   ├── Core.py                      # Robot_Cls — PyBullet robot interface
│   │   └── Configuration/
│   │       └── Environment.py            # Env/collision parameter dataclasses
│   └── RoLE/                            # Robotics Library
│       ├── Parameters/Robot.py            # DH params, joint limits, collider defs
│       ├── Transformation/Core.py         # HTM, Vector3, Quaternion, EulerAngle
│       ├── Kinematics/Core.py            # FK, IK (JT / NR / GN / LM solvers)
│       ├── Collider/Core.py              # AABB, OBB collision detection
│       ├── Trajectory/                    # Trapezoidal, polynomial profiles
│       └── Interpolation/                 # Bezier, B-Spline curves
├── Training/
│   ├── train_ddpg_robot.py                 # DDPG, Default, 500k steps
│   ├── train_ddpg.py                     # DDPG, Collision-Free, 100k steps
│   ├── train_sac.py                      # SAC, Collision-Free, 100k steps
│   └── train_td3.py                      # TD3, Collision-Free, 100k steps
├── Evaluation/
│   ├── Gym/Model/Prediction/
│   │   ├── Static/                       # Fixed-target prediction scripts
│   │   └── Random/                      # Random-target prediction scripts
│   └── PyBullet/Control/                # IK validation, config space tests
├── Data/                                # Training outputs (gitignored)
├── URDFs/                              # Robot URDF, primitives, viewpoints
├── Textures/                           # Plane texture (gitignored)
├── config/
│   └── config.yaml                      # PROJECT_FOLDER_NAME, env params, IK params
└── requirements.txt                    # Pinned dependencies
```

---

## Robot Specifications — robot

### Joint Limits (degrees)

| Joint | Min | Max |
|-------|-----|-----|
| Joint 1 | -170.0° | +170.0° |
| Joint 2 | -65.0° | +145.0° |
| Joint 3 | -116.0° | +255.0° |
| Joint 4 | -190.0° | +190.0° |
| Joint 5 | -135.0° | +135.0° |
| Joint 6 | -360.0° | +360.0° |

### Joint Limits (radians)

| Joint | Min (rad) | Max (rad) |
|-------|-----------|-----------|
| Joint 1 | -2.9671 | +2.9671 |
| Joint 2 | -1.1345 | +2.5307 |
| Joint 3 | -2.0246 | +4.4506 |
| Joint 4 | -3.3161 | +3.3161 |
| Joint 5 | -2.3562 | +2.3562 |
| Joint 6 | -3.1416 | +3.1416 |

### Denavit-Hartenberg Parameters (Standard Convention)

| Joint | theta_zero (rad) | a (m) | d (m) | alpha (rad) |
|-------|-----------------|-------|-------|-------------|
| 1 | 0.0 | 0.040 | 0.330 | 0.0 |
| 2 | -1.5708 | 0.445 | 0.0 | 0.0 |
| 3 | 0.0 | 0.040 | 0.0 | -1.5708 |
| 4 | 0.0 | 0.0 | 0.440 | +1.5708 |
| 5 | 0.0 | 0.0 | 0.0 | -1.5708 |
| 6 | 0.0 | 0.0 | 0.080 | 0.0 |

### Denavit-Hartenberg Parameters (Modified Convention)

| Joint | theta_zero (rad) | a (m) | d (m) | alpha (rad) |
|-------|-----------------|-------|-------|-------------|
| 1 | 0.0 | 0.0 | 0.330 | 0.0 |
| 2 | -1.5708 | 0.040 | 0.0 | -1.5708 |
| 3 | 0.0 | 0.445 | 0.0 | 0.0 |
| 4 | 0.0 | 0.040 | 0.440 | -1.5708 |
| 5 | 0.0 | 0.0 | 0.0 | +1.5708 |
| 6 | +0.3217 | 0.0 | 0.080 | -1.5708 |

### Home Configuration

`[0.0, 0.0, 0.0, 0.0, -90.0°, 0.0]` in degrees

### Joint Properties

| Property | Value |
|----------|-------|
| Joint Names | Joint_1 through Joint_6 |
| Joint Types | All Revolute (R) |
| Joint Axes | All Z-axis |
| Directions | All +1 |
| External Axis | None (disabled) |

### Collision Bodies

| Body | Type | Center (m) | Half-Extents (m) |
|------|------|-----------|-----------------|
| Base Collider | OBB | [0.0606, 0.0000, -0.0832] | [0.3012, 0.1805, 0.1663] |
| Joint 1-6 Colliders | OBB | Zero-size (placeholder) | — |

### Collision Pairs (optimized self-collision check)

`(0,4), (0,5), (0,6), (0,3), (1,4), (1,5), (1,6)`

---

## Environment Design — RobotReachPyBulletEnv

### Environment ID Format

```
robotRobotReachPyBullet-{Mode}-v0
```
Where `Mode` is either `Default` or `Collision-Free`.

---

### Action Space

| Property | Value |
|----------|-------|
| Type | `Box` |
| Shape | `(3,)` — 3 dimensions |
| Low | `-1.0` (per dimension) |
| High | `+1.0` (per dimension) |
| Data type | `np.float32` |
| Physical meaning | Normalized 3D Cartesian delta `[dx, dy, dz]` |

**Action scaling:** The normalized action is multiplied by `action_step` (default **0.01 m**) to produce an actual Cartesian displacement per step.

```
desired_position = current_tcp_position + (action * action_step)
```

Example: `action = [0.5, -0.3, 0.1]` → `delta = [0.005, -0.003, 0.001]` meters

---

### Observation Space

| Property | Value |
|----------|-------|
| Type | `Box` |
| Shape | `(15,)` — 15 dimensions |
| Low | `-inf` (per dimension) |
| High | `+inf` (per dimension) |
| Data type | `np.float32` |

#### Observation Vector Layout (15 dimensions)

| Indices | Field Name | Description | Unit | Typical Range |
|---------|-----------|-------------|------|--------------|
| 0 | `tcp_x` | Current TCP X position (world frame) | meter | workspace dependent |
| 1 | `tcp_y` | Current TCP Y position (world frame) | meter | workspace dependent |
| 2 | `tcp_z` | Current TCP Z position (world frame) | meter | workspace dependent |
| 3 | `target_x` | Target X position (world frame) | meter | workspace dependent |
| 4 | `target_y` | Target Y position (world frame) | meter | workspace dependent |
| 5 | `target_z` | Target Z position (world frame) | meter | workspace dependent |
| 6 | `err_x` | Target minus TCP X (`target_x - tcp_x`) | meter | ~[-0.5, +0.5] |
| 7 | `err_y` | Target minus TCP Y (`target_y - tcp_y`) | meter | ~[-0.5, +0.5] |
| 8 | `err_z` | Target minus TCP Z (`target_z - tcp_z`) | meter | ~[-0.5, +0.5] |
| 9 | `rel_obs_x` | Obstacle X position relative to TCP (normalized by workspace range) | dimensionless | ~[-1, +1] |
| 10 | `rel_obs_y` | Obstacle Y position relative to TCP (normalized by workspace range) | dimensionless | ~[-1, +1] |
| 11 | `rel_obs_z` | Obstacle Z position relative to TCP (normalized by workspace range) | dimensionless | ~[-1, +1] |
| 12 | `obs_size_x` | Obstacle X half-extent (normalized by workspace range) | dimensionless | 0 (Default) or ~[0, 1] |
| 13 | `obs_size_y` | Obstacle Y half-extent (normalized by workspace range) | dimensionless | 0 (Default) or ~[0, 1] |
| 14 | `obs_size_z` | Obstacle Z half-extent (normalized by workspace range) | dimensionless | 0 (Default) or ~[0, 1] |

**Normalization of obstacle fields:**
- `rel_obs_*` = `(obstacle_pos - tcp_pos) / (workspace_max - workspace_min + 1e-6)`
- `obs_size_*` = `obstacle_half_extent / (workspace_max - workspace_min + 1e-6)`
- In `Default` mode, indices 9–14 are all **zero**.

---

### Reward Function

The reward is **multi-component and dense** (computed at every step).

#### Default Mode

```
reward = r_distance + r_action
```

| Component | Formula | Description |
|-----------|---------|-------------|
| `r_distance` | `-euclidean_distance(tcp, target)` | Primary dense reward — always negative, larger magnitude when farther |
| `r_action` | `-0.001 * ||action||^2` | Action smoothness penalty — discourages large, jerky actions |

#### Collision-Free Mode

```
reward = r_distance + r_action + r_collision_soft
```

| Component | Formula | Description |
|-----------|---------|-------------|
| `r_distance` | `-euclidean_distance(tcp, target)` | Same as Default |
| `r_action` | `-0.001 * ||action||^2` | Same as Default |
| `r_collision_soft` | `-danger_ratio * 0.5` (when `dist < 0.05 m`), else `0.0` | Soft collision proximity penalty |

Where:
- `dist = euclidean_distance(tcp_pos, obstacle_pos)`
- `danger_zone = 0.05 m`
- `danger_ratio = 1.0 - dist / danger_zone` (linear falloff from 1.0 to 0.0)
- `collision_obj_penalty_threshold = 0.5`

#### Hard Failure Penalties (applied at episode end)

| Event | Penalty | Effect |
|-------|---------|--------|
| Obstacle collision (real contact detected) | `-5.0` | `truncated = True` |
| Workspace limit violation | `-1.0` | `truncated = True` |
| IK solver failure | `-1.0` | `truncated = True` |
| Joint limit violation | `-1.0` | `truncated = True` |
| Episode step limit reached | `0.0` (no penalty, just truncation) | `truncated = True` |
| **Success** (distance < threshold) | `+10.0` bonus | `terminated = True` |

#### Reward Scale Summary

| Scenario | Reward Range |
|----------|--------------|
| Far from target, no collision | `~[-0.7, -0.01]` |
| Near target, no collision | `~[-0.01, -0.001]` |
| Close to obstacle (soft penalty) | `[-0.5, 0.0]` extra penalty |
| Collision | `-5.0` |
| Success | `+10.0` bonus on top of step reward |

**Note:** `VecNormalize` is configured with `norm_obs=False` and `norm_reward=False`, so raw rewards are used directly.

---

### Termination Conditions

| Condition | Flag | Trigger |
|-----------|------|---------|
| **Success** | `terminated = True` | `euclidean_distance(tcp, target) < distance_thresh` (default **0.01 m = 1 cm**) |
| **Workspace limit** | `truncated = True` | Desired TCP position goes outside workspace bounds |
| **IK failure** | `truncated = True` | Inverse kinematics solver cannot find a solution |
| **Joint limit** | `truncated = True` | Any joint angle exceeds its physical limit |
| **Obstacle collision** | `truncated = True` | Real contact detected between robot and obstacle |
| **Max steps** | `truncated = True` | Episode exceeds `max_episode_steps` (default **200**) |

**`info` dictionary** always contains:
- `is_success`: `True` if `terminated == True` (success), `False` otherwise
- `distance`: Current Euclidean distance from TCP to target
- `is_collision`: `True` if real collision detected
- `contacts`: Number of PyBullet contact points
- `termination_reason`: String — `'success'`, `'workspace_limit'`, `'ik_failure'`, `'joint_limit'`, `'collision'`, `'max_steps'`, `'none'`
- `reward_distance`, `reward_action`, `reward_collision_soft`, `collision_soft_distance`, `collision_soft_penalty`

---

### Environment Parameters

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `action_step` | `0.01` | `> 0` | Cartesian displacement per normalized action unit (meters) |
| `distance_thresh` | `0.01` | `> 0` | Success threshold — TCP must be within this distance of target (meters) |
| `max_episode_steps` | `500` | `> 0` | Maximum number of `step()` calls per episode |
| `enable_gui` | `True` | `bool` | Whether to show PyBullet GUI window |

---

### Target Sampling

- Targets are sampled uniformly at random from the **Target configuration space** (a subset of the full joint configuration space).
- In `Collision-Free` mode, the sampler rejects any candidate that falls inside the obstacle AABB (with a 0.02 m margin). Up to 100 retries before falling back to the C-space center.
- Target orientation is **fixed**: quaternion `[w=0, x=1, y=0, z=0]` (180° around X-axis), meaning the TCP Z-axis points world -Z (downward).
- Target is visualized as a frame marker (`T_EE_Target`) in the PyBullet scene.

---

### Workspace Bounds

Workspace bounds are derived from the "Search" configuration space vertices of the robot. The bounds define the axis-aligned box within which the TCP is allowed to move.

---

## Training Configuration

### Hyperparameters — DDPG (train_ddpg_robot.py)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `CONST_TOTAL_TIMESTEPS` | `500,000` | Total environment steps |
| `GAMMA` | `0.95` | Discount factor |
| `LEARNING_RATE` | `0.001` | Adam optimizer learning rate |
| `ACTION_NOISE_SIGMA` | `0.1` | Stddev of Gaussian exploration noise |
| `BATCH_SIZE` | `256` | Minibatch size for replay buffer updates |
| `NET_ARCH` | `[256, 256, 256]` | Actor/Critic MLP hidden layer sizes |
| `DEVICE` | `'cuda'` | Computation device (GPU) |
| `LOG_INTERVAL` | `10` | Print progress every N episodes |
| `EVAL_FREQ` | `5,000` | Evaluate best model every N steps |
| `N_EVAL_EPISODES` | `10` | Number of evaluation episodes per eval |
| `CHECKPOINT_EVERY` | `20,000` | Save checkpoint every N steps |
| `SEED` | `42` | Reproducibility seed |

### Other Training Scripts

| Script | Mode | Steps | Algorithm |
|--------|------|-------|-----------|
| `train_ddpg_robot.py` | Default | 500,000 | DDPG |
| `train_ddpg.py` | Collision-Free | 100,000 | DDPG |
| `train_sac.py` | Collision-Free | 100,000 | SAC |
| `train_td3.py` | Collision-Free | 100,000 | TD3 |

---

## VecNormalize Configuration

VecNormalize is used but **disabled for normalization**:

```python
VecNormalize(gym_environment,
    norm_obs=False,    # Observation normalization OFF
    norm_reward=False, # Reward normalization OFF
    clip_reward=10.0   # Unused (norm_reward=False)
)
```

This means:
- Observations are passed to the policy in **raw form** (no scaling).
- Rewards are passed to the algorithm in **raw form**.
- The `vec_normalize_stats.pkl` file is still saved so inference scripts can load it without error, but it has no effect.
- This design was chosen so that `predict_ddpg.py` does not need VecNormalize to run inference.

---

## Output Directory Structure

```
Data/Training/Environment_{MODE}/{ALGORITHM}/ROBOT/run_{TIMESTAMP}/
├── config.json              # Experiment configuration
├── model/
│   ├── final_model.zip     # Saved policy (SB3 native format)
│   ├── best_model.zip      # Best model by eval reward
│   ├── vec_normalize_stats.pkl  # VecNormalize stats (no-op)
│   ├── noise_sigma.txt      # Action noise sigma (for resume training)
│   └── checkpoints/
│       └── checkpoint_{NNNNNNN}.zip  # Periodic checkpoints
├── logs/
│   ├── progress.csv        # SB3 training metrics (losses, etc.)
│   ├── monitor.csv          # Episode rewards and lengths
│   ├── events.out.tfevents.*  # TensorBoard event files
│   └── time.txt             # Elapsed training time
└── replay_buffer.pkl        # Off-policy replay buffer (for resume)
```

---

## Installation

### Prerequisites

| Component | Version | Notes |
|-----------|---------|-------|
| Python | 3.10 or 3.11 | Tested on 3.10 |
| CUDA | 11.8 or 12.x | Optional — only needed for GPU training |

### Steps

```bash
# 1. Navigate to project root
cd DRL_Robot_Manipulator

# 2. Create virtual environment (recommended)
python -m venv .venv
# Windows:
.venv\Scripts\activate
# Linux/macOS:
source .venv/bin/activate

# 3. Install PyTorch with CUDA 12.1 support (for GPU training)
pip install torch==2.2.2 torchvision==0.17.2 torchaudio==2.2.2 \
    --index-url https://download.pytorch.org/whl/cu121

# 4. Install remaining dependencies
pip install -r requirements.txt

# 5. Verify installation
python -c "import gymnasium; import stable_baselines3; import pybullet; print('OK')"
```

> **Important:** On Windows, running `pip install -r requirements.txt` alone installs CPU-only PyTorch. Always run the CUDA wheel installation command first.

---

## Running Training

From the project root directory:

```bash
# Default mode, GUI disabled, 500k steps (main training script)
python Training/train_ddpg_robot.py

# Collision-Free mode, GUI enabled, 100k steps
python Training/train_ddpg.py

# Collision-Free mode, SAC, 100k steps
python Training/train_sac.py

# Collision-Free mode, TD3, 100k steps
python Training/train_td3.py

# To enable GUI, edit the script and set: ENABLE_GUI = True
```

---

## Switching Environment Modes

Change `CONST_ENV_MODE` at the top of the training script:

```python
CONST_ENV_MODE = 'Default'          # No obstacle — free-space reaching
CONST_ENV_MODE = 'Collision-Free'   # Cube obstacle present
```

Or pass the mode through `config/env_config.yaml` (which is loaded by the training scripts).

---

## TensorBoard

TensorBoard logging is enabled automatically when `tensorboard` is installed:

```bash
tensorboard --logdir Data/Training/
```

---

## Evaluation Scripts

| Script | Purpose |
|--------|---------|
| `Evaluation/Gym/Model/Prediction/Static/predict_ddpg.py` | Run DDPG policy to a fixed target, save trajectory |
| `Evaluation/Gym/Model/Prediction/Random/predict_ddpg.py` | Run DDPG for 100 random targets, save metrics |
| `Evaluation/Gym/Model/Training/show_train_results.py` | Plot a single algorithm's training progress |
| `Evaluation/Gym/Model/Training/show_train_comparison.py` | Compare all algorithm variants |
| `Evaluation/PyBullet/Control/test_configuration_space_rand.py` | Validate IK on random C-space targets |
| `Evaluation/PyBullet/Control/test_configuration_space_vertices.py` | Validate IK on all C-space corner vertices |

---

## Environment Modes in Detail

### Default Mode

- Robot moves freely in workspace with no obstacles.
- Observation indices 9–14 (obstacle fields) are all zero.
- Reward is purely distance-based with action smoothness penalty.
- Truncation only on workspace violation, IK failure, joint limit, or step limit.

### Collision-Free Mode

- A 0.1 m cube obstacle is placed in the workspace.
- Obstacle position and size are included in the observation (normalized).
- Soft proximity penalty when TCP gets within 0.05 m of obstacle.
- Hard collision triggers `-5.0` penalty and episode truncation.
- Target is never sampled inside the obstacle.

---

## Key Design Decisions

1. **TCP reading:** TCP pose is read directly from PyBullet's `getLinkState` on the `ee_link` frame, not from the internal RoLE FK model (which has a known ~0.35 m discrepancy).

2. **Position-only IK:** The environment uses position-only IK (`use_orientation=True` in config, but the IK solver is configured for position-only via `ik_position_tolerance`). Orientation is fixed to Z-downward for all targets.

3. **Single-process training:** All training uses `DummyVecEnv` (no multiprocessing fork). The `if __name__ == '__main__'` guard is present in all scripts.

4. **No VecNormalize normalization:** `norm_obs=False` and `norm_reward=False` so inference scripts don't need VecNormalize.

5. **Success criterion:** Throughout the codebase, `distance < 0.01` (1 cm) is the success threshold.

6. **Seeding:** All four random sources (Python `random`, NumPy, PyTorch, Gymnasium env) are seeded with `SEED = 42`.

7. **HER variants:** DDPG_HER, SAC_HER, TD3_HER are referenced in data paths and comparison scripts, but training scripts do not yet exist.

---

## Troubleshooting

### PyBullet GUI on Windows

PyBullet GUI works natively on Windows using OpenGL. If the window fails to open:

1. Ensure graphics drivers are up to date.
2. On Nvidia Optimus laptops, set the Nvidia GPU as the default for `python.exe`.
3. Run headless: set `ENABLE_GUI = False`.

### GPU Not Detected

```python
import torch
print(torch.cuda.is_available())   # Should be True with CUDA wheels
print(torch.version.cuda)          # Should be '12.1'
```

If False, reinstall PyTorch:

```bash
pip install torch==2.2.2 --index-url https://download.pytorch.org/whl/cu121
```

### Module Import Errors

Run from the project root directory, or ensure `src/` is in `PYTHONPATH`. Training scripts automatically add `src/` to `sys.path`.

### URDF Mesh Not Found

Some URDF files reference mesh paths that differ between Windows and Linux. The `Evaluation/URDFs/Robots/ROBOT/a.py` script resolves these automatically.

---

## Suggested Improvements

- Add HER (Hindsight Experience Replay) training scripts using SB3's `HerReplayBuffer`.
- Replace hardcoded `device='cuda'` with `torch.cuda.is_available()` check.
- Use `pathlib.Path` consistently for all path operations.
- Add automated tests using `stable_baselines3.common.env_checker.check_env`.

---

## License

MIT License — Copyright (c) 2024 Roman Parak
