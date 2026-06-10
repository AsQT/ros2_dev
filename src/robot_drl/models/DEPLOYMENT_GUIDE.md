# Model Deployment Guide

This guide explains how to deploy a trained DDPG/SAC/TD3 policy model to a real robot or to an inference-only environment. It covers everything from required files and dependencies to writing the control loop and understanding environment parameters.

---

## Overview

The trained model receives a **15-dimensional observation** and outputs a **3D Cartesian delta action** `[dx, dy, dz]`. You are responsible for:

1. Sending the current TCP position from the real robot
2. Sending target positions to the model
3. Translating the model's action output into robot motion commands

The model does **not** include PyBullet, Gymnasium, or any physics simulation at inference time — it is a pure PyTorch neural network loaded via Stable-Baselines3.

---

## Files to Copy to Deployment Machine

At minimum, copy these files from your training output directory:

```
model/
├── best_model.zip           # Best model (auto-saved during training by eval callback)
├── vec_normalize_stats.pkl  # VecNormalize statistics (may be a no-op — see below)
├── config.json              # Experiment configuration (hyperparameters, env params)
config/
└── env_config.yaml          # Environment parameters (action_step, distance_thresh, IK settings)
```

**Path example:**
```
Data/Training/Environment_Default/DDPG/ROBOT/run_20260508_100002/model/best_model.zip
```

---

## Dependencies

```bash
pip install numpy torch gymnasium stable-baselines3
```

| Package | Required | Notes |
|---------|----------|-------|
| `numpy` | Yes | For array operations |
| `torch` | Yes | Model inference (CPU is fine) |
| `gymnasium` | Only if using `gym.make` | Only needed if you want to run in the PyBullet sim for testing |
| `stable_baselines3` | Yes | Loads `.zip` model via `DDPG.load()` |

> **Note:** You do **not** need PyBullet, the full `src/` directory, or any of the training scripts at inference time. Only the model file and the SB3 library are required.

---

## Environment Parameters

All values are read from `config/env_config.yaml`. Verify these match what was used during training.

### Critical Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `action_step` | `0.01` | TCP displacement (meters) per unit action. Action ∈ [-1, 1] → max delta = ±0.01 m per step. |
| `distance_thresh` | `0.02` | Success threshold — TCP must reach within this distance of target (meters) |
| `max_episode_steps` | `500` | Maximum control loop iterations before episode terminates |

### IK Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `ik.delta_time` | `0.01` | IK solver timestep |
| `ik.num_of_iteration` | `50` | Maximum IK iterations |
| `ik.tolerance` | `1e-30` | Numerical tolerance (PyBullet internal) |
| `ik.use_orientation` | `true` | Must be True |
| `ik.ik_position_tolerance` | `0.01` | IK success threshold (meters) |

### Collision-Free Mode Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `collision_free.size` | `[0.05, 0.05, 0.05]` | Obstacle half-extents in meters [x, y, z] |
| `collision_free.penalty_threshold` | `0.5` | Collision penalty threshold |

---

## Observation Space — 15 Dimensions

The observation is a **15-dimensional float32 array**. You must construct this manually for the real robot.

| Indices | Field Name | Description | Unit | Notes |
|---------|-----------|-------------|------|-------|
| 0 | `tcp_x` | Current TCP X position | meter | Read from robot |
| 1 | `tcp_y` | Current TCP Y position | meter | Read from robot |
| 2 | `tcp_z` | Current TCP Z position | meter | Read from robot |
| 3 | `target_x` | Target X position | meter | Your goal |
| 4 | `target_y` | Target Y position | meter | Your goal |
| 5 | `target_z` | Target Z position | meter | Your goal |
| 6 | `err_x` | `target_x - tcp_x` | meter | Error vector |
| 7 | `err_y` | `target_y - tcp_y` | meter | Error vector |
| 8 | `err_z` | `target_z - tcp_z` | meter | Error vector |
| 9 | `rel_obs_x` | Obstacle X relative to TCP (normalized) | dimensionless | **Set to 0 if no obstacle** |
| 10 | `rel_obs_y` | Obstacle Y relative to TCP (normalized) | dimensionless | **Set to 0 if no obstacle** |
| 11 | `rel_obs_z` | Obstacle Z relative to TCP (normalized) | dimensionless | **Set to 0 if no obstacle** |
| 12 | `obs_size_x` | Obstacle X half-extent (normalized) | dimensionless | **Set to 0 if no obstacle** |
| 13 | `obs_size_y` | Obstacle Y half-extent (normalized) | dimensionless | **Set to 0 if no obstacle** |
| 14 | `obs_size_z` | Obstacle Z half-extent (normalized) | dimensionless | **Set to 0 if no obstacle** |

**In Default mode (no obstacle):** indices 9–14 are all **zero**.

**Normalization formula for obstacle fields** (Collision-Free mode only):
```
rel_obs_*  = (obstacle_pos - tcp_pos) / (workspace_range)
obs_size_* = obstacle_half_extent / workspace_range
```
Where `workspace_range ≈ 0.5` meters for the robot.

### Building the Observation Array

```python
import numpy as np

# 1. Read TCP position from robot (replace with your robot API)
tcp_pos = robot.get_tcp_position()      # [x, y, z] in metres

# 2. Define your target position
target_pos = np.array([0.20, -0.40, 0.50])

# 3. Build error vector
error = target_pos - tcp_pos

# 4. Obstacle fields (set to zero if Default mode / no obstacle)
obstacle_rel_pos = np.zeros(3)   # [rel_obs_x, rel_obs_y, rel_obs_z]
obstacle_size   = np.zeros(3)   # [obs_size_x, obs_size_y, obs_size_z]

# 5. Assemble 15-D observation
obs = np.concatenate([
    tcp_pos,          # indices 0-2
    target_pos,       # indices 3-5
    error,            # indices 6-8
    obstacle_rel_pos,  # indices 9-11
    obstacle_size,    # indices 12-14
], dtype=np.float32)

assert obs.shape == (15,), f"Expected shape (15,), got {obs.shape}"
```

---

## Action Space

The model outputs a **3-dimensional action** `action ∈ [-1, 1]^3`, representing a normalized Cartesian TCP delta.

| Component | Range | Description |
|-----------|-------|-------------|
| `action[0]` | [-1, +1] | Delta X direction (left/right) |
| `action[1]` | [-1, +1] | Delta Y direction (forward/back) |
| `action[2]` | [-1, +1] | Delta Z direction (up/down) |

**Conversion to real-world displacement:**
```
delta_meters = action * action_step
```
With `action_step = 0.01`, a maximum action of `[1, 1, 1]` moves the TCP by `0.01 m` in each axis per control step.

---

## Reward Function (for reference)

You do not need to compute rewards at inference time, but it helps to understand the training signal:

| Component | Formula | Description |
|-----------|---------|-------------|
| `r_distance` | `-euclidean_distance(tcp, target)` | Primary reward — always negative |
| `r_action` | `-0.001 * ||action||^2` | Action smoothness penalty |
| `r_collision_soft` | `-danger_ratio * 0.5` (only when `dist < 0.05 m`) | Soft proximity penalty |
| **Success bonus** | `+10.0` | Added on successful termination |
| **Collision penalty** | `-5.0` | Applied when robot contacts obstacle |
| **Failure penalty** | `-1.0` | Applied on workspace/IK/joint limit truncation |

---

## VecNormalize — Will the Model Work Without It?

**Yes.** The training script uses `VecNormalize` with `norm_obs=False` and `norm_reward=False`, which means normalization is **disabled**. The `vec_normalize_stats.pkl` file exists but is a no-op.

However, if the model was trained with `norm_obs=True`, you must apply the same normalization during inference. To check:

```python
import pickle

with open('vec_normalize_stats.pkl', 'rb') as f:
    vn = pickle.load(f)

norm_obs_enabled = getattr(vn, 'norm_obs', True)
print(f"norm_obs enabled: {norm_obs_enabled}")

if norm_obs_enabled:
    # Apply normalization to observations
    obs = np.clip((raw_obs - vn.obs_rms.mean) / np.sqrt(vn.obs_rms.var + 1e-8), -10, 10)
```

---

## Control Loop — Minimal Inference

### With the Gymnasium Environment (PyBullet simulation)

If you want to test inference using the PyBullet simulation (no real robot required):

```python
import numpy as np
import gymnasium as gym
from stable_baselines3 import DDPG
import pickle
import sys
from pathlib import Path

# --- Path setup (adjust for your deployment machine) ---
SRC_DIR = Path(__file__).parent / 'src'
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

import Industrial_Robotics_Gym
from config_loader import load_env_config

_env_cfg = load_env_config()

# --- Load model ---
model = DDPG.load("best_model.zip", device="cpu")

# --- VecNormalize (if enabled in training) ---
vec_norm_path = Path("model/vec_normalize_stats.pkl")
obs_rms = None
if vec_norm_path.exists():
    with open(vec_norm_path, 'rb') as f:
        vn = pickle.load(f)
    if getattr(vn, 'norm_obs', True):
        obs_rms = vn.obs_rms
        print("[INFO] VecNormalize norm_obs=True — applying normalization")

def _normalize(obs, obs_rms):
    if obs_rms is None:
        return obs
    return np.clip((obs - obs_rms.mean) / np.sqrt(obs_rms.var + 1e-8), -10, 10).astype(np.float32)

# --- Create environment ---
env_id = "robotRobotReachPyBullet-Default-v0"   # or "Collision-Free"
env = gym.make(
    env_id,
    enable_gui=True,
    action_step=_env_cfg['action_step'],
    distance_thresh=_env_cfg['distance_thresh'],
    max_episode_steps=_env_cfg['max_episode_steps'],
)

# --- Run one episode with a static target ---
obs, info = env.reset(seed=42)
obs = _normalize(obs, obs_rms)

while True:
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(action)
    obs = _normalize(obs, obs_rms)

    if terminated or truncated:
        print(f"Done — success={info['is_success']}, distance={info['distance']:.4f}m")
        break

env.close()
```

### With a Real Robot (no PyBullet)

This is the production control loop template. Replace the robot API calls with your actual robot SDK:

```python
import numpy as np
from stable_baselines3 import DDPG
import pickle
from pathlib import Path

# =============================================================================
# CONFIGURATION — adjust these for your deployment
# =============================================================================
MODEL_PATH      = "best_model.zip"          # Path to trained model
VEC_NORM_PATH    = "vec_normalize_stats.pkl" # Next to model.zip

ACTION_STEP     = 0.01    # metres per unit action (from env_config.yaml)
DIST_THRESH     = 0.02    # success threshold in metres
ENV_MODE        = 'Default'  # 'Default' or 'Collision-Free'
# =============================================================================

# --- Load model ---
model = DDPG.load(MODEL_PATH, device="cpu")

# --- Load VecNormalize stats (check if normalization is needed) ---
obs_rms = None
if Path(VEC_NORM_PATH).exists():
    with open(VEC_NORM_PATH, 'rb') as f:
        vn = pickle.load(f)
    if getattr(vn, 'norm_obs', True):
        obs_rms = vn.obs_rms
        print("[INFO] VecNormalize enabled — normalization will be applied")

def _normalize(obs, obs_rms):
    if obs_rms is None:
        return obs.astype(np.float32)
    return np.clip(
        (obs - obs_rms.mean) / np.sqrt(obs_rms.var + 1e-8),
        -10, 10
    ).astype(np.float32)

def build_observation(tcp_pos, target_pos):
    """Build the 15-D observation array."""
    error = target_pos - tcp_pos
    if ENV_MODE == 'Collision-Free':
        # Replace with actual obstacle position and size (normalized)
        obstacle_rel = (obstacle_pos - tcp_pos) / workspace_range
        obstacle_size = np.array([0.05, 0.05, 0.05]) / workspace_range
    else:
        obstacle_rel  = np.zeros(3)
        obstacle_size = np.zeros(3)

    return np.concatenate([
        tcp_pos,
        target_pos,
        error,
        obstacle_rel,
        obstacle_size,
    ], dtype=np.float32)

# =============================================================================
# MAIN CONTROL LOOP
# =============================================================================
target_pos = np.array([0.20, -0.40, 0.50])

while True:
    # 1. Read current TCP position from robot
    #    Replace this with your robot's API call:
    #    tcp_pos = robot.get_tcp_position()  # returns [x, y, z] in metres
    tcp_pos = np.array([0.10, -0.20, 0.30])  # <-- placeholder

    # 2. Check success
    distance = np.linalg.norm(tcp_pos - target_pos)
    if distance < DIST_THRESH:
        print(f"SUCCESS — reached target in {distance:.4f}m")
        break

    # 3. Build observation
    obs = build_observation(tcp_pos, target_pos)
    obs = _normalize(obs, obs_rms)

    # 4. Get action from model
    action, _ = model.predict(obs, deterministic=True)
    # action = [dx, dy, dz] in [-1, +1]

    # 5. Convert to metres and send to robot
    delta = action * ACTION_STEP
    #    Replace with your robot's API call:
    #    robot.move_tcp(delta)   # move TCP by delta metres
    print(f"  TCP={tcp_pos} | target={target_pos} | dist={distance:.4f} | action={action} | delta={delta}")

# =============================================================================
```

---

## Random Target Inference — Batch Evaluation

Run the model against 100 randomly sampled targets and log success/failure:

```python
import numpy as np
from stable_baselines3 import DDPG
import pickle
from pathlib import Path

MODEL_PATH    = "best_model.zip"
VEC_NORM_PATH = "vec_normalize_stats.pkl"
ACTION_STEP  = 0.01
DIST_THRESH  = 0.02
NUM_EPISODES = 100
ENV_MODE     = 'Default'

model = DDPG.load(MODEL_PATH, device="cpu")

# VecNormalize
obs_rms = None
if Path(VEC_NORM_PATH).exists():
    with open(VEC_NORM_PATH, 'rb') as f:
        vn = pickle.load(f)
    if getattr(vn, 'norm_obs', True):
        obs_rms = vn.obs_rms

def _normalize(obs):
    if obs_rms is None:
        return obs.astype(np.float32)
    return np.clip(
        (obs - obs_rms.mean) / np.sqrt(obs_rms.var + 1e-8),
        -10, 10
    ).astype(np.float32)

def build_observation(tcp_pos, target_pos):
    error = target_pos - tcp_pos
    obstacle_rel  = np.zeros(3)
    obstacle_size = np.zeros(3)
    return np.concatenate([tcp_pos, target_pos, error, obstacle_rel, obstacle_size], dtype=np.float32)

# robot workspace bounds (approximate)
X_MIN, X_MAX = 0.05, 0.35
Y_MIN, Y_MAX = -0.20, -0.80
Z_MIN, Z_MAX = 0.40, 0.60

successes = 0
for ep in range(1, NUM_EPISODES + 1):
    # Random target within workspace
    target_pos = np.array([
        np.random.uniform(X_MIN, X_MAX),
        np.random.uniform(Y_MIN, Y_MAX),
        np.random.uniform(Z_MIN, Z_MAX),
    ])

    # Move robot to home first
    # robot.move_to_home()

    # Reset TCP to home (placeholder — replace with real robot call)
    tcp_pos = np.array([0.10, -0.20, 0.30])

    steps = 0
    while True:
        obs = _normalize(build_observation(tcp_pos, target_pos))
        action, _ = model.predict(obs, deterministic=True)

        delta = action * ACTION_STEP
        # robot.move_tcp(delta)
        tcp_pos = tcp_pos + delta  # placeholder

        dist = np.linalg.norm(tcp_pos - target_pos)
        steps += 1

        if dist < DIST_THRESH:
            successes += 1
            result = "SUCCESS"
            break
        if steps >= 500:
            result = "FAIL"
            break

    print(f"EP {ep:03d}: {result} | dist={dist:.4f}m | steps={steps}")

print(f"\nSuccess rate: {successes}/{NUM_EPISODES} = {successes/NUM_EPISODES:.1%}")
```

---

## Using the Prediction Scripts (with PyBullet simulation)

The project includes ready-to-use prediction scripts. Edit the `CONST_MODEL_PATH` at the top of each script, then run:

```bash
# Static (single) target — logs TCP trajectory
python Evaluation/Gym/Model/Prediction/Static/predict_ddpg.py

# Random targets — runs 100 episodes and saves results
python Evaluation/Gym/Model/Prediction/Random/predict_ddpg.py

# For SAC or TD3 models:
python Evaluation/Gym/Model/Prediction/Random/predict_sac.py
python Evaluation/Gym/Model/Prediction/Random/predict_td3.py
```

In the prediction scripts, set `CONST_MODEL_PATH` to the path of your trained model `.zip` file and set `CONST_ENV_MODE` to match the environment the model was trained on (`Default` or `Collision-Free`).

---

## Success Condition

```python
import numpy as np

distance = np.linalg.norm(tcp_pos - target_pos)

if distance < 0.02:       # 2 cm threshold (from env_config.yaml)
    print("SUCCESS")
else:
    print(f"Not yet reached — distance: {distance:.4f}m")
```

---

## Collision-Free Mode — Obstacle Handling

If deploying a model trained in `Collision-Free` mode, you must provide obstacle information in the observation:

```python
import numpy as np

# Workspace range for normalization (robot approximate)
WORKSPACE_RANGE = 0.5  # metres

# Obstacle position and size (from your environment setup)
OBSTACLE_POS = np.array([0.20, -0.40, 0.50])   # metres [x, y, z]
OBSTACLE_SIZE = np.array([0.05, 0.05, 0.05])     # half-extents [x, y, z]

def build_observation_collision_free(tcp_pos, target_pos, obstacle_pos, obstacle_size):
    error = target_pos - tcp_pos
    rel_obs   = (obstacle_pos - tcp_pos) / WORKSPACE_RANGE
    norm_size = obstacle_size / WORKSPACE_RANGE
    return np.concatenate([
        tcp_pos, target_pos, error, rel_obs, norm_size
    ], dtype=np.float32)
```

---

## Deployment Checklist

Before deploying to a real robot:

- [ ] Copy `best_model.zip` and `config/env_config.yaml` to the deployment machine
- [ ] Verify `action_step` and `distance_thresh` match between training and deployment config
- [ ] Verify `ENV_MODE` (`Default` or `Collision-Free`) matches the trained model
- [ ] If the model was trained with `norm_obs=True`, ensure `vec_normalize_stats.pkl` is loaded and normalization is applied
- [ ] Confirm the robot's TCP coordinate frame matches the PyBullet convention (world frame, meters)
- [ ] Set `deterministic=True` in `model.predict()` for reproducible behavior
- [ ] Run the PyBullet simulation version first to validate the model performs as expected
- [ ] Start with a slow control loop (1–10 Hz) and increase frequency once behavior is validated
