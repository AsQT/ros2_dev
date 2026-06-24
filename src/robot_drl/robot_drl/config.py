"""Constants for the robot_drl ROS2 package.

Values are sourced from models/env_config.yaml and match the training environment
exactly so the deployed policy sees the same inputs it was trained on.
"""

import os
from pathlib import Path
from typing import Optional

import numpy as np

# Topic subscriptions
TOPIC_TCP_POSE = "/tcp_pose"                 # Real robot only, if provided by hardware bridge.
TOPIC_TCP_TF_SOURCE = "base_link"            # Simulation & real: TF base -> tcp_link
TOPIC_TCP_TF_TARGET = "tcp_link"
TOPIC_TARGET_POSE = "/detected_object/pose"
TOPIC_BBOX = "/vision/box_detection"
TOPIC_ACTION_OUT = "/drl/action"

# Topic publications
TOPIC_ACTION_BRIDGE_NEXT_POSE = "/drl/next_pose"

# Mock environment
TOPIC_OBSTACLE_POSE = "/obstacle/pose"

# Services
SERVICE_MOVE_TO_CARTESIAN = "/move_to_cartesian_target"
SERVICE_EXECUTE_TRAJECTORY = "/drl/execute_trajectory"
SERVICE_CLEAR_TRAJECTORY = "/drl/clear_trajectory"
SERVICE_REPLAN = "/drl/replan"

# Dimension constants
OBS_DIM = 15
ACTION_DIM = 3

# Default paths
DEFAULT_MODEL_NAME = "run2/model/best_model.zip"
DEFAULT_VEC_NORMALIZE_NAME = "run2/model/vec_normalize_stats.pkl"
DEFAULT_RATE_HZ = 10.0

# Trained action step in metres (from env_config.yaml action_step)
ACTION_STEP = 0.01

# Z-offset between real-world robot frame and the PyBullet simulation frame.
# PyBullet places the table surface at z=0; the real TCP z (from /tcp_pose)
# is measured from the world origin which is 330 mm above the table.
# Training input:  z_training = z_world - Z_OFFSET
# Output inversion: z_world += Z_OFFSET * action_z
FRAME_Z_OFFSET = 0.0

# Planning defaults
DEFAULT_DISTANCE_THRESH = 0.02   # metres — success threshold
DEFAULT_MAX_EPISODE_STEPS = 500  # maximum planning loop iterations
DEFAULT_WORKSPACE_RANGE = 0.5    # metres — for obstacle normalization
DEFAULT_EXECUTE_RATE_HZ = 10.0   # Hz — waypoint streaming rate during execution
DEFAULT_OBSTACLE_SAFETY_MARGIN = 0.04  # metres — TCP clearance used by rollout filter
DEFAULT_WORKSPACE_MIN = np.array([0.2500, -0.150, 0.020], dtype=np.float32)
DEFAULT_WORKSPACE_MAX = np.array([0.5000, 0.150, 0.300], dtype=np.float32)
DEFAULT_START_TCP_BASE = np.array([0.350, 0.000, 0.250], dtype=np.float32)
DEFAULT_TARGET_BASE = np.array([0.450, 0.100, 0.120], dtype=np.float32)
DEFAULT_OBSTACLE_CENTER_BASE = np.array([0.400, 0.000, 0.120], dtype=np.float32)
DEFAULT_OBSTACLE_SIZE = np.array([0.100, 0.100, 0.100], dtype=np.float32)

# -------------------------------------------------------------------------
# Frame conversion helpers
# -------------------------------------------------------------------------

def base_to_drl_world(pos_base) -> np.ndarray:
    """Convert a position from BASE frame to WORLD/DRL frame.

    Adds 330 mm to Z. The DRL model was trained in WORLD/DRL coordinates,
    so all positions fed to the model must be in this frame.
    """
    return np.asarray(pos_base, dtype=np.float32) + np.array(
        [0.0, 0.0, FRAME_Z_OFFSET], dtype=np.float32
    )


def drl_world_to_base(pos_drl) -> np.ndarray:
    """Convert a position from WORLD/DRL frame to BASE frame.

    Subtracts 330 mm from Z. All RViz and executor output must be in BASE frame.
    """
    return np.asarray(pos_drl, dtype=np.float32) - np.array(
        [0.0, 0.0, FRAME_Z_OFFSET], dtype=np.float32
    )


# -------------------------------------------------------------------------
# VecNormalize
# -------------------------------------------------------------------------

class VecNormalizeStats:
    """Loaded VecNormalize statistics (lazily loaded from pickle)."""

    def __init__(self) -> None:
        self.obs_rms: Optional[object] = None  # running mean/std
        self.norm_obs: bool = True              # whether normalization is enabled


def load_vec_normalize_stats(pkl_path: str | Path) -> VecNormalizeStats:
    """Load VecNormalize statistics from a pickle file.

    Checks norm_obs flag and only stores obs_rms if normalization is enabled.

    Args:
        pkl_path: Path to vec_normalize_stats.pkl.

    Returns:
        VecNormalizeStats with obs_rms set if norm_obs==True, else None.

    Raises:
        FileNotFoundError: If the file does not exist.
    """
    import pickle

    pkl_path = Path(pkl_path)
    if not pkl_path.exists():
        raise FileNotFoundError(f"VecNormalize stats not found: {pkl_path}")

    with open(pkl_path, "rb") as f:
        vn = pickle.load(f)

    stats = VecNormalizeStats()
    stats.norm_obs = bool(getattr(vn, "norm_obs", True))
    if stats.norm_obs:
        stats.obs_rms = getattr(vn, "obs_rms", None)
    return stats


def normalize_if_needed(
    raw_obs: np.ndarray, stats: Optional[VecNormalizeStats]
) -> np.ndarray:
    """Apply VecNormalize observation normalization if enabled.

    Args:
        raw_obs: Raw 15D observation from build_observation_15d.
        stats: Loaded VecNormalize stats, or None if file absent / norm_obs=False.

    Returns:
        Normalized observation if stats is not None and norm_obs==True,
        otherwise the raw observation unchanged.
    """
    if stats is None or stats.obs_rms is None:
        return raw_obs.astype(np.float32)
    obs_rms = stats.obs_rms
    return np.clip(
        (raw_obs - obs_rms.mean) / np.sqrt(obs_rms.var + 1e-8),
        -10.0,
        10.0,
    ).astype(np.float32)


# -------------------------------------------------------------------------
# Environment config loading
# -------------------------------------------------------------------------

def load_env_config(env_config_path: Optional[str] = None) -> dict:
    """Load environment configuration from YAML.

    Searches in the usual install location if no explicit path is given.
    Returns defaults for any missing keys.
    """
    import yaml

    defaults = {
        "action_step": ACTION_STEP,
        "distance_thresh": DEFAULT_DISTANCE_THRESH,
        "max_episode_steps": DEFAULT_MAX_EPISODE_STEPS,
        "workspace_range": DEFAULT_WORKSPACE_RANGE,
        "workspace_min": DEFAULT_WORKSPACE_MIN.tolist(),
        "workspace_max": DEFAULT_WORKSPACE_MAX.tolist(),
        "frame_z_offset": FRAME_Z_OFFSET,
    }

    if env_config_path is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory("robot_drl")
            candidates = [
                Path(share) / "models" / "env_config.yaml",
                Path(share) / "config" / "env_config.yaml",
            ]
            for cand in candidates:
                if cand.exists():
                    env_config_path = str(cand)
                    break
        except Exception:
            pass

    if env_config_path and Path(env_config_path).exists():
        with open(env_config_path) as f:
            data = yaml.safe_load(f) or {}
            if "workspace" in data:
                workspace = data["workspace"] or {}
                if "min" in workspace:
                    data["workspace_min"] = workspace["min"]
                if "max" in workspace:
                    data["workspace_max"] = workspace["max"]
            return {**defaults, **data}

    return defaults
