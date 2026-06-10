"""Load a Stable-Baselines3 DDPG model from a .zip file.

The model was trained on Windows with numpy/cloudpickle metadata that can be
incompatible with the loading environment.  Compatibility patches and
custom_objects are applied so that DDPG.load() succeeds without errors.

VecNormalize handling is delegated to config.py — this module only loads
the policy network.
"""

import sys
from pathlib import Path
from typing import Optional

import numpy as np
from gymnasium import spaces
from stable_baselines3 import DDPG

from gp7_drl_inference import config


def _patch_numpy_pickle_alias() -> None:
    """Patch numpy pickle aliases for cross-environment compatibility."""
    sys.modules.setdefault("numpy._core", np.core)
    sys.modules.setdefault("numpy._core.numeric", np.core.numeric)
    sys.modules.setdefault("numpy._core.multiarray", np.core.multiarray)


def load_model(model_path: str | Path) -> DDPG:
    """Load a trained DDPG model.

    Args:
        model_path: Path to the model.zip file.

    Returns:
        Loaded DDPG model on CPU.

    Raises:
        FileNotFoundError: If the model file does not exist.
    """
    model_path = Path(model_path)

    if not model_path.exists():
        raise FileNotFoundError(f"Model file not found: {model_path}")

    _patch_numpy_pickle_alias()

    custom_objects = {
        "observation_space": spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(config.OBS_DIM,),
            dtype=np.float32,
        ),
        "action_space": spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(config.ACTION_DIM,),
            dtype=np.float32,
        ),
        "lr_schedule": lambda _: 0.0,
        "_last_obs": None,
        "_last_episode_starts": None,
        "env": None,
        "replay_buffer": None,
    }

    return DDPG.load(
        str(model_path),
        device="cpu",
        custom_objects=custom_objects,
    )


def predict(model: DDPG, observation: np.ndarray) -> np.ndarray:
    """Run one-step inference. Returns action in [-1, 1]."""
    obs = observation.reshape(1, -1).astype(np.float32)
    action, _ = model.predict(obs, deterministic=True)
    return action.flatten()
