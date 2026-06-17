"""Load a Stable-Baselines3 off-policy model from a .zip file.

The model was trained on Windows with numpy/cloudpickle metadata that can be
incompatible with the loading environment.  Compatibility patches and
custom_objects are applied so that SB3 load succeeds without errors.

VecNormalize handling is delegated to config.py — this module only loads
the policy network.
"""

import json
import os
import sys
from zipfile import ZipFile
from zipfile import BadZipFile
from pathlib import Path

import numpy as np


def _add_active_venv_site_packages() -> None:
    """Let ROS console scripts launched by /usr/bin/python see the active venv."""
    venv = os.environ.get("VIRTUAL_ENV")
    if not venv:
        return
    version = f"python{sys.version_info.major}.{sys.version_info.minor}"
    site_packages = Path(venv) / "lib" / version / "site-packages"
    if site_packages.exists():
        sys.path.insert(0, str(site_packages))


_add_active_venv_site_packages()

from gymnasium import spaces
from stable_baselines3 import DDPG, SAC, TD3

from robot_drl import config


def _patch_numpy_pickle_alias() -> None:
    """Patch numpy pickle aliases for cross-environment compatibility."""
    sys.modules.setdefault("numpy._core", np.core)
    sys.modules.setdefault("numpy._core.numeric", np.core.numeric)
    sys.modules.setdefault("numpy._core.multiarray", np.core.multiarray)


def _detect_algorithm(model_path: Path) -> str:
    """Infer the SB3 algorithm from the saved zip metadata."""
    with ZipFile(model_path) as archive:
        raw_data = archive.read("data").decode("utf-8", errors="replace")
    metadata = json.loads(raw_data)
    policy = metadata.get("policy_class", {})
    module = str(policy.get("__module__", "")).lower()
    serialized = str(policy.get(":serialized:", "")).lower()
    text = f"{module} {serialized} {raw_data[:2000].lower()}"
    if "stable_baselines3.sac" in text or "sacpolicy" in text:
        return "SAC"
    if "stable_baselines3.td3" in text or "td3policy" in text:
        return "TD3"
    if "stable_baselines3.ddpg" in text or "ddpgpolicy" in text:
        return "DDPG"
    return "DDPG"


def load_model(model_path: str | Path):
    """Load a trained DDPG/SAC/TD3 model.

    Args:
        model_path: Path to the model.zip file.

    Returns:
        Loaded SB3 model on CPU.

    Raises:
        FileNotFoundError: If the model file does not exist.
    """
    model_path = Path(model_path)

    if not model_path.exists():
        raise FileNotFoundError(f"Model file not found: {model_path}")
    if model_path.stat().st_size == 0:
        raise ValueError(f"Model file is empty: {model_path}")

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

    try:
        algo_name = _detect_algorithm(model_path)
    except BadZipFile as exc:
        raise ValueError(f"Model file is not a valid SB3 zip: {model_path}") from exc
    algo_cls = {"DDPG": DDPG, "SAC": SAC, "TD3": TD3}[algo_name]
    print(f"[model_loader] Loading {algo_name} model: {model_path}")

    return algo_cls.load(
        str(model_path),
        device="cpu",
        custom_objects=custom_objects,
    )


def predict(model, observation: np.ndarray) -> np.ndarray:
    """Run one-step inference. Returns action in [-1, 1]."""
    obs = observation.reshape(1, -1).astype(np.float32)
    action, _ = model.predict(obs, deterministic=True)
    return action.flatten()
