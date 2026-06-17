"""Build the 15D observation vector consumed by the DRL policy.

EXACT layout matching the PyBullet training environment (robotReachPyBulletEnv):
  0-2   tcp_x, tcp_y, tcp_z           TCP position [m, WORLD/DRL frame]
  3-5   target_x, target_y, target_z  Target position [m, WORLD/DRL frame]
  6-8   err_x, err_y, err_z          target - tcp [m]
  9-11  rel_obs_x, rel_obs_y, rel_obs_z  normalized obstacle relative position
  12-14 obs_size_x, obs_size_y, obs_size_z  normalized obstacle half-extents

Used by drl_unified_planner_node via build_observation_15d().
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from robot_drl import config


# -------------------------------------------------------------------------
# Standalone 15D observation builder
# -------------------------------------------------------------------------

def build_observation_15d(
    current_tcp_drl: np.ndarray,
    target_drl: np.ndarray,
    obstacle_center_drl: Optional[np.ndarray] = None,
    obstacle_half_extent: Optional[np.ndarray] = None,
    has_obstacle: bool = False,
    workspace_range: float = 0.5,
) -> np.ndarray:
    """Build a 15D observation array in WORLD/DRL frame.

    Used by drl_unified_planner_node for planning from terminal or vision input.
    All positions must already be in WORLD/DRL frame (i.e. after base_to_drl_world conversion).

    Args:
        current_tcp_drl:  Current TCP position (WORLD/DRL frame), shape (3,).
        target_drl:       Target position (WORLD/DRL frame), shape (3,).
        workspace_range:  Normalization constant (default 0.5 m).

    Returns:
        15-element np.float32 array.

    Raises:
        ValueError: If has_obstacle=True but obstacle params are missing,
                    or if any array has the wrong shape.
    """
    if np.asarray(current_tcp_drl).shape != (3,):
        raise ValueError(f"current_tcp_drl must be shape (3,), got {np.asarray(current_tcp_drl).shape}")
    if np.asarray(target_drl).shape != (3,):
        raise ValueError(f"target_drl must be shape (3,), got {np.asarray(target_drl).shape}")

    tcp = np.asarray(current_tcp_drl, dtype=np.float32)
    tgt = np.asarray(target_drl, dtype=np.float32)
    error = tgt - tcp

    if has_obstacle:
        if obstacle_center_drl is None or obstacle_half_extent is None:
            raise ValueError(
                "has_obstacle=True requires obstacle_center_drl and obstacle_half_extent"
            )
        oc = np.asarray(obstacle_center_drl, dtype=np.float32)
        oh = np.asarray(obstacle_half_extent, dtype=np.float32)
        if oc.shape != (3,) or oh.shape != (3,):
            raise ValueError(
                f"obstacle arrays must be shape (3,), got center={oc.shape}, extent={oh.shape}"
            )
        rel_obs = (oc - tcp) / workspace_range
        obs_size = oh / workspace_range
    else:
        rel_obs = np.zeros(3, dtype=np.float32)
        obs_size = np.zeros(3, dtype=np.float32)

    obs = np.concatenate([
        tcp,       # 0-2  TCP (WORLD/DRL)
        tgt,       # 3-5  target (WORLD/DRL)
        error,     # 6-8  error
        rel_obs,   # 9-11 relative obstacle position
        obs_size,  # 12-14 obstacle half-extents
    ], dtype=np.float32)

    assert obs.shape == (config.OBS_DIM,), f"Expected shape ({config.OBS_DIM},), got {obs.shape}"
    assert obs.dtype == np.float32, f"Expected dtype float32, got {obs.dtype}"
    assert np.all(np.isfinite(obs)), f"Observation contains non-finite values: {obs}"
    return obs
