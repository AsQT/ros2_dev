"""DRL trajectory planner core — pure DRL logic, no ROS dependencies.

Encapsulates all model-loading, observation-building, and trajectory-computation logic.
Used by drl_unified_planner_node.

Responsibilities:
  - Load DDPG/SAC/TD3 model from best_model.zip.
  - Load VecNormalize stats if present.
  - Load env_config.yaml.
  - Maintain calibrated start TCP in BASE and WORLD/DRL frames.
  - Convert between BASE and WORLD/DRL frames.
  - Build the 15D observation vector.
  - Compute trajectory from start to target via closed-loop DRL simulation.
  - Append exact target as final waypoint.
  - Build forward and backward trajectory lists.
  - Build PoseArray and MarkerArray messages for the DRL path.

This module does NOT:
  - Subscribe to vision topics.
  - Publish /vision/* markers.
  - Ask terminal input.
  - Know about camera images or TF.

Usage:
    planner = DrlTrajectoryPlannerCore(model, vn_stats, env_cfg, calibrated_start_tcp_base)
    result = planner.compute_trajectory(target_base, has_obstacle, obstacle_center_base, obstacle_full_size)
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

from gp7_drl_inference import config
from gp7_drl_inference.model_loader import load_model, predict
from gp7_drl_inference.state_builder import build_observation_15d


# -------------------------------------------------------------------------
# Planning result
# -------------------------------------------------------------------------

@dataclass
class PlanningResult:
    """Output of DrlTrajectoryPlannerCore.compute_trajectory()."""

    #: Whether the DRL converged to the target within distance_thresh.
    converged: bool

    #: Wall-clock time spent planning (seconds).
    planning_time_sec: float

    #: Euclidean distance from the DRL's final simulated TCP to the target
    #: (in WORLD/DRL frame), before appending the exact target waypoint.
    convergence_dist: float

    #: Forward trajectory in BASE frame.  waypoints[0] = start, waypoints[-1] = target.
    trajectory_forward_base: list[np.ndarray]

    #: Backward trajectory in BASE frame.  waypoints[0] = target, waypoints[-1] = start.
    trajectory_backward_base: list[np.ndarray]

    #: The planned-for target position in BASE frame.
    target_base: np.ndarray

    #: The planned-for target position in WORLD/DRL frame.
    target_drl: np.ndarray

    #: Whether an obstacle was used during planning.
    has_obstacle: bool

    #: Obstacle center in BASE frame (zero array if no obstacle).
    obstacle_center_base: np.ndarray

    #: Obstacle center in WORLD/DRL frame (zero array if no obstacle).
    obstacle_center_drl: np.ndarray

    #: Obstacle full D/W/H dimensions (zero array if no obstacle).
    obstacle_full_size: np.ndarray


# -------------------------------------------------------------------------
# Core planner class
# -------------------------------------------------------------------------

class DrlTrajectoryPlannerCore:
    """Reusable DRL trajectory planner.

    Pure Python / numpy — no ROS types.  Thread-safe for concurrent planning
    calls only if each caller owns its own instance (the usual ROS pattern).

    Args:
        model: Loaded Stable-Baselines3 DDPG model (from model_loader.load_model).
        vn_stats: VecNormalizeStats or None (from config.load_vec_normalize_stats).
        env_cfg: Dict loaded from env_config.yaml (from config.load_env_config).
        calibrated_start_tcp_base: Calibrated start TCP in BASE frame (shape (3,)).
            Obtained from the ``calibrated_start_tcp_base`` ROS parameter or CLI.
    """

    def __init__(
        self,
        model,
        vn_stats: Optional[config.VecNormalizeStats],
        env_cfg: dict,
        calibrated_start_tcp_base: np.ndarray,
    ) -> None:
        self._model = model
        self._vn_stats = vn_stats
        self._env_cfg = env_cfg

        # Planning parameters from env config
        self._action_step = float(env_cfg.get("action_step", config.ACTION_STEP))
        self._distance_thresh = float(
            env_cfg.get("distance_thresh", config.DEFAULT_DISTANCE_THRESH)
        )
        self._max_episode_steps = int(
            env_cfg.get("max_episode_steps", config.DEFAULT_MAX_EPISODE_STEPS)
        )
        self._workspace_range = float(
            env_cfg.get("workspace_range", config.DEFAULT_WORKSPACE_RANGE)
        )

        # Calibrated start TCP
        tcp = np.asarray(calibrated_start_tcp_base, dtype=np.float32)
        if tcp.shape != (3,):
            raise ValueError(
                f"calibrated_start_tcp_base must have shape (3,), got {tcp.shape}"
            )
        self._calibrated_start_tcp_base: np.ndarray = tcp
        self._start_tcp_base: np.ndarray = tcp.copy()
        self._start_tcp_drl: np.ndarray = config.base_to_drl_world(tcp)

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------

    @property
    def start_tcp_base(self) -> np.ndarray:
        """Calibrated start TCP in BASE frame."""
        return self._start_tcp_base.copy()

    @property
    def start_tcp_drl(self) -> np.ndarray:
        """Calibrated start TCP in WORLD/DRL frame."""
        return self._start_tcp_drl.copy()

    @property
    def calibrated_start_tcp_base(self) -> np.ndarray:
        """Original calibrated value in BASE frame."""
        return self._calibrated_start_tcp_base.copy()

    @property
    def action_step(self) -> float:
        return self._action_step

    @property
    def distance_thresh(self) -> float:
        return self._distance_thresh

    @property
    def max_episode_steps(self) -> int:
        return self._max_episode_steps

    @property
    def workspace_range(self) -> float:
        return self._workspace_range

    @property
    def vn_stats(self) -> Optional[config.VecNormalizeStats]:
        return self._vn_stats

    # -------------------------------------------------------------------------
    # Frame helpers (delegated to config)
    # -------------------------------------------------------------------------

    def base_to_drl(self, pos_base: np.ndarray) -> np.ndarray:
        """Convert BASE -> WORLD/DRL frame."""
        return config.base_to_drl_world(pos_base)

    def drl_to_base(self, pos_drl: np.ndarray) -> np.ndarray:
        """Convert WORLD/DRL -> BASE frame."""
        return config.drl_world_to_base(pos_drl)

    def update_start_tcp(self, new_tcp_base: np.ndarray) -> None:
        """Override the start TCP at runtime (e.g., from CLI override).

        Args:
            new_tcp_base: New start TCP in BASE frame, shape (3,).
        """
        tcp = np.asarray(new_tcp_base, dtype=np.float32)
        if tcp.shape != (3,):
            raise ValueError(f"new_tcp_base must have shape (3,), got {tcp.shape}")
        self._start_tcp_base = tcp.copy()
        self._start_tcp_drl = config.base_to_drl_world(tcp)

    # -------------------------------------------------------------------------
    # Core planning
    # -------------------------------------------------------------------------

    def compute_trajectory(
        self,
        target_base: np.ndarray,
        has_obstacle: bool,
        obstacle_center_base: Optional[np.ndarray] = None,
        obstacle_full_size: Optional[np.ndarray] = None,
        source: str = "unknown",
    ) -> PlanningResult:
        """Pre-compute the full DRL trajectory from calibrated start to target.

        This is a PURE SIMULATION loop.  The robot does not move.
        No real TCP feedback is consumed.  No FK is called.

        Args:
            target_base: Target TCP position in BASE frame, shape (3,).
            has_obstacle: If True, obstacle_center_base and obstacle_full_size
                must also be provided.
            obstacle_center_base: Obstacle center in BASE frame, shape (3,).
            obstacle_full_size: Obstacle full D/W/H dimensions, shape (3,).
            source: Arbitrary label for logging (e.g. "manual", "vision_realtime").

        Returns:
            PlanningResult with converged flag, timing, and trajectory lists.
        """
        planning_t0 = time.perf_counter()

        # Resolve and validate inputs
        target_base_arr = np.asarray(target_base, dtype=np.float32)
        if target_base_arr.shape != (3,):
            raise ValueError(f"target_base must be shape (3,), got {target_base_arr.shape}")

        target_drl = config.base_to_drl_world(target_base_arr)
        start_tcp_drl = self._start_tcp_drl.copy()

        if has_obstacle:
            if obstacle_center_base is None or obstacle_full_size is None:
                raise ValueError(
                    "has_obstacle=True requires obstacle_center_base and obstacle_full_size"
                )
            obstacle_center_base_arr = np.asarray(obstacle_center_base, dtype=np.float32)
            obstacle_full_size_arr = np.asarray(obstacle_full_size, dtype=np.float32)
            if obstacle_center_base_arr.shape != (3,):
                raise ValueError(
                    f"obstacle_center_base must be shape (3,), "
                    f"got {obstacle_center_base_arr.shape}"
                )
            if obstacle_full_size_arr.shape != (3,):
                raise ValueError(
                    f"obstacle_full_size must be shape (3,), "
                    f"got {obstacle_full_size_arr.shape}"
                )
            obstacle_center_drl = config.base_to_drl_world(obstacle_center_base_arr)
            obstacle_half_extent = obstacle_full_size_arr / 2.0
        else:
            obstacle_center_base_arr = np.zeros(3, dtype=np.float32)
            obstacle_center_drl = np.zeros(3, dtype=np.float32)
            obstacle_full_size_arr = np.zeros(3, dtype=np.float32)
            obstacle_half_extent = np.zeros(3, dtype=np.float32)

        # Initialize simulated TCP
        current_tcp_drl = start_tcp_drl.copy()
        trajectory_drl: list[np.ndarray] = [current_tcp_drl.copy()]
        trajectory_base: list[np.ndarray] = [
            config.drl_world_to_base(current_tcp_drl)
        ]

        converged = False

        for step in range(self._max_episode_steps):
            # Build raw 15D observation
            raw_obs = build_observation_15d(
                current_tcp_drl=current_tcp_drl,
                target_drl=target_drl,
                obstacle_center_drl=obstacle_center_drl,
                obstacle_half_extent=obstacle_half_extent,
                has_obstacle=has_obstacle,
                workspace_range=self._workspace_range,
            )

            # Normalize if VecNormalize is enabled
            obs = config.normalize_if_needed(raw_obs, self._vn_stats)

            # Model inference
            action = predict(self._model, obs)
            action = np.asarray(action, dtype=np.float32).reshape(3)

            # Convert action to displacement
            delta = action * self._action_step

            # Advance simulated TCP
            next_tcp_drl = current_tcp_drl + delta
            next_tcp_base = config.drl_world_to_base(next_tcp_drl)

            # Store
            trajectory_drl.append(next_tcp_drl.copy())
            trajectory_base.append(next_tcp_base.copy())

            # Update
            current_tcp_drl = next_tcp_drl

            # Check convergence
            dist = float(np.linalg.norm(current_tcp_drl - target_drl))
            if dist < self._distance_thresh:
                converged = True
                break

        # Capture model convergence state BEFORE appending exact target
        model_final_tcp_drl = trajectory_drl[-1]
        convergence_dist = float(np.linalg.norm(model_final_tcp_drl - target_drl))

        # Append exact target as final waypoint
        if np.linalg.norm(trajectory_drl[-1] - target_drl) > 1e-6:
            trajectory_drl.append(target_drl.copy())
            trajectory_base.append(target_base_arr.copy())

        # Build forward/backward lists
        trajectory_forward_base = list(trajectory_base)
        trajectory_backward_base = list(reversed(trajectory_base))

        elapsed = time.perf_counter() - planning_t0

        return PlanningResult(
            converged=converged,
            planning_time_sec=elapsed,
            convergence_dist=convergence_dist,
            trajectory_forward_base=trajectory_forward_base,
            trajectory_backward_base=trajectory_backward_base,
            target_base=target_base_arr,
            target_drl=target_drl,
            has_obstacle=has_obstacle,
            obstacle_center_base=obstacle_center_base_arr,
            obstacle_center_drl=obstacle_center_drl,
            obstacle_full_size=obstacle_full_size_arr,
        )


# -------------------------------------------------------------------------
# Convenience factory (loads everything from package share path)
# -------------------------------------------------------------------------

def load_planner(
    model_subpath: str = config.DEFAULT_MODEL_NAME,
    vec_normalize_subpath: str = config.DEFAULT_VEC_NORMALIZE_NAME,
    env_config_path: Optional[str] = None,
    calibrated_start_tcp_base: Optional[list[float]] = None,
) -> tuple[DrlTrajectoryPlannerCore, dict]:
    """Load model, VecNormalize, env config, and instantiate the planner.

    This is a convenience wrapper around the individual loaders.  It locates
    files relative to the package share directory so nodes don't need to
    compute paths themselves.

    Args:
        model_subpath: Relative path under ``share/gp7_drl_inference/models/``.
        vec_normalize_subpath: Relative path under ``share/gp7_drl_inference/models/``.
        env_config_path: Explicit path to env_config.yaml, or None for auto-detect.
        calibrated_start_tcp_base: Override for the start TCP, or None for default.

    Returns:
        (planner, env_cfg) tuple.
    """
    from ament_index_python.packages import get_package_share_directory

    pkg_share = get_package_share_directory("gp7_drl_inference")
    model_path = Path(pkg_share) / "models" / model_subpath
    vn_path = Path(pkg_share) / "models" / vec_normalize_subpath

    # VecNormalize
    vn_stats = None
    if vn_path.exists():
        try:
            vn_stats = config.load_vec_normalize_stats(vn_path)
            print(f"[drl_planner_core] VecNormalize loaded: norm_obs={vn_stats.norm_obs}")
        except Exception as e:
            print(f"[drl_planner_core] WARNING: failed to load VecNormalize: {e}")

    # Env config
    env_cfg = config.load_env_config(env_config_path)

    # Model
    model = load_model(str(model_path))

    # Start TCP
    if calibrated_start_tcp_base is not None:
        tcp = np.array(calibrated_start_tcp_base, dtype=np.float32)
    else:
        tcp = np.array([0.350, -0.330, 0.060], dtype=np.float32)

    planner = DrlTrajectoryPlannerCore(
        model=model,
        vn_stats=vn_stats,
        env_cfg=env_cfg,
        calibrated_start_tcp_base=tcp,
    )

    return planner, env_cfg
