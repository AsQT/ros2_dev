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

from robot_drl import config
from robot_drl.model_loader import load_model, predict
from robot_drl.state_builder import build_observation_15d


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

    #: First raw 15D observation built for this rollout.
    first_raw_observation: np.ndarray

    #: First observation after optional VecNormalize.
    first_model_observation: np.ndarray

    #: Last raw 15D observation built for this rollout.
    last_raw_observation: np.ndarray

    #: Obstacle slice from the first raw observation, indices 9:15.
    first_obstacle_slice: np.ndarray

    #: Human-readable diagnostic lines for the first rollout steps.
    rollout_diagnostics: list[str] = field(default_factory=list)

    #: Number of rollout steps where the safety filter replaced the model action.
    safety_filter_adjustments: int = 0


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
        if self._workspace_range <= 0.0:
            raise ValueError(f"workspace_range must be > 0, got {self._workspace_range}")
        self._workspace_min_base = np.asarray(
            env_cfg.get("workspace_min", config.DEFAULT_WORKSPACE_MIN),
            dtype=np.float32,
        )
        self._workspace_max_base = np.asarray(
            env_cfg.get("workspace_max", config.DEFAULT_WORKSPACE_MAX),
            dtype=np.float32,
        )
        if self._workspace_min_base.shape != (3,):
            raise ValueError(
                f"workspace_min must have shape (3,), got {self._workspace_min_base.shape}"
            )
        if self._workspace_max_base.shape != (3,):
            raise ValueError(
                f"workspace_max must have shape (3,), got {self._workspace_max_base.shape}"
            )
        if np.any(self._workspace_min_base >= self._workspace_max_base):
            raise ValueError(
                "workspace_min must be strictly smaller than workspace_max on all axes"
            )
        self._obstacle_safety_filter_enabled = bool(
            env_cfg.get("obstacle_safety_filter_enabled", True)
        )
        self._obstacle_safety_margin = max(
            0.0,
            float(
                env_cfg.get(
                    "obstacle_safety_margin",
                    config.DEFAULT_OBSTACLE_SAFETY_MARGIN,
                )
            ),
        )
        self._safety_check_step = max(
            0.001,
            float(env_cfg.get("obstacle_safety_check_step_m", self._action_step / 2.0)),
        )

        model_obs_space = getattr(model, "observation_space", None)
        model_obs_shape = getattr(model_obs_space, "shape", None)
        if model_obs_shape is not None and tuple(model_obs_shape) != (config.OBS_DIM,):
            raise ValueError(
                f"Model observation_space.shape={model_obs_shape}, "
                f"but robot_drl builds ({config.OBS_DIM},)"
            )

        # Calibrated start TCP
        tcp = np.asarray(calibrated_start_tcp_base, dtype=np.float32)
        if tcp.shape != (3,):
            raise ValueError(
                f"calibrated_start_tcp_base must have shape (3,), got {tcp.shape}"
            )
        self._validate_workspace_position(tcp, "calibrated_start_tcp_base")
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
    def workspace_min_base(self) -> np.ndarray:
        return self._workspace_min_base.copy()

    @property
    def workspace_max_base(self) -> np.ndarray:
        return self._workspace_max_base.copy()

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
        self._validate_workspace_position(tcp, "new_tcp_base")
        self._start_tcp_base = tcp.copy()
        self._start_tcp_drl = config.base_to_drl_world(tcp)

    def _validate_workspace_position(self, pos_base: np.ndarray, label: str) -> None:
        if np.any(pos_base < self._workspace_min_base) or np.any(pos_base > self._workspace_max_base):
            raise ValueError(
                f"{label}={pos_base.tolist()} is outside trained workspace "
                f"min={self._workspace_min_base.tolist()}, "
                f"max={self._workspace_max_base.tolist()}"
            )

    def _clip_to_workspace(self, pos_base: np.ndarray) -> np.ndarray:
        return np.clip(pos_base, self._workspace_min_base, self._workspace_max_base)

    def _point_aabb_signed_distance(
        self,
        point_base: np.ndarray,
        center_base: np.ndarray,
        half_extent_base: np.ndarray,
    ) -> float:
        delta = np.abs(point_base - center_base) - half_extent_base
        outside = np.maximum(delta, 0.0)
        outside_dist = float(np.linalg.norm(outside))
        if outside_dist > 0.0:
            return outside_dist
        return float(np.max(delta))

    def _segment_aabb_min_clearance(
        self,
        start_base: np.ndarray,
        end_base: np.ndarray,
        center_base: np.ndarray,
        half_extent_base: np.ndarray,
    ) -> float:
        dist = float(np.linalg.norm(end_base - start_base))
        n_steps = max(1, int(np.ceil(dist / self._safety_check_step)))
        min_clearance = float("inf")
        for idx in range(n_steps + 1):
            alpha = idx / n_steps
            sample = ((1.0 - alpha) * start_base + alpha * end_base).astype(np.float32)
            clearance = self._point_aabb_signed_distance(
                sample,
                center_base,
                half_extent_base,
            )
            min_clearance = min(min_clearance, clearance)
        return min_clearance

    def _segment_obstacles_min_clearance(
        self,
        start_base: np.ndarray,
        end_base: np.ndarray,
        safety_obstacles_base: list[tuple[np.ndarray, np.ndarray]],
    ) -> tuple[float, int]:
        min_clearance = float("inf")
        min_index = -1
        for idx, (center_base, full_size) in enumerate(safety_obstacles_base):
            inflated_half = full_size / 2.0 + self._obstacle_safety_margin
            clearance = self._segment_aabb_min_clearance(
                start_base,
                end_base,
                center_base,
                inflated_half,
            )
            if clearance < min_clearance:
                min_clearance = clearance
                min_index = idx
        return min_clearance, min_index

    def _normalize_safety_obstacles(
        self,
        primary_center_base: np.ndarray,
        primary_full_size: np.ndarray,
        has_primary_obstacle: bool,
        safety_obstacles_base: Optional[list[tuple[np.ndarray, np.ndarray]]],
    ) -> list[tuple[np.ndarray, np.ndarray]]:
        obstacles: list[tuple[np.ndarray, np.ndarray]] = []
        if has_primary_obstacle:
            obstacles.append((primary_center_base.copy(), primary_full_size.copy()))
        if safety_obstacles_base:
            for idx, (center_base, full_size) in enumerate(safety_obstacles_base):
                center = np.asarray(center_base, dtype=np.float32)
                size = np.asarray(full_size, dtype=np.float32)
                if center.shape != (3,) or size.shape != (3,):
                    raise ValueError(
                        "safety_obstacles_base entries must be "
                        f"(center(3), full_size(3)); entry {idx} has "
                        f"center={center.shape}, size={size.shape}"
                    )
                if np.any(size <= 0.0):
                    raise ValueError(
                        f"safety obstacle {idx} full_size must be > 0, got {size}"
                    )
                obstacles.append((center.copy(), size.copy()))

        unique: list[tuple[np.ndarray, np.ndarray]] = []
        for center, size in obstacles:
            if not any(
                np.allclose(center, existing_center, atol=1e-6, rtol=0.0)
                and np.allclose(size, existing_size, atol=1e-6, rtol=0.0)
                for existing_center, existing_size in unique
            ):
                unique.append((center, size))
        return unique

    def _candidate_delta_from_direction(self, direction: np.ndarray) -> Optional[np.ndarray]:
        direction = np.asarray(direction, dtype=np.float32)
        norm = float(np.linalg.norm(direction))
        if norm <= 1e-8:
            return None
        unit = direction / norm
        max_component = float(np.max(np.abs(unit)))
        if max_component <= 1e-8:
            return None
        # The policy action is bounded per axis, so keep every delta component
        # within the same action_step limit while preserving the candidate direction.
        return (unit * (self._action_step / max_component)).astype(np.float32)

    def _aabb_escape_directions(
        self,
        point_base: np.ndarray,
        center_base: np.ndarray,
        half_extent_base: np.ndarray,
    ) -> list[np.ndarray]:
        lower = center_base - half_extent_base
        upper = center_base + half_extent_base
        directions: list[np.ndarray] = []
        basis = np.eye(3, dtype=np.float32)
        for axis in range(3):
            if lower[axis] <= point_base[axis] <= upper[axis]:
                directions.append(basis[axis])
                directions.append(-basis[axis])
            elif point_base[axis] < lower[axis]:
                directions.append(-basis[axis])
            else:
                directions.append(basis[axis])
        closest = np.minimum(np.maximum(point_base, lower), upper)
        away = point_base - closest
        if float(np.linalg.norm(away)) <= 1e-8:
            face_distances: list[tuple[float, np.ndarray]] = []
            for axis in range(3):
                pos_dir = basis[axis]
                neg_dir = -basis[axis]
                face_distances.append((float(abs(upper[axis] - point_base[axis])), pos_dir))
                face_distances.append((float(abs(point_base[axis] - lower[axis])), neg_dir))
            face_distances.sort(key=lambda item: item[0])
            directions.append(face_distances[0][1])
        else:
            directions.append(away)
        return directions

    def _preferred_bypass_directions(
        self,
        current_base: np.ndarray,
        target_base: np.ndarray,
        center_base: np.ndarray,
        half_extent_base: np.ndarray,
    ) -> list[np.ndarray]:
        lower = center_base - half_extent_base
        upper = center_base + half_extent_base
        basis = np.eye(3, dtype=np.float32)
        preferred: list[np.ndarray] = []
        for crossing_axis in range(3):
            crosses_box = (
                current_base[crossing_axis] > upper[crossing_axis]
                and target_base[crossing_axis] < lower[crossing_axis]
            ) or (
                current_base[crossing_axis] < lower[crossing_axis]
                and target_base[crossing_axis] > upper[crossing_axis]
            )
            if not crosses_box:
                continue
            for axis in range(3):
                if axis == crossing_axis:
                    continue
                if lower[axis] <= current_base[axis] <= upper[axis]:
                    # Leave through the face closest to the target-side corridor.
                    dist_to_upper_from_target = abs(float(target_base[axis] - upper[axis]))
                    dist_to_lower_from_target = abs(float(target_base[axis] - lower[axis]))
                    sign = 1.0 if dist_to_upper_from_target <= dist_to_lower_from_target else -1.0
                    preferred.append(sign * basis[axis])
        return preferred

    def _safety_filter_delta_base(
        self,
        current_base: np.ndarray,
        target_base: np.ndarray,
        desired_delta_base: np.ndarray,
        safety_obstacles_base: list[tuple[np.ndarray, np.ndarray]],
    ) -> tuple[np.ndarray, bool, str]:
        if (
            not self._obstacle_safety_filter_enabled
            or self._obstacle_safety_margin <= 0.0
            or not safety_obstacles_base
        ):
            return desired_delta_base, False, "disabled"

        desired_next = self._clip_to_workspace(current_base + desired_delta_base)
        desired_clearance, blocking_index = self._segment_obstacles_min_clearance(
            current_base,
            desired_next,
            safety_obstacles_base,
        )
        if desired_clearance >= -1e-6:
            return desired_next - current_base, False, f"clearance={desired_clearance:.5f}"

        current_dist = float(np.linalg.norm(target_base - current_base))
        goal_dir = target_base - current_base
        desired_dir = desired_delta_base.copy()
        away_dirs: list[np.ndarray] = []
        preferred_dirs: list[np.ndarray] = []
        for idx, (center_base, full_size) in enumerate(safety_obstacles_base):
            inflated_half = full_size / 2.0 + self._obstacle_safety_margin
            current_clearance = self._point_aabb_signed_distance(
                current_base,
                center_base,
                inflated_half,
            )
            is_close = current_clearance < max(3.0 * self._action_step, 0.03)
            if idx == blocking_index or is_close:
                away_dirs.extend(
                    self._aabb_escape_directions(
                        current_base,
                        center_base,
                        inflated_half,
                    )
                )
                preferred_dirs.extend(
                    self._preferred_bypass_directions(
                        current_base,
                        target_base,
                        center_base,
                        inflated_half,
                    )
                )

        raw_dirs: list[np.ndarray] = [
            desired_dir,
            goal_dir,
            0.7 * desired_dir + 0.3 * goal_dir,
        ]
        raw_dirs.extend(preferred_dirs)
        raw_dirs.extend(away_dirs)
        for preferred in preferred_dirs:
            raw_dirs.append(goal_dir + 2.5 * preferred)
            raw_dirs.append(desired_dir + 2.5 * preferred)
        for away in away_dirs:
            raw_dirs.append(goal_dir + 2.0 * away)
            raw_dirs.append(desired_dir + 2.0 * away)
            goal_norm = float(np.linalg.norm(goal_dir))
            away_norm = float(np.linalg.norm(away))
            if goal_norm > 1e-8 and away_norm > 1e-8:
                tangent = np.cross(goal_dir / goal_norm, away / away_norm)
                if float(np.linalg.norm(tangent)) > 1e-8:
                    raw_dirs.append(goal_dir + tangent)
                    raw_dirs.append(goal_dir - tangent)
                    raw_dirs.append(desired_dir + tangent)
                    raw_dirs.append(desired_dir - tangent)

        scored: list[tuple[float, np.ndarray, float]] = []
        for raw_dir in raw_dirs:
            candidate_delta = self._candidate_delta_from_direction(raw_dir)
            if candidate_delta is None:
                continue
            candidate_next = self._clip_to_workspace(current_base + candidate_delta)
            clipped_penalty = float(
                np.linalg.norm((current_base + candidate_delta) - candidate_next)
            )
            if float(np.linalg.norm(candidate_next - current_base)) <= 1e-8:
                continue
            clearance, _ = self._segment_obstacles_min_clearance(
                current_base,
                candidate_next,
                safety_obstacles_base,
            )
            if clearance < -1e-6:
                continue
            dist_after = float(np.linalg.norm(target_base - candidate_next))
            progress = current_dist - dist_after
            deviation = float(np.linalg.norm(candidate_delta - desired_delta_base))
            preferred_alignment = 0.0
            candidate_norm = float(np.linalg.norm(candidate_delta))
            if candidate_norm > 1e-8:
                candidate_dir = candidate_delta / candidate_norm
                for preferred in preferred_dirs:
                    preferred_norm = float(np.linalg.norm(preferred))
                    if preferred_norm > 1e-8:
                        preferred_alignment = max(
                            preferred_alignment,
                            float(np.dot(candidate_dir, preferred / preferred_norm)),
                        )
            score = (
                dist_after
                - 0.50 * progress
                - 1.50 * min(clearance, 0.02)
                + 0.25 * deviation
                + 5.0 * clipped_penalty
                - 0.08 * max(preferred_alignment, 0.0)
            )
            scored.append((score, candidate_next - current_base, clearance))

        if scored:
            scored.sort(key=lambda item: item[0])
            _, best_delta, best_clearance = scored[0]
            return best_delta.astype(np.float32), True, (
                f"desired_clearance={desired_clearance:.5f}, "
                f"selected_clearance={best_clearance:.5f}"
            )

        return desired_delta_base, False, (
            f"blocked: no safe local action, desired_clearance={desired_clearance:.5f}"
        )

    # -------------------------------------------------------------------------
    # Core planning
    # -------------------------------------------------------------------------

    def compute_trajectory(
        self,
        target_base: np.ndarray,
        has_obstacle: bool,
        obstacle_center_base: Optional[np.ndarray] = None,
        obstacle_full_size: Optional[np.ndarray] = None,
        safety_obstacles_base: Optional[list[tuple[np.ndarray, np.ndarray]]] = None,
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
            safety_obstacles_base: Optional list of all PlanningScene obstacles
                as ``(center_base, full_size)`` tuples.  The 15D policy still
                receives only the selected obstacle, but the rollout safety
                filter checks every obstacle in this list.
            source: Arbitrary label for logging (e.g. "manual", "vision_realtime").

        Returns:
            PlanningResult with converged flag, timing, and trajectory lists.
        """
        planning_t0 = time.perf_counter()

        # Resolve and validate inputs
        target_base_arr = np.asarray(target_base, dtype=np.float32)
        if target_base_arr.shape != (3,):
            raise ValueError(f"target_base must be shape (3,), got {target_base_arr.shape}")
        self._validate_workspace_position(target_base_arr, "target_base")

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

        safety_obstacles = self._normalize_safety_obstacles(
            primary_center_base=obstacle_center_base_arr,
            primary_full_size=obstacle_full_size_arr,
            has_primary_obstacle=has_obstacle,
            safety_obstacles_base=safety_obstacles_base,
        )

        # Initialize simulated TCP
        current_tcp_drl = start_tcp_drl.copy()
        trajectory_drl: list[np.ndarray] = [current_tcp_drl.copy()]
        trajectory_base: list[np.ndarray] = [
            config.drl_world_to_base(current_tcp_drl)
        ]

        converged = False
        first_raw_obs: Optional[np.ndarray] = None
        first_model_obs: Optional[np.ndarray] = None
        last_raw_obs: Optional[np.ndarray] = None
        rollout_diagnostics: list[str] = []
        safety_filter_adjustments = 0

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
            if raw_obs.shape != (config.OBS_DIM,):
                raise ValueError(
                    f"raw_obs shape mismatch: expected ({config.OBS_DIM},), got {raw_obs.shape}"
                )
            if raw_obs.dtype != np.float32:
                raise TypeError(f"raw_obs dtype must be float32, got {raw_obs.dtype}")
            if not np.all(np.isfinite(raw_obs)):
                raise ValueError(f"raw_obs contains non-finite values: {raw_obs}")

            # Normalize if VecNormalize is enabled
            obs = config.normalize_if_needed(raw_obs, self._vn_stats)
            if obs.shape != (config.OBS_DIM,):
                raise ValueError(
                    f"model obs shape mismatch: expected ({config.OBS_DIM},), got {obs.shape}"
                )
            if obs.dtype != np.float32:
                raise TypeError(f"model obs dtype must be float32, got {obs.dtype}")
            if not np.all(np.isfinite(obs)):
                raise ValueError(f"model obs contains non-finite values: {obs}")

            if first_raw_obs is None:
                first_raw_obs = raw_obs.copy()
                first_model_obs = obs.copy()
            last_raw_obs = raw_obs.copy()

            # Model inference
            action = predict(self._model, obs)
            action = np.asarray(action, dtype=np.float32).reshape(3)

            # Convert action to displacement
            delta = action * self._action_step

            # Advance simulated TCP
            next_tcp_drl = current_tcp_drl + delta
            next_tcp_base = config.drl_world_to_base(next_tcp_drl)
            safety_adjusted = False
            safety_message = ""
            if safety_obstacles:
                current_tcp_base = config.drl_world_to_base(current_tcp_drl)
                filtered_delta_base, safety_adjusted, safety_message = (
                    self._safety_filter_delta_base(
                        current_base=current_tcp_base,
                        target_base=target_base_arr,
                        desired_delta_base=next_tcp_base - current_tcp_base,
                        safety_obstacles_base=safety_obstacles,
                    )
                )
                if safety_adjusted:
                    safety_filter_adjustments += 1
                    next_tcp_base = current_tcp_base + filtered_delta_base
                    next_tcp_drl = config.base_to_drl_world(next_tcp_base)
                    delta = filtered_delta_base
            clipped_next_tcp_base = self._clip_to_workspace(next_tcp_base)
            hit_workspace_limit = not np.allclose(
                clipped_next_tcp_base, next_tcp_base, rtol=0.0, atol=1e-7
            )
            if hit_workspace_limit:
                next_tcp_base = clipped_next_tcp_base
                next_tcp_drl = config.base_to_drl_world(next_tcp_base)

            # Store
            trajectory_drl.append(next_tcp_drl.copy())
            trajectory_base.append(next_tcp_base.copy())

            # Update
            current_tcp_drl = next_tcp_drl

            # Check convergence
            dist = float(np.linalg.norm(current_tcp_drl - target_drl))
            if step < 5:
                if has_obstacle:
                    nearest_obstacle_dist = float(
                        np.linalg.norm(obstacle_center_drl - current_tcp_drl)
                    )
                else:
                    nearest_obstacle_dist = float("inf")
                rollout_diagnostics.append(
                    "step={step} tcp_base=({x:.4f},{y:.4f},{z:.4f}) "
                    "dist_goal={dist:.4f} action=({ax:.4f},{ay:.4f},{az:.4f}) "
                    "nearest_obs={obs_dist:.4f} obs_slice={obs_slice}".format(
                        step=step,
                        x=float(next_tcp_base[0]),
                        y=float(next_tcp_base[1]),
                        z=float(next_tcp_base[2]),
                        dist=dist,
                        ax=float(action[0]),
                        ay=float(action[1]),
                        az=float(action[2]),
                        obs_dist=nearest_obstacle_dist,
                        obs_slice=np.array2string(raw_obs[9:15], precision=4),
                    )
                )
                if safety_message:
                    rollout_diagnostics[-1] += (
                        f" safety_adjusted={safety_adjusted} {safety_message}"
                    )
            if dist < self._distance_thresh:
                converged = True
                break
            if hit_workspace_limit:
                break

        if first_raw_obs is None or first_model_obs is None or last_raw_obs is None:
            first_raw_obs = build_observation_15d(
                current_tcp_drl=start_tcp_drl,
                target_drl=target_drl,
                obstacle_center_drl=obstacle_center_drl,
                obstacle_half_extent=obstacle_half_extent,
                has_obstacle=has_obstacle,
                workspace_range=self._workspace_range,
            )
            first_model_obs = config.normalize_if_needed(first_raw_obs, self._vn_stats)
            last_raw_obs = first_raw_obs.copy()

        # Capture model convergence state BEFORE appending exact target
        model_final_tcp_drl = trajectory_drl[-1]
        convergence_dist = float(np.linalg.norm(model_final_tcp_drl - target_drl))

        # Append exact target as final waypoint
        if np.linalg.norm(trajectory_drl[-1] - target_drl) > 1e-6:
            append_target = True
            if safety_obstacles and self._obstacle_safety_filter_enabled:
                final_clearance, _ = self._segment_obstacles_min_clearance(
                    trajectory_base[-1],
                    target_base_arr,
                    safety_obstacles,
                )
                append_target = final_clearance >= -1e-6
                if not append_target:
                    rollout_diagnostics.append(
                        "final target append rejected by safety filter "
                        f"clearance={final_clearance:.5f}"
                    )
            if append_target:
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
            first_raw_observation=first_raw_obs,
            first_model_observation=first_model_obs,
            last_raw_observation=last_raw_obs,
            first_obstacle_slice=first_raw_obs[9:15].copy(),
            rollout_diagnostics=rollout_diagnostics,
            safety_filter_adjustments=safety_filter_adjustments,
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
        model_subpath: Relative path under ``share/robot_drl/models/``.
        vec_normalize_subpath: Relative path under ``share/robot_drl/models/``.
        env_config_path: Explicit path to env_config.yaml, or None for auto-detect.
        calibrated_start_tcp_base: Override for the start TCP, or None for default.

    Returns:
        (planner, env_cfg) tuple.
    """
    from ament_index_python.packages import get_package_share_directory

    pkg_share = get_package_share_directory("robot_drl")
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
        tcp = config.DEFAULT_START_TCP_BASE.copy()

    planner = DrlTrajectoryPlannerCore(
        model=model,
        vn_stats=vn_stats,
        env_cfg=env_cfg,
        calibrated_start_tcp_base=tcp,
    )

    return planner, env_cfg
