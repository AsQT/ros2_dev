"""CSV experiment logger for DRL planner runs.

This module is intentionally independent from ROS node logic.  Callers pass
plain values in, and any CSV write failure is contained inside the logger.
"""

from __future__ import annotations

import csv
import math
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional

import numpy as np


CSV_HEADER = [
    "run_id",
    "plan_id",
    "row_type",
    "timestamp_iso",
    "t_rel_sec",
    "event",
    "model_path",
    "planning_time_ms",
    "converged",
    "final_distance_m",
    "start_x",
    "start_y",
    "start_z",
    "target_x",
    "target_y",
    "target_z",
    "reach_x",
    "reach_y",
    "reach_z",
    "pick_x",
    "pick_y",
    "pick_z",
    "obstacle_enabled",
    "obstacle_x",
    "obstacle_y",
    "obstacle_z",
    "obstacle_size_x",
    "obstacle_size_y",
    "obstacle_size_z",
    "planned_direction",
    "planned_index",
    "planned_x",
    "planned_y",
    "planned_z",
    "planned_segment_length_m",
    "planned_cumulative_length_m",
    "planned_dist_to_start_m",
    "planned_dist_to_target_m",
    "planned_dist_to_obstacle_center_m",
    "forward_waypoints",
    "backward_waypoints",
    "pick_path_length_m",
    "place_path_length_m",
    "start_to_target_distance_m",
    "actual_index",
    "actual_x",
    "actual_y",
    "actual_z",
    "actual_dist_to_start_m",
    "actual_dist_to_target_m",
    "actual_dist_to_obstacle_center_m",
    "nearest_planned_index",
    "nearest_planned_x",
    "nearest_planned_y",
    "nearest_planned_z",
    "tracking_error_x",
    "tracking_error_y",
    "tracking_error_z",
    "tracking_error_norm_m",
    "task_executor_status",
    "robot_in_motion",
    "service_success",
    "service_message",
    "reached",
    "time_to_reach_sec",
    "actual_path_length_m",
    "mean_tracking_error_m",
    "max_tracking_error_m",
    "rmse_tracking_error_m",
    "final_error_to_reach_m",
    "actual_samples_count",
    "execute_direction",
    "execute_mode",
    "sample_rate_hz",
    "reach_threshold_m",
    "base_frame",
    "tcp_frame",
    "note",
]


class ExperimentLogger:
    """Write DRL experiment rows to one timestamped CSV file."""

    def __init__(
        self,
        log_dir: str,
        sample_rate_hz: float,
        reach_threshold_m: float,
        base_frame: str,
        tcp_frame: str,
        warn: Optional[Callable[[str], None]] = None,
    ) -> None:
        self.sample_rate_hz = float(sample_rate_hz)
        self.reach_threshold_m = float(reach_threshold_m)
        self.base_frame = str(base_frame)
        self.tcp_frame = str(tcp_frame)
        self._warn = warn or (lambda message: None)
        self._lock = threading.RLock()
        self._closed = False
        self._start_monotonic = time.monotonic()

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_id, run_dir, csv_path = self._make_unique_paths(Path(log_dir), stamp)
        run_dir.mkdir(parents=True, exist_ok=False)

        self.csv_path = csv_path
        self._file = csv_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=CSV_HEADER)
        self._writer.writeheader()
        self._file.flush()

        self._actual_positions: list[np.ndarray] = []
        self._tracking_errors: list[float] = []
        self._actual_sample_count = 0

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            try:
                self._file.flush()
                self._file.close()
            except Exception as exc:
                self._warn(f"experiment logger close failed: {exc}")
            self._closed = True

    def log_plan_summary(
        self,
        plan_id: int,
        result,
        model_path: str,
        start_base: np.ndarray,
    ) -> None:
        start = self._vec3(start_base)
        target = self._vec3(result.target_base)
        forward = [self._vec3(p) for p in result.trajectory_forward_base]
        backward = [self._vec3(p) for p in result.trajectory_backward_base]
        forward_len = self._path_length(forward)
        backward_len = self._path_length(backward)

        row = {
            "plan_id": plan_id,
            "row_type": "plan_summary",
            "event": "planned",
            "model_path": model_path,
            "planning_time_ms": float(result.planning_time_sec) * 1000.0,
            "converged": bool(result.converged),
            "final_distance_m": float(result.convergence_dist),
            "start_x": start[0],
            "start_y": start[1],
            "start_z": start[2],
            "target_x": target[0],
            "target_y": target[1],
            "target_z": target[2],
            "reach_x": target[0],
            "reach_y": target[1],
            "reach_z": target[2],
            "obstacle_enabled": bool(result.has_obstacle),
            "forward_waypoints": len(forward),
            "backward_waypoints": len(backward),
            "pick_path_length_m": forward_len,
            "place_path_length_m": backward_len,
            "start_to_target_distance_m": self._distance(start, target),
            "sample_rate_hz": self.sample_rate_hz,
            "reach_threshold_m": self.reach_threshold_m,
            "base_frame": self.base_frame,
            "tcp_frame": self.tcp_frame,
            "note": (
                "reach defaults to target; pick blank; "
                "pick/place path lengths follow forward/backward paths"
            ),
        }
        row.update(self._obstacle_fields(result))
        self._safe_write(row)

    def log_planned_waypoints(
        self,
        plan_id: int,
        planned_direction: str,
        trajectory_base: list[np.ndarray],
        start_base: np.ndarray,
        target_base: np.ndarray,
        has_obstacle: bool,
        obstacle_center_base: np.ndarray,
    ) -> None:
        start = self._vec3(start_base)
        target = self._vec3(target_base)
        obstacle = self._vec3(obstacle_center_base) if has_obstacle else None
        cumulative = 0.0
        prev: Optional[np.ndarray] = None

        for index, point_base in enumerate(trajectory_base):
            point = self._vec3(point_base)
            segment = 0.0 if prev is None else self._distance(point, prev)
            cumulative += segment
            row = {
                "plan_id": plan_id,
                "row_type": "planned_waypoint",
                "event": "planned_waypoint",
                "planned_direction": planned_direction,
                "planned_index": index,
                "planned_x": point[0],
                "planned_y": point[1],
                "planned_z": point[2],
                "planned_segment_length_m": segment,
                "planned_cumulative_length_m": cumulative,
                "planned_dist_to_start_m": self._distance(point, start),
                "planned_dist_to_target_m": self._distance(point, target),
                "planned_dist_to_obstacle_center_m": (
                    self._distance(point, obstacle) if obstacle is not None else ""
                ),
                "sample_rate_hz": self.sample_rate_hz,
                "reach_threshold_m": self.reach_threshold_m,
                "base_frame": self.base_frame,
                "tcp_frame": self.tcp_frame,
            }
            self._safe_write(row)
            prev = point

    def log_execution_start(
        self,
        plan_id: int,
        status: str,
        execute_direction: str,
        execute_mode: str,
    ) -> None:
        with self._lock:
            self._actual_positions = []
            self._tracking_errors = []
            self._actual_sample_count = 0

        self._safe_write(
            {
                "plan_id": plan_id,
                "row_type": "execution_start",
                "event": "execute_started",
                "task_executor_status": status,
                "robot_in_motion": True,
                "execute_direction": execute_direction,
                "execute_mode": execute_mode,
                "sample_rate_hz": self.sample_rate_hz,
                "reach_threshold_m": self.reach_threshold_m,
                "base_frame": self.base_frame,
                "tcp_frame": self.tcp_frame,
            }
        )

    def log_actual_sample(
        self,
        plan_id: int,
        actual_base: np.ndarray,
        planned_path_base: list[np.ndarray],
        start_base: np.ndarray,
        target_base: np.ndarray,
        has_obstacle: bool,
        obstacle_center_base: np.ndarray,
        status: str,
        execute_direction: str,
        execute_mode: str,
    ) -> None:
        actual = self._vec3(actual_base)
        start = self._vec3(start_base)
        target = self._vec3(target_base)
        obstacle = self._vec3(obstacle_center_base) if has_obstacle else None

        nearest_index = ""
        nearest = None
        tracking = None
        tracking_norm = ""
        if planned_path_base:
            planned_points = [self._vec3(p) for p in planned_path_base]
            distances = [self._distance(actual, point) for point in planned_points]
            nearest_index_int = int(np.argmin(distances))
            nearest_index = nearest_index_int
            nearest = planned_points[nearest_index_int]
            tracking = actual - nearest
            tracking_norm = float(np.linalg.norm(tracking))

        with self._lock:
            actual_index = self._actual_sample_count
            self._actual_sample_count += 1
            self._actual_positions.append(actual.copy())
            if tracking_norm != "":
                self._tracking_errors.append(float(tracking_norm))

        row = {
            "plan_id": plan_id,
            "row_type": "actual_sample",
            "event": "actual_sampled",
            "actual_index": actual_index,
            "actual_x": actual[0],
            "actual_y": actual[1],
            "actual_z": actual[2],
            "actual_dist_to_start_m": self._distance(actual, start),
            "actual_dist_to_target_m": self._distance(actual, target),
            "actual_dist_to_obstacle_center_m": (
                self._distance(actual, obstacle) if obstacle is not None else ""
            ),
            "nearest_planned_index": nearest_index,
            "task_executor_status": status,
            "robot_in_motion": True,
            "execute_direction": execute_direction,
            "execute_mode": execute_mode,
            "sample_rate_hz": self.sample_rate_hz,
            "reach_threshold_m": self.reach_threshold_m,
            "base_frame": self.base_frame,
            "tcp_frame": self.tcp_frame,
        }
        if nearest is not None and tracking is not None:
            row.update(
                {
                    "nearest_planned_x": nearest[0],
                    "nearest_planned_y": nearest[1],
                    "nearest_planned_z": nearest[2],
                    "tracking_error_x": tracking[0],
                    "tracking_error_y": tracking[1],
                    "tracking_error_z": tracking[2],
                    "tracking_error_norm_m": tracking_norm,
                }
            )
        self._safe_write(row)

    def log_execution_summary(
        self,
        plan_id: int,
        status: str,
        service_success: bool,
        service_message: str,
        time_to_reach_sec: float,
        reach_base: np.ndarray,
        execute_direction: str,
        execute_mode: str,
    ) -> None:
        reach = self._vec3(reach_base)
        with self._lock:
            actual_positions = [p.copy() for p in self._actual_positions]
            tracking_errors = list(self._tracking_errors)
            sample_count = self._actual_sample_count

        actual_path_length = self._path_length(actual_positions)
        final_error = ""
        reached = ""
        note = ""
        if actual_positions:
            final_error = self._distance(actual_positions[-1], reach)
            reached = bool(service_success and final_error <= self.reach_threshold_m)
        else:
            note = "no actual TF samples; reached left blank"

        row = {
            "plan_id": plan_id,
            "row_type": "execution_summary",
            "event": "execute_finished",
            "task_executor_status": status,
            "robot_in_motion": False,
            "service_success": bool(service_success),
            "service_message": service_message,
            "reached": reached,
            "time_to_reach_sec": float(time_to_reach_sec),
            "actual_path_length_m": actual_path_length if actual_positions else "",
            "mean_tracking_error_m": (
                float(np.mean(tracking_errors)) if tracking_errors else ""
            ),
            "max_tracking_error_m": (
                float(np.max(tracking_errors)) if tracking_errors else ""
            ),
            "rmse_tracking_error_m": (
                math.sqrt(float(np.mean(np.square(tracking_errors))))
                if tracking_errors
                else ""
            ),
            "final_error_to_reach_m": final_error,
            "actual_samples_count": sample_count,
            "execute_direction": execute_direction,
            "execute_mode": execute_mode,
            "sample_rate_hz": self.sample_rate_hz,
            "reach_threshold_m": self.reach_threshold_m,
            "base_frame": self.base_frame,
            "tcp_frame": self.tcp_frame,
            "note": note,
        }
        self._safe_write(row)

    @staticmethod
    def _make_unique_paths(base_dir: Path, stamp: str) -> tuple[str, Path, Path]:
        for suffix in [""] + [f"_{i:03d}" for i in range(1, 1000)]:
            run_id = f"run_{stamp}{suffix}"
            run_dir = base_dir / run_id
            csv_path = run_dir / f"results_{stamp}{suffix}.csv"
            if not run_dir.exists() and not csv_path.exists():
                return run_id, run_dir, csv_path
        raise RuntimeError("could not allocate unique experiment log path")

    @staticmethod
    def _vec3(value) -> np.ndarray:
        arr = np.asarray(value, dtype=np.float64).reshape(3)
        return arr

    @staticmethod
    def _distance(a: np.ndarray, b: np.ndarray) -> float:
        return float(np.linalg.norm(a - b))

    @classmethod
    def _path_length(cls, points: list[np.ndarray]) -> float:
        if len(points) < 2:
            return 0.0
        return float(
            sum(cls._distance(points[i], points[i - 1]) for i in range(1, len(points)))
        )

    @staticmethod
    def _obstacle_fields(result) -> dict[str, object]:
        if not bool(result.has_obstacle):
            return {
                "obstacle_x": "",
                "obstacle_y": "",
                "obstacle_z": "",
                "obstacle_size_x": "",
                "obstacle_size_y": "",
                "obstacle_size_z": "",
            }
        center = np.asarray(result.obstacle_center_base, dtype=np.float64).reshape(3)
        size = np.asarray(result.obstacle_full_size, dtype=np.float64).reshape(3)
        return {
            "obstacle_x": center[0],
            "obstacle_y": center[1],
            "obstacle_z": center[2],
            "obstacle_size_x": size[0],
            "obstacle_size_y": size[1],
            "obstacle_size_z": size[2],
        }

    def _base_row(self) -> dict[str, object]:
        return {
            "run_id": self.run_id,
            "timestamp_iso": datetime.now().isoformat(timespec="milliseconds"),
            "t_rel_sec": time.monotonic() - self._start_monotonic,
        }

    def _safe_write(self, row: dict[str, object]) -> None:
        with self._lock:
            if self._closed:
                return
            try:
                full_row = {key: "" for key in CSV_HEADER}
                full_row.update(self._base_row())
                full_row.update(row)
                self._writer.writerow(full_row)
                self._file.flush()
            except Exception as exc:
                self._warn(f"experiment CSV write failed: {exc}")
