"""End-to-end helper for DRL mock-hardware obstacle regression tests."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from robot_drl.planning_scene_adapter import (
    SceneObstacle,
    collision_object_to_obstacles,
    validate_cartesian_path_against_obstacles,
)
from robot_drl.state_builder import build_observation_15d


@dataclass
class ObstacleSpec:
    object_id: str
    center: np.ndarray
    size: np.ndarray
    orientation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)


@dataclass
class TestMetrics:
    label: str
    observation: np.ndarray
    waypoint_count: int
    path_length: float
    trajectory: list[np.ndarray]


class DrlMockHwObstacleTest(Node):
    """Small ROS client that drives Tests A/B/C from codex.md."""

    def __init__(self) -> None:
        super().__init__("drl_mock_hw_obstacle_test")
        self.declare_parameter("start_base", [0.375, 0.0, 0.25])
        self.declare_parameter("target_base", [0.45, 0.05, 0.12])
        self.declare_parameter("obstacle_id", "drl_blocking_box")
        self.declare_parameter("obstacle_size", [0.02, 0.02, 0.02])
        self.declare_parameter("workspace_range", 0.5)
        self.declare_parameter("timeout_sec", 30.0)
        self.declare_parameter("execute", True)
        self.declare_parameter("output_file", "/tmp/robot_drl_mock_hw_obstacle_test.npz")
        self.declare_parameter("case_count", 1)
        self.declare_parameter("randomize_start", True)
        self.declare_parameter("randomize_target", True)
        self.declare_parameter("random_seed", 2)
        self.declare_parameter("start_min_base", [0.33, -0.08, 0.20])
        self.declare_parameter("start_max_base", [0.40, 0.08, 0.27])
        self.declare_parameter("target_min_base", [0.43, 0.02, 0.10])
        self.declare_parameter("target_max_base", [0.47, 0.08, 0.13])
        self.declare_parameter("random_target_min_distance_m", 0.13)
        self.declare_parameter("obstacle_count", 3)
        self.declare_parameter("randomize_obstacle_count", False)
        self.declare_parameter("obstacle_count_min", 2)
        self.declare_parameter("obstacle_count_max", 4)
        self.declare_parameter("randomize_obstacle_size", True)
        self.declare_parameter("obstacle_size_min", [0.018, 0.018, 0.018])
        self.declare_parameter("obstacle_size_max", [0.045, 0.045, 0.045])
        self.declare_parameter("randomize_obstacle_positions", True)
        self.declare_parameter("randomize_obstacle_orientation", True)
        self.declare_parameter("obstacle_roll_pitch_max_rad", 0.25)
        self.declare_parameter("obstacle_yaw_min_rad", -math.pi)
        self.declare_parameter("obstacle_yaw_max_rad", math.pi)
        self.declare_parameter("max_random_attempts", 5)
        self.declare_parameter("retry_cooldown_sec", 5.0)
        self.declare_parameter("obstacle_path_fraction_min", 0.35)
        self.declare_parameter("obstacle_path_fraction_max", 0.80)
        self.declare_parameter("obstacle_lateral_offset_max_m", 0.150)
        self.declare_parameter("obstacle_lateral_min_m", 0.090)
        self.declare_parameter("obstacle_primary_lateral_min_m", 0.100)
        self.declare_parameter("obstacle_path_clearance_m", 0.035)
        self.declare_parameter("obstacle_endpoint_clearance_m", 0.080)
        self.declare_parameter("obstacle_min_separation_m", 0.060)
        self.declare_parameter("extra_obstacle_offset_m", 0.11)
        self.declare_parameter("extra_obstacle_z_jitter_m", 0.015)

        self._latest_poses: PoseArray | None = None
        self._active_start_base: np.ndarray | None = None
        self._active_target_base: np.ndarray | None = None
        self._poses_sub = self.create_subscription(
            PoseArray,
            "/drl/forward_trajectory_poses",
            self._on_poses,
            10,
        )
        self._plan_client = self.create_client(Trigger, "/drl/plan")
        self._execute_client = self.create_client(Trigger, "/drl/execute_forward")
        self._clear_client = self.create_client(Trigger, "/drl/clear_trajectory")
        self._status_client = self.create_client(Trigger, "/drl/get_execution_status")
        self._apply_scene_client = self.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
        )
        self._get_scene_client = self.create_client(
            GetPlanningScene,
            "/get_planning_scene",
        )
        self._set_planner_params_client = self.create_client(
            SetParameters,
            "/drl_unified_planner_node/set_parameters",
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def _on_poses(self, msg: PoseArray) -> None:
        self._latest_poses = msg

    def _wait_for_service(self, client, name: str) -> None:
        timeout = float(self.get_parameter("timeout_sec").value)
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"Service {name} not available after {timeout:.1f}s")

    def _call_trigger(self, client, name: str) -> str:
        self._wait_for_service(client, name)
        future = client.call_async(Trigger.Request())
        self._spin_until_future(future, name)
        resp = future.result()
        if resp is None:
            raise RuntimeError(f"{name} returned no response")
        if not resp.success:
            raise RuntimeError(f"{name} failed: {resp.message}")
        return resp.message

    def _wait_for_execution_idle(self) -> str:
        self._wait_for_service(self._status_client, "/drl/get_execution_status")
        timeout = float(self.get_parameter("timeout_sec").value)
        deadline = time.monotonic() + timeout
        last_message = ""
        while rclpy.ok() and time.monotonic() < deadline:
            future = self._status_client.call_async(Trigger.Request())
            self._spin_until_future(future, "get execution status")
            resp = future.result()
            if resp is not None:
                last_message = resp.message
                if resp.success:
                    if not resp.message.startswith("SUCCEEDED"):
                        raise RuntimeError(
                            f"Execution finished without success: {resp.message}"
                        )
                    return resp.message
            time.sleep(0.2)
        raise TimeoutError(
            f"Execution did not finish after {timeout:.1f}s; last={last_message}"
        )

    def _spin_until_future(self, future, label: str) -> None:
        timeout = float(self.get_parameter("timeout_sec").value)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                raise TimeoutError(f"{label} timed out after {timeout:.1f}s")
            rclpy.spin_once(self, timeout_sec=0.05)

    def _spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + max(0.0, seconds)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _clear_object(self, object_id: str) -> None:
        self._wait_for_service(self._apply_scene_client, "/apply_planning_scene")
        obj = CollisionObject()
        obj.header.frame_id = "base_link"
        obj.id = object_id
        obj.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self._apply_scene_client.call_async(req)
        self._spin_until_future(future, "clear obstacle")
        resp = future.result()
        if resp is None or not resp.success:
            self.get_logger().warn(
                f"PlanningScene clear for '{object_id}' returned success=false; "
                "continuing because the object may already be absent."
            )

    def _clear_objects(self, object_ids: list[str]) -> None:
        seen: set[str] = set()
        for object_id in object_ids:
            if object_id in seen:
                continue
            seen.add(object_id)
            self._clear_object(object_id)

    def _add_box(
        self,
        object_id: str,
        center: np.ndarray,
        size: np.ndarray,
        orientation_xyzw: tuple[float, float, float, float],
    ) -> SceneObstacle:
        self._wait_for_service(self._apply_scene_client, "/apply_planning_scene")
        obj = CollisionObject()
        obj.header.frame_id = "base_link"
        obj.id = object_id
        obj.operation = CollisionObject.ADD
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [float(size[0]), float(size[1]), float(size[2])]
        pose = Pose()
        pose.position.x = float(center[0])
        pose.position.y = float(center[1])
        pose.position.z = float(center[2])
        pose.orientation.x = float(orientation_xyzw[0])
        pose.orientation.y = float(orientation_xyzw[1])
        pose.orientation.z = float(orientation_xyzw[2])
        pose.orientation.w = float(orientation_xyzw[3])
        obj.primitives.append(prim)
        obj.primitive_poses.append(pose)
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self._apply_scene_client.call_async(req)
        self._spin_until_future(future, "add obstacle")
        resp = future.result()
        if resp is None or not resp.success:
            raise RuntimeError("Failed to add PlanningScene obstacle")
        confirmed = self._wait_for_known_object(object_id)
        obstacles = collision_object_to_obstacles(
            confirmed,
            tf_buffer=self._tf_buffer,
            target_frame="base_link",
            timeout_sec=float(self.get_parameter("timeout_sec").value),
        )
        if not obstacles:
            raise RuntimeError(f"PlanningScene object '{object_id}' has no usable geometry")
        return obstacles[0]

    def _add_obstacles(self, obstacles: list[ObstacleSpec]) -> list[SceneObstacle]:
        confirmed: list[SceneObstacle] = []
        for obstacle in obstacles:
            confirmed.append(
                self._add_box(
                    obstacle.object_id,
                    obstacle.center,
                    obstacle.size,
                    obstacle.orientation_xyzw,
                )
            )
        self._wait_for_known_objects([obstacle.object_id for obstacle in obstacles])
        return confirmed

    def _wait_for_known_object(self, object_id: str) -> CollisionObject:
        self._wait_for_service(self._get_scene_client, "/get_planning_scene")
        timeout = float(self.get_parameter("timeout_sec").value)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            req = GetPlanningScene.Request()
            req.components.components = (
                PlanningSceneComponents.WORLD_OBJECT_NAMES
                | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            )
            future = self._get_scene_client.call_async(req)
            self._spin_until_future(future, "get planning scene")
            resp = future.result()
            if resp is not None:
                for obj in resp.scene.world.collision_objects:
                    if obj.id == object_id:
                        return obj
            time.sleep(0.1)
        raise TimeoutError(f"PlanningScene did not confirm object '{object_id}'")

    def _wait_for_known_objects(self, object_ids: list[str]) -> None:
        pending = set(object_ids)
        if not pending:
            return
        self._wait_for_service(self._get_scene_client, "/get_planning_scene")
        timeout = float(self.get_parameter("timeout_sec").value)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            req = GetPlanningScene.Request()
            req.components.components = (
                PlanningSceneComponents.WORLD_OBJECT_NAMES
                | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            )
            future = self._get_scene_client.call_async(req)
            self._spin_until_future(future, "get planning scene")
            resp = future.result()
            if resp is not None:
                known = {obj.id for obj in resp.scene.world.collision_objects}
                pending = set(object_ids) - known
                if not pending:
                    return
            time.sleep(0.1)
        raise TimeoutError(
            "PlanningScene did not confirm objects: "
            + ", ".join(sorted(pending))
        )

    def _get_vector_param(self, name: str) -> np.ndarray:
        value = self.get_parameter(name).value
        arr = np.asarray(value, dtype=np.float32)
        if arr.shape != (3,):
            raise ValueError(f"{name} must have 3 elements, got {arr.shape}")
        return arr

    def _sample_vector_in_bounds(
        self,
        min_name: str,
        max_name: str,
        rng: np.random.Generator,
    ) -> np.ndarray:
        lower = self._get_vector_param(min_name)
        upper = self._get_vector_param(max_name)
        if np.any(lower > upper):
            raise ValueError(f"{min_name} must be <= {max_name}")
        return rng.uniform(lower, upper).astype(np.float32)

    def _set_planner_scene_defaults(
        self,
        start: np.ndarray,
        target: np.ndarray,
    ) -> None:
        self._wait_for_service(
            self._set_planner_params_client,
            "/drl_unified_planner_node/set_parameters",
        )
        req = SetParameters.Request()
        for name, values in (
            ("manual_default_target", target),
            ("calibrated_start_tcp_base", start),
            ("preposition_tcp_base", start),
        ):
            req.parameters.append(
                Parameter(
                    name=name,
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                        double_array_value=[float(v) for v in values],
                    ),
                )
            )
        future = self._set_planner_params_client.call_async(req)
        self._spin_until_future(future, "set planner randomized scene params")
        resp = future.result()
        if resp is None or not resp.results:
            raise RuntimeError("Planner parameter update returned no result")
        failures = [
            f"{param.name}: {result.reason}"
            for param, result in zip(req.parameters, resp.results)
            if not result.successful
        ]
        if failures:
            raise RuntimeError(
                "Failed to set planner randomized scene params: "
                + "; ".join(failures)
            )

    def _choose_start(self, rng: np.random.Generator) -> np.ndarray:
        fallback = self._get_vector_param("start_base")
        if not bool(self.get_parameter("randomize_start").value):
            return fallback
        return self._sample_vector_in_bounds("start_min_base", "start_max_base", rng)

    def _choose_target(
        self,
        start: np.ndarray,
        rng: np.random.Generator,
    ) -> np.ndarray:
        fallback = self._get_vector_param("target_base")
        if not bool(self.get_parameter("randomize_target").value):
            return fallback

        min_distance = max(
            0.0,
            float(self.get_parameter("random_target_min_distance_m").value),
        )
        for _ in range(64):
            target = self._sample_vector_in_bounds(
                "target_min_base",
                "target_max_base",
                rng,
            )
            if float(np.linalg.norm(target - start)) >= min_distance:
                return target
        self.get_logger().warn(
            "Could not sample a random target satisfying min distance; "
            "falling back to target_base parameter."
        )
        return fallback

    def _object_ids(self, base_id: str, count: int) -> list[str]:
        if count <= 1:
            return [base_id]
        return [f"{base_id}_{idx:02d}" for idx in range(count)]

    def _cleanup_object_ids(self, base_id: str, count: int) -> list[str]:
        max_count = max(count + 3, 6)
        return (
            [base_id, f"{base_id}_goal"]
            + [f"{base_id}_{idx:02d}" for idx in range(max_count)]
        )

    def _quat_from_rpy(
        self,
        roll: float,
        pitch: float,
        yaw: float,
    ) -> tuple[float, float, float, float]:
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 1e-12:
            return (0.0, 0.0, 0.0, 1.0)
        return (qx / norm, qy / norm, qz / norm, qw / norm)

    def _sample_obstacle_orientation(
        self,
        rng: np.random.Generator,
    ) -> tuple[float, float, float, float]:
        if not bool(self.get_parameter("randomize_obstacle_orientation").value):
            return (0.0, 0.0, 0.0, 1.0)
        roll_pitch_max = max(
            0.0,
            float(self.get_parameter("obstacle_roll_pitch_max_rad").value),
        )
        yaw_min = float(self.get_parameter("obstacle_yaw_min_rad").value)
        yaw_max = float(self.get_parameter("obstacle_yaw_max_rad").value)
        if yaw_min > yaw_max:
            yaw_min, yaw_max = yaw_max, yaw_min
        roll = float(rng.uniform(-roll_pitch_max, roll_pitch_max))
        pitch = float(rng.uniform(-roll_pitch_max, roll_pitch_max))
        yaw = float(rng.uniform(yaw_min, yaw_max))
        return self._quat_from_rpy(roll, pitch, yaw)

    def _build_obstacle_specs(
        self,
        start: np.ndarray,
        target: np.ndarray,
        base_id: str,
        base_size: np.ndarray,
        count: int,
        rng: np.random.Generator,
    ) -> list[ObstacleSpec]:
        count = max(1, count)
        object_ids = self._object_ids(base_id, count)
        randomize_positions = bool(
            self.get_parameter("randomize_obstacle_positions").value
        )
        randomize_sizes = bool(self.get_parameter("randomize_obstacle_size").value)
        size_min = self._get_vector_param("obstacle_size_min")
        size_max = self._get_vector_param("obstacle_size_max")
        if np.any(size_min <= 0.0) or np.any(size_max <= 0.0):
            raise ValueError("obstacle_size_min/max must be positive")
        if np.any(size_min > size_max):
            raise ValueError("obstacle_size_min must be <= obstacle_size_max")
        workspace_min = np.array([0.25, -0.15, 0.02], dtype=np.float32)
        workspace_max = np.array([0.50, 0.15, 0.30], dtype=np.float32)
        path = target - start
        side = np.array([-path[1], path[0], 0.0], dtype=np.float32)
        side_norm = float(np.linalg.norm(side))
        if side_norm <= 1e-8:
            side = np.array([0.0, 1.0, 0.0], dtype=np.float32)
        else:
            side /= side_norm

        frac_min = float(self.get_parameter("obstacle_path_fraction_min").value)
        frac_max = float(self.get_parameter("obstacle_path_fraction_max").value)
        frac_min = float(np.clip(frac_min, 0.0, 1.0))
        frac_max = float(np.clip(frac_max, 0.0, 1.0))
        if frac_min > frac_max:
            frac_min, frac_max = frac_max, frac_min
        lateral_max = max(
            0.0,
            float(self.get_parameter("obstacle_lateral_offset_max_m").value),
        )
        lateral_min = max(
            0.0,
            float(self.get_parameter("obstacle_lateral_min_m").value),
        )
        primary_lateral_min = max(
            0.0,
            float(self.get_parameter("obstacle_primary_lateral_min_m").value),
        )
        z_jitter = max(0.0, float(self.get_parameter("extra_obstacle_z_jitter_m").value))
        endpoint_clearance = max(
            0.0,
            float(self.get_parameter("obstacle_endpoint_clearance_m").value),
        )
        min_separation = max(
            0.0,
            float(self.get_parameter("obstacle_min_separation_m").value),
        )
        path_clearance = max(
            0.0,
            float(self.get_parameter("obstacle_path_clearance_m").value),
        )
        path_len_sq = float(np.dot(path, path))

        def sample_size() -> np.ndarray:
            if randomize_sizes:
                return rng.uniform(size_min, size_max).astype(np.float32)
            return base_size.copy()

        def clamp_center(center: np.ndarray, size: np.ndarray) -> np.ndarray:
            padding = size / 2.0 + 0.015
            return np.minimum(
                np.maximum(center, workspace_min + padding),
                workspace_max - padding,
            ).astype(np.float32)

        def is_endpoint_clear(center: np.ndarray, size: np.ndarray) -> bool:
            inflated = endpoint_clearance + float(np.linalg.norm(size / 2.0))
            return (
                float(np.linalg.norm(center - start)) >= inflated
                and float(np.linalg.norm(center - target)) >= inflated
            )

        def is_path_clear(center: np.ndarray, size: np.ndarray) -> bool:
            if path_len_sq <= 1e-12:
                closest = start
            else:
                t = float(np.clip(np.dot(center - start, path) / path_len_sq, 0.0, 1.0))
                closest = start + t * path
            half_diag = float(np.linalg.norm(size / 2.0))
            return float(np.linalg.norm(center - closest)) >= half_diag + path_clearance

        def sample_center(size: np.ndarray, idx: int) -> np.ndarray:
            if not randomize_positions:
                if idx == 0:
                    return ((start + target) / 2.0).astype(np.float32)
                frac = fractions[(idx - 1) % len(fractions)]
                sign = 1.0 if idx % 2 else -1.0
                return clamp_center(start + frac * path + sign * offset * side, size)

            last_center = None
            for _ in range(64):
                frac = float(rng.uniform(frac_min, frac_max))
                if idx == 0:
                    bounded_lateral = max(primary_lateral_min, min(lateral_max, 0.080))
                    sign = -1.0 if rng.random() < 0.5 else 1.0
                    lateral = sign * float(rng.uniform(primary_lateral_min, bounded_lateral))
                else:
                    bounded_lateral = max(lateral_min, lateral_max)
                    sign = -1.0 if rng.random() < 0.5 else 1.0
                    lateral = sign * float(rng.uniform(lateral_min, bounded_lateral))
                center = start + frac * path + lateral * side
                if z_jitter > 0.0:
                    center[2] += float(rng.uniform(-z_jitter, z_jitter))
                center = clamp_center(center, size)
                last_center = center
                if is_endpoint_clear(center, size) and is_path_clear(center, size):
                    return center
            assert last_center is not None
            self.get_logger().warn(
                "Could not sample an obstacle satisfying endpoint/path clearance; "
                f"using last candidate idx={idx} center={last_center.tolist()}"
            )
            return last_center

        primary_size = sample_size()
        primary_center = sample_center(primary_size, 0)
        specs = [
            ObstacleSpec(
                object_id=object_ids[0],
                center=primary_center,
                size=primary_size,
                orientation_xyzw=self._sample_obstacle_orientation(rng),
            )
        ]
        if count == 1:
            return specs

        offset = max(0.0, float(self.get_parameter("extra_obstacle_offset_m").value))
        fractions = [0.32, 0.68, 0.42, 0.58, 0.25, 0.75]

        for idx in range(1, count):
            size = sample_size()
            center = sample_center(size, idx)
            for _ in range(64):
                if all(
                    float(np.linalg.norm(center - existing.center)) >= min_separation
                    for existing in specs
                ):
                    break
                center = sample_center(size, idx)
            specs.append(
                ObstacleSpec(
                    object_id=object_ids[idx],
                    center=center,
                    size=size,
                    orientation_xyzw=self._sample_obstacle_orientation(rng),
                )
            )
        return specs


    def _plan_and_capture(
        self,
        label: str,
        obstacle_center=None,
        obstacle_size=None,
        wait_timeout_sec: float | None = None,
    ) -> TestMetrics:
        self._latest_poses = None
        self._call_trigger(self._clear_client, "/drl/clear_trajectory")
        self._call_trigger(self._plan_client, "/drl/plan")
        timeout = (
            float(wait_timeout_sec)
            if wait_timeout_sec is not None
            else float(self.get_parameter("timeout_sec").value)
        )
        deadline = time.monotonic() + timeout
        last_count = 0
        stable_since = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            count = len(self._latest_poses.poses) if self._latest_poses is not None else 0
            if count > 0 and count != last_count:
                last_count = count
                stable_since = time.monotonic()
            if count > 0 and stable_since is not None and time.monotonic() - stable_since > 0.5:
                break
        if self._latest_poses is None or not self._latest_poses.poses:
            raise TimeoutError(f"No trajectory poses received for {label}")

        trajectory = [
            np.array([p.position.x, p.position.y, p.position.z], dtype=np.float32)
            for p in self._latest_poses.poses
        ]
        path_length = sum(
            float(np.linalg.norm(trajectory[i + 1] - trajectory[i]))
            for i in range(len(trajectory) - 1)
        )
        start = (
            self._active_start_base.copy()
            if self._active_start_base is not None
            else self._get_vector_param("start_base")
        )
        target = (
            self._active_target_base.copy()
            if self._active_target_base is not None
            else np.asarray(self.get_parameter("target_base").value, dtype=np.float32)
        )
        workspace_range = float(self.get_parameter("workspace_range").value)
        has_obstacle = obstacle_center is not None and obstacle_size is not None
        observation = build_observation_15d(
            current_tcp_drl=start,
            target_drl=target,
            obstacle_center_drl=obstacle_center if has_obstacle else None,
            obstacle_half_extent=(obstacle_size / 2.0) if has_obstacle else None,
            has_obstacle=has_obstacle,
            workspace_range=workspace_range,
        )
        return TestMetrics(label, observation, len(trajectory), path_length, trajectory)

    def _case_output_path(self, base_output: Path, case_idx: int, case_count: int) -> Path:
        if case_count <= 1:
            return base_output
        return base_output.with_name(
            f"{base_output.stem}_case{case_idx:03d}{base_output.suffix or '.npz'}"
        )

    def run(self) -> None:
        base_seed = int(self.get_parameter("random_seed").value)
        case_count = max(1, int(self.get_parameter("case_count").value))
        base_output = Path(str(self.get_parameter("output_file").value))
        completed: list[Path] = []
        for case_idx in range(case_count):
            if base_seed < 0:
                rng = np.random.default_rng(None)
                seed_label = "entropy"
            else:
                case_seed = base_seed + case_idx
                rng = np.random.default_rng(case_seed)
                seed_label = str(case_seed)
            out = self._case_output_path(base_output, case_idx, case_count)
            self.get_logger().info(
                f"Random mock-hw case {case_idx + 1}/{case_count} | "
                f"seed={seed_label} | output={out}"
            )
            self._run_single_case(rng, seed_label, out)
            completed.append(out)
        self.get_logger().info(
            f"Completed {len(completed)}/{case_count} random mock-hw case(s): "
            + ", ".join(str(path) for path in completed)
        )

    def _run_single_case(
        self,
        rng: np.random.Generator,
        seed_label: str,
        out: Path,
    ) -> None:
        object_id = str(self.get_parameter("obstacle_id").value)
        obstacle_size = self._get_vector_param("obstacle_size")
        execute = bool(self.get_parameter("execute").value)
        start = self._choose_start(rng)
        target = self._choose_target(start, rng)
        self._active_start_base = start.copy()
        self._active_target_base = target.copy()
        self._set_planner_scene_defaults(start, target)
        min_count = max(1, int(self.get_parameter("obstacle_count_min").value))
        max_count = max(min_count, int(self.get_parameter("obstacle_count_max").value))
        fixed_count = max(1, int(self.get_parameter("obstacle_count").value))
        cleanup_count = max(max_count, fixed_count)
        max_attempts = max(1, int(self.get_parameter("max_random_attempts").value))
        retry_cooldown_sec = max(
            0.0,
            float(self.get_parameter("retry_cooldown_sec").value),
        )

        self.get_logger().info(
            "Test setup | "
            f"start={np.array2string(start, precision=4)} | "
            f"target={np.array2string(target, precision=4)} | "
            f"seed={seed_label} | "
            f"max_random_attempts={max_attempts}"
        )

        self._clear_objects(self._cleanup_object_ids(object_id, cleanup_count))
        metrics_a = self._plan_and_capture("A_no_obstacle")

        metrics_b = None
        obstacle_specs: list[ObstacleSpec] = []
        obstacles: list[SceneObstacle] = []
        last_error = ""
        for attempt in range(1, max_attempts + 1):
            self._clear_objects(self._cleanup_object_ids(object_id, cleanup_count))
            if bool(self.get_parameter("randomize_obstacle_count").value):
                obstacle_count = int(rng.integers(min_count, max_count + 1))
            else:
                obstacle_count = fixed_count
            obstacle_specs = self._build_obstacle_specs(
                start=start,
                target=target,
                base_id=object_id,
                base_size=obstacle_size,
                count=obstacle_count,
                rng=rng,
            )
            self.get_logger().info(
                "Test B random scene | "
                f"attempt={attempt}/{max_attempts} | "
                f"obstacle_count={len(obstacle_specs)} | "
                f"obstacle_sizes={[np.array2string(o.size, precision=3) for o in obstacle_specs]} | "
                f"obstacle_quat={[tuple(round(v, 4) for v in o.orientation_xyzw) for o in obstacle_specs]}"
            )
            try:
                obstacles = self._add_obstacles(obstacle_specs)
                candidate_b = self._plan_and_capture(
                    "B_multi_obstacle",
                    obstacle_center=obstacles[0].center_base,
                    obstacle_size=obstacles[0].full_size,
                )
                validation_b = validate_cartesian_path_against_obstacles(
                    candidate_b.trajectory,
                    obstacles,
                    max_step_m=0.01,
                )
                if not validation_b.valid:
                    raise RuntimeError(
                        f"Test B trajectory collision: {validation_b.message}"
                    )
                self.get_logger().info(
                    "Test B confirmed PlanningScene obstacles | "
                    f"aabb_sizes={[np.array2string(o.full_size, precision=3) for o in obstacles]} | "
                    f"min_clearance={validation_b.min_clearance:.5f} m"
                )
                if np.allclose(metrics_a.observation[9:15], candidate_b.observation[9:15]):
                    raise RuntimeError("Test B obstacle observation slice did not change")
                if (
                    candidate_b.waypoint_count == metrics_a.waypoint_count
                    and math.isclose(
                        candidate_b.path_length,
                        metrics_a.path_length,
                        abs_tol=1e-4,
                    )
                ):
                    raise RuntimeError("Test B path did not differ from Test A")
                metrics_b = candidate_b
                break
            except Exception as exc:
                last_error = str(exc)
                self.get_logger().warn(
                    f"Test B random scene attempt {attempt}/{max_attempts} failed: {exc}"
                )
                self._clear_objects([obstacle.object_id for obstacle in obstacle_specs])
                if attempt < max_attempts and retry_cooldown_sec > 0.0:
                    self.get_logger().info(
                        f"Waiting {retry_cooldown_sec:.1f}s before retrying Test B"
                    )
                    self._spin_for(retry_cooldown_sec)

        if metrics_b is None:
            raise RuntimeError(
                "No random obstacle scene passed Test B after "
                f"{max_attempts} attempt(s); last_error={last_error}"
            )
        if execute:
            self._call_trigger(self._execute_client, "/drl/execute_forward")
            self._wait_for_execution_idle()

        self._clear_objects([obstacle.object_id for obstacle in obstacle_specs])
        if execute:
            self.get_logger().info("Prepositioning back to start before Test C")
            self._plan_and_capture(
                "C_preposition_no_obstacle",
                wait_timeout_sec=float(self.get_parameter("timeout_sec").value),
            )

        goal_object_id = f"{object_id}_goal"
        self._add_obstacles(
            [
                ObstacleSpec(
                    object_id=goal_object_id,
                    center=target.copy(),
                    size=obstacle_size.copy(),
                )
            ]
        )
        try:
            self._plan_and_capture(
                "C_goal_inside_obstacle",
                obstacle_center=target,
                obstacle_size=obstacle_size,
                wait_timeout_sec=5.0,
            )
        except Exception as exc:
            self.get_logger().info(f"Test C correctly failed: {exc}")
        else:
            raise RuntimeError("Test C unexpectedly produced a trajectory")

        np.savez(
            out,
            seed=seed_label,
            target=target,
            start=start,
            obstacle_centers=np.asarray(
                [obstacle.center for obstacle in obstacle_specs],
                dtype=np.float32,
            ),
            obstacle_sizes=np.asarray(
                [obstacle.size for obstacle in obstacle_specs],
                dtype=np.float32,
            ),
            obstacle_quat_xyzw=np.asarray(
                [obstacle.orientation_xyzw for obstacle in obstacle_specs],
                dtype=np.float32,
            ),
            obstacle_aabb_centers=np.asarray(
                [obstacle.center_base for obstacle in obstacles],
                dtype=np.float32,
            ),
            obstacle_aabb_sizes=np.asarray(
                [obstacle.full_size for obstacle in obstacles],
                dtype=np.float32,
            ),
            obstacle_aabb_quat_xyzw=np.asarray(
                [obstacle.pose_quat_xyzw for obstacle in obstacles],
                dtype=np.float32,
            ),
            obstacle_ids=np.asarray(
                [obstacle.object_id for obstacle in obstacle_specs],
            ),
            obs_a=metrics_a.observation,
            obs_b=metrics_b.observation,
            path_a=np.asarray(metrics_a.trajectory, dtype=np.float32),
            path_b=np.asarray(metrics_b.trajectory, dtype=np.float32),
            length_a=metrics_a.path_length,
            length_b=metrics_b.path_length,
        )
        self._clear_object(goal_object_id)
        self.get_logger().info(
            "Tests A/B/C passed | "
            f"start={np.array2string(start, precision=4)} | "
            f"target={np.array2string(target, precision=4)} | "
            f"obstacles={len(obstacle_specs)} | "
            f"A: wp={metrics_a.waypoint_count}, len={metrics_a.path_length:.4f} | "
            f"B: wp={metrics_b.waypoint_count}, len={metrics_b.path_length:.4f} | "
            f"obs_a[9:15]={np.array2string(metrics_a.observation[9:15], precision=4)} | "
            f"obs_b[9:15]={np.array2string(metrics_b.observation[9:15], precision=4)} | "
            f"saved={out}"
        )


def main() -> None:
    rclpy.init()
    node = DrlMockHwObstacleTest()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
