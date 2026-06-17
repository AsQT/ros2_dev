"""Gazebo wood-block DRL obstacle regression runner."""

from __future__ import annotations

import math
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from robot_task_executor_msgs.srv import MoveCartesianPoseSequence

from robot_drl.planning_scene_adapter import (
    SceneObstacle,
    collision_object_to_obstacles,
    validate_cartesian_path_against_obstacles,
)


WOOD_BLOCK_SIZE = np.array([0.10, 0.20, 0.30], dtype=np.float32)


@dataclass
class GazeboCase:
    name: str
    seed_label: str
    start: np.ndarray
    target: np.ndarray
    center_base: np.ndarray
    center_world: np.ndarray
    rpy: tuple[float, float, float]
    quat_xyzw: tuple[float, float, float, float]
    mode: str


class GazeboObstacleTest(Node):
    """Spawn Gazebo wood blocks, sync them to MoveIt, then drive DRL planning."""

    def __init__(self) -> None:
        super().__init__("drl_gazebo_obstacle_test")
        self.declare_parameter("world_name", "default")
        self.declare_parameter("case_count", 1)
        self.declare_parameter("random_seed", 2)
        self.declare_parameter("execute", True)
        self.declare_parameter("obstacle_mode", "block_direct_path")
        self.declare_parameter("max_random_attempts", 5)
        self.declare_parameter("retry_cooldown_sec", 5.0)
        self.declare_parameter("timeout_sec", 60.0)
        self.declare_parameter("output_file", "/tmp/robot_drl_gazebo_obstacle_test.npz")
        self.declare_parameter("start_base", [0.375, 0.0, 0.25])
        self.declare_parameter("target_base", [0.45, 0.05, 0.12])
        self.declare_parameter("randomize_start", True)
        self.declare_parameter("randomize_target", True)
        self.declare_parameter("start_min_base", [0.30, -0.10, 0.23])
        self.declare_parameter("start_max_base", [0.35, 0.02, 0.28])
        self.declare_parameter("target_min_base", [0.46, 0.03, 0.09])
        self.declare_parameter("target_max_base", [0.50, 0.12, 0.15])
        self.declare_parameter("random_target_min_distance_m", 0.22)
        self.declare_parameter("base_to_world_xyz", [0.0, 0.0, 1.02])
        self.declare_parameter("free_min_base", [0.32, -0.11, 0.15])
        self.declare_parameter("free_max_base", [0.49, 0.12, 0.23])
        self.declare_parameter("path_fraction_min", 0.35)
        self.declare_parameter("path_fraction_max", 0.80)
        self.declare_parameter("path_lateral_max_m", 0.035)
        self.declare_parameter("endpoint_clearance_m", 0.12)
        self.declare_parameter("roll_pitch_max_rad", 0.35)
        self.declare_parameter("yaw_min_rad", -math.pi)
        self.declare_parameter("yaw_max_rad", math.pi)
        self.declare_parameter("goal_tolerance_m", 0.035)

        world_name = str(self.get_parameter("world_name").value)
        self._world_name = world_name
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
        self._plan_client = self.create_client(Trigger, "/drl/plan")
        self._execute_client = self.create_client(Trigger, "/drl/execute_forward")
        self._clear_client = self.create_client(Trigger, "/drl/clear_trajectory")
        self._status_client = self.create_client(Trigger, "/drl/get_execution_status")
        self._pose_sequence_client = self.create_client(
            MoveCartesianPoseSequence,
            "/move_cartesian_pose_sequence",
        )
        self._latest_poses: PoseArray | None = None
        self._poses_sub = self.create_subscription(
            PoseArray,
            "/drl/forward_trajectory_poses",
            self._on_poses,
            10,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._sdf_path = (
            Path(get_package_share_directory("robot_description"))
            / "worlds"
            / "wood_block"
            / "wood_model.sdf"
        )

    def _on_poses(self, msg: PoseArray) -> None:
        self._latest_poses = msg

    def _timeout(self) -> float:
        return float(self.get_parameter("timeout_sec").value)

    def _wait_for_service(self, client, name: str) -> None:
        deadline = time.monotonic() + self._timeout()
        while rclpy.ok() and time.monotonic() < deadline:
            if client.service_is_ready():
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(f"Service {name} not available after {self._timeout():.1f}s")

    def _spin_until_future(self, future, label: str, timeout: float | None = None) -> None:
        wait = self._timeout() if timeout is None else float(timeout)
        deadline = time.monotonic() + wait
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                raise TimeoutError(f"{label} timed out after {wait:.1f}s")
            rclpy.spin_once(self, timeout_sec=0.05)

    def _spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + max(0.0, float(seconds))
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

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

    def _move_tcp_to_start(self, start: np.ndarray) -> str:
        self._wait_for_service(self._pose_sequence_client, "/move_cartesian_pose_sequence")
        req = MoveCartesianPoseSequence.Request()
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = float(start[0])
        pose.pose.position.y = float(start[1])
        pose.pose.position.z = float(start[2])
        pose.pose.orientation.x = 0.7071068
        pose.pose.orientation.y = 0.7071068
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0
        req.poses.append(pose)
        req.execute = True
        future = self._pose_sequence_client.call_async(req)
        self._spin_until_future(future, "move tcp to random start")
        resp = future.result()
        if resp is None:
            raise RuntimeError("move tcp to random start returned no response")
        if not resp.success:
            raise RuntimeError(
                f"move tcp to random start failed (fraction={resp.fraction:.4f}): {resp.message}"
            )
        return f"fraction={resp.fraction:.4f}: {resp.message}"

    def _vector_param(self, name: str) -> np.ndarray:
        arr = np.asarray(self.get_parameter(name).value, dtype=np.float32)
        if arr.shape != (3,):
            raise ValueError(f"{name} must have 3 elements, got {arr.shape}")
        return arr

    def _sample_vector(self, min_name: str, max_name: str, rng: np.random.Generator) -> np.ndarray:
        lower = self._vector_param(min_name)
        upper = self._vector_param(max_name)
        if np.any(lower > upper):
            raise ValueError(f"{min_name} must be <= {max_name}")
        return rng.uniform(lower, upper).astype(np.float32)

    def _quat_from_rpy(self, roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 1e-12:
            return (0.0, 0.0, 0.0, 1.0)
        return (qx / norm, qy / norm, qz / norm, qw / norm)

    def _sample_orientation(
        self,
        rng: np.random.Generator,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        rp = max(0.0, float(self.get_parameter("roll_pitch_max_rad").value))
        yaw_min = float(self.get_parameter("yaw_min_rad").value)
        yaw_max = float(self.get_parameter("yaw_max_rad").value)
        if yaw_min > yaw_max:
            yaw_min, yaw_max = yaw_max, yaw_min
        rpy = (
            float(rng.uniform(-rp, rp)),
            float(rng.uniform(-rp, rp)),
            float(rng.uniform(yaw_min, yaw_max)),
        )
        return rpy, self._quat_from_rpy(*rpy)

    def _choose_start_target(self, rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray]:
        start = (
            self._sample_vector("start_min_base", "start_max_base", rng)
            if bool(self.get_parameter("randomize_start").value)
            else self._vector_param("start_base")
        )
        if not bool(self.get_parameter("randomize_target").value):
            return start, self._vector_param("target_base")
        min_distance = max(0.0, float(self.get_parameter("random_target_min_distance_m").value))
        for _ in range(64):
            target = self._sample_vector("target_min_base", "target_max_base", rng)
            if float(np.linalg.norm(target - start)) >= min_distance:
                return start, target
        return start, self._vector_param("target_base")

    def _sample_obstacle_center(
        self,
        mode: str,
        start: np.ndarray,
        target: np.ndarray,
        rng: np.random.Generator,
    ) -> np.ndarray:
        endpoint_clearance = max(0.0, float(self.get_parameter("endpoint_clearance_m").value))
        if mode == "random_free":
            for _ in range(64):
                center = self._sample_vector("free_min_base", "free_max_base", rng)
                if (
                    float(np.linalg.norm(center - start)) >= endpoint_clearance
                    and float(np.linalg.norm(center - target)) >= endpoint_clearance
                ):
                    return center
            return self._sample_vector("free_min_base", "free_max_base", rng)

        path = target - start
        side = np.array([-path[1], path[0], 0.0], dtype=np.float32)
        side_norm = float(np.linalg.norm(side))
        side = side / side_norm if side_norm > 1e-8 else np.array([0.0, 1.0, 0.0], dtype=np.float32)
        frac_min = float(np.clip(self.get_parameter("path_fraction_min").value, 0.0, 1.0))
        frac_max = float(np.clip(self.get_parameter("path_fraction_max").value, 0.0, 1.0))
        if frac_min > frac_max:
            frac_min, frac_max = frac_max, frac_min
        lateral_max = max(0.0, float(self.get_parameter("path_lateral_max_m").value))
        lower = self._vector_param("free_min_base")
        upper = self._vector_param("free_max_base")
        for _ in range(64):
            frac = float(rng.uniform(frac_min, frac_max))
            lateral = float(rng.uniform(-lateral_max, lateral_max))
            center = start + frac * path + lateral * side
            center[2] = float(np.clip(center[2], lower[2], upper[2]))
            center = np.minimum(np.maximum(center, lower), upper).astype(np.float32)
            if (
                float(np.linalg.norm(center - start)) >= endpoint_clearance
                and float(np.linalg.norm(center - target)) >= endpoint_clearance
            ):
                return center
        return np.minimum(np.maximum((start + target) / 2.0, lower), upper).astype(np.float32)

    def _set_planner_params(self, start: np.ndarray, target: np.ndarray) -> None:
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
        req.parameters.append(
            Parameter(
                name="preposition_before_plan",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_BOOL,
                    bool_value=False,
                ),
            )
        )
        future = self._set_planner_params_client.call_async(req)
        self._spin_until_future(future, "set planner gazebo params")
        resp = future.result()
        if resp is None or any(not r.successful for r in resp.results):
            reasons = "; ".join(r.reason for r in (resp.results if resp else []))
            raise RuntimeError(f"Failed to set planner params: {reasons}")

    def _spawn_gazebo(self, case: GazeboCase) -> None:
        cmd = [
            "ros2",
            "run",
            "ros_gz_sim",
            "create",
            "-name",
            case.name,
            "-x",
            str(float(case.center_world[0])),
            "-y",
            str(float(case.center_world[1])),
            "-z",
            str(float(case.center_world[2])),
            "-R",
            str(float(case.rpy[0])),
            "-P",
            str(float(case.rpy[1])),
            "-Y",
            str(float(case.rpy[2])),
            "-file",
            str(self._sdf_path),
            "-allow_renaming",
            "true",
        ]
        self.get_logger().info(
            f"Spawn Gazebo entity {case.name} @ world={np.array2string(case.center_world, precision=4)} "
            f"RPY={tuple(round(v, 4) for v in case.rpy)}"
        )
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=self._timeout(),
            check=False,
        )
        if result.returncode != 0:
            raise RuntimeError(
                f"Gazebo create command failed rc={result.returncode}: "
                f"{(result.stderr or result.stdout).strip()}"
            )
        if result.stdout.strip():
            self.get_logger().info(f"Gazebo create stdout: {result.stdout.strip()}")

    def _delete_gazebo(self, name: str) -> None:
        cmd = [
            "ros2",
            "service",
            "call",
            f"/world/{self._world_name}/remove",
            "ros_gz_interfaces/srv/DeleteEntity",
            f"{{entity: {{name: '{name}', type: 2}}}}",
        ]
        try:
            subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=5.0,
                check=False,
            )
        except Exception:
            return

    def _clear_scene_object(self, object_id: str) -> None:
        deadline = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < deadline:
            if self._apply_scene_client.service_is_ready():
                break
            rclpy.spin_once(self, timeout_sec=0.05)
        else:
            return
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
        try:
            self._spin_until_future(future, "clear planning scene object", timeout=5.0)
        except Exception:
            return

    def _add_scene_box(self, case: GazeboCase) -> SceneObstacle:
        self._wait_for_service(self._apply_scene_client, "/apply_planning_scene")
        obj = CollisionObject()
        obj.header.frame_id = "base_link"
        obj.id = case.name
        obj.operation = CollisionObject.ADD
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [float(v) for v in WOOD_BLOCK_SIZE]
        pose = Pose()
        pose.position.x = float(case.center_base[0])
        pose.position.y = float(case.center_base[1])
        pose.position.z = float(case.center_base[2])
        pose.orientation.x = float(case.quat_xyzw[0])
        pose.orientation.y = float(case.quat_xyzw[1])
        pose.orientation.z = float(case.quat_xyzw[2])
        pose.orientation.w = float(case.quat_xyzw[3])
        obj.primitives.append(prim)
        obj.primitive_poses.append(pose)
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self._apply_scene_client.call_async(req)
        self._spin_until_future(future, "add planning scene wood block")
        resp = future.result()
        if resp is None or not resp.success:
            raise RuntimeError("Failed to add wood block to PlanningScene")
        confirmed = self._wait_for_scene_object(case.name)
        obstacles = collision_object_to_obstacles(
            confirmed,
            tf_buffer=self._tf_buffer,
            target_frame="base_link",
            timeout_sec=self._timeout(),
        )
        if not obstacles:
            raise RuntimeError(f"PlanningScene object '{case.name}' has no usable geometry")
        return obstacles[0]

    def _wait_for_scene_object(self, object_id: str) -> CollisionObject:
        self._wait_for_service(self._get_scene_client, "/get_planning_scene")
        deadline = time.monotonic() + self._timeout()
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
            self._spin_for(0.1)
        raise TimeoutError(f"PlanningScene did not confirm object '{object_id}'")

    def _plan_and_capture(self, label: str) -> tuple[list[np.ndarray], float]:
        self._latest_poses = None
        self._call_trigger(self._clear_client, "/drl/clear_trajectory")
        self._call_trigger(self._plan_client, "/drl/plan")
        deadline = time.monotonic() + self._timeout()
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
        length = sum(
            float(np.linalg.norm(trajectory[i + 1] - trajectory[i]))
            for i in range(len(trajectory) - 1)
        )
        return trajectory, length

    def _wait_for_execution_idle(self) -> str:
        self._wait_for_service(self._status_client, "/drl/get_execution_status")
        deadline = time.monotonic() + self._timeout()
        last_message = ""
        while rclpy.ok() and time.monotonic() < deadline:
            future = self._status_client.call_async(Trigger.Request())
            self._spin_until_future(future, "get execution status")
            resp = future.result()
            if resp is not None:
                last_message = resp.message
                if resp.success:
                    if not resp.message.startswith("SUCCEEDED"):
                        raise RuntimeError(f"Execution finished without success: {resp.message}")
                    return resp.message
            self._spin_for(0.2)
        raise TimeoutError(f"Execution did not finish; last={last_message}")

    def _current_tcp_error(self, target: np.ndarray) -> tuple[float, bool]:
        try:
            tf = self._tf_buffer.lookup_transform(
                "base_link",
                "tcp_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
        except Exception:
            return math.inf, False
        p = tf.transform.translation
        current = np.array([p.x, p.y, p.z], dtype=np.float32)
        return float(np.linalg.norm(current - target)), True

    def _case_output_path(self, base_output: Path, idx: int, count: int) -> Path:
        if count <= 1:
            return base_output
        return base_output.with_name(f"{base_output.stem}_case{idx:03d}{base_output.suffix or '.npz'}")

    def _make_case(self, idx: int, rng: np.random.Generator, seed_label: str) -> GazeboCase:
        start, target = self._choose_start_target(rng)
        mode = str(self.get_parameter("obstacle_mode").value)
        center_base = self._sample_obstacle_center(mode, start, target, rng)
        base_to_world = self._vector_param("base_to_world_xyz")
        center_world = center_base + base_to_world
        rpy, quat_xyzw = self._sample_orientation(rng)
        return GazeboCase(
            name=f"drl_wood_block_{idx:03d}",
            seed_label=seed_label,
            start=start,
            target=target,
            center_base=center_base,
            center_world=center_world,
            rpy=rpy,
            quat_xyzw=quat_xyzw,
            mode=mode,
        )

    def run(self) -> None:
        base_seed = int(self.get_parameter("random_seed").value)
        case_count = max(1, int(self.get_parameter("case_count").value))
        execute = bool(self.get_parameter("execute").value)
        max_attempts = max(1, int(self.get_parameter("max_random_attempts").value))
        retry_cooldown = max(0.0, float(self.get_parameter("retry_cooldown_sec").value))
        goal_tolerance = max(0.0, float(self.get_parameter("goal_tolerance_m").value))
        base_output = Path(str(self.get_parameter("output_file").value))
        completed: list[Path] = []

        for case_idx in range(case_count):
            if base_seed < 0:
                rng = np.random.default_rng(None)
                seed_label = "entropy"
            else:
                seed = base_seed + case_idx
                rng = np.random.default_rng(seed)
                seed_label = str(seed)
            out = self._case_output_path(base_output, case_idx, case_count)
            last_error = ""
            for attempt in range(1, max_attempts + 1):
                case = self._make_case(case_idx, rng, seed_label)
                obstacle = None
                try:
                    self._delete_gazebo(case.name)
                    self._clear_scene_object(case.name)
                    self._set_planner_params(case.start, case.target)
                    preposition_msg = self._move_tcp_to_start(case.start)
                    self.get_logger().info(
                        f"Gazebo case {case_idx + 1}/{case_count} attempt {attempt}/{max_attempts} | "
                        f"seed={seed_label} mode={case.mode} start={np.array2string(case.start, precision=4)} "
                        f"target={np.array2string(case.target, precision=4)} "
                        f"center_base={np.array2string(case.center_base, precision=4)} "
                        f"rpy={tuple(round(v, 4) for v in case.rpy)} "
                        f"quat={tuple(round(v, 4) for v in case.quat_xyzw)} | "
                        f"preposition={preposition_msg}"
                    )
                    self._spawn_gazebo(case)
                    obstacle = self._add_scene_box(case)
                    trajectory, path_length = self._plan_and_capture("gazebo_obstacle")
                    validation = validate_cartesian_path_against_obstacles(
                        trajectory,
                        [obstacle],
                        max_step_m=0.01,
                        margin_m=0.05,
                    )
                    if not validation.valid:
                        raise RuntimeError(validation.message)
                    execution_msg = "not_executed"
                    final_error = math.inf
                    final_tf_ok = False
                    if execute:
                        self._call_trigger(self._execute_client, "/drl/execute_forward")
                        execution_msg = self._wait_for_execution_idle()
                        final_error, final_tf_ok = self._current_tcp_error(case.target)
                        if final_tf_ok and final_error > goal_tolerance:
                            raise RuntimeError(
                                f"Final TCP error {final_error:.5f} exceeds tolerance {goal_tolerance:.5f}"
                            )

                    np.savez(
                        out,
                        seed=case.seed_label,
                        mode=case.mode,
                        start=case.start,
                        goal=case.target,
                        gazebo_center_world=case.center_world,
                        planning_scene_center_base=case.center_base,
                        dimensions=WOOD_BLOCK_SIZE,
                        rpy=np.asarray(case.rpy, dtype=np.float32),
                        quaternion_xyzw=np.asarray(case.quat_xyzw, dtype=np.float32),
                        obstacle_aabb_center=obstacle.center_base,
                        obstacle_aabb_size=obstacle.full_size,
                        path=np.asarray(trajectory, dtype=np.float32),
                        waypoint_count=len(trajectory),
                        path_length=path_length,
                        min_clearance=validation.min_clearance,
                        planning_result="success",
                        execution_result=execution_msg,
                        final_error=final_error,
                        final_tf_ok=final_tf_ok,
                    )
                    self.get_logger().info(
                        "Gazebo obstacle case passed | "
                        f"seed={seed_label} wp={len(trajectory)} len={path_length:.4f} "
                        f"min_clearance={validation.min_clearance:.5f} "
                        f"execution={execution_msg} final_error={final_error:.5f} saved={out}"
                    )
                    completed.append(out)
                    break
                except Exception as exc:
                    last_error = str(exc)
                    self.get_logger().warn(
                        f"Gazebo case {case_idx + 1}/{case_count} attempt "
                        f"{attempt}/{max_attempts} failed: {exc}"
                    )
                    if attempt < max_attempts and retry_cooldown > 0.0:
                        self._spin_for(retry_cooldown)
                finally:
                    self._delete_gazebo(case.name)
                    self._clear_scene_object(case.name)
            else:
                raise RuntimeError(
                    f"Gazebo case {case_idx + 1}/{case_count} failed after "
                    f"{max_attempts} attempts; last_error={last_error}"
                )

        self.get_logger().info(
            f"Completed {len(completed)}/{case_count} Gazebo obstacle case(s): "
            + ", ".join(str(path) for path in completed)
        )


def main() -> None:
    rclpy.init()
    node = GazeboObstacleTest()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
