#!/usr/bin/env python3
import math
import time
from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

from robot_task_manager.action import DrlPickPlace, MoveToPoseCartesian


@dataclass
class TrialResult:
    trial_id: int
    seed: int
    start: np.ndarray
    obstacle_center: np.ndarray
    obstacle_size: np.ndarray
    obstacle_orientation_xyzw: tuple[float, float, float, float]
    pick: np.ndarray
    place: np.ndarray
    success: bool
    message: str
    failed_stage: str
    elapsed_sec: float


class DrlPickPlaceRandomTestClient(Node):
    def __init__(self) -> None:
        super().__init__("drl_pick_place_random_test_client")
        self.declare_parameter("number_of_trials", 20)
        self.declare_parameter("random_seed", 0)
        self.declare_parameter("gripper_close_width_m", 0.028)
        self.declare_parameter("action_name", "drl_pickplace")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("workspace_min", [0.30, -0.12, 0.08])
        self.declare_parameter("workspace_max", [0.48, 0.12, 0.18])
        self.declare_parameter("start_min", [0.34, -0.08, 0.20])
        self.declare_parameter("start_max", [0.41, 0.08, 0.27])
        self.declare_parameter("min_pick_place_distance_m", 0.08)
        self.declare_parameter("goal_timeout_sec", 300.0)
        self.declare_parameter("setup_timeout_sec", 60.0)
        self.declare_parameter("action_server_timeout_sec", 30.0)
        self.declare_parameter("wait_for_joint_states", False)
        self.declare_parameter("joint_states_timeout_sec", 30.0)
        self.declare_parameter("start_velocity_scale", 0.35)
        self.declare_parameter("obstacle_id", "drl_pick_place_random_obstacle")
        self.declare_parameter("obstacle_size_min", [0.018, 0.018, 0.018])
        self.declare_parameter("obstacle_size_max", [0.040, 0.040, 0.045])
        self.declare_parameter("obstacle_path_fraction_min", 0.35)
        self.declare_parameter("obstacle_path_fraction_max", 0.75)
        self.declare_parameter("obstacle_lateral_min_m", 0.090)
        self.declare_parameter("obstacle_lateral_max_m", 0.130)
        self.declare_parameter("obstacle_endpoint_clearance_m", 0.090)
        self.declare_parameter("target_obstacle_clearance_m", 0.060)
        self.declare_parameter("obstacle_roll_pitch_max_rad", 0.20)
        self.declare_parameter("obstacle_yaw_min_rad", -math.pi)
        self.declare_parameter("obstacle_yaw_max_rad", math.pi)

        self._client = ActionClient(
            self,
            DrlPickPlace,
            str(self.get_parameter("action_name").value),
        )
        self._cartesian_client = ActionClient(
            self,
            MoveToPoseCartesian,
            "move_to_pose_cartesian",
        )
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

    def _pose(self, xyz: np.ndarray) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = str(self.get_parameter("frame_id").value)
        pose.pose.position.x = float(xyz[0])
        pose.pose.position.y = float(xyz[1])
        pose.pose.position.z = float(xyz[2])
        pose.pose.orientation.x = 0.7071068
        pose.pose.orientation.y = 0.7071068
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0
        return pose

    def _get_vector_param(self, name: str) -> np.ndarray:
        arr = np.asarray(self.get_parameter(name).value, dtype=float)
        if arr.shape != (3,):
            raise ValueError(f"{name} must have exactly 3 elements")
        return arr

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
        roll_pitch_max = max(
            0.0,
            float(self.get_parameter("obstacle_roll_pitch_max_rad").value),
        )
        yaw_min = float(self.get_parameter("obstacle_yaw_min_rad").value)
        yaw_max = float(self.get_parameter("obstacle_yaw_max_rad").value)
        if yaw_min > yaw_max:
            yaw_min, yaw_max = yaw_max, yaw_min
        return self._quat_from_rpy(
            float(rng.uniform(-roll_pitch_max, roll_pitch_max)),
            float(rng.uniform(-roll_pitch_max, roll_pitch_max)),
            float(rng.uniform(yaw_min, yaw_max)),
        )

    def _point_clear_of_box(
        self,
        point: np.ndarray,
        center: np.ndarray,
        size: np.ndarray,
        margin: float,
    ) -> bool:
        half = size / 2.0 + margin
        return bool(np.any(np.abs(point - center) > half))

    def _sample_start(self, rng: np.random.Generator) -> np.ndarray:
        lower = self._get_vector_param("start_min")
        upper = self._get_vector_param("start_max")
        return rng.uniform(lower, upper)

    def _sample_pair(
        self,
        rng: np.random.Generator,
        obstacle_center: np.ndarray,
        obstacle_size: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        lower = np.asarray(self.get_parameter("workspace_min").value, dtype=float)
        upper = np.asarray(self.get_parameter("workspace_max").value, dtype=float)
        min_dist = float(self.get_parameter("min_pick_place_distance_m").value)
        for _ in range(200):
            pick = rng.uniform(lower, upper)
            place = rng.uniform(lower, upper)
            if (
                float(np.linalg.norm(place - pick)) >= min_dist
                and self._point_clear_of_box(
                    pick,
                    obstacle_center,
                    obstacle_size,
                    float(self.get_parameter("target_obstacle_clearance_m").value),
                )
                and self._point_clear_of_box(
                    place,
                    obstacle_center,
                    obstacle_size,
                    float(self.get_parameter("target_obstacle_clearance_m").value),
                )
            ):
                return pick, place
        raise RuntimeError("Could not sample a valid pick/place pair")

    def _sample_obstacle(
        self,
        start: np.ndarray,
        rng: np.random.Generator,
    ) -> tuple[np.ndarray, np.ndarray, tuple[float, float, float, float]]:
        size = rng.uniform(
            self._get_vector_param("obstacle_size_min"),
            self._get_vector_param("obstacle_size_max"),
        )
        workspace_min = self._get_vector_param("workspace_min")
        workspace_max = self._get_vector_param("workspace_max")
        provisional_goal = rng.uniform(workspace_min, workspace_max)
        path = provisional_goal - start
        side = np.array([-path[1], path[0], 0.0], dtype=float)
        side_norm = float(np.linalg.norm(side))
        if side_norm <= 1e-8:
            side = np.array([0.0, 1.0, 0.0], dtype=float)
        else:
            side /= side_norm

        frac_min = float(self.get_parameter("obstacle_path_fraction_min").value)
        frac_max = float(self.get_parameter("obstacle_path_fraction_max").value)
        lateral_min = float(self.get_parameter("obstacle_lateral_min_m").value)
        lateral_max = float(self.get_parameter("obstacle_lateral_max_m").value)
        endpoint_clearance = float(self.get_parameter("obstacle_endpoint_clearance_m").value)
        if frac_min > frac_max:
            frac_min, frac_max = frac_max, frac_min
        if lateral_min > lateral_max:
            lateral_min, lateral_max = lateral_max, lateral_min

        last_center = (start + provisional_goal) / 2.0
        for _ in range(100):
            frac = float(rng.uniform(frac_min, frac_max))
            sign = -1.0 if rng.random() < 0.5 else 1.0
            lateral = sign * float(rng.uniform(lateral_min, lateral_max))
            center = start + frac * path + lateral * side
            center[2] = float(np.clip(center[2], workspace_min[2] + 0.02, workspace_max[2] + 0.08))
            center = np.minimum(
                np.maximum(center, workspace_min - np.array([0.02, 0.02, 0.02])),
                workspace_max + np.array([0.02, 0.02, 0.10]),
            )
            last_center = center
            if float(np.linalg.norm(center - start)) >= endpoint_clearance:
                break

        return last_center, size, self._sample_obstacle_orientation(rng)

    def _wait_for_future(self, future, label: str, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                raise TimeoutError(f"{label} timed out after {timeout:.1f}s")
            rclpy.spin_once(self, timeout_sec=0.05)

    def _wait_for_joint_states(self) -> None:
        if not bool(self.get_parameter("wait_for_joint_states").value):
            return

        timeout = float(self.get_parameter("joint_states_timeout_sec").value)
        deadline = time.monotonic() + timeout
        received = False

        def _on_joint_state(msg: JointState) -> None:
            nonlocal received
            received = bool(msg.name) and bool(msg.position)

        sub = self.create_subscription(JointState, "/joint_states", _on_joint_state, 10)
        try:
            while rclpy.ok() and not received:
                if time.monotonic() > deadline:
                    raise TimeoutError(f"/joint_states timed out after {timeout:.1f}s")
                rclpy.spin_once(self, timeout_sec=0.1)
            self.get_logger().info("/joint_states is publishing")
        finally:
            self.destroy_subscription(sub)

    def _clear_obstacle(self) -> None:
        if not self._apply_scene_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/apply_planning_scene is not available")
        obj = CollisionObject()
        obj.header.frame_id = str(self.get_parameter("frame_id").value)
        obj.id = str(self.get_parameter("obstacle_id").value)
        obj.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self._apply_scene_client.call_async(req)
        self._wait_for_future(future, "clear obstacle", 10.0)

    def _wait_for_obstacle(self, object_id: str) -> None:
        if not self._get_scene_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/get_planning_scene is not available")
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and time.monotonic() < deadline:
            req = GetPlanningScene.Request()
            req.components.components = (
                PlanningSceneComponents.WORLD_OBJECT_NAMES
                | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            )
            future = self._get_scene_client.call_async(req)
            self._wait_for_future(future, "get planning scene", 10.0)
            resp = future.result()
            if resp is not None and any(
                obj.id == object_id for obj in resp.scene.world.collision_objects
            ):
                return
            time.sleep(0.1)
        raise TimeoutError(f"PlanningScene did not confirm obstacle '{object_id}'")

    def _add_obstacle(
        self,
        center: np.ndarray,
        size: np.ndarray,
        orientation_xyzw: tuple[float, float, float, float],
    ) -> None:
        if not self._apply_scene_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/apply_planning_scene is not available")
        object_id = str(self.get_parameter("obstacle_id").value)
        obj = CollisionObject()
        obj.header.frame_id = str(self.get_parameter("frame_id").value)
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
        self._wait_for_future(future, "add obstacle", 10.0)
        resp = future.result()
        if resp is None or not resp.success:
            raise RuntimeError("Failed to add PlanningScene obstacle")
        self._wait_for_obstacle(object_id)

    def _set_planner_start(self, start: np.ndarray) -> None:
        if not self._set_planner_params_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/drl_unified_planner_node/set_parameters is not available")
        req = SetParameters.Request()
        for name in ("preposition_tcp_base", "calibrated_start_tcp_base"):
            req.parameters.append(
                Parameter(
                    name=name,
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                        double_array_value=[float(v) for v in start],
                    ),
                )
            )
        future = self._set_planner_params_client.call_async(req)
        self._wait_for_future(future, "set planner start params", 10.0)
        resp = future.result()
        if resp is None:
            raise RuntimeError("Planner parameter update returned no response")
        failures = [
            f"{param.name}: {result.reason}"
            for param, result in zip(req.parameters, resp.results)
            if not result.successful
        ]
        if failures:
            raise RuntimeError("Failed to set planner start params: " + "; ".join(failures))

    def _move_to_start(self, start: np.ndarray) -> None:
        if not self._cartesian_client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("MoveToPoseCartesian action server not available")
        goal = MoveToPoseCartesian.Goal()
        goal.target_pose = self._pose(start).pose
        goal.velocity_scale = float(self.get_parameter("start_velocity_scale").value)
        future = self._cartesian_client.send_goal_async(goal)
        self._wait_for_future(
            future,
            "send MoveToPoseCartesian start goal",
            float(self.get_parameter("setup_timeout_sec").value),
        )
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("MoveToPoseCartesian start goal rejected")
        result_future = goal_handle.get_result_async()
        self._wait_for_future(
            result_future,
            "MoveToPoseCartesian start result",
            float(self.get_parameter("setup_timeout_sec").value),
        )
        result = result_future.result().result
        if not result.success:
            raise RuntimeError(f"MoveToPoseCartesian start failed: {result.message}")

    def _send_trial(self, trial_id: int, seed: int) -> TrialResult:
        rng = np.random.default_rng(seed)
        start_pose = self._sample_start(rng)
        obstacle_center, obstacle_size, obstacle_orientation = self._sample_obstacle(
            start_pose,
            rng,
        )
        pick, place = self._sample_pair(rng, obstacle_center, obstacle_size)
        close_width = float(self.get_parameter("gripper_close_width_m").value)

        self._clear_obstacle()
        self._move_to_start(start_pose)
        self._set_planner_start(start_pose)
        self._add_obstacle(obstacle_center, obstacle_size, obstacle_orientation)

        goal = DrlPickPlace.Goal()
        goal.target_pick = self._pose(pick)
        goal.target_place = self._pose(place)
        goal.gripper_close_width_m = close_width

        self.get_logger().info(
            f"trial={trial_id} seed={seed} "
            f"start=({start_pose[0]:.4f}, {start_pose[1]:.4f}, {start_pose[2]:.4f}) "
            f"obstacle=({obstacle_center[0]:.4f}, {obstacle_center[1]:.4f}, {obstacle_center[2]:.4f}) "
            f"obstacle_size=({obstacle_size[0]:.4f}, {obstacle_size[1]:.4f}, {obstacle_size[2]:.4f}) "
            f"obstacle_quat=({obstacle_orientation[0]:.4f}, {obstacle_orientation[1]:.4f}, "
            f"{obstacle_orientation[2]:.4f}, {obstacle_orientation[3]:.4f}) "
            f"pick=({pick[0]:.4f}, {pick[1]:.4f}, {pick[2]:.4f}) "
            f"place=({place[0]:.4f}, {place[1]:.4f}, {place[2]:.4f}) "
            f"gripper={close_width:.4f}"
        )

        start = time.monotonic()
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return TrialResult(
                trial_id,
                seed,
                start_pose,
                obstacle_center,
                obstacle_size,
                obstacle_orientation,
                pick,
                place,
                False,
                "goal rejected",
                "SEND_GOAL",
                0.0,
            )

        result_future = goal_handle.get_result_async()
        timeout = float(self.get_parameter("goal_timeout_sec").value)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not result_future.done():
            if time.monotonic() > deadline:
                return TrialResult(
                    trial_id,
                    seed,
                    start_pose,
                    obstacle_center,
                    obstacle_size,
                    obstacle_orientation,
                    pick,
                    place,
                    False,
                    f"goal timeout after {timeout:.1f}s",
                    "TIMEOUT",
                    time.monotonic() - start,
                )
            rclpy.spin_once(self, timeout_sec=0.1)

        wrapped = result_future.result()
        elapsed = time.monotonic() - start
        result = wrapped.result
        return TrialResult(
            trial_id,
            seed,
            start_pose,
            obstacle_center,
            obstacle_size,
            obstacle_orientation,
            pick,
            place,
            bool(result.success),
            result.message,
            result.failed_stage,
            elapsed,
        )

    def run(self) -> int:
        self._wait_for_joint_states()

        action_server_timeout = float(
            self.get_parameter("action_server_timeout_sec").value
        )
        if not self._client.wait_for_server(timeout_sec=action_server_timeout):
            self.get_logger().error("DrlPickPlace action server not available")
            return 2

        count = int(self.get_parameter("number_of_trials").value)
        base_seed = int(self.get_parameter("random_seed").value)
        results: list[TrialResult] = []
        for trial_id in range(count):
            try:
                result = self._send_trial(trial_id, base_seed + trial_id)
            except Exception as exc:
                self.get_logger().error(
                    f"trial={trial_id} setup_or_send_exception={exc}"
                )
                nan3 = np.array([math.nan, math.nan, math.nan], dtype=float)
                result = TrialResult(
                    trial_id,
                    base_seed + trial_id,
                    nan3,
                    nan3,
                    nan3,
                    (math.nan, math.nan, math.nan, math.nan),
                    nan3,
                    nan3,
                    False,
                    str(exc),
                    "SETUP",
                    0.0,
                )
            results.append(result)
            self.get_logger().info(
                f"trial={result.trial_id} "
                f"result={'PASS' if result.success else 'FAIL'} "
                f"failed_stage={result.failed_stage} "
                f"elapsed={result.elapsed_sec:.2f}s "
                f"message={result.message}"
            )

        passed = sum(1 for r in results if r.success)
        failed = count - passed
        avg_time = sum(r.elapsed_sec for r in results) / count if count else math.nan
        failed_seeds = [r.seed for r in results if not r.success]
        failed_stages = [r.failed_stage for r in results if not r.success]

        self.get_logger().info("=" * 72)
        self.get_logger().info(f"Total trials: {count}")
        self.get_logger().info(f"Passed: {passed}")
        self.get_logger().info(f"Failed: {failed}")
        self.get_logger().info(f"Success rate: {100.0 * passed / count if count else 0.0:.1f}%")
        self.get_logger().info(f"Average execution time: {avg_time:.2f}s")
        self.get_logger().info("Maximum pose error: reported by action server logs")
        self.get_logger().info(f"Failed seeds: {failed_seeds}")
        self.get_logger().info(f"Failed stages: {failed_stages}")
        self.get_logger().info("=" * 72)
        return 0 if failed == 0 else 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DrlPickPlaceRandomTestClient()
    code = 1
    try:
        code = node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted by user")
        code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
