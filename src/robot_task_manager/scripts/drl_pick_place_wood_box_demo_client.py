#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParameters
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker

from robot_task_manager.action import DrlPickPlace


class DrlPickPlaceWoodBoxDemoClient(Node):
    def __init__(self) -> None:
        super().__init__("drl_pick_place_wood_box_demo_client")
        self.declare_parameter("wood_info_topic", "/sim/pick_wood_info")
        self.declare_parameter("box_info_topic", "/sim/obstacle_box_info")
        self.declare_parameter("start_info_topic", "/sim/start_pose_info")
        self.declare_parameter("place_info_topic", "/sim/place_pose_info")
        self.declare_parameter("use_scene_start_pose", True)
        self.declare_parameter("use_scene_place_pose", True)
        self.declare_parameter("action_name", "drl_pickplace")
        self.declare_parameter("planner_node_name", "/drl_unified_planner_node")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("place_xyz", [0.46, 0.12, 0.12])
        self.declare_parameter("grasp_tcp_offset_z", 0.015)
        self.declare_parameter("place_z_offset_m", 0.0)
        self.declare_parameter("min_pick_z_m", 0.025)
        self.declare_parameter("gripper_close_width_m", 0.025)
        self.declare_parameter("execute", True)
        self.declare_parameter("obstacle_id", "obstacle_box")
        self.declare_parameter("object_timeout_sec", 60.0)
        self.declare_parameter("action_server_timeout_sec", 120.0)
        self.declare_parameter("planning_scene_timeout_sec", 20.0)
        self.declare_parameter("goal_timeout_sec", 420.0)
        self._node_t0 = time.monotonic()

        self._wood_marker: Marker | None = None
        self._box_marker: Marker | None = None
        self._start_marker: Marker | None = None
        self._place_marker: Marker | None = None
        self._obstacle_object: CollisionObject | None = None

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._wood_sub = self.create_subscription(
            Marker,
            str(self.get_parameter("wood_info_topic").value),
            self._on_wood_marker,
            qos,
        )
        self._box_sub = self.create_subscription(
            Marker,
            str(self.get_parameter("box_info_topic").value),
            self._on_box_marker,
            qos,
        )
        self._start_sub = self.create_subscription(
            Marker,
            str(self.get_parameter("start_info_topic").value),
            self._on_start_marker,
            qos,
        )
        self._place_sub = self.create_subscription(
            Marker,
            str(self.get_parameter("place_info_topic").value),
            self._on_place_marker,
            qos,
        )
        self._client = ActionClient(
            self,
            DrlPickPlace,
            str(self.get_parameter("action_name").value),
        )
        planner_node = str(self.get_parameter("planner_node_name").value)
        self._set_planner_params_client = self.create_client(
            SetParameters,
            planner_node.rstrip("/") + "/set_parameters",
        )
        self._apply_scene_client = self.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
        )
        self._get_scene_client = self.create_client(
            GetPlanningScene,
            "/get_planning_scene",
        )
        self._collision_object_pub = self.create_publisher(
            CollisionObject,
            "/collision_object",
            10,
        )
        self._obstacle_timer = self.create_timer(0.2, self._republish_obstacle)

    def _timing(self, phase: str) -> None:
        self.get_logger().info(
            f"[timing] {phase}: {time.monotonic() - self._node_t0:.3f}s since demo client start"
        )

    def _valid_cube_marker(self, msg: Marker) -> bool:
        return (
            msg.type == Marker.CUBE
            and msg.scale.x > 0.0
            and msg.scale.y > 0.0
            and msg.scale.z > 0.0
        )

    def _on_wood_marker(self, msg: Marker) -> None:
        if self._valid_cube_marker(msg):
            self._wood_marker = msg

    def _on_box_marker(self, msg: Marker) -> None:
        if self._valid_cube_marker(msg):
            self._box_marker = msg

    def _valid_pose_marker(self, msg: Marker) -> bool:
        return (
            msg.scale.x > 0.0
            and msg.scale.y > 0.0
            and msg.scale.z > 0.0
            and math.isfinite(msg.pose.position.x)
            and math.isfinite(msg.pose.position.y)
            and math.isfinite(msg.pose.position.z)
        )

    def _on_start_marker(self, msg: Marker) -> None:
        if self._valid_pose_marker(msg):
            self._start_marker = msg

    def _on_place_marker(self, msg: Marker) -> None:
        if self._valid_pose_marker(msg):
            self._place_marker = msg

    def _wait_for_markers(self) -> tuple[Marker, Marker, Marker | None, Marker | None]:
        timeout = float(self.get_parameter("object_timeout_sec").value)
        deadline = time.monotonic() + timeout
        use_start = bool(self.get_parameter("use_scene_start_pose").value)
        use_place = bool(self.get_parameter("use_scene_place_pose").value)
        while rclpy.ok() and (
            self._wood_marker is None
            or self._box_marker is None
            or (use_start and self._start_marker is None)
            or (use_place and self._place_marker is None)
        ):
            if time.monotonic() > deadline:
                missing = []
                if self._wood_marker is None:
                    missing.append(str(self.get_parameter("wood_info_topic").value))
                if self._box_marker is None:
                    missing.append(str(self.get_parameter("box_info_topic").value))
                if use_start and self._start_marker is None:
                    missing.append(str(self.get_parameter("start_info_topic").value))
                if use_place and self._place_marker is None:
                    missing.append(str(self.get_parameter("place_info_topic").value))
                raise TimeoutError("Timed out waiting for sim ground truth: " + ", ".join(missing))
            rclpy.spin_once(self, timeout_sec=0.1)
        assert self._wood_marker is not None
        assert self._box_marker is not None
        return self._wood_marker, self._box_marker, self._start_marker, self._place_marker

    def _wait_for_future(self, future, label: str, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                raise TimeoutError(f"{label} timed out after {timeout:.1f}s")
            rclpy.spin_once(self, timeout_sec=0.05)

    def _check_frame(self, marker: Marker, label: str) -> None:
        frame_id = str(self.get_parameter("frame_id").value)
        marker_frame = marker.header.frame_id or frame_id
        if marker_frame != frame_id:
            raise RuntimeError(
                f"{label} frame '{marker_frame}' does not match expected frame '{frame_id}'"
            )

    def _pose_from_xyz(self, xyz: list[float], frame_id: str) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(xyz[0])
        pose.pose.position.y = float(xyz[1])
        pose.pose.position.z = float(xyz[2])
        pose.pose.orientation.x = 0.7071068
        pose.pose.orientation.y = 0.7071068
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0
        return pose

    def _clear_obstacle(self, object_id: str, frame_id: str) -> None:
        obj = CollisionObject()
        obj.header.frame_id = frame_id
        obj.id = object_id
        obj.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self._apply_scene_client.call_async(req)
        self._wait_for_future(
            future,
            "remove old obstacle",
            float(self.get_parameter("planning_scene_timeout_sec").value),
        )

    def _build_obstacle_object(self, marker: Marker) -> CollisionObject:
        frame_id = str(self.get_parameter("frame_id").value)
        object_id = str(self.get_parameter("obstacle_id").value)
        obj = CollisionObject()
        obj.header.frame_id = frame_id
        obj.id = object_id
        obj.operation = CollisionObject.ADD

        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [
            float(marker.scale.x),
            float(marker.scale.y),
            float(marker.scale.z),
        ]

        pose = Pose()
        pose.position.x = float(marker.pose.position.x)
        pose.position.y = float(marker.pose.position.y)
        pose.position.z = float(marker.pose.position.z)
        pose.orientation = marker.pose.orientation

        obj.primitives.append(prim)
        obj.primitive_poses.append(pose)
        return obj

    def _republish_obstacle(self) -> None:
        if self._obstacle_object is None:
            return
        self._obstacle_object.header.stamp = self.get_clock().now().to_msg()
        self._collision_object_pub.publish(self._obstacle_object)

    def _add_obstacle(self, marker: Marker) -> None:
        frame_id = str(self.get_parameter("frame_id").value)
        object_id = str(self.get_parameter("obstacle_id").value)
        timeout = float(self.get_parameter("planning_scene_timeout_sec").value)

        if not self._apply_scene_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("/apply_planning_scene is not available")
        if not self._get_scene_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("/get_planning_scene is not available")

        self._clear_obstacle(object_id, frame_id)

        obj = self._build_obstacle_object(marker)
        self._obstacle_object = obj
        self._republish_obstacle()

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)

        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self._apply_scene_client.call_async(req)
        self._wait_for_future(future, "add obstacle", timeout)
        resp = future.result()
        if resp is None or not resp.success:
            raise RuntimeError("Failed to add obstacle_box to PlanningScene")
        self._wait_for_scene_object(object_id, timeout)

    def _wait_for_scene_object(self, object_id: str, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            req = GetPlanningScene.Request()
            req.components.components = (
                PlanningSceneComponents.WORLD_OBJECT_NAMES
                | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            )
            future = self._get_scene_client.call_async(req)
            self._wait_for_future(future, "get planning scene", timeout)
            resp = future.result()
            if resp is not None and any(
                obj.id == object_id for obj in resp.scene.world.collision_objects
            ):
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise TimeoutError(f"PlanningScene did not confirm obstacle '{object_id}'")

    def _marker_xyz(self, marker: Marker) -> list[float]:
        return [
            float(marker.pose.position.x),
            float(marker.pose.position.y),
            float(marker.pose.position.z),
        ]

    def _build_goal(self, wood_marker: Marker, place_marker: Marker | None) -> DrlPickPlace.Goal:
        frame_id = str(self.get_parameter("frame_id").value)
        wood_center_z = float(wood_marker.pose.position.z)
        wood_size_z = float(wood_marker.scale.z)
        wood_top_z = wood_center_z + wood_size_z / 2.0
        grasp_tcp_offset_z = float(self.get_parameter("grasp_tcp_offset_z").value)
        pick_z = wood_center_z + grasp_tcp_offset_z
        pick_z = max(pick_z, float(self.get_parameter("min_pick_z_m").value))

        if bool(self.get_parameter("use_scene_place_pose").value) and place_marker is not None:
            place_xyz = self._marker_xyz(place_marker)
        else:
            place_xyz = [float(v) for v in self.get_parameter("place_xyz").value]
        if len(place_xyz) != 3 or any(not math.isfinite(v) for v in place_xyz):
            raise ValueError("place_xyz must contain exactly 3 finite numbers")
        place_xyz[2] += float(self.get_parameter("place_z_offset_m").value)

        goal = DrlPickPlace.Goal()
        goal.target_pick = self._pose_from_xyz(
            [
                float(wood_marker.pose.position.x),
                float(wood_marker.pose.position.y),
                pick_z,
            ],
            frame_id,
        )
        goal.target_place = self._pose_from_xyz(place_xyz, frame_id)
        goal.gripper_close_width_m = float(
            self.get_parameter("gripper_close_width_m").value
        )
        goal.execute = bool(self.get_parameter("execute").value)
        self.get_logger().info(
            f"[Z_DEBUG] wood_center_z={wood_center_z:.4f} wood_top_z={wood_top_z:.4f} "
            f"grasp_tcp_offset_z={grasp_tcp_offset_z:.4f} pick_z={pick_z:.4f} "
            f"(offset_above_top={pick_z - wood_top_z:.4f}) "
            f"place_z={goal.target_place.pose.position.z:.4f}"
        )
        return goal

    @staticmethod
    def _sign(value: float) -> int:
        return 1 if value >= 0.0 else -1

    @staticmethod
    def _xy_distance(a: list[float], b: list[float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _xy_point_to_segment_distance(
        point: list[float],
        a: list[float],
        b: list[float],
    ) -> float:
        px, py = point[0], point[1]
        ax, ay = a[0], a[1]
        bx, by = b[0], b[1]
        vx, vy = bx - ax, by - ay
        denom = vx * vx + vy * vy
        if denom <= 1e-12:
            return math.hypot(px - ax, py - ay)
        t = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / denom))
        return math.hypot(px - (ax + t * vx), py - (ay + t * vy))

    def _make_double_array_param(self, name: str, values: list[float]) -> Parameter:
        param = Parameter()
        param.name = name
        param.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        param.value.double_array_value = [float(v) for v in values]
        return param

    def _set_planner_start(self, start_marker: Marker | None) -> None:
        if not bool(self.get_parameter("use_scene_start_pose").value) or start_marker is None:
            return
        timeout = float(self.get_parameter("planning_scene_timeout_sec").value)
        if not self._set_planner_params_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("Planner parameter service is not available")

        start_xyz = self._marker_xyz(start_marker)
        req = SetParameters.Request()
        req.parameters = [
            self._make_double_array_param("preposition_tcp_base", start_xyz),
            self._make_double_array_param("calibrated_start_tcp_base", start_xyz),
        ]
        future = self._set_planner_params_client.call_async(req)
        self._wait_for_future(future, "set planner start pose", timeout)
        resp = future.result()
        if resp is None:
            raise RuntimeError("Planner parameter service returned no response")
        failures = [
            result.reason or req.parameters[idx].name
            for idx, result in enumerate(resp.results)
            if not result.successful
        ]
        if failures:
            raise RuntimeError("Failed to set planner start pose: " + "; ".join(failures))
        self.get_logger().info(
            f"[CORRIDOR_SCENE] planner_start_pose=({start_xyz[0]:.4f}, "
            f"{start_xyz[1]:.4f}, {start_xyz[2]:.4f})"
        )

    def _validate_and_log_corridor(
        self,
        wood_marker: Marker,
        box_marker: Marker,
        start_marker: Marker | None,
        goal: DrlPickPlace.Goal,
    ) -> None:
        wood_xyz = self._marker_xyz(wood_marker)
        box_xyz = self._marker_xyz(box_marker)
        place_xyz = [
            float(goal.target_place.pose.position.x),
            float(goal.target_place.pose.position.y),
            float(goal.target_place.pose.position.z),
        ]
        start_xyz = self._marker_xyz(start_marker) if start_marker is not None else [0.0, 0.0, 0.0]
        wood_sign = self._sign(wood_xyz[1])
        start_ok = start_marker is None or self._sign(start_xyz[1]) == -wood_sign
        place_ok = self._sign(place_xyz[1]) == -wood_sign
        box_y_ok = abs(box_xyz[1]) <= 0.025
        min_sep = 0.06
        distances_ok = (
            self._xy_distance(box_xyz, wood_xyz) >= min_sep
            and self._xy_distance(box_xyz, place_xyz) >= min_sep
            and (start_marker is None or self._xy_distance(box_xyz, start_xyz) >= min_sep)
        )
        dist_to_nominal = self._xy_point_to_segment_distance(box_xyz, wood_xyz, place_xyz)

        self.get_logger().info(
            "[CORRIDOR_SCENE] "
            f"wood_y_sign={wood_sign:+d} "
            f"wood_pose=({wood_xyz[0]:.4f}, {wood_xyz[1]:.4f}, {wood_xyz[2]:.4f}) "
            f"box_pose=({box_xyz[0]:.4f}, {box_xyz[1]:.4f}, {box_xyz[2]:.4f}) "
            f"box_size=({box_marker.scale.x:.4f}, {box_marker.scale.y:.4f}, {box_marker.scale.z:.4f}) "
            f"start_pose=({start_xyz[0]:.4f}, {start_xyz[1]:.4f}, {start_xyz[2]:.4f}) "
            f"place_pose=({place_xyz[0]:.4f}, {place_xyz[1]:.4f}, {place_xyz[2]:.4f}) "
            f"box_to_wood_place_segment_xy={dist_to_nominal:.4f}"
        )
        self.get_logger().info(
            "[CORRIDOR_CHECK] "
            f"sign(start_y)==-sign(wood_y): {start_ok}; "
            f"sign(place_y)==-sign(wood_y): {place_ok}; "
            f"abs(box_y)<=0.025: {box_y_ok}; "
            f"min_distances_ok: {distances_ok}; "
            f"dist_box_wood={self._xy_distance(box_xyz, wood_xyz):.4f}; "
            f"dist_box_place={self._xy_distance(box_xyz, place_xyz):.4f}; "
            f"dist_box_start={self._xy_distance(box_xyz, start_xyz):.4f}"
        )
        if not (start_ok and place_ok and box_y_ok and distances_ok):
            raise RuntimeError("Corridor scene validation failed")

    def run(self) -> int:
        wood_marker, box_marker, start_marker, place_marker = self._wait_for_markers()
        self._timing("ground-truth markers ready")
        self._check_frame(wood_marker, "wood")
        self._check_frame(box_marker, "box")
        if start_marker is not None:
            self._check_frame(start_marker, "start")
        if place_marker is not None:
            self._check_frame(place_marker, "place")
        self._add_obstacle(box_marker)
        self._timing("planning scene obstacle ready")
        self._set_planner_start(start_marker)
        self._timing("planner start pose set")
        goal = self._build_goal(wood_marker, place_marker)
        self._validate_and_log_corridor(wood_marker, box_marker, start_marker, goal)

        self.get_logger().info(
            "Using Gazebo ground truth: "
            f"wood={wood_marker.text or 'pick_wood'} "
            f"pose=({wood_marker.pose.position.x:.4f}, {wood_marker.pose.position.y:.4f}, "
            f"{wood_marker.pose.position.z:.4f}) "
            f"size=({wood_marker.scale.x:.4f}, {wood_marker.scale.y:.4f}, {wood_marker.scale.z:.4f}); "
            f"obstacle={box_marker.text or 'obstacle_box'} "
            f"pose=({box_marker.pose.position.x:.4f}, {box_marker.pose.position.y:.4f}, "
            f"{box_marker.pose.position.z:.4f}) "
            f"size=({box_marker.scale.x:.4f}, {box_marker.scale.y:.4f}, {box_marker.scale.z:.4f}); "
            f"pick_z={goal.target_pick.pose.position.z:.4f} "
            f"place=({goal.target_place.pose.position.x:.4f}, "
            f"{goal.target_place.pose.position.y:.4f}, {goal.target_place.pose.position.z:.4f}) "
            f"gripper={goal.gripper_close_width_m:.4f} execute={goal.execute}"
        )

        server_timeout = float(self.get_parameter("action_server_timeout_sec").value)
        if not self._client.wait_for_server(timeout_sec=server_timeout):
            self.get_logger().error("DrlPickPlace action server not available")
            return 2
        self._timing("DrlPickPlace action server ready")

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        self._timing("DrlPickPlace goal response received")
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("DrlPickPlace goal rejected")
            return 3

        result_future = goal_handle.get_result_async()
        timeout = float(self.get_parameter("goal_timeout_sec").value)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not result_future.done():
            if time.monotonic() > deadline:
                self.get_logger().error(f"DrlPickPlace timed out after {timeout:.1f}s")
                return 4
            rclpy.spin_once(self, timeout_sec=0.1)

        wrapped = result_future.result()
        result = wrapped.result
        if result.success:
            self._timing("DrlPickPlace result success")
            self.get_logger().info(f"DrlPickPlace demo succeeded: {result.message}")
            return 0
        self._timing("DrlPickPlace result failure")
        self.get_logger().error(
            f"DrlPickPlace demo failed at {result.failed_stage}: {result.message}"
        )
        return 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DrlPickPlaceWoodBoxDemoClient()
    code = 1
    try:
        code = node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted by user")
        code = 130
    except Exception as exc:
        node.get_logger().error(str(exc))
        code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
