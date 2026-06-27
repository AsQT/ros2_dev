#!/usr/bin/env python3
import math
import os
import random
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker


@dataclass(frozen=True)
class SimBox:
    name: str
    role: str
    topic: str
    ns: str
    center_world: tuple[float, float, float]
    size: tuple[float, float, float]
    yaw: float
    color: tuple[float, float, float, float]
    sdf_path: str


class PickWoodObstacleBoxSpawner(Node):
    def __init__(self) -> None:
        super().__init__("pick_wood_obstacle_box_spawner")

        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("robot_base_world_z", 1.02)
        self.declare_parameter("table_height", -1.0)
        self.declare_parameter("world_file", "")
        self.declare_parameter("startup_delay", 3.0)
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("spawn", True)
        self.declare_parameter("randomize", False)
        self.declare_parameter("corridor_randomization", False)
        self.declare_parameter("seed", 0)

        self.declare_parameter("wood_name", "pick_wood")
        self.declare_parameter("wood_info_topic", "/sim/pick_wood_info")
        self.declare_parameter("wood_size", [0.03, 0.03, 0.03])
        self.declare_parameter("wood_x", 0.44)
        self.declare_parameter("wood_y", 0.06)
        self.declare_parameter("wood_yaw", 0.0)
        self.declare_parameter("wood_x_min", 0.38)
        self.declare_parameter("wood_x_max", 0.48)
        self.declare_parameter("wood_y_min", 0.02)
        self.declare_parameter("wood_y_max", 0.12)
        self.declare_parameter("wood_edge_margin_m", 0.08)

        self.declare_parameter("box_name", "obstacle_box")
        self.declare_parameter("box_info_topic", "/sim/obstacle_box_info")
        self.declare_parameter("box_size", [0.05, 0.05, 0.06])
        self.declare_parameter("randomize_box_size", True)
        self.declare_parameter("box_size_min", [0.05, 0.05, 0.05])
        self.declare_parameter("box_size_max", [0.15, 0.15, 0.15])
        self.declare_parameter("box_x", 0.34)
        self.declare_parameter("box_y", -0.09)
        self.declare_parameter("box_yaw", 0.0)
        self.declare_parameter("box_x_min", 0.34)
        self.declare_parameter("box_x_max", 0.43)
        self.declare_parameter("box_y_min", -0.08)
        self.declare_parameter("box_y_max", -0.05)
        self.declare_parameter("min_xy_separation", 0.06)
        self.declare_parameter("place_xyz", [0.46, 0.12, 0.12])
        self.declare_parameter("place_info_topic", "/sim/place_pose_info")
        self.declare_parameter("place_x_min", 0.43)
        self.declare_parameter("place_x_max", 0.48)
        self.declare_parameter("place_y_abs_min", 0.08)
        self.declare_parameter("place_y_abs_max", 0.15)
        self.declare_parameter("start_info_topic", "/sim/start_pose_info")
        self.declare_parameter("start_xyz", [0.375, 0.0, 0.25])
        self.declare_parameter("start_x_min", 0.35)
        self.declare_parameter("start_x_max", 0.40)
        self.declare_parameter("start_y_abs_min", 0.08)
        self.declare_parameter("start_y_abs_max", 0.13)
        self.declare_parameter("box_y_center_min", -0.025)
        self.declare_parameter("box_y_center_max", 0.025)
        self.declare_parameter("box_midpoint_noise_x", 0.015)
        self.declare_parameter("avoidance_margin_m", 0.02)
        self.declare_parameter("gz_world_name", "arm_and_table")

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._node_t0 = time.monotonic()
        self._table_height, self._table_xy_region = self._resolve_table_surface()

        seed = int(self.get_parameter("seed").value)
        self._rng = random.Random(seed if seed != 0 else None)
        self._wood_y_sign = 1
        self._start_pose_base = self._start_xyz_base()
        self._place_pose_base = self._place_xyz_base()
        self._objects = self._make_objects()
        self._log_scene_layout()
        self._log_z_audit()
        self._log_timing("scene sampled")

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._marker_publishers = {
            obj.name: self.create_publisher(Marker, obj.topic, qos)
            for obj in self._objects
        }
        self._start_pose_pub = self.create_publisher(
            Marker, str(self.get_parameter("start_info_topic").value), qos
        )
        self._place_pose_pub = self.create_publisher(
            Marker, str(self.get_parameter("place_info_topic").value), qos
        )

        delay = float(self.get_parameter("startup_delay").value)
        if delay > 0.0:
            self.get_logger().info(f"[timing] fixed startup_delay sleep: {delay:.3f}s")
            time.sleep(delay)
        self._log_timing("startup delay complete")

        if bool(self.get_parameter("spawn").value):
            spawn_t0 = time.monotonic()
            for obj in self._objects:
                self._spawn_object(obj)
            self.get_logger().info(
                f"[timing] scene spawned: {time.monotonic() - spawn_t0:.3f}s"
            )

        rate_hz = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self._timer = self.create_timer(1.0 / rate_hz, self._publish_markers)
        self._publish_markers()

        self._respawn_srv = self.create_service(
            Trigger, "/sim/respawn_objects", self._on_respawn
        )

    def _default_world_file(self) -> str:
        pkg_share = get_package_share_directory("robot_gazebo")
        return os.path.join(pkg_share, "worlds", "table", "arm_on_the_table.sdf")

    def _log_timing(self, phase: str) -> None:
        self.get_logger().info(
            f"[timing] {phase}: {time.monotonic() - self._node_t0:.3f}s since spawner start"
        )

    def _resolve_table_surface(
        self,
    ) -> tuple[float, tuple[float, float, float, float]]:
        explicit = float(self.get_parameter("table_height").value)
        world_file = str(self.get_parameter("world_file").value) or self._default_world_file()
        surface = self._read_table_surface(world_file)
        if surface is None:
            region = (-0.75, 0.75, -0.40, 0.40)
            self.get_logger().warn(
                f"Could not infer table XY bounds from '{world_file}', using fallback "
                f"region x=[{region[0]:.3f}, {region[1]:.3f}] "
                f"y=[{region[2]:.3f}, {region[3]:.3f}]"
            )
        else:
            _, region = surface

        if explicit >= 0.0:
            return explicit, region

        if surface is None:
            height = 1.015
            self.get_logger().warn(
                f"Could not infer table height from '{world_file}', using {height:.3f} m"
            )
        else:
            height = surface[0]
            self.get_logger().info(f"Inferred table height {height:.3f} m from {world_file}")
        self.get_logger().info(
            "[WOOD_EDGE] "
            f"region_x_min={region[0]:.4f} region_x_max={region[1]:.4f} "
            f"region_y_min={region[2]:.4f} region_y_max={region[3]:.4f} "
            f"wood_edge_margin_m={float(self.get_parameter('wood_edge_margin_m').value):.4f}"
        )
        return height, region

    def _read_table_surface(
        self,
        world_file: str,
    ) -> tuple[float, tuple[float, float, float, float]] | None:
        try:
            root = ET.parse(world_file).getroot()
        except Exception as exc:
            self.get_logger().warn(f"Failed to parse world file '{world_file}': {exc}")
            return None

        for model in root.findall(".//model"):
            if model.get("name") != "table":
                continue
            for elem in model.findall(".//collision") + model.findall(".//visual"):
                if elem.get("name") not in ("surface", "surface_visual"):
                    continue
                pose_text = elem.findtext("pose", default="0 0 0 0 0 0").split()
                size_text = elem.findtext(".//box/size", default="").split()
                if len(pose_text) >= 3 and len(size_text) >= 3:
                    cx = float(pose_text[0])
                    cy = float(pose_text[1])
                    cz = float(pose_text[2])
                    sx = float(size_text[0])
                    sy = float(size_text[1])
                    sz = float(size_text[2])
                    height = cz + sz / 2.0
                    region = (
                        cx - sx / 2.0,
                        cx + sx / 2.0,
                        cy - sy / 2.0,
                        cy + sy / 2.0,
                    )
                    return height, region
        return None

    def _vector3_param(self, name: str) -> tuple[float, float, float]:
        values = [float(v) for v in self.get_parameter(name).value]
        if len(values) != 3 or any((not math.isfinite(v) or v <= 0.0) for v in values):
            raise ValueError(f"{name} must contain exactly 3 positive finite numbers")
        return (values[0], values[1], values[2])

    def _sample_box_size(self) -> tuple[float, float, float]:
        if not bool(self.get_parameter("randomize_box_size").value):
            return self._vector3_param("box_size")

        size_min = self._vector3_param("box_size_min")
        size_max = self._vector3_param("box_size_max")
        for low, high in zip(size_min, size_max):
            if low < 0.05 or high > 0.15 or low > high:
                raise ValueError(
                    "box_size_min/max must satisfy 0.05 <= min <= max <= 0.15 for every axis"
                )
        return tuple(
            self._rng.uniform(low, high)
            for low, high in zip(size_min, size_max)
        )

    @staticmethod
    def _sign(value: float) -> int:
        return 1 if value >= 0.0 else -1

    @staticmethod
    def _xy_distance(
        a: tuple[float, float, float],
        b: tuple[float, float, float],
    ) -> float:
        return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))

    @staticmethod
    def _xy_point_to_segment_distance(
        point: tuple[float, float, float],
        a: tuple[float, float, float],
        b: tuple[float, float, float],
    ) -> float:
        px, py = float(point[0]), float(point[1])
        ax, ay = float(a[0]), float(a[1])
        bx, by = float(b[0]), float(b[1])
        vx, vy = bx - ax, by - ay
        denom = vx * vx + vy * vy
        if denom <= 1e-12:
            return math.hypot(px - ax, py - ay)
        t = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / denom))
        cx, cy = ax + t * vx, ay + t * vy
        return math.hypot(px - cx, py - cy)

    def _sample_xy(
        self,
        prefix: str,
    ) -> tuple[float, float, float]:
        if bool(self.get_parameter("randomize").value):
            return self._sample_random_xy(prefix)
        return (
            float(self.get_parameter(f"{prefix}_x").value),
            float(self.get_parameter(f"{prefix}_y").value),
            float(self.get_parameter(f"{prefix}_yaw").value),
        )

    def _sample_random_xy(self, prefix: str) -> tuple[float, float, float]:
        x = self._rng.uniform(
            float(self.get_parameter(f"{prefix}_x_min").value),
            float(self.get_parameter(f"{prefix}_x_max").value),
        )
        y = self._rng.uniform(
            float(self.get_parameter(f"{prefix}_y_min").value),
            float(self.get_parameter(f"{prefix}_y_max").value),
        )
        yaw = self._rng.uniform(-math.pi, math.pi)
        return x, y, yaw

    def _point_clear_of_box(
        self,
        point: tuple[float, float, float],
        center: tuple[float, float, float],
        size: tuple[float, float, float],
        margin: float,
    ) -> bool:
        return any(
            abs(float(p) - float(c)) > (float(s) / 2.0 + margin)
            for p, c, s in zip(point, center, size)
        )

    def _place_xyz_base(self) -> tuple[float, float, float]:
        values = [float(v) for v in self.get_parameter("place_xyz").value]
        if len(values) != 3:
            raise ValueError("place_xyz must contain exactly 3 numbers")
        return (values[0], values[1], values[2])

    def _start_xyz_base(self) -> tuple[float, float, float]:
        values = [float(v) for v in self.get_parameter("start_xyz").value]
        if len(values) != 3:
            raise ValueError("start_xyz must contain exactly 3 numbers")
        return (values[0], values[1], values[2])

    def _wood_edge_distances(
        self,
        wood_base_xy: tuple[float, float],
        wood_size: tuple[float, float, float],
    ) -> tuple[float, float, float, float]:
        region_x_min, region_x_max, region_y_min, region_y_max = self._table_xy_region
        wood_x, wood_y = wood_base_xy
        return (
            wood_x - wood_size[0] / 2.0 - region_x_min,
            region_x_max - (wood_x + wood_size[0] / 2.0),
            wood_y - wood_size[1] / 2.0 - region_y_min,
            region_y_max - (wood_y + wood_size[1] / 2.0),
        )

    def _wood_edge_ok(
        self,
        wood_base_xy: tuple[float, float],
        wood_size: tuple[float, float, float],
        log_rejection: bool = False,
    ) -> bool:
        margin = float(self.get_parameter("wood_edge_margin_m").value)
        distances = self._wood_edge_distances(wood_base_xy, wood_size)
        ok = all(distance >= margin for distance in distances)
        if not ok and log_rejection:
            self.get_logger().warn(
                "Rejected wood pose: too close to edge | "
                f"wood_pose=({wood_base_xy[0]:.4f}, {wood_base_xy[1]:.4f}) "
                f"wood_size=({wood_size[0]:.4f}, {wood_size[1]:.4f}, {wood_size[2]:.4f}) "
                f"region_x_min={self._table_xy_region[0]:.4f} "
                f"region_x_max={self._table_xy_region[1]:.4f} "
                f"region_y_min={self._table_xy_region[2]:.4f} "
                f"region_y_max={self._table_xy_region[3]:.4f} "
                f"wood_edge_distance_x_min={distances[0]:.4f} "
                f"wood_edge_distance_x_max={distances[1]:.4f} "
                f"wood_edge_distance_y_min={distances[2]:.4f} "
                f"wood_edge_distance_y_max={distances[3]:.4f} "
                f"required={margin:.4f}"
            )
        return ok

    def _sample_corridor_endpoints(
        self,
        wood_size: tuple[float, float, float],
    ) -> tuple[tuple[float, float, float], float, tuple[float, float, float], tuple[float, float, float]]:
        for attempt in range(100):
            sign = self._rng.choice([-1, 1])
            wood_x = self._rng.uniform(
                float(self.get_parameter("wood_x_min").value),
                float(self.get_parameter("wood_x_max").value),
            )
            wood_y = sign * self._rng.uniform(0.08, 0.13)
            if self._wood_edge_ok(
                (wood_x, wood_y),
                wood_size,
                log_rejection=(attempt < 5),
            ):
                break
        else:
            raise RuntimeError(
                "Could not sample wood pose satisfying Y corridor and edge margin. "
                f"region={self._table_xy_region} margin="
                f"{float(self.get_parameter('wood_edge_margin_m').value):.3f}"
            )
        wood_yaw = self._rng.uniform(-math.pi, math.pi)

        start_pose = (
            self._rng.uniform(
                float(self.get_parameter("start_x_min").value),
                float(self.get_parameter("start_x_max").value),
            ),
            -sign * self._rng.uniform(
                float(self.get_parameter("start_y_abs_min").value),
                float(self.get_parameter("start_y_abs_max").value),
            ),
            self._start_xyz_base()[2],
        )
        place_fallback = self._place_xyz_base()
        place_pose = (
            self._rng.uniform(
                float(self.get_parameter("place_x_min").value),
                float(self.get_parameter("place_x_max").value),
            ),
            -sign * self._rng.uniform(
                float(self.get_parameter("place_y_abs_min").value),
                float(self.get_parameter("place_y_abs_max").value),
            ),
            place_fallback[2],
        )
        wood_center_world = (wood_x, wood_y, self._table_height + wood_size[2] / 2.0)
        return wood_center_world, wood_yaw, start_pose, place_pose

    def _sample_valid_box_pose(
        self,
        wood_center_base: tuple[float, float, float],
        box_size: tuple[float, float, float],
        start_base: tuple[float, float, float],
        place_base: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        min_sep = float(self.get_parameter("min_xy_separation").value)
        robot_base = (0.0, 0.0, 0.0)
        randomize = bool(self.get_parameter("randomize").value)
        corridor = bool(self.get_parameter("corridor_randomization").value)

        candidates: list[tuple[float, float, float]] = []
        if randomize and corridor:
            mid_x = (wood_center_base[0] + place_base[0]) * 0.5
            noise = max(0.0, float(self.get_parameter("box_midpoint_noise_x").value))
            box_x_min = float(self.get_parameter("box_x_min").value)
            box_x_max = float(self.get_parameter("box_x_max").value)
            box_y_min = float(self.get_parameter("box_y_center_min").value)
            box_y_max = float(self.get_parameter("box_y_center_max").value)
            for _ in range(300):
                candidates.append((
                    min(max(mid_x + self._rng.uniform(-noise, noise), box_x_min), box_x_max),
                    self._rng.uniform(box_y_min, box_y_max),
                    float(self.get_parameter("box_yaw").value),
                ))
        else:
            candidates.append((
                float(self.get_parameter("box_x").value),
                float(self.get_parameter("box_y").value),
                float(self.get_parameter("box_yaw").value),
            ))
            for _ in range(200):
                candidates.append(self._sample_random_xy("box"))

        box_z_base = self._table_height + box_size[2] / 2.0 - float(
            self.get_parameter("robot_base_world_z").value
        )
        for box_x, box_y, box_yaw in candidates:
            box_center_base = (box_x, box_y, box_z_base)
            if self._xy_distance(wood_center_base, box_center_base) < min_sep:
                continue
            if self._xy_distance(place_base, box_center_base) < min_sep:
                continue
            if self._xy_distance(start_base, box_center_base) < min_sep:
                continue
            if self._xy_distance(robot_base, box_center_base) < min_sep:
                continue
            return box_x, box_y, box_yaw

        raise RuntimeError(
            "Could not sample obstacle_box pose clear of wood, place target, and robot base"
        )

    def _make_objects(self) -> list[SimBox]:
        randomize = bool(self.get_parameter("randomize").value)
        corridor = bool(self.get_parameter("corridor_randomization").value)
        wood_size = self._vector3_param("wood_size")

        for _ in range(500):
            box_size = self._sample_box_size()
            if randomize and corridor:
                wood_center_world, wood_yaw, start_pose, place_pose = (
                    self._sample_corridor_endpoints(wood_size)
                )
            else:
                wood_x, wood_y, wood_yaw = self._sample_xy("wood")
                if not self._wood_edge_ok((wood_x, wood_y), wood_size, log_rejection=True):
                    continue
                wood_center_world = (
                    wood_x,
                    wood_y,
                    self._table_height + wood_size[2] / 2.0,
                )
                start_pose = self._start_xyz_base()
                place_pose = self._place_xyz_base()

            wood_center_base = (
                wood_center_world[0],
                wood_center_world[1],
                wood_center_world[2] - float(self.get_parameter("robot_base_world_z").value),
            )
            try:
                box_x, box_y, box_yaw = self._sample_valid_box_pose(
                    wood_center_base, box_size, start_pose, place_pose
                )
            except RuntimeError:
                continue
            box_center_base = (
                box_x,
                box_y,
                self._table_height + box_size[2] / 2.0 -
                float(self.get_parameter("robot_base_world_z").value),
            )
            if corridor and not self._scene_rules_ok(
                wood_center_base, box_center_base, start_pose, place_pose
            ):
                continue
            self._wood_y_sign = self._sign(wood_center_base[1])
            self._start_pose_base = start_pose
            self._place_pose_base = place_pose
            break
        else:
            raise RuntimeError("Could not sample a valid pick/place obstacle corridor scene")

        wood = SimBox(
            name=str(self.get_parameter("wood_name").value),
            role="pick_object",
            topic=str(self.get_parameter("wood_info_topic").value),
            ns="sim_pick_wood",
            center_world=wood_center_world,
            size=wood_size,
            yaw=wood_yaw,
            color=(0.55, 0.32, 0.12, 1.0),
            sdf_path=self._make_box_sdf("pick_wood", wood_size, (0.55, 0.32, 0.12, 1.0)),
        )
        obstacle = SimBox(
            name=str(self.get_parameter("box_name").value),
            role="obstacle",
            topic=str(self.get_parameter("box_info_topic").value),
            ns="sim_obstacle_box",
            center_world=(box_x, box_y, self._table_height + box_size[2] / 2.0),
            size=box_size,
            yaw=box_yaw,
            color=(0.95, 0.95, 0.95, 1.0),
            sdf_path=self._make_box_sdf("obstacle_box", box_size, (0.95, 0.95, 0.95, 1.0)),
        )
        self.get_logger().info(
            "Sampled obstacle_box size=(%.3f, %.3f, %.3f) in required [0.050, 0.150] m range"
            % obstacle.size
        )
        return [wood, obstacle]

    def _scene_rules_ok(
        self,
        wood_base: tuple[float, float, float],
        box_base: tuple[float, float, float],
        start_base: tuple[float, float, float],
        place_base: tuple[float, float, float],
    ) -> bool:
        min_sep = float(self.get_parameter("min_xy_separation").value)
        wood_sign = self._sign(wood_base[1])
        return (
            self._sign(start_base[1]) == -wood_sign
            and self._sign(place_base[1]) == -wood_sign
            and abs(box_base[1]) <= 0.025
            and self._xy_distance(box_base, wood_base) >= min_sep
            and self._xy_distance(box_base, place_base) >= min_sep
            and self._xy_distance(box_base, start_base) >= min_sep
        )

    def _log_scene_layout(self) -> None:
        wood = next(obj for obj in self._objects if obj.role == "pick_object")
        box = next(obj for obj in self._objects if obj.role == "obstacle")
        base_world_z = float(self.get_parameter("robot_base_world_z").value)
        wood_base = (
            wood.center_world[0],
            wood.center_world[1],
            wood.center_world[2] - base_world_z,
        )
        box_base = (
            box.center_world[0],
            box.center_world[1],
            box.center_world[2] - base_world_z,
        )
        dist_to_nominal = self._xy_point_to_segment_distance(
            box_base, wood_base, self._place_pose_base
        )
        self.get_logger().info(
            "[CORRIDOR_SCENE] "
            f"wood_y_sign={self._wood_y_sign:+d} "
            f"wood_pose=({wood_base[0]:.4f}, {wood_base[1]:.4f}, {wood_base[2]:.4f}) "
            f"box_pose=({box_base[0]:.4f}, {box_base[1]:.4f}, {box_base[2]:.4f}) "
            f"box_size=({box.size[0]:.4f}, {box.size[1]:.4f}, {box.size[2]:.4f}) "
            f"start_pose=({self._start_pose_base[0]:.4f}, {self._start_pose_base[1]:.4f}, "
            f"{self._start_pose_base[2]:.4f}) "
            f"place_pose=({self._place_pose_base[0]:.4f}, {self._place_pose_base[1]:.4f}, "
            f"{self._place_pose_base[2]:.4f}) "
            f"box_to_wood_place_segment_xy={dist_to_nominal:.4f}"
        )
        self.get_logger().info(
            "[CORRIDOR_CHECK] "
            f"start_opposite_y={self._sign(self._start_pose_base[1]) == -self._wood_y_sign} "
            f"place_opposite_y={self._sign(self._place_pose_base[1]) == -self._wood_y_sign} "
            f"box_y_centered={abs(box_base[1]) <= 0.025} "
            f"dist_box_wood={self._xy_distance(box_base, wood_base):.4f} "
            f"dist_box_place={self._xy_distance(box_base, self._place_pose_base):.4f} "
            f"dist_box_start={self._xy_distance(box_base, self._start_pose_base):.4f}"
        )
        edge = self._wood_edge_distances((wood_base[0], wood_base[1]), wood.size)
        self.get_logger().info(
            "[WOOD_EDGE] "
            f"wood_pose=({wood_base[0]:.4f}, {wood_base[1]:.4f}, {wood_base[2]:.4f}) "
            f"wood_size=({wood.size[0]:.4f}, {wood.size[1]:.4f}, {wood.size[2]:.4f}) "
            f"region_x_min={self._table_xy_region[0]:.4f} "
            f"region_x_max={self._table_xy_region[1]:.4f} "
            f"region_y_min={self._table_xy_region[2]:.4f} "
            f"region_y_max={self._table_xy_region[3]:.4f} "
            f"wood_edge_distance_x_min={edge[0]:.4f} "
            f"wood_edge_distance_x_max={edge[1]:.4f} "
            f"wood_edge_distance_y_min={edge[2]:.4f} "
            f"wood_edge_distance_y_max={edge[3]:.4f}"
        )

    def _log_z_audit(self) -> None:
        base_world_z = float(self.get_parameter("robot_base_world_z").value)
        self.get_logger().info(
            f"[Z_DEBUG] table_top_z = {self._table_height:.4f}"
        )
        self.get_logger().info(
            f"[Z_DEBUG] robot_base_world_z = {base_world_z:.4f}"
        )
        self.get_logger().info(
            f"[Z_AUDIT][spawner] table_top_world_z={self._table_height:.4f} "
            f"robot_base_world_z={base_world_z:.4f} "
            f"model_origin=box_center marker_frame={self._frame_id}"
        )
        for obj in self._objects:
            center_world_z = obj.center_world[2]
            top_world_z = center_world_z + obj.size[2] / 2.0
            bottom_world_z = center_world_z - obj.size[2] / 2.0
            center_base_z = center_world_z - base_world_z
            top_base_z = top_world_z - base_world_z
            bottom_base_z = bottom_world_z - base_world_z
            if obj.role == "pick_object":
                self.get_logger().info(
                    f"[Z_DEBUG] wood_height = {obj.size[2]:.4f}"
                )
                self.get_logger().info(
                    f"[Z_DEBUG] wood_pose_world.z = {center_world_z:.4f}"
                )
                self.get_logger().info(
                    f"[Z_DEBUG] before_transform_world_z = {center_world_z:.4f}"
                )
                self.get_logger().info(
                    f"[Z_DEBUG] after_transform_base_z = {center_base_z:.4f}"
                )
                self.get_logger().info(
                    f"[Z_DEBUG] transform_delta_z = {center_base_z - center_world_z:.4f}"
                )
                self.get_logger().info(
                    f"[Z_DEBUG] wood_pose_base.z = {center_base_z:.4f}"
                )
                self.get_logger().info(
                    f"[Z_DEBUG] expected_wood_top_z = {top_world_z:.4f}"
                )
                self.get_logger().info(
                    f"[Z_DEBUG] measured_expected_z = {center_world_z:.4f}"
                )
                self.get_logger().info(
                    f"[Z_DEBUG] current_used_z = {base_world_z:.4f}"
                )
                self.get_logger().info(
                    f"[Z_DEBUG] z_error = {center_world_z - base_world_z:.4f}"
                )
            self.get_logger().info(
                f"[Z_AUDIT][spawner] {obj.name} role={obj.role} size_z={obj.size[2]:.4f} "
                f"world_z bottom/center/top=({bottom_world_z:.4f} "
                f"{center_world_z:.4f} {top_world_z:.4f}) "
                f"base_z bottom/center/top=({bottom_base_z:.4f} "
                f"{center_base_z:.4f} {top_base_z:.4f})"
            )

    def _make_box_sdf(
        self,
        model_name: str,
        size: tuple[float, float, float],
        color: tuple[float, float, float, float],
    ) -> str:
        sx, sy, sz = size
        r, g, b, a = color
        content = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{model_name}">
    <static>false</static>
    <link name="link">
      <inertial><mass>0.05</mass></inertial>
      <collision name="collision">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} {a}</ambient>
          <diffuse>{r} {g} {b} {a}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""
        tmp = tempfile.NamedTemporaryFile("w", suffix=".sdf", delete=False)
        tmp.write(content)
        tmp.close()
        return tmp.name

    def _spawn_object(self, obj: SimBox) -> None:
        x, y, z = obj.center_world
        cmd = [
            "ros2",
            "run",
            "ros_gz_sim",
            "create",
            "-name",
            obj.name,
            "-x",
            str(x),
            "-y",
            str(y),
            "-z",
            str(z),
            "-Y",
            str(obj.yaw),
            "-file",
            obj.sdf_path,
            "-allow_renaming",
            "false",
        ]
        self.get_logger().info(
            f"Spawn {obj.name} role={obj.role} size={obj.size} "
            f"world=({x:.3f}, {y:.3f}, {z:.3f}) yaw={obj.yaw:.3f}"
        )
        t0 = time.monotonic()
        result = subprocess.run(cmd, capture_output=True, text=True)
        elapsed = time.monotonic() - t0
        if result.returncode != 0:
            self.get_logger().error(
                f"Spawn {obj.name} failed: {(result.stderr or result.stdout).strip()}"
            )
        else:
            self.get_logger().info(f"Spawn {obj.name} OK")
        self.get_logger().info(f"[timing] spawn {obj.name}: {elapsed:.3f}s")

    def _delete_object(self, name: str) -> None:
        gz_world = str(self.get_parameter("gz_world_name").value)
        req_str = f'name: "{name}" type: 2'
        cmd = [
            "gz", "service",
            "-s", f"/world/{gz_world}/remove",
            "--reqtype", "gz.msgs.Entity",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000",
            "--req", req_str,
        ]
        self.get_logger().info(f"Deleting '{name}' from Gazebo world '{gz_world}'")
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10.0)
            if result.returncode != 0:
                self.get_logger().warn(
                    f"Delete '{name}' returned non-zero: "
                    f"{(result.stderr or result.stdout).strip()}"
                )
            else:
                self.get_logger().info(f"Delete '{name}' OK")
        except Exception as exc:
            self.get_logger().warn(f"Delete '{name}' exception: {exc}")

    def _on_respawn(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self.get_logger().info("[respawn] Respawning objects with new random poses...")
        try:
            for obj in self._objects:
                self._delete_object(obj.name)
                try:
                    os.unlink(obj.sdf_path)
                except Exception:
                    pass
            self._objects = self._make_objects()
            self._log_scene_layout()
            self._log_z_audit()
            spawn_t0 = time.monotonic()
            for obj in self._objects:
                self._spawn_object(obj)
            self.get_logger().info(
                f"[timing] respawn scene spawned: {time.monotonic() - spawn_t0:.3f}s"
            )
            self._publish_markers()
            response.success = True
            response.message = f"Respawned {len(self._objects)} objects"
            self.get_logger().info(f"[respawn] {response.message}")
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(f"[respawn] Failed: {exc}")
        return response

    def _publish_markers(self) -> None:
        for idx, obj in enumerate(self._objects):
            marker = self._marker_for_object(obj, idx)
            self._marker_publishers[obj.name].publish(marker)
        self._start_pose_pub.publish(
            self._pose_marker("start_pose", "sim_start_pose", 100, self._start_pose_base)
        )
        self._place_pose_pub.publish(
            self._pose_marker("place_pose", "sim_place_pose", 101, self._place_pose_base)
        )

    def _pose_marker(
        self,
        name: str,
        ns: str,
        idx: int,
        xyz: tuple[float, float, float],
    ) -> Marker:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self._frame_id
        marker.ns = ns
        marker.id = idx
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(xyz[0])
        marker.pose.position.y = float(xyz[1])
        marker.pose.position.z = float(xyz[2])
        marker.pose.orientation.x = 0.7071068
        marker.pose.orientation.y = 0.7071068
        marker.pose.orientation.w = 0.0
        marker.scale.x = 0.025
        marker.scale.y = 0.025
        marker.scale.z = 0.025
        if name == "start_pose":
            marker.color.r = 0.1
            marker.color.g = 0.4
            marker.color.b = 1.0
        else:
            marker.color.r = 0.1
            marker.color.g = 0.9
            marker.color.b = 0.3
        marker.color.a = 1.0
        marker.text = name
        return marker

    def _marker_for_object(self, obj: SimBox, idx: int) -> Marker:
        x, y, z_world = obj.center_world
        z = z_world
        if self._frame_id == "base_link":
            z -= float(self.get_parameter("robot_base_world_z").value)

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self._frame_id
        marker.ns = obj.ns
        marker.id = idx
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.z = math.sin(obj.yaw * 0.5)
        marker.pose.orientation.w = math.cos(obj.yaw * 0.5)
        marker.scale.x = obj.size[0]
        marker.scale.y = obj.size[1]
        marker.scale.z = obj.size[2]
        marker.color.r = obj.color[0]
        marker.color.g = obj.color[1]
        marker.color.b = obj.color[2]
        marker.color.a = obj.color[3]
        marker.text = f"{obj.name};role={obj.role}"
        return marker


def main() -> None:
    rclpy.init()
    node = PickWoodObstacleBoxSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
