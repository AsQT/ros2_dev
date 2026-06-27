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
        self.declare_parameter("min_xy_separation", 0.09)
        self.declare_parameter("place_xyz", [0.46, 0.12, 0.12])
        self.declare_parameter("avoidance_margin_m", 0.02)

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._table_height = self._resolve_table_height()

        seed = int(self.get_parameter("seed").value)
        self._rng = random.Random(seed if seed != 0 else None)
        self._objects = self._make_objects()

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._marker_publishers = {
            obj.name: self.create_publisher(Marker, obj.topic, qos)
            for obj in self._objects
        }

        delay = float(self.get_parameter("startup_delay").value)
        time.sleep(max(0.0, delay))

        if bool(self.get_parameter("spawn").value):
            for obj in self._objects:
                self._spawn_object(obj)

        rate_hz = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self._timer = self.create_timer(1.0 / rate_hz, self._publish_markers)
        self._publish_markers()

    def _default_world_file(self) -> str:
        pkg_share = get_package_share_directory("robot_gazebo")
        return os.path.join(pkg_share, "worlds", "table", "arm_on_the_table.sdf")

    def _resolve_table_height(self) -> float:
        explicit = float(self.get_parameter("table_height").value)
        if explicit >= 0.0:
            return explicit

        world_file = str(self.get_parameter("world_file").value) or self._default_world_file()
        height = self._read_table_height(world_file)
        if height is None:
            height = 1.015
            self.get_logger().warn(
                f"Could not infer table height from '{world_file}', using {height:.3f} m"
            )
        else:
            self.get_logger().info(f"Inferred table height {height:.3f} m from {world_file}")
        return height

    def _read_table_height(self, world_file: str) -> float | None:
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
                    return float(pose_text[2]) + float(size_text[2]) / 2.0
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

    def _sample_valid_box_pose(
        self,
        wood_center_base: tuple[float, float, float],
        box_size: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        margin = max(0.0, float(self.get_parameter("avoidance_margin_m").value))
        min_sep = float(self.get_parameter("min_xy_separation").value)
        place_base = self._place_xyz_base()
        robot_base = (0.0, 0.0, 0.0)

        candidates: list[tuple[float, float, float]] = []
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
            if math.hypot(wood_center_base[0] - box_x, wood_center_base[1] - box_y) < min_sep:
                continue
            if not self._point_clear_of_box(wood_center_base, box_center_base, box_size, margin):
                continue
            if not self._point_clear_of_box(place_base, box_center_base, box_size, margin):
                continue
            if not self._point_clear_of_box(robot_base, box_center_base, box_size, margin):
                continue
            return box_x, box_y, box_yaw

        raise RuntimeError(
            "Could not sample obstacle_box pose clear of wood, place target, and robot base"
        )

    def _make_objects(self) -> list[SimBox]:
        wood_size = self._vector3_param("wood_size")
        box_size = self._sample_box_size()
        wood_x, wood_y, wood_yaw = self._sample_xy("wood")
        wood_center_world = (wood_x, wood_y, self._table_height + wood_size[2] / 2.0)
        wood_center_base = (
            wood_x,
            wood_y,
            wood_center_world[2] - float(self.get_parameter("robot_base_world_z").value),
        )
        box_x, box_y, box_yaw = self._sample_valid_box_pose(wood_center_base, box_size)

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
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            self.get_logger().error(
                f"Spawn {obj.name} failed: {(result.stderr or result.stdout).strip()}"
            )
        else:
            self.get_logger().info(f"Spawn {obj.name} OK")

    def _publish_markers(self) -> None:
        for idx, obj in enumerate(self._objects):
            marker = self._marker_for_object(obj, idx)
            self._marker_publishers[obj.name].publish(marker)

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
