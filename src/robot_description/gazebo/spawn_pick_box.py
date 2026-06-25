#!/usr/bin/env python3
import math
import os
import random
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker


class PickBoxSpawner(Node):
    def __init__(self):
        super().__init__("pick_box_spawner")

        self.declare_parameter("object_name", "pick_box")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("info_topic", "/sim/pick_box_info")
        self.declare_parameter("robot_base_world_z", 1.02)
        self.declare_parameter("box_size", 0.03)
        self.declare_parameter("table_height", -1.0)
        self.declare_parameter("world_file", "")
        self.declare_parameter("x", 0.42)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("randomize", False)
        self.declare_parameter("seed", 0)
        self.declare_parameter("x_min", 0.35)
        self.declare_parameter("x_max", 0.50)
        self.declare_parameter("y_min", -0.12)
        self.declare_parameter("y_max", 0.12)
        self.declare_parameter("startup_delay", 3.0)
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("spawn", True)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._publisher = self.create_publisher(
            Marker,
            str(self.get_parameter("info_topic").value),
            qos,
        )

        self._object_name = str(self.get_parameter("object_name").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._size = float(self.get_parameter("box_size").value)
        self._table_height = self._resolve_table_height()
        self._pose = self._choose_pose()
        self._sdf_path = self._sdf_path_for_size(self._size)

        delay = float(self.get_parameter("startup_delay").value)
        time.sleep(max(0.0, delay))

        if bool(self.get_parameter("spawn").value):
            self._spawn_box()

        rate_hz = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self._timer = self.create_timer(1.0 / rate_hz, self._publish_marker)
        self._publish_marker()

    def _default_world_file(self) -> str:
        pkg_share = get_package_share_directory("robot_description")
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

    def _choose_pose(self) -> tuple[float, float, float, float]:
        seed = int(self.get_parameter("seed").value)
        if seed != 0:
            random.seed(seed)

        if bool(self.get_parameter("randomize").value):
            x = random.uniform(
                float(self.get_parameter("x_min").value),
                float(self.get_parameter("x_max").value),
            )
            y = random.uniform(
                float(self.get_parameter("y_min").value),
                float(self.get_parameter("y_max").value),
            )
            yaw = random.uniform(-math.pi, math.pi)
        else:
            x = float(self.get_parameter("x").value)
            y = float(self.get_parameter("y").value)
            yaw = float(self.get_parameter("yaw").value)

        z = self._table_height + self._size / 2.0
        return x, y, z, yaw

    def _sdf_path_for_size(self, size: float) -> str:
        pkg_share = get_package_share_directory("robot_description")
        fixed_path = os.path.join(
            pkg_share,
            "worlds",
            "pick_box_3cm",
            "pick_box_3cm.sdf",
        )
        if abs(size - 0.03) < 1e-9 and os.path.exists(fixed_path):
            return fixed_path

        content = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="pick_box">
    <static>false</static>
    <link name="link">
      <inertial><mass>0.03</mass></inertial>
      <collision name="collision"><geometry><box><size>{size} {size} {size}</size></box></geometry></collision>
      <visual name="visual"><geometry><box><size>{size} {size} {size}</size></box></geometry></visual>
    </link>
  </model>
</sdf>
"""
        tmp = tempfile.NamedTemporaryFile("w", suffix=".sdf", delete=False)
        tmp.write(content)
        tmp.close()
        return tmp.name

    def _spawn_box(self) -> None:
        x, y, z, yaw = self._pose
        cmd = [
            "ros2",
            "run",
            "ros_gz_sim",
            "create",
            "-name",
            self._object_name,
            "-x",
            str(x),
            "-y",
            str(y),
            "-z",
            str(z),
            "-Y",
            str(yaw),
            "-file",
            self._sdf_path,
            "-allow_renaming",
            "false",
        ]
        self.get_logger().info(
            f"Spawn {self._object_name} size={self._size:.3f} at "
            f"x={x:.3f}, y={y:.3f}, z={z:.3f}, yaw={yaw:.3f}"
        )
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            self.get_logger().error(f"Spawn failed: {result.stderr.strip()}")
        else:
            self.get_logger().info("Spawn OK")

    def _publish_marker(self) -> None:
        x, y, z, yaw = self._pose
        marker_z = z
        if self._frame_id == "base_link":
            marker_z = z - float(self.get_parameter("robot_base_world_z").value)

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self._frame_id
        marker.ns = "sim_pick_box"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = marker_z
        marker.pose.orientation.z = math.sin(yaw * 0.5)
        marker.pose.orientation.w = math.cos(yaw * 0.5)
        marker.scale.x = self._size
        marker.scale.y = self._size
        marker.scale.z = self._size
        marker.color.r = 0.95
        marker.color.g = 0.78
        marker.color.b = 0.22
        marker.color.a = 1.0
        marker.text = self._object_name
        self._publisher.publish(marker)


def main():
    rclpy.init()
    node = PickBoxSpawner()
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
