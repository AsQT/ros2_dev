#!/usr/bin/env python3
import os
import math
import subprocess
import time

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class FixedOnTableSpawner(Node):
    def __init__(self):
        super().__init__("fixed_wood_spawner_on_table")

        # --- Params ---
        self.declare_parameter("startup_delay", 2.0)

        # Độ cao mặt bàn trong world
        self.declare_parameter("table_z", 1.10)

        # Chiều cao khối gỗ (m).
        self.declare_parameter("block_height", 0.05)

        # Nhích lên 1 chút để tránh cấn mặt bàn
        self.declare_parameter("z_epsilon", 0.005)

        pkg_share = get_package_share_directory("robot_description")
        self.sdf_path = os.path.join(pkg_share, "worlds", "wood_block", "wood_model.sdf")

        if not os.path.exists(self.sdf_path):
            self.get_logger().error(f"LOI: Khong tim thay file SDF: {self.sdf_path}")
            return

        time.sleep(max(0.0, float(self.get_parameter("startup_delay").value)))
        self.spawn_3_blocks_neat_for_pick_place()

    def spawn_3_blocks_neat_for_pick_place(self):

        table_z      = float(self.get_parameter("table_z").value)
        block_h      = float(self.get_parameter("block_height").value)
        z_eps        = float(self.get_parameter("z_epsilon").value)

        z_on_table = table_z + block_h * 0.5 + z_eps

        # 3 vị trí cố định 
        # (name, x, y, yaw_deg)
        targets = [
            ("wood_1", 0.30, 0.10, 0.0),
            ("wood_2", 0.45, -0.10, 0.0),
            ("wood_3", 0.60, -0.10, 0.0),
        ]
        self.get_logger().info("--- Spawn 3 khoi go tren MAT BAN  ---")
        self.get_logger().info(f"SDF: {self.sdf_path}")
        self.get_logger().info(f"table_z={table_z:.3f}, block_height={block_h:.3f} => z_on_table={z_on_table:.3f}")

        for name, x, y, yaw_deg in targets:
            roll  = 0.0
            pitch = 0.0
            yaw   = math.radians(yaw_deg)

            self.spawn_via_command(name, x, y, z_on_table, roll, pitch, yaw)
            time.sleep(0.10)

    def spawn_via_command(self, name, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        cmd = [
            "ros2", "run", "ros_gz_sim", "create",
            "-name", name,
            "-x", str(x), "-y", str(y), "-z", str(z),
            "-R", str(roll), "-P", str(pitch), "-Y", str(yaw),
            "-file", self.sdf_path,
            "-allow_renaming", "true",
        ]

        self.get_logger().info(
            f"Spawn {name} @ (x={x:.3f}, y={y:.3f}, z={z:.3f}) | RPY=({roll:.2f},{pitch:.2f},{yaw:.2f})"
        )

        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode == 0:
            self.get_logger().info(f"-> OK: {name}")
        else:
            self.get_logger().error(f"-> LOI spawn {name}: {result.stderr.strip()}")


def main():
    rclpy.init()
    node = FixedOnTableSpawner()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
