#!/usr/bin/env python3
import os
import math
import random
import subprocess
import time

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class RandomSpawner(Node):
    def __init__(self):
        super().__init__('random_wood_spawner')

        self.declare_parameter('count', 10)
        self.declare_parameter('seed', 0)

        self.declare_parameter('x_min', 0.35)
        self.declare_parameter('x_max', 0.5)
        self.declare_parameter('y_min',-0.30)
        self.declare_parameter('y_max', 0.30)
        self.declare_parameter('z',     1.25)
        self.declare_parameter('yaw_min', 0.0)
        self.declare_parameter('yaw_max', 2.0 * math.pi)
        self.declare_parameter('yaw_step_deg', 0.0)

        self.declare_parameter('startup_delay', 4.0)

        pkg_share     = get_package_share_directory('robot_description')
        self.sdf_path = os.path.join(pkg_share, 'worlds', 'wood_block', 'wood_model.sdf')

        if not os.path.exists(self.sdf_path):
            self.get_logger().error(f"LOI: Khong tim thay file SDF: {self.sdf_path}")
            return

        delay = float(self.get_parameter('startup_delay').value)
        time.sleep(max(0.0, delay))

        self.spawn_random_objects()

    def spawn_random_objects(self):
        count = int(self.get_parameter('count').value)
        seed  = int(self.get_parameter('seed').value)

        if seed != 0:
            random.seed(seed)

        x_min        = float(self.get_parameter('x_min').value)
        x_max        = float(self.get_parameter('x_max').value)
        y_min        = float(self.get_parameter('y_min').value)
        y_max        = float(self.get_parameter('y_max').value)
        z            = float(self.get_parameter('z').value)
        yaw_min      = float(self.get_parameter('yaw_min').value)
        yaw_max      = float(self.get_parameter('yaw_max').value)
        yaw_step_deg = float(self.get_parameter('yaw_step_deg').value)

        face_poses   = [  
                        ( math.pi/2,  math.pi/2), 
                        ( math.pi/2, -math.pi/2), 
                        (-math.pi/2,  math.pi/2), 
                        (-math.pi/2, -math.pi/2),      ]

        self.get_logger().info(f"--- Bat dau tao {count} khoi go ---")
        self.get_logger().info(f"SDF: {self.sdf_path}")

        for i in range(count):
            name        = f"wood_{i+1}"
            x           = random.uniform(x_min, x_max)
            y           = random.uniform(y_min, y_max)

            roll, pitch = random.choice(face_poses)

            if yaw_step_deg and yaw_step_deg > 0.0:
                step = math.radians(yaw_step_deg)
                n    = max(1, int((yaw_max - yaw_min) / step))
                yaw  = yaw_min + step * random.randint(0, n)
            else:
                yaw  = random.uniform(yaw_min, yaw_max)

            self.spawn_via_command(name, x, y, z, roll, pitch, yaw)
            time.sleep(0.15)

    def spawn_via_command(self, name, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', name,
            '-x', str(x), '-y', str(y), '-z', str(z),
            '-R', str(roll), '-P', str(pitch), '-Y', str(yaw),
            '-file', self.sdf_path,
            '-allow_renaming', 'true'
        ]

        self.get_logger().info(
            f"Spawn {name} @ x={x:.3f}, y={y:.3f}, z={z:.3f} | RPY=({roll:.2f},{pitch:.2f},{yaw:.2f})"
        )

        try:
            result               = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode == 0:
                self.get_logger().info(f"-> OK: {name}")
            else:
                self.get_logger().error(f"-> LOI spawn {name}: {result.stderr.strip()}")
        except Exception as e:
            self.get_logger().error(f"Exception spawn {name}: {e}")

def main():
    rclpy.init()
    node = RandomSpawner()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()