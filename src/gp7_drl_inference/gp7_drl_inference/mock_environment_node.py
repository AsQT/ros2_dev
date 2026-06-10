"""Mock environment node — publishes synthetic target/object data for simulation testing.

Replaces the vision pipeline during development so the DRL inference node can be
tested in a closed loop without hardware or a RealSense camera.

Published topics:
  /detected_object/pose    geometry_msgs/PoseStamped  — synthetic target pose
  /vision/box_detection    gp7_vision_pipeline/BoxDetection  — synthetic bbox

Supports two object classes matching the YOLO model:
  box     — the primary manipulation target
  target  — alternative class (switch via target_class_name param)

Parameters:
  publish_rate_hz    — publish rate in Hz (default 10.0)
  target_class_name — YOLO class name to publish ("box" or "target", default "box")
  target_x/y/z      — world-frame position of the target object
  frame_id          — TF frame ID for published poses (default "world")
  distance_m        — fake depth from camera in BoxDetection (default 0.5)
  bbox_width_px     — fake bbox width in pixels (default 200)
  bbox_height_px    — fake bbox height in pixels (default 200)
  confidence        — fake YOLO confidence score (default 0.95)

Usage:
  ros2 run gp7_drl_inference mock_environment_node

  # Simulate a "target" class instead of "box"
  ros2 run gp7_drl_inference mock_environment_node --ros-args \
      -p target_class_name:=target

  # Custom position
  ros2 run gp7_drl_inference mock_environment_node --ros-args \
      -p target_x:=0.2 -p target_y:=-0.3 -p target_z:=0.4 \
      -p target_class_name:=box
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from gp7_vision_pipeline.msg import BoxDetection


class MockEnvironmentNode(Node):

    def __init__(self) -> None:
        super().__init__("mock_environment_node")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        self._target_pub = self.create_publisher(
            PoseStamped, "/detected_object/pose", qos_profile=qos
        )
        self._box_pub = self.create_publisher(
            BoxDetection, "/vision/box_detection", qos_profile=qos
        )

        rate = float(self.declare_parameter("publish_rate_hz", 10.0).value)
        self._period_s = 1.0 / rate if rate > 0 else 0.1

        self._target_class_name = str(
            self.declare_parameter("target_class_name", "box").value
        ).lower()
        if self._target_class_name not in ("box", "target"):
            self.get_logger().warn(
                f"Unknown target_class_name '{self._target_class_name}' "
                "— falling back to 'box'. Valid values: box, target"
            )
            self._target_class_name = "box"

        self._target_x = float(self.declare_parameter("target_x", 0.0).value)
        self._target_y = float(self.declare_parameter("target_y", -0.3).value)
        self._target_z = float(self.declare_parameter("target_z", 0.4).value)

        self._frame_id = str(self.declare_parameter("frame_id", "world").value)

        self._distance_m = float(self.declare_parameter("distance_m", 0.5).value)
        self._bbox_w = int(self.declare_parameter("bbox_width_px", 200).value)
        self._bbox_h = int(self.declare_parameter("bbox_height_px", 200).value)
        self._confidence = float(self.declare_parameter("confidence", 0.95).value)

        self.get_logger().info(
            f"Mock environment  |  rate={rate} Hz  "
            f"|  class={self._target_class_name}  "
            f"|  pos=({self._target_x}, {self._target_y}, {self._target_z})  "
            f"|  frame_id={self._frame_id}"
        )

        self._timer = self.create_timer(self._period_s, self._publish)

    def _publish(self) -> None:
        now = self.get_clock().now().to_msg()

        target = PoseStamped()
        target.header = Header(stamp=now, frame_id=self._frame_id)
        target.pose.position.x = self._target_x
        target.pose.position.y = self._target_y
        target.pose.position.z = self._target_z
        target.pose.orientation.w = 1.0
        self._target_pub.publish(target)

        box = BoxDetection()
        box.header = Header(stamp=now, frame_id=self._frame_id)
        box.class_name = self._target_class_name
        box.confidence = self._confidence
        box.x_min = 320 - self._bbox_w // 2
        box.y_min = 240 - self._bbox_h // 2
        box.x_max = 320 + self._bbox_w // 2
        box.y_max = 240 + self._bbox_h // 2
        box.center_x_px = 320
        box.center_y_px = 240
        box.width_px = self._bbox_w
        box.height_px = self._bbox_h
        box.distance_m = self._distance_m
        self._box_pub.publish(box)


def main(argv=None):
    rclpy.init(args=argv)
    try:
        node = MockEnvironmentNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
