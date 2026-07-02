"""Publish RViz MarkerArray for wood and box detections.

Subscribes to:
  /vision/wood_objects    robot_vision_pipeline_msgs/WoodArray
  /vision/box_objects   robot_vision_pipeline_msgs/BoxArray

Publishes:
  /vision/object_markers   visualization_msgs/MarkerArray

Marker design (codex2.md sections 5/6):
  wood: green CUBE, 30x30x30 mm by default (Wood.msg has no size field),
        pose.orientation taken from wood.pose.orientation.
  box:  red/orange CUBE, scale from box.size, box.pose.orientation.
Both wood and box markers are dropped once their source detection is older
than their *_detection_timeout_sec parameter, so a lost detection cannot
leave a stale marker on screen (codex2.md section 5/15).
"""

from __future__ import annotations

import threading
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from visualization_msgs.msg import Marker, MarkerArray

from robot_vision_pipeline_msgs.msg import WoodArray, BoxArray


class VisionDetectionMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_detection_marker_node")

        self._data_lock = threading.Lock()
        self._latest_wood_arr: Optional[WoodArray] = None
        self._latest_box_arr: Optional[BoxArray] = None

        self.declare_parameter("marker_frame_id", "camera_color_optical_frame")
        self.declare_parameter("marker_publish_rate_hz", 5.0)
        self.declare_parameter("marker_lifetime_sec", 1.0)

        # codex2.md section 6: wood target cube is fixed at 30x30x30 mm by
        # default (Wood.msg carries no size field), still overridable via
        # params/yaml if a different fixed size is ever needed.
        self.declare_parameter("wood_marker_size_x_m", 0.03)
        self.declare_parameter("wood_marker_size_y_m", 0.03)
        self.declare_parameter("wood_marker_size_z_m", 0.03)

        self.declare_parameter("box_marker_size_x_m", 0.08)
        self.declare_parameter("box_marker_size_y_m", 0.08)
        self.declare_parameter("box_marker_size_z_m", 0.05)

        self.declare_parameter("wood_detection_timeout_sec", 1.0)
        self.declare_parameter("wood_confidence_threshold", 0.5)
        self.declare_parameter("box_detection_timeout_sec", 1.0)

        self._marker_frame_id = str(self.get_parameter("marker_frame_id").value)
        rate_hz = float(self.get_parameter("marker_publish_rate_hz").value)
        self._marker_lifetime_sec = float(self.get_parameter("marker_lifetime_sec").value)

        self._wood_size_x = float(self.get_parameter("wood_marker_size_x_m").value)
        self._wood_size_y = float(self.get_parameter("wood_marker_size_y_m").value)
        self._wood_size_z = float(self.get_parameter("wood_marker_size_z_m").value)

        self._default_box_x = float(self.get_parameter("box_marker_size_x_m").value)
        self._default_box_y = float(self.get_parameter("box_marker_size_y_m").value)
        self._default_box_z = float(self.get_parameter("box_marker_size_z_m").value)

        self._wood_timeout = float(self.get_parameter("wood_detection_timeout_sec").value)
        self._wood_conf_thresh = float(self.get_parameter("wood_confidence_threshold").value)
        self._box_timeout = float(self.get_parameter("box_detection_timeout_sec").value)

        det_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub_markers = self.create_publisher(MarkerArray, "/vision/object_markers", 10)

        self.create_subscription(
            WoodArray, "/vision/wood_objects", self._on_wood_array, det_qos
        )
        self.create_subscription(
            BoxArray, "/vision/box_objects", self._on_box_array, det_qos
        )

        period_sec = 1.0 / rate_hz if rate_hz > 0 else 0.2
        self._publish_timer = self.create_timer(period_sec, self._publish_markers)

        # Tracks whether a box marker was shown last cycle so the "cleared"
        # message is logged only on the have-box -> no-box transition.
        self._had_box_marker = False

        self.get_logger().info(
            f"vision_detection_marker_node started | "
            f"frame={self._marker_frame_id}, "
            f"wood_size=({self._wood_size_x},{self._wood_size_y},{self._wood_size_z}), "
            f"default_box=({self._default_box_x},{self._default_box_y},{self._default_box_z})"
        )

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_wood_array(self, msg: WoodArray) -> None:
        with self._data_lock:
            self._latest_wood_arr = msg

    def _on_box_array(self, msg: BoxArray) -> None:
        with self._data_lock:
            self._latest_box_arr = msg

    def _make_delete_all(self) -> Marker:
        m = Marker()
        m.action = Marker.DELETEALL
        return m

    def _set_lifetime(self, m: Marker) -> None:
        lifetime_sec = max(self._marker_lifetime_sec, 0.0)
        m.lifetime.sec = int(lifetime_sec)
        m.lifetime.nanosec = int((lifetime_sec % 1.0) * 1e9)

    def _make_wood_marker(
        self, marker_id: int, frame_id: str, stamp_sec: float,
        pose,
    ) -> Marker:
        m = Marker()
        m.header.stamp.sec = int(stamp_sec)
        m.header.stamp.nanosec = int((stamp_sec % 1.0) * 1e9)
        m.header.frame_id = frame_id
        m.ns = "wood"
        # Stable ns/id (codex.md section 7.1): reuse the same id each cycle so
        # RViz UPDATES the marker instead of stacking ghosts. DELETEALL below
        # still clears any surplus id when the object count drops.
        m.id = marker_id
        m.type = Marker.CUBE
        m.action = Marker.ADD
        # codex2.md section 6: use both position and orientation from the
        # wood detection so the cube rotates with the detected yaw.
        m.pose = pose
        m.scale.x = self._wood_size_x
        m.scale.y = self._wood_size_y
        m.scale.z = self._wood_size_z
        m.color.r = 0.0
        m.color.g = 0.8
        m.color.b = 0.2
        m.color.a = 0.7
        self._set_lifetime(m)
        return m

    def _make_box_marker(
        self, marker_id: int, frame_id: str, stamp_sec: float,
        pose,
        sx: float, sy: float, sz: float,
    ) -> Marker:
        if sx <= 0.0:
            sx = self._default_box_x
        if sy <= 0.0:
            sy = self._default_box_y
        if sz <= 0.0:
            sz = self._default_box_z

        m = Marker()
        m.header.stamp.sec = int(stamp_sec)
        m.header.stamp.nanosec = int((stamp_sec % 1.0) * 1e9)
        m.header.frame_id = frame_id
        m.ns = "box"
        # Stable ns/id (codex.md section 7.1): box uses ns="box" starting at
        # id=0, reused each cycle so a moved box updates in place (no ghost).
        m.id = marker_id
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose = pose
        m.scale.x = sx
        m.scale.y = sy
        m.scale.z = sz
        # codex2.md section 5: obstacle box in red/orange, alpha 0.4-0.7.
        m.color.r = 1.0
        m.color.g = 0.35
        m.color.b = 0.0
        m.color.a = 0.55
        self._set_lifetime(m)
        return m

    def _publish_markers(self) -> None:
        now = self._now_seconds()

        with self._data_lock:
            wood_arr = self._latest_wood_arr
            box_arr = self._latest_box_arr

        arr = MarkerArray()
        # DELETEALL first every cycle (codex.md section 7.2) so any surplus id
        # from a previous cycle (e.g. box count dropped) is cleared before the
        # current markers are added.
        arr.markers.append(self._make_delete_all())

        if wood_arr is not None:
            frame = wood_arr.header.frame_id or self._marker_frame_id
            stamp = float(wood_arr.header.stamp.sec) + float(
                wood_arr.header.stamp.nanosec
            ) * 1e-9
            wood_marker_id = 0
            for wood in wood_arr.woods:
                if wood.confidence < self._wood_conf_thresh:
                    continue
                if (now - stamp) > self._wood_timeout:
                    continue
                arr.markers.append(
                    self._make_wood_marker(
                        wood_marker_id, frame, stamp, wood.pose,
                    )
                )
                wood_marker_id += 1

        box_marker_count = 0
        if box_arr is not None:
            frame = box_arr.header.frame_id or self._marker_frame_id
            stamp = float(box_arr.header.stamp.sec) + float(
                box_arr.header.stamp.nanosec
            ) * 1e-9
            # codex2.md section 5: drop stale box detections so a lost
            # obstacle does not keep showing a marker forever.
            if (now - stamp) <= self._box_timeout:
                for box in box_arr.boxes:
                    arr.markers.append(
                        self._make_box_marker(
                            box_marker_count, frame, stamp, box.pose,
                            box.size.x,
                            box.size.y,
                            box.size.z,
                        )
                    )
                    self.get_logger().info(
                        "[vision_marker] latest box marker: "
                        f"id={box_marker_count} "
                        f"center=({box.pose.position.x:.3f},{box.pose.position.y:.3f},"
                        f"{box.pose.position.z:.3f}) "
                        f"size=({box.size.x:.3f},{box.size.y:.3f},{box.size.z:.3f}) "
                        f"stamp={stamp:.3f} action=ADD",
                        throttle_duration_sec=1.0,
                    )
                    box_marker_count += 1

        if box_marker_count == 0 and self._had_box_marker:
            # Transition to no fresh box: DELETEALL already cleared the previous
            # box marker; log once so the clear is visible.
            self.get_logger().info(
                "[vision_detection_marker_node] no valid latest box; clearing box marker"
            )
        self._had_box_marker = box_marker_count > 0

        if arr.markers:
            self._pub_markers.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionDetectionMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()