#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker

from robot_task_manager.action import DrlPickPlace


class DrlPickPlaceBoxDemoClient(Node):
    def __init__(self) -> None:
        super().__init__("drl_pick_place_box_demo_client")
        self.declare_parameter("object_info_topic", "/sim/pick_box_info")
        self.declare_parameter("action_name", "drl_pickplace")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("place_xyz", [0.34, -0.10, 0.035])
        self.declare_parameter("pick_z_offset_m", 0.0)
        self.declare_parameter("place_z_offset_m", 0.0)
        self.declare_parameter("min_pick_z_m", 0.025)
        self.declare_parameter("gripper_close_width_m", 0.025)
        self.declare_parameter("object_timeout_sec", 60.0)
        self.declare_parameter("action_server_timeout_sec", 120.0)
        self.declare_parameter("goal_timeout_sec", 420.0)

        self._latest_marker: Marker | None = None
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._sub = self.create_subscription(
            Marker,
            str(self.get_parameter("object_info_topic").value),
            self._on_marker,
            qos,
        )
        self._client = ActionClient(
            self,
            DrlPickPlace,
            str(self.get_parameter("action_name").value),
        )

    def _on_marker(self, msg: Marker) -> None:
        if msg.type != Marker.CUBE:
            return
        if msg.scale.x <= 0.0 or msg.scale.y <= 0.0 or msg.scale.z <= 0.0:
            return
        self._latest_marker = msg

    def _wait_for_marker(self) -> Marker:
        timeout = float(self.get_parameter("object_timeout_sec").value)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and self._latest_marker is None:
            if time.monotonic() > deadline:
                raise TimeoutError(
                    f"Timed out waiting for {self.get_parameter('object_info_topic').value}"
                )
            rclpy.spin_once(self, timeout_sec=0.1)
        assert self._latest_marker is not None
        return self._latest_marker

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

    def _build_goal(self, marker: Marker) -> DrlPickPlace.Goal:
        frame_id = str(self.get_parameter("frame_id").value)
        marker_frame = marker.header.frame_id or frame_id
        if marker_frame != frame_id:
            raise RuntimeError(
                f"Object info frame '{marker_frame}' does not match action frame '{frame_id}'. "
                "Launch spawn_pick_box with frame_id:=base_link or provide matching frames."
            )

        pick_z = (
            float(marker.pose.position.z)
            + float(self.get_parameter("pick_z_offset_m").value)
        )
        pick_z = max(pick_z, float(self.get_parameter("min_pick_z_m").value))

        place_xyz = [
            float(v) for v in self.get_parameter("place_xyz").value
        ]
        if len(place_xyz) != 3 or any(not math.isfinite(v) for v in place_xyz):
            raise ValueError("place_xyz must contain exactly 3 finite numbers")
        place_xyz[2] += float(self.get_parameter("place_z_offset_m").value)

        goal = DrlPickPlace.Goal()
        goal.target_pick = self._pose_from_xyz(
            [
                float(marker.pose.position.x),
                float(marker.pose.position.y),
                pick_z,
            ],
            frame_id,
        )
        goal.target_place = self._pose_from_xyz(place_xyz, frame_id)
        goal.gripper_close_width_m = float(
            self.get_parameter("gripper_close_width_m").value
        )
        self.get_logger().info(
            "Using sim perception: "
            f"name='{marker.text or 'pick_box'}' "
            f"pose=({marker.pose.position.x:.4f}, {marker.pose.position.y:.4f}, {marker.pose.position.z:.4f}) "
            f"size=({marker.scale.x:.4f}, {marker.scale.y:.4f}, {marker.scale.z:.4f}) "
            f"pick_z={pick_z:.4f} gripper={goal.gripper_close_width_m:.4f}"
        )
        return goal

    def run(self) -> int:
        marker = self._wait_for_marker()
        goal = self._build_goal(marker)

        server_timeout = float(self.get_parameter("action_server_timeout_sec").value)
        if not self._client.wait_for_server(timeout_sec=server_timeout):
            self.get_logger().error("DrlPickPlace action server not available")
            return 2

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
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
            self.get_logger().info(f"DrlPickPlace demo succeeded: {result.message}")
            return 0
        self.get_logger().error(
            f"DrlPickPlace demo failed at {result.failed_stage}: {result.message}"
        )
        return 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DrlPickPlaceBoxDemoClient()
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
