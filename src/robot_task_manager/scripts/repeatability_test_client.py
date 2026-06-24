#!/usr/bin/env python3
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node

from robot_task_manager.action import RepeatabilityTest


class RepeatabilityTestClient(Node):
    def __init__(self) -> None:
        super().__init__("repeatability_test_client")
        self.declare_parameter("action_name", "repeatability_test")
        self.declare_parameter("axis", int(RepeatabilityTest.Goal.AXIS_X))
        self.declare_parameter("repeat_count", 3)
        self.declare_parameter("meas_offset", 0.02)
        self.declare_parameter("velocity_scale", 0.25)
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("goal_timeout_sec", 600.0)

        self._client = ActionClient(
            self,
            RepeatabilityTest,
            str(self.get_parameter("action_name").value),
        )

    def _pose(self, x: float, y: float, z: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = str(self.get_parameter("frame_id").value)
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = 0.7071068
        pose.pose.orientation.y = 0.7071068
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0
        return pose

    def send_goal(self) -> bool:
        timeout = float(self.get_parameter("goal_timeout_sec").value)
        if not self._client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("RepeatabilityTest action server not available")
            return False

        goal = RepeatabilityTest.Goal()
        goal.retract_pose = self._pose(0.40, 0.00, 0.18)
        goal.disturb_pose_1 = self._pose(0.35, -0.08, 0.18)
        goal.disturb_pose_2 = self._pose(0.45, 0.08, 0.18)
        goal.axis = int(self.get_parameter("axis").value)
        goal.meas_offset = float(self.get_parameter("meas_offset").value)
        goal.repeat_count = int(self.get_parameter("repeat_count").value)
        goal.velocity_scale = float(self.get_parameter("velocity_scale").value)

        self.get_logger().info(
            "Sending RepeatabilityTest goal: "
            f"axis={goal.axis} repeat_count={goal.repeat_count} "
            f"offset={goal.meas_offset:.3f} velocity={goal.velocity_scale:.2f}"
        )

        send_future = self._client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        self._wait_for_future(send_future, "send goal", timeout)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("RepeatabilityTest goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        self._wait_for_future(result_future, "result", timeout)
        result = result_future.result()
        if result is None:
            self.get_logger().error("RepeatabilityTest result is None")
            return False

        self.get_logger().info(f"Result code: {result.status}")
        self.get_logger().info(
            f"success={result.result.success} "
            f"completed_count={result.result.completed_count} "
            f"message={result.result.message}"
        )
        return bool(result.result.success)

    def _feedback_callback(self, msg) -> None:
        feedback = msg.feedback
        self.get_logger().info(
            f"Feedback: loop={feedback.current_index} "
            f"step={feedback.current_step}"
        )

    def _wait_for_future(self, future, label: str, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                raise TimeoutError(f"{label} timed out after {timeout:.1f}s")
            rclpy.spin_once(self, timeout_sec=0.05)


def main() -> None:
    rclpy.init()
    node = RepeatabilityTestClient()
    ok = False
    try:
        ok = node.send_goal()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
