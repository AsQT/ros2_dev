#!/usr/bin/env python3
"""
Reachability scan node for ROS 2 + MoveIt 2.

This node samples XYZ positions and roll/pitch orientations, calls MoveIt
/compute_ik and optionally /check_state_validity, then publishes reachable
points as MarkerArray and saves CSV results.

Typical use:
  ros2 launch robot_reachability reachability_scan.launch.py
"""

import csv
import math
from typing import Iterable, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from visualization_msgs.msg import Marker, MarkerArray


def frange(start: float, stop: float, step: float) -> Iterable[float]:
    """Inclusive float range with rounding to reduce floating-point noise."""
    if step <= 0.0:
        raise ValueError("step must be > 0")

    value = float(start)
    stop = float(stop)
    step = float(step)

    while value <= stop + 1e-9:
        yield round(value, 10)
        value += step


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convert roll, pitch, yaw in radians to quaternion x, y, z, w."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw


class ReachabilityScanNode(Node):
    def __init__(self) -> None:
        super().__init__("reachability_scan_node")

        # Core MoveIt parameters
        self.declare_parameter("base_frame", "world")
        self.declare_parameter("group_name", "arm")
        self.declare_parameter("ik_link_name", "tcp_link")
        self.declare_parameter("compute_ik_service", "/compute_ik")
        self.declare_parameter("state_validity_service", "/check_state_validity")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("require_joint_state_seed", True)
        self.declare_parameter("joint_state_wait_sec", 5.0)

        # XYZ workspace scan range in meters
        self.declare_parameter("x_min", 0.20)
        self.declare_parameter("x_max", 0.75)
        self.declare_parameter("y_min", -0.35)
        self.declare_parameter("y_max", 0.35)
        self.declare_parameter("z_min", 0.05)
        self.declare_parameter("z_max", 0.55)
        self.declare_parameter("xyz_step", 0.05)

        # Orientation scan range in degrees.
        # roll is rotation around X, pitch is rotation around Y, yaw is fixed by default.
        self.declare_parameter("roll_min_deg", -45.0)
        self.declare_parameter("roll_max_deg", 45.0)
        self.declare_parameter("pitch_min_deg", -45.0)
        self.declare_parameter("pitch_max_deg", 45.0)
        self.declare_parameter("angle_step_deg", 15.0)
        self.declare_parameter("yaw_deg", 0.0)

        # Check approach pose above each tested point. Useful for pick/place.
        self.declare_parameter("check_approach", False)
        self.declare_parameter("approach_offset_z", 0.10)

        # Collision/state validity check
        self.declare_parameter("avoid_collisions_in_ik", True)
        self.declare_parameter("check_state_validity", True)
        self.declare_parameter("ik_timeout_sec", 0.05)

        # Output and visualization
        self.declare_parameter("csv_path", "/tmp/reachability_map.csv")
        self.declare_parameter("marker_topic", "/reachability_markers")
        self.declare_parameter("marker_scale", 0.025)
        self.declare_parameter("publish_unreachable", False)
        self.declare_parameter("unreachable_stride", 1)

        self.base_frame = self.get_parameter("base_frame").value
        self.group_name = self.get_parameter("group_name").value
        self.ik_link_name = self.get_parameter("ik_link_name").value
        self.compute_ik_service = self.get_parameter("compute_ik_service").value
        self.state_validity_service = self.get_parameter("state_validity_service").value
        self.joint_state_topic = self.get_parameter("joint_state_topic").value
        self.last_joint_state: Optional[JointState] = None

        marker_topic = self.get_parameter("marker_topic").value
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, qos)
        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            20,
        )

        self.ik_client = self.create_client(GetPositionIK, self.compute_ik_service)
        self.validity_client = self.create_client(GetStateValidity, self.state_validity_service)

        self.started = False
        self.timer = self.create_timer(1.0, self._start_once)

        self.get_logger().info("ReachabilityScanNode started")
        self.get_logger().info(f"IK service             : {self.compute_ik_service}")
        self.get_logger().info(f"State validity service : {self.state_validity_service}")
        self.get_logger().info(f"Base frame             : {self.base_frame}")
        self.get_logger().info(f"Group / IK link         : {self.group_name} / {self.ik_link_name}")
        self.get_logger().info(f"JointState topic        : {self.joint_state_topic}")

    def joint_state_callback(self, msg: JointState) -> None:
        # Ignore invalid empty JointState messages. MoveIt needs at least joint names and positions
        # as a seed for stable IK requests.
        if not msg.name or not msg.position:
            self.get_logger().warn(
                "Received empty /joint_states message. Check which node is publishing it with: "
                "ros2 topic info /joint_states -v",
                throttle_duration_sec=5.0,
            )
            return
        self.last_joint_state = msg

    def wait_for_joint_state_seed(self) -> bool:
        if not self._param_bool("require_joint_state_seed"):
            return True

        wait_sec = max(0.0, self._param_float("joint_state_wait_sec"))
        deadline = self.get_clock().now() + Duration(seconds=wait_sec)
        while rclpy.ok() and self.last_joint_state is None and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        if self.last_joint_state is None:
            self.get_logger().error(
                "No non-empty JointState received. MoveIt IK needs current joint names/positions as seed. "
                "Check /joint_states and joint_state_broadcaster."
            )
            return False

        self.get_logger().info(
            f"Using JointState seed with {len(self.last_joint_state.name)} joints: "
            f"{list(self.last_joint_state.name)}"
        )
        return True

    def _param_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _param_bool(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)

    def _param_int(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _start_once(self) -> None:
        if self.started:
            return
        self.started = True
        self.timer.cancel()

        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f"Service {self.compute_ik_service} not available. Is move_group running?")
            return

        if self._param_bool("check_state_validity"):
            if not self.validity_client.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(
                    f"Service {self.state_validity_service} not available. "
                    "Set check_state_validity:=false or start move_group."
                )
                return

        if not self.wait_for_joint_state_seed():
            return

        self.scan_workspace()

    def make_pose(self, x: float, y: float, z: float, roll_deg: float, pitch_deg: float, yaw_deg: float) -> PoseStamped:
        qx, qy, qz, qw = rpy_to_quaternion(
            math.radians(roll_deg),
            math.radians(pitch_deg),
            math.radians(yaw_deg),
        )

        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        return pose

    def compute_ik(self, pose: PoseStamped) -> Tuple[bool, Optional[RobotState], int]:
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ik_link_name
        req.ik_request.pose_stamped = pose
        req.ik_request.avoid_collisions = self._param_bool("avoid_collisions_in_ik")

        # Seed IK with the latest real/current joint state. Without this, MoveIt may log
        # "Found empty JointState message" and IK can be unstable or slow.
        if self.last_joint_state is not None:
            req.ik_request.robot_state.joint_state = self.last_joint_state
            req.ik_request.robot_state.is_diff = False
        else:
            req.ik_request.robot_state.is_diff = True

        timeout_sec = max(0.001, self._param_float("ik_timeout_sec"))
        req.ik_request.timeout = Duration(seconds=timeout_sec).to_msg()

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            return False, None, MoveItErrorCodes.FAILURE

        result = future.result()
        ok = result.error_code.val == MoveItErrorCodes.SUCCESS
        return ok, result.solution if ok else None, result.error_code.val

    def check_state_validity(self, robot_state: RobotState) -> bool:
        if not self._param_bool("check_state_validity"):
            return True

        req = GetStateValidity.Request()
        req.robot_state = robot_state
        req.group_name = self.group_name

        future = self.validity_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            return False

        return bool(future.result().valid)

    def pose_is_reachable(self, x: float, y: float, z: float, roll_deg: float, pitch_deg: float, yaw_deg: float) -> bool:
        target_pose = self.make_pose(x, y, z, roll_deg, pitch_deg, yaw_deg)
        ok_ik, solution, _ = self.compute_ik(target_pose)
        if not ok_ik or solution is None:
            return False
        if not self.check_state_validity(solution):
            return False

        if self._param_bool("check_approach"):
            approach_z = z + self._param_float("approach_offset_z")
            approach_pose = self.make_pose(x, y, approach_z, roll_deg, pitch_deg, yaw_deg)
            ok_ik_approach, solution_approach, _ = self.compute_ik(approach_pose)
            if not ok_ik_approach or solution_approach is None:
                return False
            if not self.check_state_validity(solution_approach):
                return False

        return True

    def make_marker(self, marker_id: int, x: float, y: float, z: float, reachable: bool, score: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "reachable_points" if reachable else "unreachable_points"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.pose.orientation.w = 1.0

        scale = self._param_float("marker_scale")
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        if reachable:
            # More valid orientations => more saturated green.
            marker.color.r = 0.0
            marker.color.g = min(1.0, 0.35 + 0.03 * float(score))
            marker.color.b = 0.0
            marker.color.a = 0.85
        else:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.25

        return marker

    def scan_workspace(self) -> None:
        x_min = self._param_float("x_min")
        x_max = self._param_float("x_max")
        y_min = self._param_float("y_min")
        y_max = self._param_float("y_max")
        z_min = self._param_float("z_min")
        z_max = self._param_float("z_max")
        xyz_step = self._param_float("xyz_step")

        roll_min = self._param_float("roll_min_deg")
        roll_max = self._param_float("roll_max_deg")
        pitch_min = self._param_float("pitch_min_deg")
        pitch_max = self._param_float("pitch_max_deg")
        angle_step = self._param_float("angle_step_deg")
        yaw_deg = self._param_float("yaw_deg")

        csv_path = self.get_parameter("csv_path").value
        publish_unreachable = self._param_bool("publish_unreachable")
        unreachable_stride = max(1, self._param_int("unreachable_stride"))

        self.get_logger().info("Start reachability scan")
        self.get_logger().info(
            f"XYZ range: x=[{x_min}, {x_max}], y=[{y_min}, {y_max}], z=[{z_min}, {z_max}], step={xyz_step}"
        )
        self.get_logger().info(
            f"Orientation: roll=[{roll_min}, {roll_max}], pitch=[{pitch_min}, {pitch_max}], "
            f"yaw={yaw_deg}, angle_step={angle_step} deg"
        )

        reachable_rows: List[List[object]] = []
        markers = MarkerArray()

        marker_id = 0
        total_xyz = 0
        reachable_xyz = 0
        total_orientation_tests = 0

        for x in frange(x_min, x_max, xyz_step):
            self.get_logger().info(f"Scanning x={x:.3f} ...")
            for y in frange(y_min, y_max, xyz_step):
                for z in frange(z_min, z_max, xyz_step):
                    total_xyz += 1

                    valid_pairs: List[str] = []
                    best_roll: Optional[float] = None
                    best_pitch: Optional[float] = None

                    for roll_deg in frange(roll_min, roll_max, angle_step):
                        for pitch_deg in frange(pitch_min, pitch_max, angle_step):
                            total_orientation_tests += 1
                            if self.pose_is_reachable(x, y, z, roll_deg, pitch_deg, yaw_deg):
                                valid_pairs.append(f"{roll_deg:.1f}:{pitch_deg:.1f}")
                                if best_roll is None:
                                    best_roll = roll_deg
                                    best_pitch = pitch_deg

                    valid_count = len(valid_pairs)
                    if valid_count > 0:
                        reachable_xyz += 1
                        reachable_rows.append([
                            f"{x:.5f}",
                            f"{y:.5f}",
                            f"{z:.5f}",
                            valid_count,
                            "" if best_roll is None else f"{best_roll:.2f}",
                            "" if best_pitch is None else f"{best_pitch:.2f}",
                            f"{yaw_deg:.2f}",
                            ";".join(valid_pairs),
                        ])
                        markers.markers.append(self.make_marker(marker_id, x, y, z, True, valid_count))
                        marker_id += 1
                    elif publish_unreachable and (total_xyz % unreachable_stride == 0):
                        markers.markers.append(self.make_marker(marker_id, x, y, z, False, 0))
                        marker_id += 1

        with open(csv_path, "w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([
                "x_m",
                "y_m",
                "z_m",
                "valid_orientation_count",
                "first_valid_roll_deg",
                "first_valid_pitch_deg",
                "yaw_deg",
                "valid_roll_pitch_pairs_deg",
            ])
            writer.writerows(reachable_rows)

        self.marker_pub.publish(markers)

        ratio = 0.0 if total_xyz == 0 else 100.0 * float(reachable_xyz) / float(total_xyz)
        self.get_logger().info("Reachability scan finished")
        self.get_logger().info(f"Total XYZ points          : {total_xyz}")
        self.get_logger().info(f"Reachable XYZ points      : {reachable_xyz} ({ratio:.2f}%)")
        self.get_logger().info(f"Total orientation tests   : {total_orientation_tests}")
        self.get_logger().info(f"CSV saved at              : {csv_path}")
        self.get_logger().info("RViz: Add -> MarkerArray -> /reachability_markers")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReachabilityScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
