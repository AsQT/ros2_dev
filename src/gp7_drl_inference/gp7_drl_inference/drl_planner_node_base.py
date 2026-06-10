"""Shared ROS node base for DRL planner nodes.

Provides all publishers, services, and execution logic that are common to both
the manual input node and the vision-driven preview node.

Responsibilities:
  Publishers:
    /drl/forward_trajectory_marker — forward trajectory MarkerArray
    /drl/forward_trajectory_poses  — forward trajectory PoseArray
    /drl/backward_trajectory_marker — backward trajectory MarkerArray
    /drl/backward_trajectory_poses  — backward trajectory PoseArray
    /drl/next_pose               — streaming waypoint during execution
    /drl/execution_status        — execution state at 2 Hz

  Services:
    /drl/execute_forward    — std_srvs/Trigger
    /drl/execute_backward   — std_srvs/Trigger
    /drl/execute_trajectory — std_srvs/Trigger  (alias for execute_forward)
    /drl/get_execution_status — std_srvs/Trigger
    /drl/clear_trajectory   — std_srvs/Trigger

  Service client:
    /move_cartesian_pose_sequence

  Execution:
    Non-blocking background thread execution.
    Copies trajectory on service call to avoid race conditions.

This base does NOT:
  - Ask terminal input.
  - Subscribe to vision topics.
  - Publish /vision/* markers.
  - Own the DRL model or planning logic.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseArray,
    PoseStamped,
    Quaternion,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

# Lazy import — resolves at runtime so the node starts even if the service
# definition is not available.
try:
    from gp7_task_executor_msgs.srv._move_cartesian_pose_sequence import (
        MoveCartesianPoseSequence,
    )
except ImportError:
    MoveCartesianPoseSequence = None  # type: ignore


# -------------------------------------------------------------------------
# Constants
# -------------------------------------------------------------------------

_EXECUTION_STATUS_RATE_HZ = 2.0

_EXECUTION_QUATERNION = (1.0, 0.0, 0.0, 0.0)  # RPY=[pi, 0, 0]


# -------------------------------------------------------------------------
# Marker helper functions (pure functions — no ROS state needed)
# -------------------------------------------------------------------------

def build_trajectory_markers(
    trajectory_base: list[np.ndarray],
    path_ns: str,
    start_color: tuple[float, float, float],
    end_color: tuple[float, float, float],
    line_color: tuple[float, float, float],
    line_width: float = 0.005,
    sphere_scale: float = 0.01,
    now=None,
    frame_id: str = "base_link",
) -> list[Marker]:
    """Build a MarkerArray for one trajectory.

    Args:
        trajectory_base: Waypoints in BASE frame.
        path_ns: Namespace string for this trajectory's markers.
        start_color: RGB for start sphere.
        end_color: RGB for end/target sphere.
        line_color: RGB for the line strip.
        line_width: Line strip width.
        sphere_scale: Sphere marker scale.
        now: ROS time message (uses get_clock().now() if None).
        frame_id: Frame ID for all markers.

    Returns:
        List of Marker objects ready to publish.
    """
    if not trajectory_base:
        return []

    markers: list[Marker] = []
    start_pt = trajectory_base[0]
    end_pt = trajectory_base[-1]

    # Line strip through all waypoints
    line = Marker()
    line.header.stamp = now if now else rclpy.time.Time().to_msg()
    line.header.frame_id = frame_id
    line.ns = f"{path_ns}_path"
    line.id = 0
    line.type = Marker.LINE_STRIP
    line.action = Marker.ADD
    line.scale.x = float(line_width)
    line.color.r = float(line_color[0])
    line.color.g = float(line_color[1])
    line.color.b = float(line_color[2])
    line.color.a = 1.0
    for pt in trajectory_base:
        line.points.append(Point(x=float(pt[0]), y=float(pt[1]), z=float(pt[2])))
    markers.append(line)

    # Start sphere
    start_marker = Marker()
    start_marker.header.stamp = now if now else rclpy.time.Time().to_msg()
    start_marker.header.frame_id = frame_id
    start_marker.ns = f"{path_ns}_start"
    start_marker.id = 1
    start_marker.type = Marker.SPHERE
    start_marker.action = Marker.ADD
    start_marker.pose.position.x = float(start_pt[0])
    start_marker.pose.position.y = float(start_pt[1])
    start_marker.pose.position.z = float(start_pt[2])
    start_marker.pose.orientation.w = 1.0
    start_marker.scale.x = start_marker.scale.y = start_marker.scale.z = float(sphere_scale)
    start_marker.color.r = float(start_color[0])
    start_marker.color.g = float(start_color[1])
    start_marker.color.b = float(start_color[2])
    start_marker.color.a = 1.0
    markers.append(start_marker)

    # End/target sphere
    end_marker = Marker()
    end_marker.header.stamp = now if now else rclpy.time.Time().to_msg()
    end_marker.header.frame_id = frame_id
    end_marker.ns = f"{path_ns}_end"
    end_marker.id = 2
    end_marker.type = Marker.SPHERE
    end_marker.action = Marker.ADD
    end_marker.pose.position.x = float(end_pt[0])
    end_marker.pose.position.y = float(end_pt[1])
    end_marker.pose.position.z = float(end_pt[2])
    end_marker.pose.orientation.w = 1.0
    end_marker.scale.x = end_marker.scale.y = end_marker.scale.z = float(sphere_scale)
    end_marker.color.r = float(end_color[0])
    end_marker.color.g = float(end_color[1])
    end_marker.color.b = float(end_color[2])
    end_marker.color.a = 1.0
    markers.append(end_marker)

    return markers


def build_trajectory_poses(
    trajectory_base: list[np.ndarray],
    now=None,
    frame_id: str = "base_link",
) -> PoseArray:
    """Build a PoseArray from a trajectory.

    All poses use the calibrated tool orientation (RPY=[pi, 0, 0],
    quaternion xyzw=[1, 0, 0, 0]).

    Returns:
        PoseArray with all waypoints.
    """
    pose_array = PoseArray()
    pose_array.header.stamp = now if now else rclpy.time.Time().to_msg()
    pose_array.header.frame_id = frame_id
    qx, qy, qz, qw = _EXECUTION_QUATERNION
    for pt in trajectory_base:
        p = Pose()
        p.position.x = float(pt[0])
        p.position.y = float(pt[1])
        p.position.z = float(pt[2])
        p.orientation.x = qx
        p.orientation.y = qy
        p.orientation.z = qz
        p.orientation.w = qw
        pose_array.poses.append(p)
    return pose_array


# -------------------------------------------------------------------------
# ROS node base class
# -------------------------------------------------------------------------

class DrlPlannerNodeBase(Node):
    """Shared ROS node base for DRL trajectory nodes.

    Subclasses must:
      1. Call ``super().__init__(node_name)``.
      2. Set ``self._planner: DrlTrajectoryPlannerCore`` after super().__init__.
      3. Call ``self._declare_base_parameters()`` before creating any publishers/subs.
      4. Call ``self._init_base_publishers()`` to set up all topic publishers.
      5. Call ``self._init_base_services()`` to register all services.
      6. Call ``self._init_base_execution()`` to set up service clients and TF.
      7. Call ``self._publish_empty_all_paths()`` at the end of ``__init__``.

    Subclasses MAY override:
      - ``publish_planning_result()`` to publish trajectory markers from a PlanningResult.
      - ``_get_trajectory_to_execute(direction)`` to customize trajectory selection.

    Subclasses MUST NOT call the following base methods from __init__ before
    ``_publish_empty_all_paths()``:
      - ``publish_all_paths()``  (call the empty version instead)
    """

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        # To be set by subclass
        self._planner: Optional[object] = None

        # Trajectory state (published via the publish_* methods)
        self._trajectory_computed: bool = False
        self._trajectory_forward_base: list[np.ndarray] = []
        self._trajectory_backward_base: list[np.ndarray] = []
        self._last_target_base: Optional[np.ndarray] = None
        self._last_obstacle_center_base: Optional[np.ndarray] = None
        self._last_obstacle_full_size: Optional[np.ndarray] = None
        self._last_has_obstacle: bool = False
        self._last_planning_time_sec: Optional[float] = None

        # Execution state (guarded by _execution_lock)
        self._execution_lock = threading.RLock()
        self._execution_active: bool = False
        self._execution_direction: str = ""
        self._execution_status: str = "IDLE"
        self._execution_last_message: str = ""
        self._execution_thread: Optional[threading.Thread] = None

    # -------------------------------------------------------------------------
    # Parameter declaration
    # -------------------------------------------------------------------------

    def _declare_base_parameters(self) -> None:
        """Declare all common ROS parameters.

        Call this after super().__init__ but before creating publishers.
        """
        self.declare_parameter("calibrated_start_tcp_base", [0.350, -0.330, 0.060])
        self.declare_parameter("use_fk_fallback", False)
        self.declare_parameter("fk_fallback_warning", True)
        self.declare_parameter(
            "move_to_cartesian_service_name", "/move_to_cartesian_target"
        )
        self.declare_parameter("execute_mode", "pose_sequence")
        self.declare_parameter("task_executor_service_timeout_sec", 5.0)
        self.declare_parameter("task_executor_result_timeout_sec", 120.0)
        self.declare_parameter("publish_next_pose_during_execute", True)
        self.declare_parameter("start_pose_tolerance", 0.001)
        self.declare_parameter("tf_timeout_sec", 2.0)
        self.declare_parameter(
            "cartesian_pose_sequence_service_name", "/move_cartesian_pose_sequence"
        )

    def _get_calibrated_start_tcp_base(self) -> np.ndarray:
        tcp_param = self.get_parameter("calibrated_start_tcp_base").value
        tcp = np.array(tcp_param, dtype=np.float32)
        if tcp.shape != (3,):
            raise ValueError(
                f"calibrated_start_tcp_base must have 3 elements, got shape {tcp.shape}"
            )
        return tcp

    # -------------------------------------------------------------------------
    # Publishers
    # -------------------------------------------------------------------------

    def _init_base_publishers(self) -> None:
        """Create all common topic publishers.

        Call this after super().__init__ and _declare_base_parameters.
        """
        trajectory_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Forward trajectory (primary)
        self._forward_marker_pub = self.create_publisher(
            MarkerArray, "/drl/forward_trajectory_marker", trajectory_qos
        )
        self._forward_poses_pub = self.create_publisher(
            PoseArray, "/drl/forward_trajectory_poses", trajectory_qos
        )

        # Backward trajectory
        self._backward_marker_pub = self.create_publisher(
            MarkerArray, "/drl/backward_trajectory_marker", trajectory_qos
        )
        self._backward_poses_pub = self.create_publisher(
            PoseArray, "/drl/backward_trajectory_poses", trajectory_qos
        )

        # Streaming waypoint during execution
        self._next_pose_pub = self.create_publisher(PoseStamped, "/drl/next_pose", 10)

        # Execution status
        self._execution_status_pub = self.create_publisher(
            String, "/drl/execution_status", 10
        )

    # -------------------------------------------------------------------------
    # Services
    # -------------------------------------------------------------------------

    def _init_base_services(self) -> None:
        """Register all common services.

        Call this after _init_base_publishers.
        """
        self._execute_forward_srv = self.create_service(
            Trigger, "/drl/execute_forward", self._on_execute_forward
        )
        self._execute_backward_srv = self.create_service(
            Trigger, "/drl/execute_backward", self._on_execute_backward
        )
        self._execute_trajectory_srv = self.create_service(
            Trigger, "/drl/execute_trajectory", self._on_execute_trajectory
        )
        self._clear_srv = self.create_service(
            Trigger, "/drl/clear_trajectory", self._on_clear_trajectory
        )
        self._get_status_srv = self.create_service(
            Trigger, "/drl/get_execution_status", self._on_get_execution_status
        )
        self._execution_status_timer = self.create_timer(
            1.0 / _EXECUTION_STATUS_RATE_HZ, self._publish_execution_status
        )

    # -------------------------------------------------------------------------
    # Execution client setup
    # -------------------------------------------------------------------------

    def _init_base_execution(self) -> None:
        """Set up TF listener and service clients.

        Call this after _init_base_services.
        """
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_timeout_sec = float(
            self.get_parameter("tf_timeout_sec").value
        )

        self._service_timeout_sec = float(
            self.get_parameter("task_executor_service_timeout_sec").value
        )
        self._result_timeout_sec = float(
            self.get_parameter("task_executor_result_timeout_sec").value
        )
        self._publish_next_pose_during_execute = bool(
            self.get_parameter("publish_next_pose_during_execute").value
        )

        self._pose_seq_service_name = str(
            self.get_parameter("cartesian_pose_sequence_service_name").value
        )

        if MoveCartesianPoseSequence is not None:
            self._pose_seq_client = self.create_client(
                MoveCartesianPoseSequence, self._pose_seq_service_name
            )
        else:
            self._pose_seq_client = None
            self.get_logger().warn(
                "gp7_task_executor_msgs not available; pose sequence execution disabled"
            )

    # -------------------------------------------------------------------------
    # Publishing helpers
    # -------------------------------------------------------------------------

    def _publish_empty_marker(self, marker_pub, ns: str) -> None:
        """Publish DELETEALL for a given publisher."""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "base_link"
        marker.ns = ns
        marker.id = 0
        marker.action = Marker.DELETEALL
        marker_pub.publish(MarkerArray(markers=[marker]))

    def _publish_empty_all_paths(self) -> None:
        """Clear all trajectory markers on startup."""
        for pub, ns in [
            (self._forward_marker_pub, "drl_forward"),
            (self._backward_marker_pub, "drl_backward"),
        ]:
            self._publish_empty_marker(pub, ns)

    def _build_obstacle_marker(
        self,
        ns: str,
        center_base: np.ndarray,
        full_size: np.ndarray,
        now=None,
    ) -> Optional[Marker]:
        """Build an obstacle cube marker, or None if center/size are zero."""
        if center_base is None or full_size is None:
            return None
        if np.allclose(center_base, 0.0) and np.allclose(full_size, 0.0):
            return None
        obs = Marker()
        obs.header.stamp = now if now else self.get_clock().now().to_msg()
        obs.header.frame_id = "base_link"
        obs.ns = ns
        obs.id = 0
        obs.type = Marker.CUBE
        obs.action = Marker.ADD
        obs.pose.position.x = float(center_base[0])
        obs.pose.position.y = float(center_base[1])
        obs.pose.position.z = float(center_base[2])
        obs.pose.orientation.w = 1.0
        obs.scale.x = float(full_size[0])
        obs.scale.y = float(full_size[1])
        obs.scale.z = float(full_size[2])
        obs.color.r = 1.0
        obs.color.g = 0.4
        obs.color.b = 0.0
        obs.color.a = 0.4
        return obs

    def _build_target_block_marker(
        self,
        target_base: np.ndarray,
        now=None,
    ) -> Marker:
        """Build a target block cylinder marker for visual representation.

        Uses the user-provided target x/y but fixes z=0.045 (table surface level)
        so the block always appears on the table, independent of the DRL planner
        target z (which may be above the table for reaching).

        Args:
            target_base: User input target in base_link frame.
            now: ROS time message.

        Returns:
            Marker configured as a blue CYLINDER on the table surface.
        """
        marker = Marker()
        marker.header.stamp = now if now else self.get_clock().now().to_msg()
        marker.header.frame_id = "base_link"
        marker.ns = "drl_forward_target_block"
        marker.id = 3
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = float(target_base[0])
        marker.pose.position.y = float(target_base[1])
        marker.pose.position.z = 0.045
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.01
        marker.color.r = 0.0
        marker.color.g = 0.2
        marker.color.b = 1.0
        marker.color.a = 1.0
        return marker

    def publish_planning_result(self, result) -> None:
        """Publish trajectory markers and poses from a PlanningResult.

        Subclasses can override this to customize visualization.
        The default implementation publishes forward, backward, and legacy topics.

        Args:
            result: A PlanningResult from DrlTrajectoryPlannerCore.compute_trajectory().
        """
        if not result.trajectory_forward_base:
            self._publish_empty_all_paths()
            return

        now = self.get_clock().now().to_msg()
        fwd = result.trajectory_forward_base
        bwd = result.trajectory_backward_base

        # Forward trajectory MarkerArray
        forward_markers: list[Marker] = []
        da_fwd = Marker()
        da_fwd.header.stamp = now
        da_fwd.header.frame_id = "base_link"
        da_fwd.action = Marker.DELETEALL
        forward_markers.append(da_fwd)
        forward_markers.extend(
            build_trajectory_markers(
                trajectory_base=fwd,
                path_ns="drl_forward",
                start_color=(0.0, 1.0, 0.0),
                end_color=(1.0, 0.0, 0.0),
                line_color=(0.0, 0.7, 1.0),
                now=now,
            )
        )
        obs_marker = self._build_obstacle_marker(
            "drl_forward_obstacle",
            result.obstacle_center_base,
            result.obstacle_full_size,
            now,
        )
        if obs_marker:
            forward_markers.append(obs_marker)
        target_block_marker = self._build_target_block_marker(result.target_base, now)
        forward_markers.append(target_block_marker)
        self.get_logger().info(
            f"Published manual target block marker at "
            f"x={result.target_base[0]:.4f}, y={result.target_base[1]:.4f}, z=0.045 | "
            f"Planner target remains x={result.target_base[0]:.4f}, "
            f"y={result.target_base[1]:.4f}, z={result.target_base[2]:.4f}"
        )
        self._forward_marker_pub.publish(MarkerArray(markers=forward_markers))
        self._forward_poses_pub.publish(
            build_trajectory_poses(fwd, now=now)
        )

        # Backward trajectory MarkerArray
        backward_markers: list[Marker] = []
        da_bwd = Marker()
        da_bwd.header.stamp = now
        da_bwd.header.frame_id = "base_link"
        da_bwd.action = Marker.DELETEALL
        backward_markers.append(da_bwd)
        backward_markers.extend(
            build_trajectory_markers(
                trajectory_base=bwd,
                path_ns="drl_backward",
                start_color=(1.0, 0.6, 0.0),
                end_color=(0.0, 1.0, 0.0),
                line_color=(0.8, 0.2, 0.8),
                line_width=0.010,
                sphere_scale=0.035,
                now=now,
            )
        )
        self._backward_marker_pub.publish(MarkerArray(markers=backward_markers))
        self._backward_poses_pub.publish(
            build_trajectory_poses(bwd, now=now)
        )

        # Update stored state
        self._trajectory_computed = True
        self._trajectory_forward_base = list(result.trajectory_forward_base)
        self._trajectory_backward_base = list(result.trajectory_backward_base)
        self._last_target_base = result.target_base.copy()
        self._last_obstacle_center_base = (
            result.obstacle_center_base.copy()
            if result.has_obstacle
            else np.zeros(3, dtype=np.float32)
        )
        self._last_obstacle_full_size = (
            result.obstacle_full_size.copy()
            if result.has_obstacle
            else np.zeros(3, dtype=np.float32)
        )
        self._last_has_obstacle = result.has_obstacle
        self._last_planning_time_sec = result.planning_time_sec

        # Log summary
        n_fwd = len(self._trajectory_forward_base)
        n_bwd = len(self._trajectory_backward_base)
        fwd_first = self._trajectory_forward_base[0]
        fwd_last = self._trajectory_forward_base[-1]
        bwd_first = self._trajectory_backward_base[0]
        bwd_last = self._trajectory_backward_base[-1]
        self.get_logger().info(
            f"Published trajectories | "
            f"forward={n_fwd}wp [{fwd_first[0]:.3f},{fwd_first[1]:.3f},{fwd_first[2]:.3f}]"
            f" -> [{fwd_last[0]:.3f},{fwd_last[1]:.3f},{fwd_last[2]:.3f}] | "
            f"backward={n_bwd}wp [{bwd_first[0]:.3f},{bwd_first[1]:.3f},{bwd_first[2]:.3f}]"
            f" -> [{bwd_last[0]:.3f},{bwd_last[1]:.3f},{bwd_last[2]:.3f}]"
        )

    # -------------------------------------------------------------------------
    # Execution
    # -------------------------------------------------------------------------

    def _current_tcp_base(self) -> tuple[np.ndarray, bool]:
        """Look up current TCP pose from TF: base_link -> tool0."""
        try:
            transform = self._tf_buffer.lookup_transform(
                "base_link",
                "tool0",
                rclpy.time.Time.from_msg(self.get_clock().now().to_msg()),
                timeout=rclpy.duration.Duration(seconds=self._tf_timeout_sec),
            )
        except Exception:
            return np.zeros(3, dtype=np.float32), False
        pos = transform.transform.translation
        return np.array([pos.x, pos.y, pos.z], dtype=np.float32), True

    def _publish_execution_status(self) -> None:
        """Publish current execution status to /drl/execution_status."""
        with self._execution_lock:
            status = self._execution_status
            message = self._execution_last_message

        msg = String()
        msg.data = f"{status}: {message}" if message else status
        self._execution_status_pub.publish(msg)

    def _set_execution_state(
        self,
        active: bool,
        direction: str,
        status: str,
        message: str,
    ) -> None:
        """Atomically update all execution state fields."""
        with self._execution_lock:
            self._execution_active = active
            self._execution_direction = direction
            self._execution_status = status
            self._execution_last_message = message

    def _check_execute_ready(
        self,
    ) -> tuple[bool, str, list[np.ndarray], int]:
        """Shared pre-execution check.

        Returns:
            (ready, error_msg, waypoints, n).  If ready=False, error_msg is set.
        """
        with self._execution_lock:
            if self._execution_active:
                return False, (
                    f"Execution already running: {self._execution_direction}"
                ), [], 0

        if not self._trajectory_computed:
            return False, "No planned trajectory available. Plan a trajectory first.", [], 0
        waypoints = self._trajectory_forward_base
        n = len(waypoints)
        if n < 2:
            return False, f"Trajectory has only {n} waypoint(s); need at least 2.", waypoints, n
        for i, pt in enumerate(waypoints):
            if not np.all(np.isfinite(pt)):
                return False, f"Waypoint {i}/{n} contains non-finite values: {pt}", waypoints, n
        return True, "", waypoints, n

    def _execute_pose_sequence_blocking(
        self,
        trajectory_base: list[np.ndarray],
        label: str,
    ) -> tuple[bool, str]:
        """Send trajectory to /move_cartesian_pose_sequence (blocking).

        Must be called from a background thread.
        """
        if self._pose_seq_client is None:
            return False, "Pose sequence service client not available."

        if not self._pose_seq_client.service_is_ready():
            if not self._pose_seq_client.wait_for_service(
                timeout_sec=self._service_timeout_sec
            ):
                return False, (
                    f"Service {self._pose_seq_service_name} not available after "
                    f"{self._service_timeout_sec} s."
                )
            self.get_logger().info(f"[{label}] Connected to {self._pose_seq_service_name}")

        n = len(trajectory_base)
        self.get_logger().info(f"[{label}] calling {self._pose_seq_service_name} with {n} poses")

        req = MoveCartesianPoseSequence.Request()
        req.execute = True

        qx, qy, qz, qw = _EXECUTION_QUATERNION
        for pt in trajectory_base:
            stamped = PoseStamped()
            stamped.header.stamp = self.get_clock().now().to_msg()
            stamped.header.frame_id = "base_link"
            stamped.pose.position.x = float(pt[0])
            stamped.pose.position.y = float(pt[1])
            stamped.pose.position.z = float(pt[2])
            stamped.pose.orientation.x = qx
            stamped.pose.orientation.y = qy
            stamped.pose.orientation.z = qz
            stamped.pose.orientation.w = qw
            req.poses.append(stamped)

        if n <= 10:
            for i, pt in enumerate(trajectory_base):
                self.get_logger().info(f"  [{i}/{n}] ({pt[0]:.3f}, {pt[1]:.3f}, {pt[2]:.3f})")
        else:
            log_interval = max(1, n // 10)
            for i in range(0, n, log_interval):
                pt = trajectory_base[i]
                self.get_logger().info(f"  [{i}/{n}] ({pt[0]:.3f}, {pt[1]:.3f}, {pt[2]:.3f})")
            self.get_logger().info(
                f"  [{n-1}/{n}] ({trajectory_base[-1][0]:.3f}, "
                f"{trajectory_base[-1][1]:.3f}, {trajectory_base[-1][2]:.3f})"
            )

        if self._publish_next_pose_during_execute and trajectory_base:
            final = trajectory_base[-1]
            stamped = PoseStamped()
            stamped.header.stamp = self.get_clock().now().to_msg()
            stamped.header.frame_id = "base_link"
            stamped.pose.position.x = float(final[0])
            stamped.pose.position.y = float(final[1])
            stamped.pose.position.z = float(final[2])
            stamped.pose.orientation.x = qx
            stamped.pose.orientation.y = qy
            stamped.pose.orientation.z = qz
            stamped.pose.orientation.w = qw
            self._next_pose_pub.publish(stamped)

        future = self._pose_seq_client.call_async(req)

        deadline = time.monotonic() + self._result_timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                return False, f"Pose sequence [{label}] timed out after {self._result_timeout_sec} s."
            time.sleep(0.05)

        if not future.done():
            return False, f"Pose sequence [{label}] timed out after {self._result_timeout_sec} s."

        try:
            resp = future.result()
        except Exception as exc:
            return False, f"Pose sequence [{label}] service exception: {exc}"

        if resp is None:
            return False, f"Pose sequence [{label}] returned no result."

        if not resp.success:
            return False, (
                f"Pose sequence [{label}] failed (fraction={resp.fraction:.4f}): "
                f"{resp.message}"
            )

        self.get_logger().info(f"[{label}] result success=True fraction={resp.fraction:.4f}")
        return True, (
            f"Executed {n} [{label}] waypoints via {self._pose_seq_service_name} "
            f"(fraction={resp.fraction:.4f})."
        )

    def _execution_worker(
        self,
        trajectory_base: list[np.ndarray],
        label: str,
    ) -> None:
        """Background worker that calls the blocking executor and updates state."""
        self.get_logger().info(f"[{label}] background execution started")
        success = False
        message = ""

        try:
            success, message = self._execute_pose_sequence_blocking(trajectory_base, label)
        except Exception as e:
            message = f"{label} execution exception: {e}"
            self.get_logger().error(f"[{label}] {message}")

        status = f"SUCCEEDED_{label.upper()}" if success else f"FAILED_{label.upper()}"
        self._set_execution_state(
            active=False,
            direction="",
            status=status,
            message=message,
        )
        self.get_logger().info(
            f"[{label}] background execution finished | status={status} | message={message}"
        )

    def _start_execution_thread(
        self,
        trajectory_base: list[np.ndarray],
        label: str,
    ) -> tuple[Trigger.Response, bool]:
        """Attempt to start a background execution thread.

        Returns:
            (response, started) — started=True only if a new thread was launched.
        """
        ready, err, waypoints, n = self._check_execute_ready()
        if not ready:
            self.get_logger().warn(f"[execute_{label}] {err}")
            response = Trigger.Response()
            response.success = False
            response.message = err
            return response, False

        with self._execution_lock:
            if self._execution_active:
                direction = self._execution_direction
                self.get_logger().warn(
                    f"[execute_{label}] execution already running: {direction}"
                )
                response = Trigger.Response()
                response.success = False
                response.message = f"Execution already running: {direction}"
                return response, False

            self._execution_active = True
            self._execution_direction = label
            self._execution_status = f"RUNNING_{label.upper()}"
            self._execution_last_message = "execution started"
            trajectory_copy = [p.copy() for p in trajectory_base]

        self.get_logger().info(
            f"[execute_{label}] starting background execution | "
            f"trajectory_computed={self._trajectory_computed} | "
            f"execution_active={self._execution_active}"
        )

        thread = threading.Thread(
            target=self._execution_worker,
            args=(trajectory_copy, label),
            daemon=True,
        )
        self._execution_thread = thread
        thread.start()

        response = Trigger.Response()
        response.success = True
        response.message = f"{label.capitalize()} execution started."
        return response, True

    def _get_trajectory_for_direction(self, direction: str) -> list[np.ndarray]:
        """Return the trajectory list for the given direction.

        Subclasses can override to customize which trajectory is used.
        """
        if direction == "forward":
            return self._trajectory_forward_base
        elif direction == "backward":
            return self._trajectory_backward_base
        else:
            return self._trajectory_forward_base

    # -------------------------------------------------------------------------
    # Service handlers
    # -------------------------------------------------------------------------

    def _on_execute_forward(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Handle /drl/execute_forward."""
        self.get_logger().info("[execute_forward] callback ENTER")
        resp, started = self._start_execution_thread(
            self._get_trajectory_for_direction("forward"), "forward"
        )
        self.get_logger().info(
            f"[execute_forward] callback RETURNING IMMEDIATELY "
            f"started={started} success={resp.success} message='{resp.message}'"
        )
        return resp

    def _on_execute_backward(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Handle /drl/execute_backward."""
        self.get_logger().info("[execute_backward] callback ENTER")

        with self._execution_lock:
            if self._execution_active:
                response.success = False
                response.message = (
                    f"Execution already running: {self._execution_direction}"
                )
                self.get_logger().warn("[execute_backward] rejected: execution active")
                return response

        if not self._trajectory_computed:
            response.success = False
            response.message = "No planned trajectory available. Plan a trajectory first."
            self.get_logger().warn("[execute_backward] rejected: no trajectory")
            return response

        if not self._trajectory_backward_base:
            response.success = False
            response.message = "No backward trajectory available."
            self.get_logger().warn("[execute_backward] rejected: no backward trajectory")
            return response

        resp, started = self._start_execution_thread(
            self._get_trajectory_for_direction("backward"), "backward"
        )
        self.get_logger().info(
            f"[execute_backward] callback RETURNING IMMEDIATELY "
            f"started={started} success={resp.success} message='{resp.message}'"
        )
        return resp

    def _on_execute_trajectory(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Handle /drl/execute_trajectory — backwards-compatible alias for execute_forward."""
        self.get_logger().info("[execute_trajectory] callback ENTER")
        resp, started = self._start_execution_thread(
            self._get_trajectory_for_direction("forward"), "forward"
        )
        self.get_logger().info(
            f"[execute_trajectory] callback RETURNING IMMEDIATELY "
            f"started={started} success={resp.success} message='{resp.message}'"
        )
        return resp

    def _on_get_execution_status(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Handle /drl/get_execution_status."""
        with self._execution_lock:
            active = self._execution_active
            status = self._execution_status
            message = self._execution_last_message

        response.success = not active
        if active:
            response.message = f"{status}: {message or 'execution in progress'}"
        else:
            response.message = f"{status}: {message}"
        return response

    def _on_clear_trajectory(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Handle /drl/clear_trajectory — reject if execution is active."""
        with self._execution_lock:
            if self._execution_active:
                response.success = False
                response.message = (
                    f"Cannot clear trajectory while execution is running: "
                    f"{self._execution_direction}"
                )
                self.get_logger().warn(
                    f"[clear_trajectory] rejected: execution active={self._execution_direction}"
                )
                return response

        self._trajectory_computed = False
        self._trajectory_forward_base = []
        self._trajectory_backward_base = []
        self._last_target_base = None
        self._last_obstacle_center_base = None
        self._last_obstacle_full_size = None
        self._last_has_obstacle = False
        self.get_logger().info("Trajectory cleared.")
        self._publish_empty_all_paths()
        response.success = True
        response.message = "Trajectory cleared."
        return response

    # -------------------------------------------------------------------------
    # Logging helpers (for subclass use)
    # -------------------------------------------------------------------------

    def log_planning_result(self, result, source: str = "") -> None:
        """Log a summary of a PlanningResult.

        Args:
            result: PlanningResult from compute_trajectory().
            source: Label for the planning source (e.g. "manual", "vision").
        """
        prefix = f"[{source}] " if source else ""
        self.get_logger().info(
            f"{prefix}Trajectory planning complete | "
            f"converged={result.converged} | "
            f"planning_time={result.planning_time_sec:.4f}s ({result.planning_time_sec * 1000:.1f} ms) | "
            f"convergence_dist={result.convergence_dist:.4f} m | "
            f"target_base=({result.target_base[0]:.4f}, "
            f"{result.target_base[1]:.4f}, {result.target_base[2]:.4f}) | "
            f"target_drl=({result.target_drl[0]:.4f}, "
            f"{result.target_drl[1]:.4f}, {result.target_drl[2]:.4f}) | "
            f"has_obstacle={result.has_obstacle} | "
            f"forward_waypoints={len(result.trajectory_forward_base)} | "
            f"backward_waypoints={len(result.trajectory_backward_base)}"
        )
        if result.has_obstacle:
            self.get_logger().info(
                f"{prefix}obstacle_center_base=({result.obstacle_center_base[0]:.4f}, "
                f"{result.obstacle_center_base[1]:.4f}, {result.obstacle_center_base[2]:.4f}) | "
                f"obstacle_full_size=({result.obstacle_full_size[0]:.4f}, "
                f"{result.obstacle_full_size[1]:.4f}, {result.obstacle_full_size[2]:.4f})"
            )
        if not result.converged:
            self.get_logger().warn(
                f"{prefix}Trajectory did NOT converge after max_episode_steps steps. "
                f"convergence_dist={result.convergence_dist:.4f} m. "
                f"Target may be unreachable or outside workspace."
            )
