"""Unified DRL planner node supporting both manual and vision input modes.

This node replaces the separate manual/vision DRL input flows with one clean
interface controlled by the ``input_mode`` parameter.

Usage::

    ros2 run robot_drl drl_unified_planner_node \\
        --ros-args -p input_mode:=manual -p auto_plan_on_start:=true

    ros2 run robot_drl drl_unified_planner_node \\
        --ros-args -p input_mode:=vision

Topics published (inherited from DrlPlannerNodeBase):
  /drl/forward_trajectory_marker   — forward trajectory MarkerArray
  /drl/forward_trajectory_poses    — forward trajectory PoseArray
  /drl/backward_trajectory_marker  — backward trajectory MarkerArray
  /drl/backward_trajectory_poses   — backward trajectory PoseArray
  /drl/next_pose                  — streaming waypoint during execution
  /drl/execution_status            — execution state at 2 Hz

Services (inherited):
  /drl/execute_forward            — execute forward trajectory
  /drl/execute_backward           — execute backward trajectory
  /drl/execute_trajectory         — alias for execute_forward
  /drl/clear_trajectory           — reset all trajectory state
  /drl/get_execution_status       — query current execution state

Services (new):
  /drl/plan                       — trigger planning (manual: re-prompt; vision: use latest data)
  /drl/replan                     — alias for /drl/plan
"""

import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetPositionIK
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from robot_drl import config
from robot_drl.drl_planner_core import (
    DrlTrajectoryPlannerCore,
    PlanningResult,
    load_planner,
)
from robot_drl.drl_planner_node_base import DrlPlannerNodeBase, _EXECUTION_QUATERNION
from robot_drl.planning_scene_adapter import (
    PlanningSceneObstacleError,
    SceneObstacle,
    manual_obstacle,
    planning_scene_to_obstacles,
    point_aabb_signed_distance,
    select_obstacle_for_policy,
    validate_cartesian_path_against_obstacles,
)
from std_srvs.srv import Trigger

# Lazy import — the Box message type is optional.
try:
    from robot_vision_pipeline.msg import Box  # noqa: F401
except ImportError:
    Box = None  # type: ignore


# -------------------------------------------------------------------------
# Common input format
# -------------------------------------------------------------------------

@dataclass
class DrlSceneInput:
    """Internal common input format for DRL planning.

    Both manual and vision input modes produce this object before calling
    the shared planning logic.
    """

    #: Target TCP position in BASE/base_link frame, shape (3,).
    target_base: np.ndarray

    #: Whether to plan around an obstacle.
    has_obstacle: bool

    #: Obstacle center in BASE frame, shape (3,).  Zero array if no obstacle.
    obstacle_center_base: np.ndarray

    #: Obstacle full D/W/H dimensions, shape (3,).  Zero array if no obstacle.
    obstacle_full_size: np.ndarray

    #: Label for logging and topic naming (e.g. "manual", "vision").
    source: str

    #: Wall-clock timestamp (seconds) when this input was captured.
    stamp_sec: float


# -------------------------------------------------------------------------
# Terminal input helpers
# -------------------------------------------------------------------------

def read_user_input() -> tuple[np.ndarray, bool, np.ndarray, np.ndarray]:
    """Read target and obstacle from stdin (blocking).

    Returns:
        (target_base, has_obstacle, obstacle_center_base, obstacle_full_size)

    All values are in BASE/base_link frame, metres.
    """
    print("\n" + "=" * 60)
    print("  DRL Unified Planner — Manual Target Input")
    print("  All values are in BASE/base_link frame (metres)")
    print("=" * 60)

    def read_float(prompt: str, default: float) -> float:
        raw = input(f"  {prompt} [{default}]: ").strip()
        return float(raw) if raw else default

    tx = read_float("Target X (base_link m)", float(config.DEFAULT_TARGET_BASE[0]))
    ty = read_float("Target Y (base_link m)", float(config.DEFAULT_TARGET_BASE[1]))
    tz = read_float("Target Z (base_link m)", float(config.DEFAULT_TARGET_BASE[2]))
    target_base = np.array([tx, ty, tz], dtype=np.float32)

    print()
    print("  --- Obstacle (press Enter three times to skip) ---")

    def read_component(prompt: str) -> str:
        return input(f"  {prompt} [skip]: ").strip()

    ox = read_component("Obstacle X (base_link m)")
    oy = read_component("Obstacle Y (base_link m)")
    oz = read_component("Obstacle Z (base_link m)")

    has_all_coords = bool(ox and oy and oz)
    has_any_coords = bool(ox or oy or oz)

    if has_all_coords:
        print()
        d = read_float("  Obstacle size D (m) [0.100]", 0.100)
        w = read_float("  Obstacle size W (m) [0.100]", 0.100)
        h = read_float("  Obstacle size H (m) [0.100]", 0.100)
        obstacle_center = np.array(
            [float(ox), float(oy), float(oz)], dtype=np.float32
        )
        obstacle_size = np.array([d, w, h], dtype=np.float32)
        has_obstacle = True
        print(
            f"\n  Obstacle: center=({obstacle_center[0]:.3f}, "
            f"{obstacle_center[1]:.3f}, {obstacle_center[2]:.3f}), "
            f"size=({d}, {w}, {h})"
        )
    elif has_any_coords:
        print()
        print("  WARNING: Incomplete obstacle input. Falling back to no obstacle.")
        obstacle_center = np.zeros(3, dtype=np.float32)
        obstacle_size = np.zeros(3, dtype=np.float32)
        has_obstacle = False
    else:
        obstacle_center = np.zeros(3, dtype=np.float32)
        obstacle_size = np.zeros(3, dtype=np.float32)
        has_obstacle = False
        print("\n  No obstacle — obstacle fields set to zero.")

    print("=" * 60 + "\n")
    return target_base, has_obstacle, obstacle_center, obstacle_size


# -------------------------------------------------------------------------
# Node
# -------------------------------------------------------------------------

class DrlUnifiedPlannerNode(DrlPlannerNodeBase):
    """Unified DRL planner supporting both manual and vision input modes.

    Inherits all ROS infrastructure (publishers, services, execution) from
    DrlPlannerNodeBase.
    """

    def __init__(
        self,
        model,
        vn_stats,
        env_cfg: dict,
        calibrated_start_tcp_base: np.ndarray,
    ) -> None:
        # 1. super().__init__ FIRST — required before any declare_parameter/get_parameter
        super().__init__("drl_unified_planner_node")

        # 2. Declare unified-planner parameters (safe now that Node.__init__ has run)
        self.declare_parameter("input_mode", "manual")
        self.declare_parameter("auto_plan_on_start", True)
        self.declare_parameter("auto_execute_after_plan", False)
        self.declare_parameter("allow_replan_service", True)
        self.declare_parameter("manual_prompt_on_start", True)
        self.declare_parameter(
            "manual_default_target",
            config.DEFAULT_TARGET_BASE.tolist(),
        )
        self.declare_parameter(
            "manual_default_obstacle_center",
            config.DEFAULT_OBSTACLE_CENTER_BASE.tolist(),
        )
        self.declare_parameter(
            "manual_default_obstacle_size",
            config.DEFAULT_OBSTACLE_SIZE.tolist(),
        )
        self.declare_parameter("manual_allow_skip_obstacle", True)
        self.declare_parameter("vision_target_topic", "/vision/target_position")
        self.declare_parameter("vision_box_topic", "/vision/box")
        self.declare_parameter(
            "vision_target_detected_topic", "/vision/target_detected"
        )
        self.declare_parameter(
            "vision_box_detected_topic", "/vision/box_detected"
        )
        self.declare_parameter("vision_require_target_detected", True)
        self.declare_parameter("vision_use_obstacle_if_detected", True)
        self.declare_parameter("use_planning_scene_obstacles", True)
        self.declare_parameter(
            "planning_scene_service_name",
            "/get_planning_scene",
        )
        self.declare_parameter("planning_scene_timeout_sec", 2.0)
        self.declare_parameter("planning_scene_frame", "base_link")
        self.declare_parameter("require_planning_scene_obstacle_transform", True)
        self.declare_parameter("path_collision_check_step_m", 0.01)
        self.declare_parameter("path_collision_clearance_margin_m", 0.0)
        self.declare_parameter("publish_failed_collision_path", False)
        self.declare_parameter("obstacle_safety_filter_enabled", True)
        self.declare_parameter(
            "obstacle_safety_margin_m",
            config.DEFAULT_OBSTACLE_SAFETY_MARGIN,
        )
        self.declare_parameter("obstacle_safety_check_step_m", 0.005)
        self.declare_parameter("validate_path_with_moveit_ik", True)
        self.declare_parameter("compute_ik_service_name", "/compute_ik")
        self.declare_parameter("moveit_group_name", "arm")
        self.declare_parameter("moveit_ik_link_name", "tcp_link")
        self.declare_parameter("moveit_ik_timeout_sec", 0.2)
        self.declare_parameter("moveit_ik_max_samples", 80)
        self.declare_parameter("accept_near_convergence_dist_m", 0.20)
        self.declare_parameter(
            "calibrated_start_tcp_base",
            np.asarray(calibrated_start_tcp_base, dtype=np.float32).tolist(),
        )
        self.declare_parameter(
            "workspace_min_base",
            np.asarray(
                env_cfg.get("workspace_min", config.DEFAULT_WORKSPACE_MIN),
                dtype=np.float32,
            ).tolist(),
        )
        self.declare_parameter(
            "workspace_max_base",
            np.asarray(
                env_cfg.get("workspace_max", config.DEFAULT_WORKSPACE_MAX),
                dtype=np.float32,
            ).tolist(),
        )

        # 3. Read unified parameters into instance attributes
        self._input_mode = str(self.get_parameter("input_mode").value)
        self._auto_plan_on_start = bool(
            self.get_parameter("auto_plan_on_start").value
        )
        self._auto_execute_after_plan = bool(
            self.get_parameter("auto_execute_after_plan").value
        )
        self._allow_replan_service = bool(
            self.get_parameter("allow_replan_service").value
        )
        self._manual_prompt_on_start = bool(
            self.get_parameter("manual_prompt_on_start").value
        )
        self._manual_default_target = np.array(
            self.get_parameter("manual_default_target").value, dtype=np.float32
        )
        self._manual_default_obstacle_center = np.array(
            self.get_parameter("manual_default_obstacle_center").value,
            dtype=np.float32,
        )
        self._manual_default_obstacle_size = np.array(
            self.get_parameter("manual_default_obstacle_size").value,
            dtype=np.float32,
        )
        self._manual_allow_skip_obstacle = bool(
            self.get_parameter("manual_allow_skip_obstacle").value
        )
        self._vision_target_topic = str(
            self.get_parameter("vision_target_topic").value
        )
        self._vision_box_topic = str(
            self.get_parameter("vision_box_topic").value
        )
        self._vision_target_detected_topic = str(
            self.get_parameter("vision_target_detected_topic").value
        )
        self._vision_box_detected_topic = str(
            self.get_parameter("vision_box_detected_topic").value
        )
        self._vision_require_target_detected = bool(
            self.get_parameter("vision_require_target_detected").value
        )
        self._vision_use_obstacle_if_detected = bool(
            self.get_parameter("vision_use_obstacle_if_detected").value
        )
        self._use_planning_scene_obstacles = bool(
            self.get_parameter("use_planning_scene_obstacles").value
        )
        self._planning_scene_service_name = str(
            self.get_parameter("planning_scene_service_name").value
        )
        self._planning_scene_timeout_sec = float(
            self.get_parameter("planning_scene_timeout_sec").value
        )
        self._planning_scene_frame = str(
            self.get_parameter("planning_scene_frame").value
        )
        self._require_planning_scene_obstacle_transform = bool(
            self.get_parameter("require_planning_scene_obstacle_transform").value
        )
        self._path_collision_check_step_m = float(
            self.get_parameter("path_collision_check_step_m").value
        )
        self._path_collision_clearance_margin_m = float(
            self.get_parameter("path_collision_clearance_margin_m").value
        )
        self._publish_failed_collision_path = bool(
            self.get_parameter("publish_failed_collision_path").value
        )
        self._obstacle_safety_filter_enabled = bool(
            self.get_parameter("obstacle_safety_filter_enabled").value
        )
        self._obstacle_safety_margin_m = float(
            self.get_parameter("obstacle_safety_margin_m").value
        )
        self._obstacle_safety_check_step_m = float(
            self.get_parameter("obstacle_safety_check_step_m").value
        )
        self._validate_path_with_moveit_ik = bool(
            self.get_parameter("validate_path_with_moveit_ik").value
        )
        self._compute_ik_service_name = str(
            self.get_parameter("compute_ik_service_name").value
        )
        self._moveit_group_name = str(
            self.get_parameter("moveit_group_name").value
        )
        self._moveit_ik_link_name = str(
            self.get_parameter("moveit_ik_link_name").value
        )
        self._moveit_ik_timeout_sec = float(
            self.get_parameter("moveit_ik_timeout_sec").value
        )
        self._moveit_ik_max_samples = int(
            self.get_parameter("moveit_ik_max_samples").value
        )
        self._accept_near_convergence_dist_m = float(
            self.get_parameter("accept_near_convergence_dist_m").value
        )
        workspace_min_base = np.array(
            self.get_parameter("workspace_min_base").value,
            dtype=np.float32,
        )
        workspace_max_base = np.array(
            self.get_parameter("workspace_max_base").value,
            dtype=np.float32,
        )
        calibrated_start_tcp_base = np.array(
            self.get_parameter("calibrated_start_tcp_base").value,
            dtype=np.float32,
        )
        if workspace_min_base.shape != (3,):
            raise ValueError(
                f"workspace_min_base must have 3 elements, got {workspace_min_base.shape}"
            )
        if workspace_max_base.shape != (3,):
            raise ValueError(
                f"workspace_max_base must have 3 elements, got {workspace_max_base.shape}"
            )
        if calibrated_start_tcp_base.shape != (3,):
            raise ValueError(
                "calibrated_start_tcp_base must have 3 elements, "
                f"got {calibrated_start_tcp_base.shape}"
            )
        env_cfg = dict(env_cfg)
        env_cfg["workspace_min"] = workspace_min_base.tolist()
        env_cfg["workspace_max"] = workspace_max_base.tolist()
        env_cfg["obstacle_safety_filter_enabled"] = self._obstacle_safety_filter_enabled
        env_cfg["obstacle_safety_margin"] = self._obstacle_safety_margin_m
        env_cfg["obstacle_safety_check_step_m"] = self._obstacle_safety_check_step_m

        # 4. Plumb in the planner core (same pattern as other nodes)
        self._planner = DrlTrajectoryPlannerCore(
            model=model,
            vn_stats=vn_stats,
            env_cfg=env_cfg,
            calibrated_start_tcp_base=calibrated_start_tcp_base,
        )

        # 5. Init base ROS infrastructure
        self._declare_base_parameters()
        self._init_base_publishers()
        self._init_base_services()
        self._init_base_execution()
        self._planning_scene_client = self.create_client(
            GetPlanningScene,
            self._planning_scene_service_name,
        )
        self._compute_ik_client = self.create_client(
            GetPositionIK,
            self._compute_ik_service_name,
        )
        self._latest_joint_state: Optional[JointState] = None
        self._joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            10,
        )

        # 6. Init vision subscriptions (only when in vision mode)
        if self._input_mode == "vision":
            self._init_vision_subscriptions()

        # 7. Register /drl/plan and /drl/replan services
        self._planning_lock = threading.RLock()
        self._planning_active = False
        self._planning_thread: Optional[threading.Thread] = None
        self._auto_plan_timer = None
        self._init_unified_services()

        # 8. Startup banner
        self.get_logger().info("=" * 60)
        self.get_logger().info("drl_unified_planner_node started")
        self.get_logger().warn(
            "drl_unified_planner_node is the preferred node for PAP workflow."
        )
        self.get_logger().info(f"  input_mode: {self._input_mode}")
        self.get_logger().info(
            f"  auto_execute_after_plan: {self._auto_execute_after_plan}"
        )
        self.get_logger().info(
            f"  calibrated_start_tcp_base: ({calibrated_start_tcp_base[0]:.4f}, "
            f"{calibrated_start_tcp_base[1]:.4f}, {calibrated_start_tcp_base[2]:.4f})"
        )
        self.get_logger().info(
            f"  start_tcp_drl: ({self._planner.start_tcp_drl[0]:.4f}, "
            f"{self._planner.start_tcp_drl[1]:.4f}, "
            f"{self._planner.start_tcp_drl[2]:.4f})"
        )
        self.get_logger().info(
            f"  Env config: action_step={env_cfg['action_step']}, "
            f"distance_thresh={env_cfg['distance_thresh']}, "
            f"max_episode_steps={env_cfg['max_episode_steps']}"
        )
        self.get_logger().info(
            "  trained_workspace_base: "
            f"min={self._planner.workspace_min_base.tolist()}, "
            f"max={self._planner.workspace_max_base.tolist()}"
        )
        self.get_logger().info(
            f"  PlanningScene obstacles: enabled={self._use_planning_scene_obstacles}, "
            f"service={self._planning_scene_service_name}, "
            f"frame={self._planning_scene_frame}"
        )
        self.get_logger().info(
            f"  Obstacle safety filter: enabled={self._obstacle_safety_filter_enabled}, "
            f"margin={self._obstacle_safety_margin_m:.3f} m, "
            f"check_step={self._obstacle_safety_check_step_m:.3f} m"
        )
        self.get_logger().info(
            f"  MoveIt IK validation: enabled={self._validate_path_with_moveit_ik}, "
            f"service={self._compute_ik_service_name}, "
            f"group={self._moveit_group_name}, link={self._moveit_ik_link_name}"
        )
        if vn_stats is not None:
            self.get_logger().info(
                f"  VecNormalize: norm_obs={vn_stats.norm_obs}, "
                f"obs_rms={'loaded' if vn_stats.obs_rms is not None else 'none'}"
            )
        else:
            self.get_logger().info(
                "  VecNormalize: not loaded, using raw observations"
            )
        self.get_logger().info("=" * 60)

        # 9. Clear trajectory state
        self._publish_empty_all_paths()

        # 10. Auto-plan after the executor starts spinning.  Planning may call
        # task_executor services, so it must run in a worker thread.
        if self._auto_plan_on_start:
            self._auto_plan_timer = self.create_timer(
                1.0, self._on_auto_plan_timer
            )

    # -------------------------------------------------------------------------
    # Vision subscriptions (vision mode only)
    # -------------------------------------------------------------------------

    def _init_vision_subscriptions(self) -> None:
        """Create vision topic subscriptions (only called in vision mode)."""
        self._vision_lock = threading.RLock()
        self._vision_target_detected: bool = False
        self._vision_box_detected: bool = False
        self._vision_target_base: Optional[np.ndarray] = None
        self._vision_box_center_base: Optional[np.ndarray] = None
        self._vision_box_size: Optional[np.ndarray] = None

        self._sub_target_pos = self.create_subscription(
            PointStamped,
            self._vision_target_topic,
            self._on_vision_target_position,
            10,
        )
        self._sub_target_detected = self.create_subscription(
            Bool,
            self._vision_target_detected_topic,
            self._on_vision_target_detected,
            10,
        )
        self._sub_box = self.create_subscription(
            Box if Box else object,
            self._vision_box_topic,
            self._on_vision_box,
            10,
        )
        self._sub_box_detected = self.create_subscription(
            Bool,
            self._vision_box_detected_topic,
            self._on_vision_box_detected,
            10,
        )
        self.get_logger().info(
            f"Vision subscriptions active: target={self._vision_target_topic}, "
            f"box={self._vision_box_topic}"
        )

    def _on_vision_target_position(self, msg: PointStamped) -> None:
        """Callback for the vision target position topic."""
        with self._vision_lock:
            self._vision_target_base = np.array(
                [msg.point.x, msg.point.y, msg.point.z], dtype=np.float32
            )

    def _on_vision_target_detected(self, msg: Bool) -> None:
        """Callback for the vision target detected flag."""
        self._vision_target_detected = msg.data

    def _on_vision_box(self, msg) -> None:
        """Callback for the vision box topic."""
        if Box is None:
            return
        pos = msg.pose.position
        size = msg.size
        with self._vision_lock:
            self._vision_box_center_base = np.array(
                [pos.x, pos.y, pos.z], dtype=np.float32
            )
            self._vision_box_size = np.array(
                [size.x, size.y, size.z], dtype=np.float32
            )

    def _on_vision_box_detected(self, msg: Bool) -> None:
        """Callback for the vision box detected flag."""
        self._vision_box_detected = msg.data

    # -------------------------------------------------------------------------
    # /drl/plan and /drl/replan services
    # -------------------------------------------------------------------------

    def _init_unified_services(self) -> None:
        """Register /drl/plan and /drl/replan services."""
        self._plan_srv = self.create_service(
            Trigger, "/drl/plan", self._on_plan
        )
        self._replan_srv = self.create_service(
            Trigger, "/drl/replan", self._on_plan
        )

    def _on_plan(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Handle /drl/plan and /drl/replan.

        Starts a background worker.  The worker may pre-position the robot via
        task_executor before running RL inference, so the service callback must
        return immediately and let the executor keep spinning.
        """
        return self._start_planning_thread("service")

    def _on_auto_plan_timer(self) -> None:
        """One-shot timer used to start auto planning after node startup."""
        if self._auto_plan_timer is not None:
            self.destroy_timer(self._auto_plan_timer)
            self._auto_plan_timer = None
        resp = self._start_planning_thread("auto_start")
        if resp.success:
            self.get_logger().info(f"[auto_plan] {resp.message}")
        else:
            self.get_logger().warn(f"[auto_plan] {resp.message}")

    def _start_planning_thread(self, reason: str) -> Trigger.Response:
        """Start one background planning worker if none is active."""
        response = Trigger.Response()
        with self._planning_lock:
            if self._planning_active:
                response.success = False
                response.message = "Planning already running."
                return response
            self._planning_active = True

        thread = threading.Thread(
            target=self._planning_worker,
            args=(reason,),
            daemon=True,
        )
        self._planning_thread = thread
        thread.start()

        response.success = True
        response.message = (
            f"Planning started in background ({reason}, mode={self._input_mode})."
        )
        return response

    def _planning_worker(self, reason: str) -> None:
        """Run one plan cycle from a background thread."""
        try:
            self.get_logger().info(
                f"[plan_worker] started | reason={reason} | mode={self._input_mode}"
            )
            if self._input_mode == "manual":
                resp = self._handle_manual_plan()
            else:
                resp = self._handle_vision_plan()

            if resp.success:
                self.get_logger().info(f"[plan_worker] success: {resp.message}")
            else:
                self.get_logger().warn(f"[plan_worker] failed: {resp.message}")
        except Exception as exc:
            self.get_logger().error(f"[plan_worker] exception: {exc}")
        finally:
            with self._planning_lock:
                self._planning_active = False
            self.get_logger().info("[plan_worker] finished")

    def _handle_manual_plan(self) -> Trigger.Response:
        """Prompt terminal and plan (manual mode)."""
        if self._manual_prompt_on_start:
            try:
                target, has_obs, obs_center, obs_size = read_user_input()
            except Exception as e:
                response = Trigger.Response()
                response.success = False
                response.message = f"Failed to read input: {e}"
                self.get_logger().error(f"[/drl/plan] manual input error: {e}")
                return response
        else:
            target, has_obs, obs_center, obs_size = self._get_manual_default_input()

        scene = DrlSceneInput(
            target_base=target,
            has_obstacle=has_obs,
            obstacle_center_base=obs_center,
            obstacle_full_size=obs_size,
            source="manual",
            stamp_sec=self.get_clock().now().seconds_nanoseconds()[0],
        )
        success = self._plan_from_scene_input(scene)
        response = Trigger.Response()
        response.success = success
        if success:
            response.message = (
                f"Manual plan published. target=({target[0]:.4f}, "
                f"{target[1]:.4f}, {target[2]:.4f})"
            )
        else:
            response.message = "Planning failed — check node logs."
        return response

    def _handle_vision_plan(self) -> Trigger.Response:
        """Use latest vision data and plan (vision mode)."""
        with self._vision_lock:
            target_detected = self._vision_target_detected
            target_base = (
                self._vision_target_base.copy()
                if self._vision_target_base is not None
                else None
            )
            box_detected = self._vision_box_detected
            box_center = (
                self._vision_box_center_base.copy()
                if self._vision_box_center_base is not None
                else None
            )
            box_size = (
                self._vision_box_size.copy()
                if self._vision_box_size is not None
                else None
            )

        if self._vision_require_target_detected and not target_detected:
            response = Trigger.Response()
            response.success = False
            response.message = "No valid vision target."
            self.get_logger().warn("[/drl/plan] No valid vision target.")
            return response

        if target_base is None:
            response = Trigger.Response()
            response.success = False
            response.message = "No vision target received yet."
            self.get_logger().warn("[/drl/plan] No vision target received yet.")
            return response

        has_obstacle = (
            self._vision_use_obstacle_if_detected
            and box_detected
            and box_center is not None
            and box_size is not None
        )
        obs_center = box_center if has_obstacle else np.zeros(3, dtype=np.float32)
        obs_size = box_size if has_obstacle else np.zeros(3, dtype=np.float32)

        scene = DrlSceneInput(
            target_base=target_base,
            has_obstacle=has_obstacle,
            obstacle_center_base=obs_center,
            obstacle_full_size=obs_size,
            source="vision",
            stamp_sec=self.get_clock().now().seconds_nanoseconds()[0],
        )
        success = self._plan_from_scene_input(scene)
        response = Trigger.Response()
        response.success = success
        if success:
            response.message = (
                f"Vision plan published. target=({target_base[0]:.4f}, "
                f"{target_base[1]:.4f}, {target_base[2]:.4f})"
            )
        else:
            response.message = "Planning failed — check node logs."
        return response

    # -------------------------------------------------------------------------
    # Common planning logic
    # -------------------------------------------------------------------------

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest_joint_state = msg

    def _fetch_planning_scene_obstacles(self) -> list[SceneObstacle]:
        """Read world collision objects from MoveIt's PlanningScene service."""
        if not self._use_planning_scene_obstacles:
            return []

        if not self._planning_scene_client.service_is_ready():
            if not self._planning_scene_client.wait_for_service(
                timeout_sec=self._planning_scene_timeout_sec
            ):
                self.get_logger().warn(
                    "[planning_scene] service "
                    f"{self._planning_scene_service_name} not available after "
                    f"{self._planning_scene_timeout_sec:.1f}s"
                )
                return []

        req = GetPlanningScene.Request()
        req.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.TRANSFORMS
        )
        future = self._planning_scene_client.call_async(req)
        deadline = time.monotonic() + self._planning_scene_timeout_sec
        while rclpy.ok() and not future.done():
            now_sec = time.monotonic()
            if now_sec > deadline:
                self.get_logger().warn(
                    "[planning_scene] request timed out after "
                    f"{self._planning_scene_timeout_sec:.1f}s"
                )
                return []
            time_sleep = min(0.02, max(0.0, deadline - now_sec))
            threading.Event().wait(time_sleep)

        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().warn(f"[planning_scene] request failed: {exc}")
            return []
        if resp is None:
            self.get_logger().warn("[planning_scene] empty response")
            return []

        try:
            obstacles = planning_scene_to_obstacles(
                resp.scene,
                tf_buffer=self._tf_buffer,
                target_frame=self._planning_scene_frame,
                timeout_sec=self._planning_scene_timeout_sec,
            )
        except Exception as exc:
            message = f"[planning_scene] failed to transform collision objects: {exc}"
            if self._require_planning_scene_obstacle_transform:
                raise PlanningSceneObstacleError(message) from exc
            self.get_logger().warn(message)
            return []

        self.get_logger().info(
            f"[planning_scene] planning_frame={self._planning_scene_frame} | "
            f"world_objects={len(resp.scene.world.collision_objects)} | "
            f"usable_obstacles={len(obstacles)}"
        )
        for obs in obstacles:
            self.get_logger().info(
                f"[planning_scene] obstacle id={obs.object_id} "
                f"src_frame={obs.source_frame} type={obs.primitive_type} "
                f"center_base={np.array2string(obs.center_base, precision=4)} "
                f"full_size={np.array2string(obs.full_size, precision=4)} "
                f"quat_xyzw=({obs.pose_quat_xyzw[0]:.4f},"
                f"{obs.pose_quat_xyzw[1]:.4f},"
                f"{obs.pose_quat_xyzw[2]:.4f},"
                f"{obs.pose_quat_xyzw[3]:.4f})"
            )
        return obstacles

    def _scene_input_obstacles(self, scene: DrlSceneInput) -> list[SceneObstacle]:
        if not scene.has_obstacle:
            return []
        return [
            manual_obstacle(
                scene.obstacle_center_base,
                scene.obstacle_full_size,
                object_id=f"{scene.source}_obstacle",
            )
        ]

    def _resolve_obstacle_for_policy(
        self,
        scene: DrlSceneInput,
    ) -> tuple[DrlSceneInput, list[SceneObstacle]]:
        """Prefer PlanningScene world objects, fallback to scene/manual obstacle."""
        planning_scene_obstacles = self._fetch_planning_scene_obstacles()
        fallback_obstacles = self._scene_input_obstacles(scene)
        validation_obstacles = planning_scene_obstacles or fallback_obstacles

        start = self._planner.start_tcp_base if self._planner is not None else config.DEFAULT_START_TCP_BASE
        selected = select_obstacle_for_policy(
            validation_obstacles,
            start_base=start,
            target_base=scene.target_base,
        )
        if selected is None:
            resolved = DrlSceneInput(
                target_base=scene.target_base,
                has_obstacle=False,
                obstacle_center_base=np.zeros(3, dtype=np.float32),
                obstacle_full_size=np.zeros(3, dtype=np.float32),
                source=f"{scene.source}+scene",
                stamp_sec=scene.stamp_sec,
            )
            return resolved, []

        if planning_scene_obstacles:
            source = f"{scene.source}+planning_scene"
        else:
            source = scene.source
        resolved = DrlSceneInput(
            target_base=scene.target_base,
            has_obstacle=True,
            obstacle_center_base=selected.center_base.copy(),
            obstacle_full_size=selected.full_size.copy(),
            source=source,
            stamp_sec=scene.stamp_sec,
        )
        self.get_logger().info(
            "[/drl/plan] policy obstacle selected "
            f"id={selected.object_id} source={source} "
            f"center_base={np.array2string(selected.center_base, precision=4)} "
            f"full_size={np.array2string(selected.full_size, precision=4)}"
        )
        return resolved, validation_obstacles

    def _point_inside_any_obstacle(
        self,
        point_base: np.ndarray,
        obstacles: list[SceneObstacle],
    ) -> Optional[SceneObstacle]:
        for obstacle in obstacles:
            if point_aabb_signed_distance(point_base, obstacle) < 0.0:
                return obstacle
        return None

    def _validate_path_with_moveit(self, trajectory_base: list[np.ndarray]) -> tuple[bool, str]:
        """Sample the final Cartesian path and ask MoveIt IK to avoid collisions."""
        if not self._validate_path_with_moveit_ik:
            return True, "MoveIt IK validation disabled."
        if not trajectory_base:
            return False, "Trajectory is empty."
        if not self._compute_ik_client.service_is_ready():
            if not self._compute_ik_client.wait_for_service(
                timeout_sec=self._planning_scene_timeout_sec
            ):
                return False, (
                    f"Service {self._compute_ik_service_name} not available "
                    f"after {self._planning_scene_timeout_sec:.1f}s."
                )

        from builtin_interfaces.msg import Duration
        from robot_drl.planning_scene_adapter import iter_cartesian_path_samples

        samples = list(
            iter_cartesian_path_samples(
                trajectory_base,
                max_step_m=self._path_collision_check_step_m,
            )
        )
        if self._moveit_ik_max_samples > 0 and len(samples) > self._moveit_ik_max_samples:
            indices = np.linspace(
                0,
                len(samples) - 1,
                num=self._moveit_ik_max_samples,
                dtype=int,
            )
            samples = [samples[int(i)] for i in indices]

        qx, qy, qz, qw = _EXECUTION_QUATERNION
        failures: list[str] = []
        seed_joint_state = self._latest_joint_state
        if seed_joint_state is None:
            self.get_logger().warn(
                "[/drl/plan] no /joint_states seed yet; MoveIt IK validation "
                "will use MoveIt's default state"
            )
        for seg_idx, sample_idx, sample in samples:
            req = GetPositionIK.Request()
            req.ik_request.group_name = self._moveit_group_name
            req.ik_request.ik_link_name = self._moveit_ik_link_name
            req.ik_request.avoid_collisions = True
            if seed_joint_state is not None:
                req.ik_request.robot_state.joint_state = seed_joint_state
            timeout_sec = max(float(self._moveit_ik_timeout_sec), 0.0)
            req.ik_request.timeout = Duration(
                sec=int(timeout_sec),
                nanosec=int((timeout_sec % 1.0) * 1e9),
            )
            req.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
            req.ik_request.pose_stamped.header.frame_id = self._planning_scene_frame
            req.ik_request.pose_stamped.pose.position.x = float(sample[0])
            req.ik_request.pose_stamped.pose.position.y = float(sample[1])
            req.ik_request.pose_stamped.pose.position.z = float(sample[2])
            req.ik_request.pose_stamped.pose.orientation.x = qx
            req.ik_request.pose_stamped.pose.orientation.y = qy
            req.ik_request.pose_stamped.pose.orientation.z = qz
            req.ik_request.pose_stamped.pose.orientation.w = qw

            future = self._compute_ik_client.call_async(req)
            deadline = time.monotonic() + max(timeout_sec + 0.5, 0.5)
            while rclpy.ok() and not future.done():
                if time.monotonic() > deadline:
                    return False, (
                        f"MoveIt IK validation timed out at segment={seg_idx}, "
                        f"sample={sample_idx}"
                    )
                time.sleep(0.01)
            resp = future.result()
            if resp is None:
                return False, (
                    f"MoveIt IK returned no response at segment={seg_idx}, "
                    f"sample={sample_idx}"
                )
            if resp.error_code.val != 1:
                failures.append(
                    f"segment={seg_idx}, sample={sample_idx}, "
                    f"xyz={np.array2string(sample, precision=4)}, "
                    f"error_code={resp.error_code.val}"
                )
                break
            seed_joint_state = resp.solution.joint_state

        if failures:
            return False, "MoveIt IK/collision validation failed: " + failures[0]
        return True, f"MoveIt IK/collision validation OK for {len(samples)} sample(s)."

    def _plan_from_scene_input(self, scene: DrlSceneInput) -> bool:
        """Shared planning entry point for both manual and vision modes.

        Packages scene input, calls DrlTrajectoryPlannerCore, and publishes results.

        Returns:
            True if planning succeeded (converged or not), False on exception.
        """
        if not self._prepare_tcp_for_plan_blocking(scene.source):
            return False

        try:
            scene, validation_obstacles = self._resolve_obstacle_for_policy(scene)
        except Exception as e:
            self.get_logger().error(f"[/drl/plan] PlanningScene obstacle error: {e}")
            return False

        start_collision = self._point_inside_any_obstacle(
            self._planner.start_tcp_base,
            validation_obstacles,
        )
        if start_collision is not None:
            self.get_logger().error(
                "[/drl/plan] start TCP lies inside PlanningScene obstacle "
                f"'{start_collision.object_id}'; refusing to plan."
            )
            return False
        target_collision = self._point_inside_any_obstacle(
            scene.target_base,
            validation_obstacles,
        )
        if target_collision is not None:
            self.get_logger().error(
                "[/drl/plan] target lies inside PlanningScene obstacle "
                f"'{target_collision.object_id}'; refusing to plan."
            )
            return False

        self.get_logger().info(
            f"[/drl/plan] source={scene.source} | "
            f"planning_frame={self._planning_scene_frame} | "
            f"start_tcp_base={np.array2string(self._planner.start_tcp_base, precision=4)} | "
            f"target_base={np.array2string(scene.target_base, precision=4)} | "
            f"number_of_obstacles={len(validation_obstacles)} | "
            f"policy_obstacle={scene.has_obstacle}"
        )
        if scene.has_obstacle:
            self.get_logger().info(
                f"[/drl/plan] transformed_obstacle_center_base="
                f"{np.array2string(scene.obstacle_center_base, precision=4)} | "
                f"transformed_obstacle_full_size="
                f"{np.array2string(scene.obstacle_full_size, precision=4)}"
            )

        safety_obstacles_base = [
            (obs.center_base.copy(), obs.full_size.copy())
            for obs in validation_obstacles
        ]
        try:
            result = self._planner.compute_trajectory(
                target_base=scene.target_base,
                has_obstacle=scene.has_obstacle,
                obstacle_center_base=scene.obstacle_center_base,
                obstacle_full_size=scene.obstacle_full_size,
                safety_obstacles_base=safety_obstacles_base,
                source=scene.source,
            )
        except Exception as e:
            self.get_logger().error(f"[/drl/plan] Planning exception: {e}")
            return False

        accept_near_convergence = (
            result.convergence_dist <= self._accept_near_convergence_dist_m
            and bool(result.trajectory_forward_base)
            and np.allclose(
                result.trajectory_forward_base[-1],
                scene.target_base,
                rtol=0.0,
                atol=1e-6,
            )
        )
        if not result.converged and not accept_near_convergence:
            self.get_logger().error(
                "[/drl/plan] DRL rollout did not converge; refusing to publish "
                f"or execute partial trajectory (dist={result.convergence_dist:.4f} m)."
            )
            if self._publish_failed_collision_path:
                self.log_planning_result(result, source=scene.source)
                self.publish_planning_result(result)
            return False
        if accept_near_convergence:
            self.get_logger().warn(
                "[/drl/plan] DRL rollout stopped near target "
                f"(dist={result.convergence_dist:.4f} m <= "
                f"{self._accept_near_convergence_dist_m:.4f} m); using appended "
                "exact target waypoint after full collision/IK validation."
            )

        validation = validate_cartesian_path_against_obstacles(
            result.trajectory_forward_base,
            validation_obstacles,
            max_step_m=self._path_collision_check_step_m,
            margin_m=self._path_collision_clearance_margin_m,
        )
        if validation.valid:
            self.get_logger().info(
                "[/drl/plan] Cartesian obstacle validation OK | "
                f"samples={validation.checked_samples} | "
                f"min_clearance={validation.min_clearance:.5f} m | "
                f"{validation.message}"
            )
        else:
            self.get_logger().error(
                "[/drl/plan] Cartesian obstacle validation FAILED | "
                f"{validation.message}"
            )
            if self._publish_failed_collision_path:
                self.log_planning_result(result, source=scene.source)
                self.publish_planning_result(result)
            return False

        ok_moveit, moveit_msg = self._validate_path_with_moveit(
            result.trajectory_forward_base
        )
        if not ok_moveit:
            self.get_logger().error(f"[/drl/plan] {moveit_msg}")
            if self._publish_failed_collision_path:
                self.log_planning_result(result, source=scene.source)
                self.publish_planning_result(result)
            return False
        self.get_logger().info(f"[/drl/plan] {moveit_msg}")

        self.log_planning_result(result, source=scene.source)
        self.publish_planning_result(result)
        if self._auto_execute_after_plan:
            resp, started = self._start_execution_thread(
                self._get_trajectory_for_direction("forward"), "forward"
            )
            if started:
                self.get_logger().info(
                    f"[/drl/plan] Auto execution started: {resp.message}"
                )
            else:
                self.get_logger().warn(
                    f"[/drl/plan] Auto execution not started: {resp.message}"
                )
        return True

    # -------------------------------------------------------------------------
    # Manual auto-plan helper (called on startup when enabled)
    # -------------------------------------------------------------------------

    def _plan_manual_once(self) -> None:
        """Prompt terminal once and plan (used for auto_plan_on_start in manual mode)."""
        if self._manual_prompt_on_start:
            self.get_logger().info(
                "[auto_plan] Prompting for manual target input on startup..."
            )
            try:
                target, has_obs, obs_center, obs_size = read_user_input()
            except Exception as e:
                self.get_logger().error(f"[auto_plan] Input error: {e}")
                return
        else:
            self.get_logger().info(
                "[auto_plan] Using manual_default_target/manual_default_obstacle_* parameters."
            )
            target, has_obs, obs_center, obs_size = self._get_manual_default_input()

        scene = DrlSceneInput(
            target_base=target,
            has_obstacle=has_obs,
            obstacle_center_base=obs_center,
            obstacle_full_size=obs_size,
            source="manual",
            stamp_sec=self.get_clock().now().seconds_nanoseconds()[0],
        )
        self._plan_from_scene_input(scene)

    def _get_manual_default_input(self) -> tuple[np.ndarray, bool, np.ndarray, np.ndarray]:
        """Return non-interactive manual defaults from ROS parameters."""
        self._manual_default_target = np.array(
            self.get_parameter("manual_default_target").value,
            dtype=np.float32,
        )
        self._manual_default_obstacle_center = np.array(
            self.get_parameter("manual_default_obstacle_center").value,
            dtype=np.float32,
        )
        self._manual_default_obstacle_size = np.array(
            self.get_parameter("manual_default_obstacle_size").value,
            dtype=np.float32,
        )
        self._manual_allow_skip_obstacle = bool(
            self.get_parameter("manual_allow_skip_obstacle").value
        )
        has_obstacle = not (
            self._manual_allow_skip_obstacle
            and np.allclose(self._manual_default_obstacle_size, 0.0)
        )
        if has_obstacle:
            obstacle_center = self._manual_default_obstacle_center.copy()
            obstacle_size = self._manual_default_obstacle_size.copy()
        else:
            obstacle_center = np.zeros(3, dtype=np.float32)
            obstacle_size = np.zeros(3, dtype=np.float32)
        return (
            self._manual_default_target.copy(),
            has_obstacle,
            obstacle_center,
            obstacle_size,
        )


# -------------------------------------------------------------------------
# Main
# -------------------------------------------------------------------------

def main(argv=None):
    import argparse

    parser = argparse.ArgumentParser(
        description="DRL Unified Planner Node — manual or vision input mode"
    )
    parser.add_argument(
        "--model", type=str, default=config.DEFAULT_MODEL_NAME,
        help="Model filename in install/share/robot_drl/models/"
    )
    parser.add_argument(
        "--vec-normalize", type=str, default=config.DEFAULT_VEC_NORMALIZE_NAME,
        help="VecNormalize stats filename"
    )
    parser.add_argument(
        "--env-config", type=str, default=None,
        help="Path to env_config.yaml (auto-detected if not provided)"
    )
    parser.add_argument(
        "--calibrated-start-tcp-base", type=str, default=None,
        help="Override calibrated_start_tcp_base as 'x,y,z'"
    )
    args, unknown = parser.parse_known_args(argv or sys.argv[1:])

    # Load model, VecNormalize, env config
    planner, env_cfg = load_planner(
        model_subpath=args.model,
        vec_normalize_subpath=args.vec_normalize,
        env_config_path=args.env_config,
    )

    # Apply CLI override for start TCP
    if args.calibrated_start_tcp_base:
        try:
            parts = [float(x.strip()) for x in args.calibrated_start_tcp_base.split(",")]
            if len(parts) != 3:
                raise ValueError("Must be x,y,z")
            planner.update_start_tcp(np.array(parts, dtype=np.float32))
            print(
                f"[drl_unified_planner_node] CLI override: "
                f"calibrated_start_tcp_base = ({parts[0]:.4f}, {parts[1]:.4f}, {parts[2]:.4f})"
            )
        except Exception as e:
            sys.stderr.write(
                f"[drl_unified_planner_node] ERROR: invalid calibrated_start_tcp_base "
                f"format: {args.calibrated_start_tcp_base} — {e}\n"
            )
            sys.exit(1)

    rclpy.init(args=sys.argv)
    try:
        node = DrlUnifiedPlannerNode(
            model=planner._model,
            vn_stats=planner.vn_stats,
            env_cfg=env_cfg,
            calibrated_start_tcp_base=planner.calibrated_start_tcp_base,
        )

        # Override start TCP if CLI override was used
        if args.calibrated_start_tcp_base:
            parts = [float(x.strip()) for x in args.calibrated_start_tcp_base.split(",")]
            node._planner.update_start_tcp(np.array(parts, dtype=np.float32))

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
