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
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool

from robot_drl import config
from robot_drl.drl_planner_core import (
    DrlTrajectoryPlannerCore,
    PlanningResult,
    load_planner,
)
from robot_drl.drl_planner_node_base import DrlPlannerNodeBase
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

        # 6. Init vision subscriptions (only when in vision mode)
        if self._input_mode == "vision":
            self._init_vision_subscriptions()

        # 7. Register /drl/plan and /drl/replan services
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
        if vn_stats is not None:
            self.get_logger().info(
                f"  VecNormalize: norm_obs={vn_stats.norm_obs}, "
                f"obs_rms={'loaded' if vn_stats.obs_rms is not None else 'none'}"
            )
        else:
            self.get_logger().info(
                "  VecNormalize: file not found, using raw observations"
            )
        self.get_logger().info("=" * 60)

        # 9. Clear trajectory state
        self._publish_empty_all_paths()

        # 10. Auto-plan on start (manual mode only)
        if self._auto_plan_on_start and self._input_mode == "manual":
            self._plan_manual_once()

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

        In manual mode: re-prompt terminal and plan.
        In vision mode: use latest vision data and plan.
        """
        if self._input_mode == "manual":
            return self._handle_manual_plan()
        else:
            return self._handle_vision_plan()

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

    def _plan_from_scene_input(self, scene: DrlSceneInput) -> bool:
        """Shared planning entry point for both manual and vision modes.

        Packages scene input, calls DrlTrajectoryPlannerCore, and publishes results.

        Returns:
            True if planning succeeded (converged or not), False on exception.
        """
        self.get_logger().info(
            f"[/drl/plan] source={scene.source} | "
            f"target=({scene.target_base[0]:.4f}, {scene.target_base[1]:.4f}, "
            f"{scene.target_base[2]:.4f}) | "
            f"obstacle={scene.has_obstacle}"
        )
        if scene.has_obstacle:
            self.get_logger().info(
                f"[/drl/plan] obs_center=({scene.obstacle_center_base[0]:.4f}, "
                f"{scene.obstacle_center_base[1]:.4f}, "
                f"{scene.obstacle_center_base[2]:.4f}) | "
                f"obs_size=({scene.obstacle_full_size[0]:.4f}, "
                f"{scene.obstacle_full_size[1]:.4f}, "
                f"{scene.obstacle_full_size[2]:.4f})"
            )

        try:
            result = self._planner.compute_trajectory(
                target_base=scene.target_base,
                has_obstacle=scene.has_obstacle,
                obstacle_center_base=scene.obstacle_center_base,
                obstacle_full_size=scene.obstacle_full_size,
                source=scene.source,
            )
        except Exception as e:
            self.get_logger().error(f"[/drl/plan] Planning exception: {e}")
            return False

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
